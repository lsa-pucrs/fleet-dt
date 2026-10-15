#!/usr/bin/env python3
"""Telemetry publisher and autopilot interface of a Fleet-DT vessel (R2).

Chain on the vessel RP4: ArduRover (NAVIO2) --MAVLink/UDP--> MAVProxy --UDP--> this program.

Uplink (vessel MQTT broker, QoS 0, namespace boat/b<i>/raw):
  * imu, mag, gps, baro, power: packed binary messages of fixed size (layouts below), each topic at
    most at its DTP rate (RateLimiter: token bucket at the DTP rate plus at most n_l = ceil(r Ts)
    messages per regulator step window).
  * state: the latest EKF state (GLOBAL_POSITION_INT + ATTITUDE) once per step, at the instant
    start(t) + offset of each step t (default offset 5 ms), so the state rate equals 1/Ts.

Downlink: boat/b<i>/goals (JSON {"t", "lat", "lon", "speed"}). For each goal newer than the last
accepted one, the program switches ArduRover to GUIDED (MAV_CMD_DO_SET_MODE), sets the ground speed
(MAV_CMD_DO_CHANGE_SPEED) and sends SET_POSITION_TARGET_GLOBAL_INT.

Payload layouts (network byte order). Sensor layouts start with a version byte (LAYOUT_VERSION);
time_us is the time since boot of the autopilot in microseconds. The state layout uses the whole
48-byte bound of the topic and has no version byte.

  state 48 B  "!ii10f"   lat, lon (int32, 1e-7 deg), alt (f32, m, MSL), vx, vy, vz (f32, m/s, NED),
                         roll, pitch, yaw (f32, rad), rollspeed, pitchspeed, yawspeed (f32, rad/s)
  imu   36 B  "!BBQ6fh"  ver, imu_id, time_us, ax, ay, az (m/s^2), gx, gy, gz (rad/s), temp (cdegC)
  mag   22 B  "!BBQ3f"   ver, mag_id, time_us, mx, my, mz (gauss)
  gps   53 B  "!BQBiiiHHHHBiIIIIH"
                         ver, time_us (GPS_RAW_INT.time_usec), fix_type, lat, lon (1e-7 deg), alt (mm MSL),
                         eph, epv (1e-2), vel (cm/s), cog (cdeg), satellites, alt_ellipsoid (mm),
                         h_acc, v_acc (mm), vel_acc (mm/s), hdg_acc (1e-5 deg), yaw (cdeg)
  baro  21 B  "!BQffhh"  ver, time_us, press_abs (hPa), press_diff (hPa), temp (cdegC), temp_diff (cdegC)
  power 28 B  "!BQHhbHiihH"
                         ver, time_us, voltage (mV), current (cA), remaining (%), load (d%),
                         current_consumed (mAh), energy_consumed (hJ), battery temp (cdegC),
                         drop_rate_comm (c%)

Usage:
  mav_publisher.py --dtp /etc/fleetdt/dtp.json --vessel 1 [--mavlink udpin:0.0.0.0:14551]
                   [--host 127.0.0.1] [--port 1883] [--epoch T0] [--state-offset 0.005]
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import signal
import struct
import sys
import threading
import time
from typing import Dict, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# ArduRover and MAVProxy speak MAVLink 2; the MAVLink 2 dialect also holds the extension fields
# (RAW_IMU.id, RAW_IMU.temperature, GPS_RAW_INT.h_acc, ...).
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

from common import DTPFile, RateLimiter, StepClock, make_client, raw_topic, sleep_until  # noqa: E402

log = logging.getLogger("mav_publisher")

LAYOUT_VERSION = 1

LAYOUTS: Dict[str, struct.Struct] = {
    "state": struct.Struct("!ii10f"),
    "imu": struct.Struct("!BBQ6fh"),
    "mag": struct.Struct("!BBQ3f"),
    "gps": struct.Struct("!BQBiiiHHHHBiIIIIH"),
    "baro": struct.Struct("!BQffhh"),
    "power": struct.Struct("!BQHhbHiihH"),
}

GRAVITY = 9.80665
ROVER_GUIDED = 15                 # ArduRover custom mode number of GUIDED
POSITION_ONLY_MASK = 0x0DF8       # ignore velocity, acceleration, yaw and yaw rate of the target

IMU_SOURCES = {"RAW_IMU": 0, "SCALED_IMU": 0, "SCALED_IMU2": 1, "SCALED_IMU3": 2}

MSG_ID = {
    "RAW_IMU": mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
    "SCALED_IMU": mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU,
    "SCALED_IMU2": mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2,
    "SCALED_IMU3": mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU3,
    "GPS_RAW_INT": mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
    "SCALED_PRESSURE": mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
    "SYS_STATUS": mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
    "BATTERY_STATUS": mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
    "GLOBAL_POSITION_INT": mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    "ATTITUDE": mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
}


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _field(msg, name, default=0):
    """Field of a MAVLink message, or `default` for an extension field that a MAVLink 1 sender omits."""
    v = getattr(msg, name, default)
    return default if v is None else v


# ---------------------------------------------------------------------------------------------------
# Encoders (one per topic). Each returns bytes of the fixed size of the layout.
# ---------------------------------------------------------------------------------------------------

def encode_state(gpi, att) -> bytes:
    return LAYOUTS["state"].pack(
        int(gpi.lat), int(gpi.lon), gpi.alt / 1000.0,
        gpi.vx / 100.0, gpi.vy / 100.0, gpi.vz / 100.0,
        att.roll, att.pitch, att.yaw, att.rollspeed, att.pitchspeed, att.yawspeed)


def imu_time_us(msg) -> int:
    if msg.get_type() == "RAW_IMU":
        return int(msg.time_usec)
    return int(msg.time_boot_ms) * 1000


def encode_imu(msg, imu_id: int) -> bytes:
    # ArduPilot fills RAW_IMU in the units of SCALED_IMU: acceleration in mG, rate in mrad/s.
    a = GRAVITY / 1000.0
    return LAYOUTS["imu"].pack(
        LAYOUT_VERSION, _field(msg, "id", imu_id) & 0xFF, imu_time_us(msg),
        msg.xacc * a, msg.yacc * a, msg.zacc * a,
        msg.xgyro / 1000.0, msg.ygyro / 1000.0, msg.zgyro / 1000.0,
        _clamp(int(_field(msg, "temperature", 0)), -32768, 32767))


def encode_mag(msg, mag_id: int) -> bytes:
    return LAYOUTS["mag"].pack(
        LAYOUT_VERSION, _field(msg, "id", mag_id) & 0xFF, imu_time_us(msg),
        msg.xmag / 1000.0, msg.ymag / 1000.0, msg.zmag / 1000.0)


def encode_gps(msg) -> bytes:
    return LAYOUTS["gps"].pack(
        LAYOUT_VERSION, int(msg.time_usec), int(msg.fix_type),
        int(msg.lat), int(msg.lon), int(msg.alt),
        int(msg.eph), int(msg.epv), int(msg.vel), int(msg.cog), int(msg.satellites_visible),
        int(_field(msg, "alt_ellipsoid")), int(_field(msg, "h_acc")), int(_field(msg, "v_acc")),
        int(_field(msg, "vel_acc")), int(_field(msg, "hdg_acc")), int(_field(msg, "yaw")))


def encode_baro(msg) -> bytes:
    return LAYOUTS["baro"].pack(
        LAYOUT_VERSION, int(msg.time_boot_ms) * 1000, float(msg.press_abs), float(msg.press_diff),
        _clamp(int(msg.temperature), -32768, 32767),
        _clamp(int(_field(msg, "temperature_press_diff")), -32768, 32767))


def encode_power(sys_status, battery, time_us: int) -> bytes:
    consumed = energy = 0
    temp = 32767  # INT16_MAX: unknown, as in BATTERY_STATUS
    if battery is not None:
        consumed, energy, temp = int(battery.current_consumed), int(battery.energy_consumed), int(battery.temperature)
    return LAYOUTS["power"].pack(
        LAYOUT_VERSION, time_us, int(sys_status.voltage_battery) & 0xFFFF,
        _clamp(int(sys_status.current_battery), -32768, 32767),
        _clamp(int(sys_status.battery_remaining), -128, 127), int(sys_status.load) & 0xFFFF,
        consumed, energy, _clamp(temp, -32768, 32767), int(sys_status.drop_rate_comm) & 0xFFFF)


# Decoders (used by the tests and by GS-side tools).

def decode(name: str, payload: bytes) -> tuple:
    return LAYOUTS[name].unpack(payload)


# ---------------------------------------------------------------------------------------------------


class MavPublisher:
    def __init__(self, a: argparse.Namespace):
        self.a = a
        self.dtp = DTPFile.load(a.dtp)
        self.vessel = a.vessel
        self.clock = StepClock(self.dtp.Ts, self.dtp.T1, self.dtp.T2,
                               a.epoch if a.epoch is not None else self.dtp.epoch)
        for name, lay in LAYOUTS.items():
            if name not in self.dtp.topics:
                raise SystemExit("topic %s missing in %s" % (name, a.dtp))
            if lay.size > self.dtp.topics[name].bound:
                raise SystemExit("layout of %s has %d bytes, above the DTP bound %d"
                                 % (name, lay.size, self.dtp.topics[name].bound))
        self.limit = {n: RateLimiter(self.dtp.topics[n].rate, self.dtp.per_step(n), self.clock)
                      for n in ("imu", "mag", "gps", "baro", "power")}
        self.imu_src = a.imu_source
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.gpi = self.att = None
        self.gpi_at = self.att_at = 0.0
        self.battery = None
        self.boot_us = 0
        self.mode: Optional[int] = None
        self.target_system: Optional[int] = None
        self.target_component = 1
        self.counts: Dict[str, int] = {n: 0 for n in LAYOUTS}
        self.last_goal_t: Optional[float] = None
        self.last_goal: Optional[dict] = None
        self.pending_goal: Optional[dict] = None
        self.goal_cv = threading.Condition()
        self.last_speed: Optional[float] = None
        self.send_lock = threading.Lock()
        self.t_start = time.monotonic()

        self.mav = mavutil.mavlink_connection(a.mavlink, source_system=a.source_system,
                                              source_component=a.source_component, autoreconnect=True)
        self.mqtt = make_client("mav", self.vessel, a.host, a.port,
                                on_connect=self._on_connect, on_message=self._on_message)

    # --- MQTT -----------------------------------------------------------------------------------------

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            log.warning("MQTT connection refused: %s", reason_code)
            return
        client.subscribe("boat/b%d/goals" % self.vessel, qos=1)
        log.info("connected to the vessel MQTT broker %s:%d", self.a.host, self.a.port)

    def _publish(self, name: str, payload: bytes) -> None:
        self.mqtt.publish(raw_topic(self.vessel, name), payload, qos=0)
        self.counts[name] += 1

    def _on_message(self, client, userdata, msg):
        try:
            g = json.loads(msg.payload)
            goal = {"t": float(g["t"]), "lat": float(g["lat"]), "lon": float(g["lon"]),
                    "speed": None if g.get("speed") is None else float(g["speed"])}
        except (ValueError, KeyError, TypeError) as e:
            log.warning("invalid goal %r: %s", msg.payload[:200], e)
            return
        if not (-90.0 <= goal["lat"] <= 90.0 and -180.0 <= goal["lon"] <= 180.0):
            log.warning("goal outside the coordinate range: %s", goal)
            return
        with self.goal_cv:
            # Goals older than the last accepted goal (by t) and repeated deliveries of the last goal
            # (QoS 1 resend) are ignored.
            if self.last_goal_t is not None and (goal["t"] < self.last_goal_t or goal == self.last_goal):
                log.info("goal ignored (t=%s, last t=%s)", goal["t"], self.last_goal_t)
                return
            self.last_goal_t, self.last_goal = goal["t"], goal
            self.pending_goal = goal
            self.goal_cv.notify()

    # --- MAVLink output -------------------------------------------------------------------------------

    def _command_long(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        with self.send_lock:
            self.mav.mav.command_long_send(self.target_system, self.target_component, cmd, 0,
                                           p1, p2, p3, p4, p5, p6, p7)

    def request_intervals(self) -> None:
        """MAV_CMD_SET_MESSAGE_INTERVAL for every message that the topics need, at the DTP rates."""
        t = self.dtp.topics
        f = self.a.oversample
        state_hz = self.a.state_source_rate or 2.0 / self.dtp.Ts
        want = {
            self.imu_src: max(t["imu"].rate, t["mag"].rate) * f,
            "GPS_RAW_INT": t["gps"].rate * f,
            "SCALED_PRESSURE": t["baro"].rate * f,
            "SYS_STATUS": t["power"].rate * f,
            "BATTERY_STATUS": 1.0,
            "GLOBAL_POSITION_INT": state_hz,
            "ATTITUDE": state_hz,
        }
        for name, hz in want.items():
            self._command_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, MSG_ID[name], int(round(1e6 / hz)))
        log.info("requested message intervals: %s", {k: round(v, 2) for k, v in want.items()})

    def _goal_worker(self) -> None:
        while not self.stop.is_set():
            with self.goal_cv:
                while self.pending_goal is None and not self.stop.is_set():
                    self.goal_cv.wait(0.5)
                goal, self.pending_goal = self.pending_goal, None
            if goal is None:
                continue
            if self.target_system is None:
                log.warning("goal t=%s without a heartbeat of the autopilot; waiting", goal["t"])
                deadline = time.monotonic() + self.a.mode_timeout
                while self.target_system is None and time.monotonic() < deadline and not self.stop.is_set():
                    time.sleep(0.05)
                if self.target_system is None:
                    continue
            self._apply_goal(goal)

    def _apply_goal(self, goal: dict) -> None:
        if self.mode != ROVER_GUIDED:
            deadline = time.monotonic() + self.a.mode_timeout
            next_send = 0.0
            while self.mode != ROVER_GUIDED and time.monotonic() < deadline and not self.stop.is_set():
                if time.monotonic() >= next_send:
                    self._command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                       mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ROVER_GUIDED)
                    next_send = time.monotonic() + 0.5
                time.sleep(0.02)
            if self.mode != ROVER_GUIDED:
                log.warning("autopilot did not report GUIDED within %.1f s; sending the target anyway",
                            self.a.mode_timeout)
        if goal["speed"] is not None and goal["speed"] > 0 and goal["speed"] != self.last_speed:
            self._command_long(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 1, goal["speed"], -1)
            self.last_speed = goal["speed"]
        with self.send_lock:
            self.mav.mav.set_position_target_global_int_send(
                int((time.monotonic() - self.t_start) * 1000) & 0xFFFFFFFF,
                self.target_system, self.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, POSITION_ONLY_MASK,
                int(round(goal["lat"] * 1e7)), int(round(goal["lon"] * 1e7)), 0.0,
                0, 0, 0, 0, 0, 0, 0, 0)
        log.info("goal t=%s sent: lat=%.7f lon=%.7f speed=%s", goal["t"], goal["lat"], goal["lon"], goal["speed"])

    # --- MAVLink input --------------------------------------------------------------------------------

    def _rx_loop(self) -> None:
        last_request = None
        while not self.stop.is_set():
            msg = self.mav.recv_match(blocking=True, timeout=0.2)
            now = time.time()
            if msg is None:
                continue
            typ = msg.get_type()
            if typ == "BAD_DATA":
                continue
            tb = getattr(msg, "time_boot_ms", None)
            if tb is not None:
                self.boot_us = int(tb) * 1000
            if typ == "HEARTBEAT":
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS or msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                    continue
                first = self.target_system is None
                self.target_system = msg.get_srcSystem()
                self.target_component = msg.get_srcComponent()
                self.mode = int(msg.custom_mode)
                mono = time.monotonic()
                if first or last_request is None or mono - last_request > self.a.request_period:
                    if first:
                        log.info("autopilot sysid %d compid %d", self.target_system, self.target_component)
                    self.request_intervals()
                    last_request = mono
            elif typ == "GLOBAL_POSITION_INT":
                with self.lock:
                    self.gpi, self.gpi_at = msg, now
            elif typ == "ATTITUDE":
                with self.lock:
                    self.att, self.att_at = msg, now
            elif typ == self.imu_src:
                if self.limit["imu"].allow(now):
                    self._publish("imu", encode_imu(msg, IMU_SOURCES[typ]))
                if self.limit["mag"].allow(now):
                    self._publish("mag", encode_mag(msg, IMU_SOURCES[typ]))
            elif typ == "GPS_RAW_INT":
                if self.limit["gps"].allow(now):
                    self._publish("gps", encode_gps(msg))
            elif typ == "SCALED_PRESSURE":
                if self.limit["baro"].allow(now):
                    self._publish("baro", encode_baro(msg))
            elif typ == "BATTERY_STATUS":
                if int(msg.id) == 0:
                    self.battery = msg
            elif typ == "SYS_STATUS":
                if self.limit["power"].allow(now):
                    self._publish("power", encode_power(msg, self.battery, self.boot_us))

    # --- state: one message per step at start(t) + offset ---------------------------------------------

    def _state_loop(self) -> None:
        Ts, off = self.dtp.Ts, self.a.state_offset
        t = self.clock.step() + 1
        while not self.stop.is_set():
            at = self.clock.start(t) + off
            if not sleep_until(at, self.stop):
                return
            with self.lock:
                gpi, att, ga, aa = self.gpi, self.att, self.gpi_at, self.att_at
            now = time.time()
            if gpi is not None and att is not None and now - min(ga, aa) <= self.a.state_max_age:
                self._publish("state", encode_state(gpi, att))
            # next step; a late wake-up (overload, clock slew) skips the steps whose instant passed
            t = max(t + 1, math.floor((time.time() - self.clock.epoch - off) / Ts) + 1)

    def _heartbeat_loop(self) -> None:
        """HEARTBEAT of this onboard computer at 1 Hz. An output udpin of MAVProxy learns the address of
        a udpout client from its first packet, so without this the program would wait for the
        autopilot heartbeat forever."""
        while not self.stop.is_set():
            with self.send_lock:
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                            mavutil.mavlink.MAV_STATE_ACTIVE)
            self.stop.wait(1.0)

    def _stats_loop(self) -> None:
        last = dict(self.counts)
        while not self.stop.wait(self.a.stats_period):
            cur = dict(self.counts)
            rate = {k: round((cur[k] - last[k]) / self.a.stats_period, 2) for k in cur}
            log.info("published messages per second: %s", rate)
            last = cur

    def run(self) -> None:
        threads = [threading.Thread(target=f, name=f.__name__, daemon=True)
                   for f in (self._rx_loop, self._state_loop, self._goal_worker, self._heartbeat_loop)]
        if self.a.stats_period > 0:
            threads.append(threading.Thread(target=self._stats_loop, daemon=True))
        for th in threads:
            th.start()
        self.stop.wait()
        with self.goal_cv:
            self.goal_cv.notify_all()
        for th in threads:
            th.join(timeout=1.0)
        self.mqtt.loop_stop()
        self.mqtt.disconnect()
        self.mav.close()


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Fleet-DT telemetry publisher and autopilot interface (R2)")
    ap.add_argument("--dtp", required=True, help="DTP file (dtp.json)")
    ap.add_argument("--vessel", type=int, required=True, help="vessel index i (topics boat/b<i>/...)")
    ap.add_argument("--mavlink", default="udpin:0.0.0.0:14551",
                    help="pymavlink connection string of the MAVProxy output (default udpin:0.0.0.0:14551)")
    ap.add_argument("--host", default="127.0.0.1", help="vessel MQTT broker")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--epoch", type=float, default=None,
                    help="Unix time of the start of step 0 (default: epoch of the DTP file; 2026-01-01T00:00:00Z when null)")
    ap.add_argument("--state-offset", type=float, default=0.005,
                    help="phase of the state publication after start(t), seconds (default 0.005)")
    ap.add_argument("--state-max-age", type=float, default=1.0,
                    help="largest age of GLOBAL_POSITION_INT and ATTITUDE for a state publication, seconds")
    ap.add_argument("--state-source-rate", type=float, default=None,
                    help="requested rate of GLOBAL_POSITION_INT and ATTITUDE, Hz (default 2/Ts)")
    ap.add_argument("--imu-source", choices=sorted(IMU_SOURCES), default="RAW_IMU",
                    help="MAVLink message of the imu and mag topics (default RAW_IMU)")
    ap.add_argument("--oversample", type=float, default=1.0,
                    help="factor on the DTP rates in the message interval requests (default 1)")
    ap.add_argument("--request-period", type=float, default=30.0,
                    help="period of the repeated message interval requests, seconds (autopilot reboot)")
    ap.add_argument("--mode-timeout", type=float, default=2.0,
                    help="seconds to wait for GUIDED before sending a target")
    ap.add_argument("--source-system", type=int, default=254, help="MAVLink system id of this program")
    ap.add_argument("--source-component", type=int, default=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    ap.add_argument("--stats-period", type=float, default=10.0, help="seconds between rate logs (0: off)")
    ap.add_argument("--log-level", default="INFO")
    return ap.parse_args(argv)


def main(argv=None) -> None:
    a = parse_args(argv)
    logging.basicConfig(level=a.log_level.upper(), format="%(asctime)s %(name)s %(levelname)s %(message)s")
    p = MavPublisher(a)
    signal.signal(signal.SIGTERM, lambda *_: p.stop.set())
    signal.signal(signal.SIGINT, lambda *_: p.stop.set())
    p.run()


if __name__ == "__main__":
    main()
