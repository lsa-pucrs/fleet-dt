"""Boat-side publisher: the Navio2 sensor groups over MQTT, from ArduPilot.

Reads MAVLink from MAVProxy (``udpout:127.0.0.1:14551``; that port never
speaks first, so the heartbeat below is what makes MAVProxy start forwarding),
keeps the latest sample of each source message, and publishes every group of
``topics.py`` at its own rate. Same topics and ping/pong echo as the synthetic
``publisher.py``, which stays as the link-test load generator.

Source of each group (MAVLink message -> JSON fields):

    imu    RAW_IMU              a_* m/s^2, w_* rad/s (primary IMU)
    mag    RAW_IMU              m_* uT (the compass ArduPilot uses)
    gps    GPS_RAW_INT          lat, lon deg, h m MSL, v_n/v_e m/s from
                                ground speed and course; v_d is null because
                                GPS_RAW_INT carries no vertical velocity
    baro   SCALED_PRESSURE      p_pa Pa, t_c degC
    power  SYS_STATUS           v_b V, i_b A (null when not measured)
    state  GLOBAL_POSITION_INT  lat, lon, h and the EKF NED velocity rotated
           + ATTITUDE           into body axes (u, v, w m/s); roll, pitch,
                                yaw deg; p, q, r rad/s

Rate regulation happens at the source: the publisher asks ArduPilot for each
message at its group's rate (MAV_CMD_SET_MESSAGE_INTERVAL) and re-asks every
10 s, because MAVProxy's stream-rate request on reconnect resets it.

A group whose source has no new sample since its last publish is skipped, not
repeated: a message on the wire always carries a fresh reading.

Every payload carries three times: ``t`` (Pi system clock, chrony-disciplined,
at publish), ``t_rx`` (Pi system clock when the MAVLink sample arrived) and
``t_boot_ms`` (ArduPilot's boot clock at the sample; null for SYS_STATUS,
which has no timestamp), plus ``seq`` per group.

Run on the Pi:

    python3 mqtt/pi/mav_publisher.py --broker localhost --boat-id b1
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import paho.mqtt.client as mqtt
from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from topics import SENSOR_FIELDS, SENSOR_RATES_HZ, cmd, ping, pong, sensor  # noqa: E402

GRAVITY = 9.80665

SOURCES = {
    "imu": ("RAW_IMU",),
    "mag": ("RAW_IMU",),
    "gps": ("GPS_RAW_INT",),
    "baro": ("SCALED_PRESSURE",),
    "power": ("SYS_STATUS",),
    "state": ("GLOBAL_POSITION_INT", "ATTITUDE"),
}
"""MAVLink messages each group is built from."""

MESSAGE_IDS = {
    "RAW_IMU": 27, "GPS_RAW_INT": 24, "SCALED_PRESSURE": 29,
    "SYS_STATUS": 1, "GLOBAL_POSITION_INT": 33, "ATTITUDE": 30,
}


def unset(value, sentinel):
    """MAVLink marks an unknown field with a sentinel; JSON gets null."""
    return None if value == sentinel else value


def unwrap_us(time_usec, ref_ms):
    """Boot time in ms from a 32-bit microsecond stamp. ArduRover 4.0 fills
    RAW_IMU.time_usec from micros(), which wraps every 71.6 min; ref_ms is a
    recent time_boot_ms from any other message."""
    wrap = 1 << 32
    ref_us = int(ref_ms * 1000)
    if ref_us - time_usec > wrap // 2:
        time_usec += ((ref_us - time_usec + wrap // 2) // wrap) * wrap
    return time_usec / 1000.0


def ned_to_body(vn, ve, vd, roll, pitch, yaw):
    """Rotate a NED vector into body axes (ZYX Euler, radians)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    u = cp * cy * vn + cp * sy * ve - sp * vd
    v = (sr * sp * cy - cr * sy) * vn + (sr * sp * sy + cr * cy) * ve + sr * cp * vd
    w = (cr * sp * cy + sr * sy) * vn + (cr * sp * sy - sr * cy) * ve + cr * cp * vd
    return u, v, w


def build(group, latest, boot_ref_ms):
    """Field values and the sample's boot time for one group, from the cache."""
    if group in ("imu", "mag"):
        m = latest["RAW_IMU"][0]
        stamp = unwrap_us(m.time_usec, boot_ref_ms)
        if group == "imu":
            return (m.xacc * GRAVITY / 1000.0, m.yacc * GRAVITY / 1000.0,
                    m.zacc * GRAVITY / 1000.0, m.xgyro / 1000.0, m.ygyro / 1000.0,
                    m.zgyro / 1000.0), stamp
        return (m.xmag / 10.0, m.ymag / 10.0, m.zmag / 10.0), stamp
    if group == "gps":
        m = latest["GPS_RAW_INT"][0]
        speed = unset(m.vel, 65535)
        course = unset(m.cog, 65535)
        if speed is None or course is None:
            v_n = v_e = None
        else:
            v_n = speed / 100.0 * math.cos(math.radians(course / 100.0))
            v_e = speed / 100.0 * math.sin(math.radians(course / 100.0))
        # time_usec is the last fix time and stays 0 until the first fix.
        stamp = m.time_usec / 1000.0 if m.time_usec else None
        return (m.lat / 1e7, m.lon / 1e7, m.alt / 1000.0, v_n, v_e, None), stamp
    if group == "baro":
        m = latest["SCALED_PRESSURE"][0]
        return (m.press_abs * 100.0, m.temperature / 100.0), float(m.time_boot_ms)
    if group == "power":
        m = latest["SYS_STATUS"][0]
        current = unset(m.current_battery, -1)
        return (m.voltage_battery / 1000.0,
                None if current is None else current / 100.0), None
    pos = latest["GLOBAL_POSITION_INT"][0]
    att = latest["ATTITUDE"][0]
    u, v, w = ned_to_body(pos.vx / 100.0, pos.vy / 100.0, pos.vz / 100.0,
                          att.roll, att.pitch, att.yaw)
    return (pos.lat / 1e7, pos.lon / 1e7, pos.alt / 1000.0,
            math.degrees(att.roll), math.degrees(att.pitch), math.degrees(att.yaw),
            u, v, w, att.rollspeed, att.pitchspeed, att.yawspeed), float(pos.time_boot_ms)


def request_rates(link, rates):
    """Ask ArduPilot for each source message at the fastest rate that needs it."""
    per_message = {}
    for group, rate_hz in rates.items():
        for name in SOURCES[group]:
            per_message[name] = max(per_message.get(name, 0.0), rate_hz)
    for name, rate_hz in per_message.items():
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            MESSAGE_IDS[name], 1e6 / rate_hz, 0, 0, 0, 0, 0)


def parse_rates(overrides):
    """SENSOR_RATES_HZ with ``group=hz`` overrides from the command line."""
    rates = dict(SENSOR_RATES_HZ)
    for item in overrides:
        group, _, value = item.partition("=")
        if group not in rates:
            raise SystemExit(f"unknown group {group!r}; known: {sorted(rates)}")
        rates[group] = float(value)
    return rates


def main() -> None:
    """Connect both sides, then publish fresh samples at each group's rate."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--broker", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--boat-id", default="b1")
    parser.add_argument("--mavlink", default="udpout:127.0.0.1:14551")
    parser.add_argument("--rate", action="append", default=[], metavar="GROUP=HZ",
                        help="override a group's publish rate; repeatable")
    args = parser.parse_args()
    rates = parse_rates(args.rate)

    link = mavutil.mavlink_connection(args.mavlink, source_system=253,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    if link.wait_heartbeat(timeout=30) is None:
        raise SystemExit(f"no heartbeat on {args.mavlink}")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"mav-{args.boat_id}")

    def on_connect(_client, _userdata, _flags, _reason, _properties) -> None:
        """(Re)subscribe on every connect, so a broker restart heals."""
        client.subscribe(ping(args.boat_id), qos=0)
        client.subscribe(cmd(args.boat_id), qos=1)

    def on_message(_client, _userdata, wire) -> None:
        """Echo pings untouched; log commands (actuation is not wired)."""
        if wire.topic == ping(args.boat_id):
            client.publish(pong(args.boat_id), wire.payload, qos=0)
        else:
            print(f"cmd: {wire.payload.decode(errors='replace')}", flush=True)

    client.on_connect = on_connect
    client.on_message = on_message
    client.reconnect_delay_set(1, 10)
    client.connect_async(args.broker, args.port)
    client.loop_start()

    latest = {}      # message name -> (message, Pi time at arrival, arrival count)
    arrivals = dict.fromkeys(MESSAGE_IDS, 0)
    published = {group: None for group in rates}
    next_due = {group: 0.0 for group in rates}
    seq = dict.fromkeys(rates, 0)
    boot_ref_ms = 0.0  # newest time_boot_ms seen, to unwrap RAW_IMU
    next_request = next_heartbeat = 0.0
    print(f"publishing {rates} Hz for boat {args.boat_id} from {args.mavlink}", flush=True)
    try:
        while True:
            now = time.monotonic()
            if now >= next_heartbeat:
                link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                next_heartbeat = now + 1.0
            if now >= next_request:
                request_rates(link, rates)
                next_request = now + 10.0
            message = link.recv_match(type=list(MESSAGE_IDS), blocking=True, timeout=0.01)
            while message is not None:
                name = message.get_type()
                arrivals[name] += 1
                latest[name] = (message, time.time(), arrivals[name])
                boot_ref_ms = max(boot_ref_ms, getattr(message, "time_boot_ms", 0))
                message = link.recv_match(type=list(MESSAGE_IDS), blocking=False)
            now = time.monotonic()
            for group, rate_hz in rates.items():
                if now < next_due[group]:
                    continue
                next_due[group] = max(next_due[group] + 1.0 / rate_hz, now)
                if any(name not in latest for name in SOURCES[group]):
                    continue
                version = tuple(latest[name][2] for name in SOURCES[group])
                if version == published[group]:
                    continue  # nothing new since the last publish: skip
                values, boot_ms = build(group, latest, boot_ref_ms)
                payload = dict(zip(SENSOR_FIELDS[group], values))
                payload["seq"] = seq[group]
                payload["t_boot_ms"] = boot_ms
                payload["t_rx"] = max(latest[name][1] for name in SOURCES[group])
                payload["t"] = time.time()
                client.publish(sensor(args.boat_id, group), json.dumps(payload).encode(), qos=0)
                published[group] = version
                seq[group] += 1
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()
        print(f"done: {seq}", flush=True)


if __name__ == "__main__":
    main()
