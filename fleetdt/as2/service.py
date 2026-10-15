"""The AS v2 process: MQTT clients, phase timers on the GS clock (SPEC Section 3) and the DTA log.

  python -m fleetdt.as2.service --dtp dtp_v2.json --broker host:port [--mode fleetdt] [--log dta.jsonl]
         [--z0 1.0] [--oracle kappa.jsonl] [--init-dta old.jsonl ...] [--monitor-wait 0.002]

Phase 1 of step t: the control client collects the demands of step t that arrive in
[start(t) - eps, start(t) + T1). At start(t) + T1 (phase 2) the AS waits up to --monitor-wait for the
monitor batch that reaches the end of Q(t), runs ASCore.phase2(t) (closing of step t-1, eq. pred, eq. aci,
eq. grant, Algorithm 1), publishes the table of each vessel of M(t) on boat/b<i>/levels (QoS 1) and the
record on fleet/as, and appends the record to the DTA (one JSON line per step, "type":"as"). The data
client subscribes to boat/+/tx/# with QoS 2 and counts the accepted messages (failure.Acceptor); the
acceptances of the topics of R go to the DTA as "type":"acc" lines (age of information), and the vessel
logs (boat/b<i>/regulator) as "type":"vlog" lines.
"""
from __future__ import annotations

import argparse
import gc
import json
import logging
import os
import re
import socket
import sys
import threading
import time
import uuid
from typing import Dict, List, Optional, Tuple

from .core import ASCore, MODES, warmup
from .dtp import DTP

log = logging.getLogger("fleetdt.as2")

DEMAND_RE = re.compile(r"^boat/b(\d+)/demand$")
VLOG_RE = re.compile(r"^boat/b(\d+)/regulator$")
BRIDGE_RE = re.compile(r"^fleet/bridge/b(\d+)-data/state$")
AS_KEYS = ("t", "M", "b", "b_hat", "eta", "kappa_hat", "alpha", "Z", "K", "cB", "A", "cost_A", "p", "k",
           "f_prev", "kappa_prev", "e_prev", "z_prev", "FR", "mode")


def nodelay(client) -> None:
    """TCP_NODELAY on every socket of a paho client (PR8)."""
    def on_open(c, userdata, sock):
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except (OSError, AttributeError):
            pass
    client.on_socket_open = on_open


def _client(role: str):
    import paho.mqtt.client as mqtt
    cid = "fleetdt-as2-%s-%d-%s" % (role, os.getpid(), uuid.uuid4().hex[:6])
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid, clean_session=True)
    nodelay(c)
    c.max_inflight_messages_set(0)
    c.max_queued_messages_set(0)
    return c


def load_oracle(path: str, dtp: Optional[DTP] = None, origin: float = 0.0) -> Dict[int, float]:
    """Ground truth kappa(t): JSON lines {"t":..,"kappa":..} (or "kappa_true"), or the CSV of
    emu/gt.py windows (columns t_s, kappa; t_s in simulator seconds, `origin` = host time of simulator
    time 0, step t = dtp.step_of(origin + t_s + Ts/2))."""
    out = {}
    with open(path, encoding="utf-8") as f:
        head = f.readline()
        if not head.lstrip().startswith("{"):
            cols = [c.strip() for c in head.split(",")]
            for line in f:
                v = dict(zip(cols, line.strip().split(",")))
                if not v.get("kappa"):
                    continue
                if "t" in v:
                    t = int(float(v["t"]))
                else:
                    if dtp is None:
                        raise ValueError("a CSV with t_s needs the DTP and the origin")
                    t = dtp.step_of(origin + float(v["t_s"]) + dtp.Ts / 2)
                out[t] = float(v["kappa"])
            return out
        f.seek(0)
        for line in f:
            line = line.strip()
            if not line:
                continue
            o = json.loads(line)
            k = o.get("kappa", o.get("kappa_true"))
            if k is not None:
                out[int(o["t"])] = float(k)
    return out


def scores_from_dta(paths: List[str]) -> Tuple[List[float], Optional[float]]:
    """Scores z and the last b_hat of earlier DTA files (initial scores of the outer loop)."""
    zs, bh = [], None
    for p in paths:
        with open(p, encoding="utf-8") as f:
            for line in f:
                try:
                    o = json.loads(line)
                except ValueError:
                    continue
                if o.get("type", "as") != "as":
                    continue
                if o.get("z_prev") is not None:
                    zs.append(float(o["z_prev"]))
                if o.get("b_hat") is not None:
                    bh = float(o["b_hat"])
    return zs, bh


class Service:
    def __init__(self, dtp: DTP, host: str = "localhost", port: int = 1883, mode: str = "fleetdt",
                 log_path: Optional[str] = None, monitor_wait: float = 0.002, spin: float = 0.0005,
                 acc_log: bool = True, **core_kw):
        self.dtp = dtp
        self.host, self.port = host, port
        self.monitor_wait, self.spin = monitor_wait, spin
        self._lines: List[Dict] = []
        self._lines_lock = threading.Lock()
        self.core = ASCore(dtp, mode=mode, acc_log=self._acc_line if acc_log else None, **core_kw)
        self.log_file = open(log_path, "a", encoding="utf-8") if log_path else None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.timing: List[Dict] = []
        self._acks: Dict[int, Tuple[int, float]] = {}       # mid -> (t, deadline)
        self._ack_max: Dict[int, float] = {}                # t -> latest PUBACK - deadline (s)
        self._ack_lock = threading.Lock()
        self._early: Dict[int, float] = {}
        self.ctl = _client("ctl")
        self.data = _client("data")
        self._ctl_up, self._data_up = threading.Event(), threading.Event()
        self.ctl.on_connect = self._on_ctl_connect
        self.ctl.on_message = self._on_ctl_message
        self.ctl.on_publish = self._on_publish
        self.data.on_connect = self._on_data_connect
        self.data.on_message = self._on_data_message

    # ------------------------------------------------------------------ MQTT
    def _on_ctl_connect(self, client, userdata, flags, rc, props=None):
        client.subscribe([("boat/+/demand", 1), ("boat/+/regulator", 0), ("fleet/order", 1),
                          ("fleet/monitor", 0), ("fleet/bridge/+/state", 0)])
        self._ctl_up.set()

    def _on_data_connect(self, client, userdata, flags, rc, props=None):
        client.subscribe([("boat/+/tx/#", 2)])
        self._data_up.set()

    def _on_ctl_message(self, client, userdata, msg):
        at = time.time()
        topic = msg.topic
        try:
            m = DEMAND_RE.match(topic)
            if m:
                ok = self.core.on_demand(int(m.group(1)), json.loads(msg.payload), at)
                if not ok:
                    log.info("late demand on %s", topic)
                return
            if topic == "fleet/monitor":
                self.core.monitor.add(json.loads(msg.payload))
                return
            if topic == "fleet/order":
                self.core.on_order(json.loads(msg.payload), at)
                return
            m = VLOG_RE.match(topic)
            if m:
                o = json.loads(msg.payload)
                self._line({"type": "vlog", "i": int(m.group(1)), "at": at, **o})
                return
            m = BRIDGE_RE.match(topic)
            if m:
                self.core.on_bridge(int(m.group(1)), msg.payload.strip() == b"1")
                return
        except (ValueError, KeyError, TypeError, IndexError) as e:
            self.core.bad_inputs += 1
            log.warning("malformed message on %s: %s", topic, e)

    def _on_data_message(self, client, userdata, msg):
        self.core.acceptor.on_message(msg.topic, msg.payload, time.time())

    def _on_publish(self, client, userdata, mid, rc=None, props=None):
        at = time.time()
        with self._ack_lock:
            v = self._acks.pop(mid, None)
            if v is None:
                self._early[mid] = at        # PUBACK before publish() returned the mid (or a QoS 0 mid)
                if len(self._early) > 4096:
                    for k in list(self._early)[:2048]:
                        del self._early[k]
                return
            t, dl = v
            self._ack_max[t] = max(self._ack_max.get(t, 0.0), at - dl)

    def _acc_line(self, r: Tuple) -> None:
        i, l, t, seq, at, c = r
        self._line({"type": "acc", "i": i, "l": l, "t": t, "seq": seq, "at": at, "c": c})

    def _line(self, rec: Dict) -> None:
        with self._lines_lock:
            self._lines.append(rec)

    def _flush(self, rec: Optional[Dict] = None) -> None:
        with self._lines_lock:
            lines, self._lines = self._lines, []
        if self.log_file is None:
            return
        out = [json.dumps(x, separators=(",", ":")) for x in lines]
        if rec is not None:
            out.append(json.dumps(rec, separators=(",", ":")))
        if out:
            self.log_file.write("\n".join(out) + "\n")
            self.log_file.flush()

    def connect(self, timeout: float = 5.0) -> None:
        warmup(self.dtp, self.core.mode)
        gc.collect()
        gc.freeze()           # objects of the start-up stay out of the collections
        for c in (self.ctl, self.data):
            c.connect(self.host, self.port, keepalive=30)
            c.loop_start()
        if not self._ctl_up.wait(timeout) or not self._data_up.wait(timeout):
            raise TimeoutError("no CONNACK from %s:%d" % (self.host, self.port))
        time.sleep(0.05)   # SUBACK

    # ------------------------------------------------------------------ timing
    def _sleep_until(self, when: float) -> None:
        while not self._stop.is_set():
            rest = when - time.time()
            if rest <= 0:
                return
            if rest > self.spin:
                self._stop.wait(min(rest - self.spin, 0.05))

    def publish(self, t: int, tabs: Dict[int, Dict], rec: Dict, deadline: float) -> None:
        for i, tb in tabs.items():
            info = self.ctl.publish("boat/b%d/levels" % i, json.dumps(tb, separators=(",", ":")), qos=1)
            with self._ack_lock:
                at = self._early.pop(info.mid, None)
                if at is None:
                    self._acks[info.mid] = (t, deadline)
                else:
                    self._ack_max[t] = max(self._ack_max.get(t, 0.0), at - deadline)
        self.ctl.publish("fleet/as", json.dumps({k: rec.get(k) for k in AS_KEYS}, separators=(",", ":")), qos=0)

    def step(self, t: int, publish: bool = True) -> Dict:
        dtp = self.dtp
        dl = dtp.phase2(t)
        wake = time.time() - dl
        gc.disable()          # no collection inside phase 2; the collector runs again in phase 3
        try:
            if self.monitor_wait > 0:
                self.core.monitor.wait_until(int(round(dl * 1e6)), max(0.0, dl + self.monitor_wait - time.time()))
            tabs, rec = self.core.phase2(t, publish=publish)
            if publish:
                self.publish(t, tabs, rec, dl)
            pub = time.time() - dl
        finally:
            gc.enable()
        with self._ack_lock:
            rec["ack_ms_prev"] = self._ack_max.pop(t - 1, None)
            if rec["ack_ms_prev"] is not None:
                rec["ack_ms_prev"] *= 1e3
        rec.update({"published": publish, "wake_ms": wake * 1e3, "pub_ms": pub * 1e3})
        self.timing.append({"t": t, "wake_ms": wake * 1e3, "compute_ms": rec["t_compute_ms"], "pub_ms": pub * 1e3,
                            "m": len(rec["M"])})
        self._flush(rec)
        return rec

    def run(self, steps: Optional[int] = None, first: Optional[int] = None) -> None:
        dtp = self.dtp
        now = time.time()
        t = first if first is not None else dtp.step_of(now - dtp.T1) + 1
        done = 0
        while not self._stop.is_set() and (steps is None or done < steps):
            self._sleep_until(dtp.phase2(t))
            if self._stop.is_set():
                break
            late = time.time() - dtp.phase2(t)
            if late >= dtp.Ts:    # overrun: the steps already over run phase 2 without publishing
                skip = int(late // dtp.Ts)
                for k in range(t, t + skip):
                    self.step(k, publish=False)
                log.warning("overrun of %.3f s: steps %d..%d not published", late, t, t + skip - 1)
                t += skip
            rec = self.step(t)
            if rec["pub_ms"] > dtp.T2 * 1e3:
                log.warning("step %d: tables published %.1f ms after start(t)+T1", t, rec["pub_ms"])
            t += 1
            done += 1

    def start(self, steps: Optional[int] = None, first: Optional[int] = None) -> None:
        self.connect()
        self._thread = threading.Thread(target=self.run, kwargs={"steps": steps, "first": first}, daemon=True)
        self._thread.start()

    def join(self, timeout: Optional[float] = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for c in (self.ctl, self.data):
            c.loop_stop()
            c.disconnect()
        self._flush()
        if self.log_file:
            self.log_file.close()
            self.log_file = None

    def timing_summary(self) -> Dict:
        out = {}
        for k in ("compute_ms", "pub_ms", "wake_ms"):
            v = sorted(x[k] for x in self.timing)
            if v:
                out[k] = {"n": len(v), "mean": sum(v) / len(v), "p50": v[len(v) // 2],
                          "p99": v[min(len(v) - 1, int(0.99 * len(v)))], "max": v[-1]}
        out["over_T2"] = sum(1 for x in self.timing if x["pub_ms"] > self.dtp.T2 * 1e3)
        return out


def _broker(s: str) -> Tuple[str, int]:
    host, _, port = s.partition(":")
    return host or "localhost", int(port or 1883)


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.as2.service", description="Fleet-DT v2 application space (AS)")
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--broker", default="localhost:1883")
    ap.add_argument("--mode", default="fleetdt", choices=MODES)
    ap.add_argument("--log", default=None, help="DTA file (JSON lines)")
    ap.add_argument("--epoch", type=float, default=None, help="overrides the DTP epoch")
    ap.add_argument("--z0", type=float, default=1.0, help="fixed Z of mode b4")
    ap.add_argument("--oracle", default=None, help="ground truth kappa(t) of mode oracle (JSON lines or gt.py CSV)")
    ap.add_argument("--oracle-origin", type=float, default=0.0, help="host time of simulator time 0 (gt.py CSV)")
    ap.add_argument("--init-dta", nargs="*", default=[], help="DTA files with the initial scores and b_hat")
    ap.add_argument("--score-window", type=int, default=None)
    ap.add_argument("--z-empty", type=float, default=1.0, help="Z while no score exists")
    ap.add_argument("--monitor-wait", type=float, default=0.002)
    ap.add_argument("--tab-unsent", choices=("count", "ignore"), default="count",
                    help="tab=false of a vessel outside M(t) counts in f(t) (literal Def. aci) or not")
    ap.add_argument("--steps", type=int, default=None)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO if a.verbose else logging.WARNING,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    dtp = DTP.load(a.dtp)
    if a.epoch is not None:
        dtp.epoch = a.epoch
    zs, bh = scores_from_dta(a.init_dta) if a.init_dta else ([], None)
    kw = dict(z0=a.z0, initial_scores=zs, b_hat0=bh or 0.0, score_window=a.score_window, z_empty=a.z_empty,
              count_tab_unsent=a.tab_unsent == "count")
    if a.oracle:
        kw["oracle"] = load_oracle(a.oracle, dtp, a.oracle_origin)
    host, port = _broker(a.broker)
    svc = Service(dtp, host, port, mode=a.mode, log_path=a.log, monitor_wait=a.monitor_wait, **kw)
    svc.connect()
    print("epoch = %.3f, mode = %s" % (dtp.epoch, a.mode), flush=True)
    try:
        svc.run(steps=a.steps)
    except KeyboardInterrupt:
        pass
    finally:
        svc.stop()
        print(json.dumps({"phase2": svc.timing_summary()}), file=sys.stderr)


if __name__ == "__main__":
    main()
