"""End-to-end harness: a local Mosquitto, fake vessels (demand per SPEC 4.1, tables, tx messages with the
step header, vessel logs per SPEC 4.2) and a fake AP monitor (fleet/monitor batches every 5 ms)."""
from __future__ import annotations

import json
import os
import shutil
import socket
import struct
import subprocess
import tempfile
import threading
import time
from typing import Dict, List, Optional

from fleetdt.as2.dtp import DTP
from fleetdt.as2.service import _client

MOSQ_CANDIDATES = ["/opt/mosquitto-2.1/sbin/mosquitto", "/usr/sbin/mosquitto", "/usr/local/sbin/mosquitto"]


def mosquitto_bin() -> Optional[str]:
    for p in [os.environ.get("MOSQ_BIN", "")] + MOSQ_CANDIDATES:
        if p and os.path.exists(p):
            return p
    return shutil.which("mosquitto")


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class Broker:
    def __init__(self):
        self.port = free_port()
        self.dir = tempfile.mkdtemp(prefix="fdt-as2-")
        conf = os.path.join(self.dir, "m.conf")
        with open(conf, "w") as f:
            f.write("listener %d 127.0.0.1\nallow_anonymous true\nset_tcp_nodelay true\nmax_inflight_messages 0\n"
                    "max_queued_messages 1000000\npersistence false\n" % self.port)
        self.proc = subprocess.Popen([mosquitto_bin(), "-c", conf], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(100):
            try:
                socket.create_connection(("127.0.0.1", self.port), 0.1).close()
                return
            except OSError:
                time.sleep(0.05)
        raise RuntimeError("mosquitto did not start")

    def stop(self):
        self.proc.terminate()
        try:
            self.proc.wait(3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        shutil.rmtree(self.dir, ignore_errors=True)


def ts_content(size: int) -> bytes:
    h = struct.pack("!Q", int(time.time() * 1e6))
    return (h + bytes(max(0, size - len(h))))[:size]


class FakeVessel(threading.Thread):
    """Regulator + DTI of one vessel, without a broker of its own: publishes the demand of step t at
    start(t) + offset, applies the table that arrives before phase 3 (or the default table), publishes
    the released messages on boat/b<i>/tx/<topic>, and reports them in the next demand."""

    def __init__(self, dtp: DTP, i: int, port: int, steps: range, offset: float = 0.002, drop_demand=()):
        super().__init__(daemon=True)
        self.dtp, self.i, self.steps, self.offset = dtp, i, steps, offset
        self.drop_demand = set(drop_demand)
        self.tables: Dict[int, Dict] = {}
        self.table_at: Dict[int, float] = {}
        self.lock = threading.Lock()
        self.c = _client("v%d" % i)
        self.c.on_message = self._on_msg
        self.c.connect("127.0.0.1", port, 30)
        self.c.loop_start()
        self.c.subscribe("boat/b%d/levels" % i, 1)
        self.c.subscribe("boat/b%d/goals" % i, 1)
        self.goals = 0
        self.rep = None
        self.vlog = None
        self.errors: List[str] = []

    def _on_msg(self, c, u, m):
        if m.topic.endswith("/goals"):
            self.goals += 1
            return
        o = json.loads(m.payload)
        with self.lock:
            self.tables[o["t"]] = o
            self.table_at[o["t"]] = time.time()

    def _sleep(self, when):
        d = when - time.time()
        if d > 0:
            time.sleep(d)

    def demand(self, t: int) -> Dict:
        dtp = self.dtp
        d, held = [], []
        for tp in dtp.topics:
            if tp.output:
                h = 2
                na = tp.outputs_bound(dtp.T1 + dtp.T2)
                d.append([[tp.n * tp.s, tp.n]] * (h + na))
                held.append(h)
            else:
                N = tp.outputs_bound(dtp.Ts)
                d.append([[N * tp.n * tp.s, N * tp.n]])
                held.append(0)
        return {"t": t, "v": self.i, "d": d, "held": held, "B": {"x0": 0, "n0": 0, "x2": 0, "n2": 0},
                "rep": self.rep, "late_us": int((time.time() - dtp.start(t)) * 1e6)}

    def run(self):
        dtp = self.dtp
        try:
            for t in self.steps:
                self._sleep(dtp.start(t) + self.offset)
                dem = self.demand(t)
                if t not in self.drop_demand:
                    self.c.publish("boat/b%d/demand" % self.i, json.dumps(dem), qos=1)
                if self.vlog is not None:
                    self.c.publish("boat/b%d/regulator" % self.i, json.dumps(self.vlog), qos=0)
                v = dtp.phase3(t)
                self._sleep(v)
                with self.lock:
                    tab = self.tables.get(t)
                    got = tab is not None and self.table_at[t] <= v
                if not got:
                    tab = {"p": [2 if l in dtp.R else 0 for l in range(dtp.L)],
                           "k": [1 if l in dtp.R else 0 for l in range(dtp.L)], "o": list(range(dtp.L))}
                rel = [[] for _ in range(dtp.L)]
                xrel = [0] * dtp.L
                rx = [sum(n for _, n in u) for u in dem["d"]]
                for lvl in (2, 1):
                    for l in tab["o"]:
                        if tab["p"][l] != lvl:
                            continue
                        tp = dtp.topics[l]
                        units = dem["d"][l][:tab["k"][l]] if tp.output else dem["d"][l][:1]
                        if not got and l in dtp.R and not tp.output:
                            N = tp.outputs_bound(dtp.Ts)
                            units = [[N * tp.n * tp.s, N * tp.n]]
                        seq = 0
                        for x, n in units:
                            for _ in range(n):
                                pl = struct.pack("!IH", t, seq) + ts_content(min(tp.s, x // max(1, n)))
                                self.c.publish("boat/b%d/tx/%s" % (self.i, tp.name), pl, qos=tp.qos)
                                xrel[l] += len(pl) - 6
                                seq += 1
                            rel[l].append(n)
                self.rep = {"t": t, "rel": rel, "disc": 0, "tab": got}
                self.vlog = {"t": t, "have_tab": got, "late_us": 0, "nsd_max": 0, "outq_p1": 0, "rx": rx,
                             "rel": [sum(r) for r in rel], "disc": [0] * dtp.L, "xrel": xrel, "tick_us_max": 0}
        except Exception as e:     # noqa: BLE001
            self.errors.append(repr(e))

    def stop(self):
        self.c.loop_stop()
        self.c.disconnect()


class FakeMonitor(threading.Thread):
    """fleet/monitor batches every 5 ms: in W(t) uplink data-bridge frames of vessel 1 with the airtime
    fraction kappa, in Q(t) a busy fraction b outside the fleet."""

    def __init__(self, dtp: DTP, port: int, until: float, kappa: float = 0.5, b: float = 0.1):
        super().__init__(daemon=True)
        self.dtp, self.until, self.kappa, self.b = dtp, until, kappa, b
        self.c = _client("mon")
        self.c.connect("127.0.0.1", port, 30)
        self.c.loop_start()
        from fleetdt.as2.airtime import Phy
        self.phy = Phy(dtp.phy)
        self.batches = 0

    def frames(self, a: int, b: int):
        dtp, phy = self.dtp, self.phy
        ts = phy.t_succ(1528, 24e6)
        dur = int((ts - phy.aifs()) * 1e6)
        gap = int(ts / self.kappa * 1e6)
        fr, busy = [], []
        t = dtp.step_of(a * 1e-6)
        for s in (t - 1, t, t + 1):
            w, we = dtp.window(s)
            w, we = int(w * 1e6), int(we * 1e6)
            stop = w + int(0.8 * (we - w))
            x = w + ((max(a, w) - w + gap - 1) // gap) * gap
            while x < min(b, stop):
                fr.append([x, x + dur, 1, 0, 1528, 24000, 1, 0])
                busy.append([x, x + dur])
                x += gap
            q0, q1 = dtp.quiet(s)
            q0, q1 = int(q0 * 1e6), int(q1 * 1e6)
            e0, e1 = max(a, q0), min(b, q0 + int(self.b * (q1 - q0)))
            if e1 > e0:
                busy.append([e0, e1])
        return busy, fr

    def run(self):
        prev = int(time.time() * 1e6)
        while time.time() < self.until:
            time.sleep(0.005)
            now = int(time.time() * 1e6)
            busy, fr = self.frames(prev, now)
            self.c.publish("fleet/monitor", json.dumps({"from_us": prev, "to_us": now, "busy": busy, "fr": fr}), qos=0)
            self.batches += 1
            prev = now

    def stop(self):
        self.c.loop_stop()
        self.c.disconnect()


def run_fleet_process(dtp_dict: Dict, epoch: float, port: int, steps: int, out: str, period: int = 8,
                      drop: Optional[Dict] = None) -> None:
    """Fake vessels, the operation-control stub and the fake monitor in their own process (so that their
    Python threads do not share the GIL of the AS); writes a JSON summary to `out`."""
    from fleetdt.as2.opctl import OpCtl, Schedule
    d = DTP.from_dict(dtp_dict)
    d.epoch = epoch
    oc = OpCtl(d, Schedule(d, period=period), "127.0.0.1", port, lead=0.01)
    vs = [FakeVessel(d, i, port, range(1, steps + 1), offset=0.001 + 0.0005 * i,
                     drop_demand=(drop or {}).get(str(i), ())) for i in d.vessels]
    mon = FakeMonitor(d, port, until=d.end(steps) + 0.2)
    th = threading.Thread(target=oc.run, kwargs={"steps": steps}, daemon=True)
    th.start()
    mon.start()
    for v in vs:
        v.start()
    for v in vs:
        v.join(d.end(steps) - time.time() + 5)
    mon.join(2)
    summary = {"tables": {str(v.i): sorted(v.tables) for v in vs},
               "tables_in_time": {str(v.i): sorted(t for t in v.tables if v.table_at[t] <= d.phase3(t)) for v in vs},
               "last_table": {str(v.i): v.tables[max(v.tables)] if v.tables else None for v in vs},
               "goals": {str(v.i): v.goals for v in vs}, "errors": [e for v in vs for e in v.errors],
               "batches": mon.batches}
    for v in vs:
        v.stop()
    oc.stop()
    mon.stop()
    with open(out, "w") as f:
        json.dump(summary, f)


if __name__ == "__main__":
    import sys
    a = json.loads(sys.argv[1])
    run_fleet_process(a["dtp"], a["epoch"], a["port"], a["steps"], a["out"], a.get("period", 8), a.get("drop"))
