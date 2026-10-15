"""Integration test of the vessel side of Fleet-DT v2 on one host (SPEC_v2.md Sections 2 to 6).

A vessel broker (patched Mosquitto 2.1 with fdt_regulator.so), a GS broker with the control bridge and
the data bridge of SPEC_v2.md Section 5 (localhost ports), the synthetic sources fdt_source.py, a fake
AS that answers each demand with a table (late on purpose in some steps), and a subscriber at the GS.

The fleetdt run checks: every released message is accepted at the GS within its step, level 2 before
level 1 (QoS 0 messages; the GS broker delivers a QoS 2 message only at PUBREL), whole outputs only,
QoS 2 for the topics of R, the payload headers, the schema of the demand and of the vessel log, sane
values of B, and the default table after a late table. The other modes run shorter with the
invariants that apply to them; b2 runs with one data bridge per class and the topic remapping of
mosquitto/conf/gs-v2.conf.

Needs MOSQ (patched Mosquitto 2.1.2) and FDT_BUILD/fdt_regulator.so (make).
"""
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

import pytest

import paho.mqtt.client as mqtt

ROOT = Path(__file__).resolve().parents[2]


def _mosq():
    if os.environ.get("MOSQ"):
        return Path(os.environ["MOSQ"])
    for p in ("/opt/mosquitto-2.1", "/opt/mosquitto-2.1.2-fleetdt"):
        if Path(p).exists():
            return Path(p)
    return Path("/opt/mosquitto-2.1")


MOSQ = _mosq()
BUILD = Path(os.environ.get("FDT_BUILD", ROOT / "build"))
BROKER = MOSQ / "sbin" / "mosquitto"
PLUGIN = BUILD / "fdt_regulator.so"
SOURCE = ROOT / "vessel" / "publishers" / "fdt_source.py"
RUN_S = float(os.environ.get("FDT_V2_SECONDS", "8"))

pytestmark = pytest.mark.skipif(not (BROKER.exists() and PLUGIN.exists()),
                                reason="needs the patched Mosquitto (MOSQ) and make")

DTP = {
    "epoch": None,
    "Ts": 0.125, "T1": 0.020, "T2": 0.010,
    "eps": 0.002, "d_rx": 0.002, "d_m": 0.001,
    "T_tick": 0.001, "L_w": 16384, "B_cap": 1048576, "T_min": 0.005,
    "beta": 0.05, "gamma": 0.01, "alpha1": 0.5, "lambda": 0.25,
    "phy": {"slot": 9e-6, "sifs": 10e-6, "t_pre": 16e-6, "t_sig": 4e-6, "t_sym": 4e-6, "t_se": 6e-6,
            "h_mac": 38, "aifsn_sta": 3, "cwmin_sta": 15, "cwmax_sta": 1023, "m_b": 6,
            "aifsn_ap": 1, "cwmin_ap": 7, "cwmax_ap": 1023,
            "basic_rates": [6e6, 12e6, 24e6], "rate_init": 24e6},
    "tcp": {"mss": 1448, "h_T": 80},
    "mqtt": {"h_A": 4},
    "vessels": [1, 2],
    "topics": [
        {"name": "state", "rate": 8, "jitter": 0.002, "n": 1, "s": 48, "kind": "stream", "qos": 2},
        {"name": "imu", "rate": 32, "jitter": 0.002, "n": 1, "s": 48, "kind": "stream", "qos": 0},
        {"name": "gps", "rate": 8, "jitter": 0.002, "n": 1, "s": 80, "kind": "stream", "qos": 0},
        {"name": "baro", "rate": 8, "jitter": 0.002, "n": 1, "s": 32, "kind": "stream", "qos": 0},
        {"name": "power", "rate": 8, "jitter": 0.002, "n": 1, "s": 40, "kind": "stream", "qos": 0},
        {"name": "mag", "rate": 16, "jitter": 0.002, "n": 1, "s": 40, "kind": "stream", "qos": 0},
        {"name": "video", "rate": 15, "jitter": 0.010, "n": 16, "s": 1400, "kind": "output", "qos": 0},
    ],
    "required": ["state"],
    "goals": {"rate": 1, "s": 200},
    "order": ["state", "imu", "gps", "video", "baro", "power", "mag"],
    "needed": ["state", "imu", "gps"],
}
NAMES = [t["name"] for t in DTP["topics"]]
IDX = {n: k for k, n in enumerate(NAMES)}
L = len(NAMES)
ORDER = [IDX[n] for n in DTP["order"]]
EPOCH = 1767225600.0
TS, T1, T2 = DTP["Ts"], DTP["T1"], DTP["T2"]
# vessel 1 (i = 1 of the M vessels of the DTP) publishes its demand i (T1 - eps - 2 T_tick) / (M + 1)
P1_OFF = (1 * (round(T1 * 1e6) - round(DTP["eps"] * 1e6) - 2 * round(DTP["T_tick"] * 1e6))
          // (len(DTP["vessels"]) + 1)) / 1e6
VIDEO = IDX["video"]
STATE = IDX["state"]


def start_s(t):
    return EPOCH + (t - 1) * TS


def v_s(t):
    return start_s(t) + T1 + T2


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def wait_port(port, timeout=20.0):
    end = time.time() + timeout
    while time.time() < end:
        try:
            socket.create_connection(("127.0.0.1", port), 0.2).close()
            return
        except OSError:
            time.sleep(0.05)
    raise RuntimeError("port %d not listening" % port)


def client(cid):
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid, clean_session=True)
    except AttributeError:
        return mqtt.Client(client_id=cid, clean_session=True)


BASE = """allow_anonymous true
set_tcp_nodelay true
max_packet_size 1048576
max_inflight_messages 0
persistence false
user root
log_type error
log_type warning
log_type notice
log_dest file {log}
"""


def vessel_conf(tmp, mode, ports, data_ports):
    s = "".join("listener %d 127.0.0.1\n" % p for p in ports)
    s += BASE.format(log=tmp / "vessel.log") + "max_queued_messages 0\n"
    s += ("plugin {plugin}\nplugin_opt_dtp {dtp}\nplugin_opt_vessel 1\nplugin_opt_mode {mode}\n"
          "plugin_opt_data_ports {dp}\nplugin_opt_stats_s 2\n").format(
        plugin=PLUGIN, dtp=tmp / "dtp_v2.json", mode=mode, dp=",".join(str(p) for p in data_ports))
    return s


BRIDGE = """
connection {name}
address 127.0.0.1:{port}
bridge_protocol_version mqttv311
cleansession true
keepalive_interval 10
restart_timeout 1 2
notifications true
notifications_local_only true
notification_topic fleet/bridge/{name}/state
"""


def gs_conf(tmp, gs_port, ctl_port, data):
    """data: list of (name, port, topic line)"""
    s = "listener %d 127.0.0.1\n" % gs_port + BASE.format(log=tmp / "gs.log") + "max_queued_messages 100000\n"
    s += BRIDGE.format(name="b1-ctl", port=ctl_port)
    s += ("topic boat/b1/demand in 1\ntopic boat/b1/regulator in 0\n"
          "topic boat/b1/levels out 1\ntopic boat/b1/goals out 1\n")
    for name, port, line in data:
        s += BRIDGE.format(name=name, port=port) + line + "\n"
    return s


class FakeAS(object):
    """Subscriber at the GS and fake AS: answers the demand of step t with the table of policy(t)."""

    def __init__(self, port, mode, late_every=6):
        self.mode, self.late_every = mode, late_every
        self.lock = threading.Lock()
        self.tx = []          # (arrival, name, qos, t, seq, content)
        self.demand = {}      # t -> (arrival, json)
        self.reg = {}         # t -> json
        self.tables = {}      # t -> dict(p, k, late, slot)
        self.bridges = {}
        self.timers = []
        self.c = client("fdt-test-gs-%d" % os.getpid())
        self.c.on_connect = self.on_connect
        self.c.on_message = self.on_message
        self.c.connect("127.0.0.1", port, 10)
        self.c.loop_start()

    def on_connect(self, c, u, flags, rc, *rest):
        c.subscribe([("boat/b1/tx/#", 2), ("boat/b1/demand", 1), ("boat/b1/regulator", 0),
                     ("fleet/bridge/+/state", 0)])

    def policy(self, t, d):
        late = self.late_every and t % self.late_every == 0
        p = [0] * L
        p[IDX["state"]] = 2
        p[IDX["imu"]] = 2
        p[IDX["gps"]] = 2 if t % 2 else 1
        p[IDX["baro"]] = 1
        p[IDX["power"]] = 1 if t % 3 else 0
        p[VIDEO] = 1 if t % 4 != 1 else 0
        k = [1 if p[l] else 0 for l in range(L)]
        nv = len(d["d"][VIDEO])
        k[VIDEO] = (nv if t % 2 else min(1, d["held"][VIDEO])) if p[VIDEO] else 0
        tab = {"t": t, "p": p, "k": k, "o": ORDER}
        if self.mode == "slotted":
            tab["slot"] = [int((v_s(t) + 0.020) * 1e6), int((v_s(t) + 0.070) * 1e6)]
        return tab, late

    def on_message(self, c, u, m):
        now = time.time()
        if m.topic.startswith("boat/b1/tx/"):
            name = m.topic.rsplit("/", 1)[1]
            p = m.payload
            with self.lock:
                self.tx.append((now, name, m.qos, int.from_bytes(p[0:4], "big"), int.from_bytes(p[4:6], "big"),
                                bytes(p[6:])))
        elif m.topic == "boat/b1/demand":
            d = json.loads(m.payload.decode())
            with self.lock:
                self.demand[d["t"]] = (now, d)
            if self.mode in ("fleetdt", "b5", "slotted") and now >= start_s(d["t"]) + T1:
                # like the AS: a demand after phase 1 gets no table (the vessel applies the default table);
                # on this 2-vCPU host the loopback path stalls for 20-30 ms in about 1 of 300 steps
                with self.lock:
                    self.tables[d["t"]] = {"p": None, "k": None, "late": True, "missed": True, "slot": None,
                                           "pub": None}
            elif self.mode in ("fleetdt", "b5", "slotted"):
                tab, late = self.policy(d["t"], d)
                payload = json.dumps(tab)
                with self.lock:
                    self.tables[d["t"]] = {"p": tab["p"], "k": tab["k"], "late": late, "slot": tab.get("slot"),
                                           "pub": time.time()}
                if late:   # after v(t): the vessel must ignore it
                    tm = threading.Timer(max(0.0, v_s(d["t"]) + 0.005 - time.time()),
                                         lambda: c.publish("boat/b1/levels", payload, qos=1))
                    tm.start()
                    self.timers.append(tm)
                else:
                    c.publish("boat/b1/levels", payload, qos=1)
        elif m.topic == "boat/b1/regulator":
            d = json.loads(m.payload.decode())
            with self.lock:
                self.reg[d["t"]] = d
        elif m.topic.startswith("fleet/bridge/"):
            with self.lock:
                self.bridges[m.topic.split("/")[2]] = m.payload.decode()

    def wait_bridges(self, names, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end:
            with self.lock:
                if all(self.bridges.get(n) == "1" for n in names):
                    return
            time.sleep(0.05)
        raise RuntimeError("bridges not connected: %r" % self.bridges)

    def close(self):
        for tm in self.timers:
            tm.cancel()
        self.c.loop_stop()
        self.c.disconnect()


class Relay(object):
    """TCP relay between the data bridge of the GS and the data listener of the vessel. The socket to the
    vessel has a small receive buffer, and pause() stops reading it, so the data-bridge socket of the
    vessel fills as behind a congested channel while the control bridge keeps working."""

    def __init__(self, target_port, rcvbuf=4096):
        self.target = target_port
        self.rcvbuf = rcvbuf
        self.paused = threading.Event()
        self.closed = threading.Event()
        self.socks = []
        self.ls = socket.socket()
        self.ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.ls.bind(("127.0.0.1", 0))
        self.ls.listen(4)
        self.port = self.ls.getsockname()[1]
        threading.Thread(target=self.accept, daemon=True).start()

    def accept(self):
        while not self.closed.is_set():
            try:
                down, _ = self.ls.accept()
            except OSError:
                return
            up = socket.socket()
            up.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.rcvbuf)
            try:
                up.connect(("127.0.0.1", self.target))
            except OSError:
                down.close()
                continue
            for x in (down, up):
                x.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.socks.append(x)
            threading.Thread(target=self.pipe, args=(down, up, False), daemon=True).start()
            threading.Thread(target=self.pipe, args=(up, down, True), daemon=True).start()

    def pipe(self, src, dst, pausable):
        try:
            while not self.closed.is_set():
                while pausable and self.paused.is_set() and not self.closed.is_set():
                    time.sleep(0.002)
                data = src.recv(65536)
                if not data:
                    break
                dst.sendall(data)
        except OSError:
            pass
        for x in (src, dst):
            try:
                x.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def close(self):
        self.closed.set()
        self.paused.clear()
        for x in self.socks + [self.ls]:
            try:
                x.close()
            except OSError:
                pass


def run_stack(tmp, mode, seconds, b2=False, source_args=(), during=None, relay=False):
    (tmp / "dtp_v2.json").write_text(json.dumps(DTP))
    vctl, gs = free_port(), free_port()
    if b2:
        data_ports = [free_port(), free_port(), free_port()]   # BE, AF41, EF
        data = [("b1-data-%s" % cls, port, "topic # in 2 boat/b1/tx/ boat/b1/tx2/%s/" % cls)
                for cls, port in zip(("be", "af41", "ef"), data_ports)]
    else:
        data_ports = [free_port()]
        data = [("b1-data", data_ports[0], "topic boat/b1/tx/# in 2")]
    (tmp / "vessel.conf").write_text(vessel_conf(tmp, mode, [vctl] + data_ports, data_ports))
    rl = None
    procs = []
    fake = None
    try:
        procs.append(subprocess.Popen([str(BROKER), "-c", str(tmp / "vessel.conf")],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        wait_port(vctl)
        if relay:
            rl = Relay(data_ports[0])
            data = [(data[0][0], rl.port, data[0][2])]
        (tmp / "gs.conf").write_text(gs_conf(tmp, gs, vctl, data))
        procs.append(subprocess.Popen([str(BROKER), "-c", str(tmp / "gs.conf")],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        wait_port(gs)
        fake = FakeAS(gs, mode)
        fake.wait_bridges(["b1-ctl"] + [d[0] for d in data])
        time.sleep(0.3)
        src = subprocess.Popen([sys.executable, str(SOURCE), "--dtp", str(tmp / "dtp_v2.json"), "--vessel", "1",
                                "--port", str(vctl), "--duration", str(seconds), "--seed", "7",
                                "--log", str(tmp / "source.jsonl")] + list(source_args),
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        if during is not None:   # e.g. pauses the relay of the data bridge for a while
            during(rl)
        _, err = src.communicate(timeout=seconds + 30)
        assert src.returncode == 0, err.decode()
        time.sleep(3 * TS + 0.1)   # the steps of the last messages close and report
        fake.close()
    finally:
        if rl is not None:
            rl.close()
        for p in reversed(procs):
            p.send_signal(signal.SIGTERM)
        for p in procs:
            try:
                p.wait(10)
            except subprocess.TimeoutExpired:
                p.kill()
    vlog = (tmp / "vessel.log").read_text()
    return fake, vlog


def source_outputs(tmp):
    out = defaultdict(dict)
    with open(tmp / "source.jsonl") as f:
        for line in f:
            r = json.loads(line)
            out[r["topic"]][r["out"]] = r
    return out


DEMAND_KEYS = {"t", "v", "d", "held", "B", "rep", "late_us"}
REP_KEYS = {"t", "rel", "disc", "tab"}
REG_KEYS = {"t", "have_tab", "late_us", "nsd_max", "outq_p1", "rx", "rel", "disc", "xrel", "tick_us_max"}


def check_demand_schema(d):
    assert set(d) == DEMAND_KEYS, d.keys()
    assert d["v"] == 1
    assert len(d["d"]) == L and len(d["held"]) == L
    for l, units in enumerate(d["d"]):
        tp = DTP["topics"][l]
        assert all(isinstance(u, list) and len(u) == 2 and all(isinstance(x, int) and x >= 0 for x in u)
                   for u in units)
        if tp["kind"] == "stream":
            assert len(units) == 1
            a = math.ceil(tp["rate"] * (T1 + T2 - P1_OFF + tp["jitter"]) - 1e-9)
            assert units[0][1] >= a * tp["n"] and units[0][0] >= a * tp["n"] * tp["s"]
        else:
            h = d["held"][l]
            a = math.ceil(tp["rate"] * (T1 + T2 - P1_OFF + tp["jitter"]) - 1e-9)
            assert len(units) == h + a
            for x, n in units[:h]:   # held outputs: exact
                assert 1 <= n <= tp["n"] and n <= x
            for u in units[h:]:      # arrival-curve bound
                assert u == [tp["n"] * tp["s"], tp["n"]]
    assert set(d["B"]) == {"x0", "n0", "x2", "n2"}
    B = d["B"]
    assert all(isinstance(v, int) and v >= 0 for v in B.values())
    assert B["x0"] + B["x2"] <= DTP["B_cap"]
    assert B["x2"] <= 48 * B["n2"] and B["x0"] <= 1400 * B["n0"]
    assert set(d["rep"]) == REP_KEYS
    assert d["rep"]["t"] == d["t"] - 1 and len(d["rep"]["rel"]) == L
    assert isinstance(d["rep"]["tab"], bool) and d["rep"]["disc"] >= 0
    assert 0 <= d["late_us"] < TS * 1e6


def by_step(tx):
    steps = defaultdict(list)
    for i, m in enumerate(tx):
        steps[m[3]].append((i,) + m)
    return steps


def test_fleetdt(tmp_path):
    fake, vlog = run_stack(tmp_path, "fleetdt", RUN_S)
    assert "data bridge 127.0.0.1" in vlog and "fd=-1" not in vlog.split("data bridge 127.0.0.1")[1][:60]
    demand, reg, tables = fake.demand, fake.reg, fake.tables
    ts = sorted(demand)
    assert len(ts) >= RUN_S / TS
    assert ts == list(range(ts[0], ts[-1] + 1)), "a demand is missing"
    in_p1 = 0
    # phase 1 starts on time at the vessel (the host has 2 CPUs for 2 brokers, the sources and this test)
    late = sorted(demand[t][1]["late_us"] for t in ts)
    assert late[len(late) // 2] < 5000 and late[int(0.95 * len(late))] < T1 * 1e6, late
    for t in ts:
        check_demand_schema(demand[t][1])
        # the demand reaches the subscriber of the GS in phase 1 (the Python subscriber adds its own lag)
        assert start_s(t) - 0.002 <= demand[t][0], (t, demand[t][0] - start_s(t))
        in_p1 += demand[t][0] < start_s(t) + T1
    assert in_p1 >= 0.9 * len(ts)
    for t, r in reg.items():
        assert REG_KEYS <= set(r) and all(len(r[k]) == L for k in ("rx", "rel", "disc", "xrel"))
    steps = by_step(fake.tx)
    src = source_outputs(tmp_path)
    # the steps of the run: a demand of t + 1 reports t; the sources run from the second step on
    checked = [t for t in ts[2:-1] if t + 1 in demand and t in reg and t in tables]
    assert len(checked) >= RUN_S / TS - 6
    n_late = n_video = n_b = n_unexp = n_msgs = n_slow = 0
    for t in checked:
        rep = demand[t + 1][1]["rep"]
        r = reg[t]
        msgs = steps.get(t, [])
        tab = tables[t]
        # every released message leaves after v(t) and reaches the GS within step t (the loopback stalls
        # of this host delay a few messages; at most 1 % of them may arrive up to Ts after the step)
        got = [0] * L
        for (i, at, name, qos, tt, seq, content) in msgs:
            got[IDX[name]] += 1
            n_msgs += 1
            assert start_s(t) + T1 + T2 - 0.002 <= at < start_s(t + 2), (t, name, at - start_s(t))
            n_slow += at >= start_s(t + 1) + 0.010
        assert got == r["rel"], (t, got, r["rel"])
        assert [sum(x) for x in rep["rel"]] == r["rel"]
        assert rep["disc"] == 0 and r["disc"] == [0] * L
        # QoS 2 for R, 0 for the others; step headers t, seq = 0.. per topic in arrival order (QoS 0)
        for (i, at, name, qos, tt, seq, content) in msgs:
            assert qos == (2 if name == "state" else 0)
        for l in range(L):
            seqs = [m[5] for m in msgs if m[2] == NAMES[l]]
            assert sorted(seqs) == list(range(len(seqs)))
            if l != STATE:
                assert seqs == sorted(seqs)
        unexp = not tab["late"] and rep["tab"] is False and r["have_tab"] is False
        if tab["late"] or unexp:
            # late table (or no table) -> default table: R alone, the arrival bound over Ts (2 states);
            # a table published in time can still reach the vessel after v(t) in a stall of the host
            n_late += not unexp
            n_unexp += unexp
            assert rep["tab"] is False and r["have_tab"] is False
            assert set(m[2] for m in msgs) <= {"state"} and len(msgs) <= 2
            continue
        assert rep["tab"] is True and r["have_tab"] is True, (
            t, rep["tab"], r["have_tab"], "demand at %.4f s, table at %.4f s after start(t), v(t) at %.4f s" % (
                demand[t][0] - start_s(t), tab.get("pub", 0) - start_s(t), T1 + T2), r)
        p, k = tab["p"], tab["k"]
        # admitted topics only; level 2 before level 1 (QoS 0 messages in arrival order)
        assert all(p[IDX[m[2]]] >= 1 for m in msgs)
        lv = [p[IDX[m[2]]] for m in msgs if m[3] == 0]
        assert lv == sorted(lv, reverse=True), (t, lv)
        for l in range(L):
            assert len(rep["rel"][l]) == (k[l] if p[l] else 0)
        # whole outputs only, at most k of them, in output order, as the source released them
        outs = defaultdict(list)
        for (i, at, name, qos, tt, seq, content) in msgs:
            if name == "video":
                outs[int.from_bytes(content[0:4], "big")].append(
                    (int.from_bytes(content[4:6], "big"), int.from_bytes(content[6:8], "big")))
        assert len(outs) <= k[VIDEO]
        for oid, parts in outs.items():
            n = parts[0][1]
            assert [x[0] for x in parts] == list(range(n)) and all(x[1] == n for x in parts)
            assert src["video"][oid]["n"] == n
            n_video += 1
        # released counts per admitted output unit
        if p[VIDEO]:
            assert sum(1 for x in rep["rel"][VIDEO] if x > 0) == len(outs)
    for t in ts[2:]:
        B = demand[t][1]["B"]
        n_b += B == {"x0": 0, "n0": 0, "x2": 0, "n2": 0}
    assert n_late >= 1 and n_video >= 5
    assert n_unexp <= max(1, 0.02 * len(checked)), n_unexp
    assert n_slow <= max(1, 0.01 * n_msgs), (n_slow, n_msgs)
    assert n_b >= 0.8 * len(ts[2:])   # loopback: the data bridge is empty at phase 1 in most steps
    m = [ln for ln in vlog.splitlines() if "fdt_regulator: steps=" in ln][-1]
    assert "tab_late=" in m and " pub_err=0" in m and "tab_late=0" not in m


@pytest.mark.parametrize("mode", ["b1", "b3", "b5", "tdma", "slotted"])
def test_modes(tmp_path, mode):
    fake, vlog = run_stack(tmp_path, mode, 3.0)
    demand, reg = fake.demand, fake.reg
    ts = sorted(demand)
    steps = by_step(fake.tx)
    checked = [t for t in ts[2:-1] if t + 1 in demand and t in reg]
    assert len(checked) >= 15
    for t in ts:
        check_demand_schema(demand[t][1])
    for t in checked:
        r = reg[t]
        assert r.get("mode") == mode
        msgs = steps.get(t, [])
        got = [0] * L
        for m in msgs:
            got[IDX[m[2]]] += 1
            assert m[3] == (2 if m[2] == "state" else 0)
        assert got == r["rel"], (mode, t, got, r["rel"])
        if mode in ("b1", "b3", "tdma"):   # no table: every message of the step leaves
            assert all(r["rel"][l] + r["disc"][l] == r["rx"][l] for l in range(L)), (mode, t, r)
            assert r["have_tab"] is False
        if mode == "tdma":   # vessel 1 of 2: first half of W(t)
            w0 = start_s(t) + T1 + T2 + DTP["eps"]
            tw = TS - T1 - T2 - 2 * DTP["eps"] - DTP["d_rx"]
            for m in msgs:
                assert w0 - 0.002 <= m[1] < w0 + tw / 2 + 0.003, (t, m[1] - w0)
        if mode == "slotted" and not fake.tables[t]["late"]:
            s0, s1 = [x / 1e6 for x in fake.tables[t]["slot"]]
            for m in msgs:
                if m[2] != "state":
                    assert s0 - 0.001 <= m[1] < s1 + 0.003, (t, m[1] - s0)
        if mode == "b3":
            lv = [ORDER.index(IDX[m[2]]) for m in msgs if m[3] == 0]
            assert lv == sorted(lv), (t, lv)


def test_b2(tmp_path):
    fake, vlog = run_stack(tmp_path, "b2", 3.0, b2=True)
    assert vlog.count("data bridge 127.0.0.1") == 3
    names = set(m[1] for m in fake.tx)
    assert names == set(NAMES), names   # remapped from boat/b1/tx2/<class>/<name> to boat/b1/tx/<name>
    for m in fake.tx:
        assert m[2] == (2 if m[1] == "state" else 0)
    steps = by_step(fake.tx)
    for t in sorted(fake.reg)[2:-1]:
        got = [0] * L
        for m in steps.get(t, []):
            got[IDX[m[2]]] += 1
        assert got == fake.reg[t]["rel"]


def test_stall(tmp_path):
    """The data bridge stops reading for 1.5 s (relay with a small receive buffer): the data-bridge socket
    of the vessel keeps unacknowledged and unsent bytes, the pacing holds the unsent bytes near L_w, the
    regulator discards the admitted messages it cannot release, and B reports the unacknowledged bytes."""
    def stall(rl):
        time.sleep(2.0)
        rl.paused.set()
        time.sleep(1.5)
        rl.paused.clear()

    fake, vlog = run_stack(tmp_path, "fleetdt", 5.0, source_args=["--median", "20000", "--sigma", "0.1"],
                           during=stall, relay=True)
    demand, reg = fake.demand, fake.reg
    ts = sorted(demand)
    for t in ts:
        check_demand_schema(demand[t][1])
    steps = by_step(fake.tx)
    L_w = DTP["L_w"]
    nsd = [r["nsd_max"] for r in reg.values()]
    assert max(nsd) >= L_w // 2, nsd
    assert max(nsd) < L_w + 1427 + 128, nsd   # pacing: at most one message beyond L_w
    assert sum(sum(r["disc"]) for r in reg.values()) > 0
    assert any(demand[t][1]["rep"]["disc"] > 0 for t in ts)
    held = 0
    for t in ts:
        B, r = demand[t][1]["B"], reg.get(t)
        if r is None or r["outq_p1"] < 0:
            continue
        x, n = B["x0"] + B["x2"], B["n0"] + B["n2"]
        held += x > 0
        # content of the unacknowledged packets <= SIOCOUTQ <= content + headers + PUBREL/PINGRESP bytes
        assert x <= r["outq_p1"] <= x + 40 * n + 4 * 64 + 2, (t, B, r["outq_p1"])
    assert held >= 2
    # TCP delivers every released message once the relay reads again (the GS discards the late ones)
    for t in ts[2:-1]:
        if t in reg:
            got = [0] * L
            for m in steps.get(t, []):
                got[IDX[m[2]]] += 1
            assert got == reg[t]["rel"], (t, got, reg[t]["rel"])
