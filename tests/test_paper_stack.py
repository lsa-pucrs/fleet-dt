"""Integration test of the paper stack on one host: vessel regulator in mode paper + the AS of the paper.

A vessel broker (Mosquitto 2.1 with fdt_regulator.so, plugin_opt_mode paper), a GS broker with the
control bridge and the data bridge of SPEC.md Section 4 (localhost ports), synthetic sources on
boat/b1/raw/<topic>, fleetdt.as_service (unchanged) on the GS broker, and a subscriber at the GS.

Checks, step by step and end to end: D_i(t) leaves the vessel in the binary format of fleetdt/model.py
within phase 1 of step t (0-based clock of as_service.py), the AS answers with p(t), the regulator
forwards exactly the topics of levels 1 and 2 with QoS 0 (both levels, level 2 first), holds level 0, polices each topic to
the d_il(t) it declared, puts t and the sequence number in the header, forwards nothing of a step
without p(t) (AS stopped), and records every message not forwarded in the vessel log.

Needs MOSQ (Mosquitto 2.1.2) and FDT_BUILD/fdt_regulator.so (make).
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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from fleetdt.model import decode_demand, decode_levels, decode_step_header  # noqa: E402


def _mosq():
    if os.environ.get("MOSQ"):
        return Path(os.environ["MOSQ"])
    for p in ("/opt/mosquitto-2.1", "/opt/mosquitto-2.1.2-fleetdt", "/opt/mosquitto-2.1.2"):
        if Path(p).exists():
            return Path(p)
    return Path("/opt/mosquitto-2.1")


MOSQ = _mosq()
BUILD = Path(os.environ.get("FDT_BUILD", ROOT / "build"))
BROKER = MOSQ / "sbin" / "mosquitto"
PLUGIN = BUILD / "fdt_regulator.so"
DTI = BUILD / "fdt_dti"
RUN_S = float(os.environ.get("FDT_PAPER_SECONDS", "4"))

pytestmark = pytest.mark.skipif(not (BROKER.exists() and PLUGIN.exists()),
                                reason="needs Mosquitto 2.1 (MOSQ) and make (fdt_regulator.so)")

TS, T1, T2 = 0.125, 0.020, 0.010
C_B, C_M = 1.06, 180.0
# bulk claims more than the grant limit (level 0); the others fit (levels 2 and 1)
TOPICS = [
    {"name": "state", "rate": 8, "bound": 48},
    {"name": "imu", "rate": 32, "bound": 48, "jitter": 0.004},
    {"name": "gps", "rate": 8, "bound": 80},
    {"name": "video", "rate": 30, "bound": 1400},
    {"name": "bulk", "rate": 8, "bound": 120000},
]
NAMES = [t["name"] for t in TOPICS]
L = len(NAMES)
NEEDED = ["state", "imu"]
# messages per step that the sources publish, and their content bytes; gps sends more than its
# bound allows in one step, so the policing holds some of its messages
RATE = {"state": 8, "imu": 32, "gps": 40, "video": 30, "bulk": 8}
SIZE = {"state": 48, "imu": 48, "gps": 80, "video": 1400, "bulk": 1000}


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
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid, clean_session=True)

    def nodelay(cl, u, sock):
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except (OSError, AttributeError):
            pass
    c.on_socket_open = nodelay
    c.max_inflight_messages_set(0)
    return c


BASE = """allow_anonymous true
set_tcp_nodelay true
max_packet_size 1048576
max_inflight_messages 0
max_queued_messages 0
persistence false
user root
log_type error
log_type warning
log_type notice
log_dest file {log}
"""

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


def cost(x):
    """channel bytes of one forwarded message with x content bytes (SPEC.md Section 5)"""
    return C_B * (x + 6) + C_M


class GS(object):
    """Subscriber at the GS: every forwarded message, demand table, priority table and step summary."""

    def __init__(self, port):
        self.lock = threading.Lock()
        self.tx = []        # (arrival, name, qos, t, seq, payload length)
        self.demand = {}    # t -> (arrival, d)
        self.backlog = {}   # t -> Q_1(t) of the table that reached the GS
        self.levels = {}    # t -> p
        self.reg = {}       # t -> summary
        self.bridges = {}
        self.c = client("fdt-paper-gs-%d" % os.getpid())
        self.c.on_connect = lambda c, u, f, rc, props=None: c.subscribe(
            [("boat/b1/tx/#", 0), ("boat/b1/demand", 1), ("boat/b1/levels", 1), ("boat/b1/regulator", 0),
             ("fleet/bridge/+/state", 0)])
        self.c.on_message = self.on_message
        self.c.connect("127.0.0.1", port, 10)
        self.c.loop_start()

    def on_message(self, c, u, msg):
        at = time.time()
        with self.lock:
            if msg.topic.startswith("boat/b1/tx/"):
                t, s = decode_step_header(msg.payload)
                self.tx.append((at, msg.topic[len("boat/b1/tx/"):], msg.qos, t, s, len(msg.payload)))
            elif msg.topic == "boat/b1/demand":
                t, Q, d = decode_demand(msg.payload)
                self.demand[t] = (at, d)
                self.backlog[t] = Q
            elif msg.topic == "boat/b1/levels":
                t, p = decode_levels(msg.payload)
                self.levels[t] = p
            elif msg.topic == "boat/b1/regulator":
                o = json.loads(msg.payload)
                self.reg[int(o["t"])] = o
            elif msg.topic.startswith("fleet/bridge/"):
                self.bridges[msg.topic.split("/")[2]] = msg.payload

    def stop(self):
        self.c.loop_stop()
        self.c.disconnect()


class Sources(object):
    """Synthetic sources on the vessel broker: RATE[name] messages per second of SIZE[name] bytes."""

    def __init__(self, port):
        self.c = client("fdt-paper-src-%d" % os.getpid())
        self.c.connect("127.0.0.1", port, 10)
        self.c.loop_start()
        self.stop_ev = threading.Event()
        self.th = threading.Thread(target=self.run, daemon=True)
        self.th.start()

    def run(self):
        t0 = time.time()
        nxt = {n: t0 + 0.0011 * k for k, n in enumerate(NAMES)}
        cnt = defaultdict(int)
        while not self.stop_ev.is_set():
            now = time.time()
            for n in NAMES:
                while nxt[n] <= now:
                    body = bytes([cnt[n] & 0xFF]) * SIZE[n]
                    self.c.publish("boat/b1/raw/%s" % n, body, qos=0)
                    cnt[n] += 1
                    nxt[n] += 1.0 / RATE[n]
            time.sleep(0.0005)

    def stop(self):
        self.stop_ev.set()
        self.th.join(2)
        self.c.loop_stop()
        self.c.disconnect()


def _kill(p):
    if p and p.poll() is None:
        p.send_signal(signal.SIGTERM)
        try:
            p.wait(5)
        except subprocess.TimeoutExpired:
            p.kill()


@pytest.fixture(params=["internal", "external"])
def stack(request, tmp_path):
    """demand internal: the regulator publishes D_i(t); external: fdt_dti publishes it and the regulator
    polices to the table it sees on the vessel broker"""
    demand = request.param
    if demand == "external" and not DTI.exists():
        pytest.skip("needs FDT_BUILD/fdt_dti (make)")
    epoch = float(math.floor(time.time()) - 20)
    dtp = {"epoch": epoch, "Ts": TS, "T1": T1, "T2": T2, "alpha": 0.125, "U": 0.8, "C0": 300000,
           "c_b": C_B, "c_m": C_M, "R_PHY": 24000000, "h_W": 150, "vessels": [1], "topics": TOPICS,
           "order": ["state", "imu", "bulk", "gps", "video"], "needed": NEEDED}
    (tmp_path / "dtp.json").write_text(json.dumps(dtp))
    ctl, data, gs = free_port(), free_port(), free_port()
    vconf = "listener %d 127.0.0.1\nlistener %d 127.0.0.1\n" % (ctl, data) + BASE.format(log=tmp_path / "vessel.log")
    vconf += ("plugin %s\nplugin_opt_dtp %s\nplugin_opt_vessel 1\nplugin_opt_mode paper\nplugin_opt_data_port %d\n"
              "plugin_opt_log %s\nplugin_opt_stats_s 1\nplugin_opt_demand %s\n") % (
        PLUGIN, tmp_path / "dtp.json", data, tmp_path / "reg-b1.jsonl", demand)
    gconf = "listener %d 127.0.0.1\n" % gs + BASE.format(log=tmp_path / "gs.log")
    gconf += BRIDGE.format(name="b1-ctl", port=ctl)
    gconf += "topic boat/b1/demand in 1\ntopic boat/b1/regulator in 0\ntopic boat/b1/levels out 1\n"
    gconf += BRIDGE.format(name="b1-data", port=data) + "topic boat/b1/tx/# in 0\n"
    (tmp_path / "vessel.conf").write_text(vconf)
    (tmp_path / "gs.conf").write_text(gconf)
    procs = []
    vb = subprocess.Popen([str(BROKER), "-c", str(tmp_path / "vessel.conf")])
    procs.append(vb)
    wait_port(ctl)
    if demand == "external":   # the jitter allowance of imu (4 ms), for every topic
        procs.append(subprocess.Popen([str(DTI), "--dtp", str(tmp_path / "dtp.json"), "--vessel", "1", "--broker",
                                       "127.0.0.1:%d" % ctl, "--jitter-ms", "4", "--prio", "0"]))
    gb = subprocess.Popen([str(BROKER), "-c", str(tmp_path / "gs.conf")])
    procs.append(gb)
    wait_port(gs)
    st = {"tmp": tmp_path, "epoch": epoch, "ctl": ctl, "gs": gs, "procs": procs, "demand": demand}
    yield st
    for p in reversed(procs + st.get("extra", [])):
        _kill(p)


idx = {n: k for k, n in enumerate(NAMES)}


def start(epoch, t):
    return epoch + t * TS


def test_paper_stack_end_to_end(stack):
    tmp, epoch = stack["tmp"], stack["epoch"]
    gs = GS(stack["gs"])
    end = time.time() + 10
    while time.time() < end and not all(gs.bridges.get(b) == b"1" for b in ("b1-ctl", "b1-data")):
        time.sleep(0.05)
    assert gs.bridges.get("b1-ctl") == b"1" and gs.bridges.get("b1-data") == b"1", gs.bridges
    src = Sources(stack["ctl"])
    env = dict(os.environ, PYTHONPATH=str(ROOT))
    as_proc = subprocess.Popen([sys.executable, "-m", "fleetdt.as_service", "--dtp", str(tmp / "dtp.json"),
                                "--broker", "127.0.0.1:%d" % stack["gs"], "--log", str(tmp / "as.jsonl")],
                               env=env, cwd=str(ROOT), stderr=subprocess.PIPE)
    stack["extra"] = [as_proc]
    time.sleep(RUN_S)
    # the AS stops: the steps after its last table get no p(t)
    as_proc.send_signal(signal.SIGINT)
    as_proc.wait(10)
    t_stop = math.floor((time.time() - epoch) / TS)
    time.sleep(1.0)
    src.stop()
    time.sleep(0.5)
    gs.stop()
    for p in stack["procs"]:
        _kill(p)   # the plugin writes the queued messages (shutdown) and closes the vessel log

    dec = {}
    for line in (tmp / "as.jsonl").read_text().splitlines():
        o = json.loads(line)
        if o.get("published"):
            dec[o["t"]] = o
    assert len(dec) >= int(RUN_S / TS) - 8, "AS published %d tables" % len(dec)

    # 1. demand tables: binary !IBI format of model.py (t, L, Q_i), step index of as_service.py, within phase 1
    assert gs.demand
    for t, (at, d) in gs.demand.items():
        assert len(d) == L
        assert start(epoch, t) <= at < start(epoch, t) + T1 + 0.015, (t, at - start(epoch, t))
        # the bound of SPEC.md Section 5 with the jitter allowance inside d_il (imu: 4 ms in both modes)
        if min(d) > 0:
            assert d[idx["imu"]] == math.ceil(C_B * 5 * (48 + 6) + C_M * 5 - 1e-9)
    # the AS admitted the vessel in the steps with a demand table
    for t, o in dec.items():
        assert o["M"] == [1]
        assert t in gs.demand
        assert gs.levels.get(t) == o["p"]
        # backlog: the Q_1(t) of the table (written by the regulator, also into the table of fdt_dti) reaches
        # the AS, and the regulator logs the same value with its o and k
        assert o["Qi"] == {"1": gs.backlog[t]} and o["Q"] == gs.backlog[t], (t, o["Qi"], gs.backlog[t])
        reg = gs.reg.get(t)
        if reg is not None and reg.get("Q") is not None:
            assert reg["Q"] == gs.backlog[t], (t, reg, gs.backlog[t])
            assert reg["o"] >= 0 and reg["n"] >= 0 and (reg["o"] > 0 or reg["n"] == 0)   # n_i = 0 when o_i = 0

    tx = defaultdict(list)
    for at, name, qos, t, s, n in gs.tx:
        tx[t].append((at, name, qos, s, n))
    lev_seen = set()
    checked = 0
    for t, o in sorted(dec.items()):
        p = o["p"]
        d = gs.demand[t][1]
        msgs = tx.get(t, [])
        if t >= max(dec) - 1 or min(d) == 0:
            continue   # the last steps may end after the subscriber stopped; a topic not yet active claims 0
        assert p[idx["bulk"]] == 0 and p[idx["state"]] == 2 and p[idx["imu"]] == 2 and p[idx["gps"]] == 1
        by = defaultdict(list)
        for at, name, qos, s, n in msgs:
            # 2. QoS 0 for both levels (revised model); level 0 never leaves
            assert p[idx[name]] in (1, 2) and qos == 0, (t, name, qos)
            lev_seen.add(p[idx[name]])
            # firm deadline: received within step t on this host
            assert at < start(epoch, t + 1) + 0.02, (t, name, at - start(epoch, t))
            by[name].append((s, n))
        for name, lst in by.items():
            # 9. sequence numbers 0..k-1 per topic and step, each once
            assert sorted(s for s, n in lst) == list(range(len(lst))), (t, name, lst)
            # 4. policing: the forwarded bytes in the channel stay within the declared d_il(t)
            assert sum(cost(n - 6) for s, n in lst) <= d[idx[name]] + 1e-6, (t, name)
        reg = gs.reg.get(t)
        if reg is not None:
            assert reg["mode"] == "paper" and reg["have_p"] and reg["have_d"]
            # one source of truth: the regulator polices the table that reached the AS
            for name in NAMES:
                assert reg["fwd"][idx[name]] == len(by.get(name, [])), (t, name, reg)
            assert reg["d"] == d and reg["p"] == p
        checked += 1
    assert checked >= 10
    assert lev_seen == {1, 2}

    # 3. no uplink without p(t): after the AS stopped
    later = [t for t in tx if t > max(dec)]
    assert not later, sorted(later)
    no_p = [t for t, r in gs.reg.items() if t > max(dec)]
    assert no_p, "no step summary after the AS stopped (t_stop %d)" % t_stop
    for t in no_p:
        assert not gs.reg[t]["have_p"] and sum(gs.reg[t]["fwd"]) == 0

    # 7. vessel log: bulk held at level 0, gps over its d_il, the steps without p(t)
    reasons = defaultdict(set)
    for line in (tmp / "reg-b1.jsonl").read_text().splitlines():
        o = json.loads(line)
        assert set(o) >= {"t", "topic", "bytes", "reason", "seq"}
        reasons[o["reason"]].add((o["t"], o["topic"]))
    assert any(name == "bulk" and t in dec for t, name in reasons["level0"])
    assert any(name == "gps" and t in dec for t, name in reasons["police"])
    assert any(t > max(dec) for t, name in reasons["no_table"])
    # a message logged for step t never reaches the GS
    sent = {(t, name) for t, lst in tx.items() for at, name, qos, s, n in lst}
    assert not any((t, "bulk") in sent for t, _ in reasons["level0"])
