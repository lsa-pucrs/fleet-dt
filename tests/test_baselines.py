"""Baselines of the paper stack (Table S3 of the paper): the configuration of every method in emu/fleet.py,
the per-vessel grants of B6 in the AS, the fleet feedback of B3, B7 and B8, the bridges of B2 and B9, and
short end-to-end runs of the regulator modes paper_b1 (B9, per-topic bridges), paper_b3 (B3), b8 (B7) and
paper_tdma (B8), and of the AS in policy b6 (B6), on the local patched Mosquitto (as
tests/test_paper_stack.py)."""
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import types
from collections import defaultdict
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "emu"))

import fleet  # noqa: E402
from fleetdt.as_model import DTP, admission, admission_b6  # noqa: E402
from fleetdt.as_service import AS, BRIDGE_STATE_RE  # noqa: E402
from fleetdt.feedback import FeedbackCore, decode_feedback, encode_feedback  # noqa: E402
from fleetdt.gsconf import bridge_count, gs_conf, vessel_conf  # noqa: E402
from fleetdt.model import decode_levels, decode_step_header, encode_demand, encode_levels  # noqa: E402
from fleetdt.ve.collector import Assembler, StepClock  # noqa: E402
from test_fleet_paper import fake_session  # noqa: E402
import test_paper_stack as tps  # noqa: E402

# --------------------------------------------------------------------------- configuration of the methods

# method -> (AS policy, plugin mode, gsconf gs options, feedback publisher)
EXPECT = {
    "fleetdt": ("alg1", "paper", "", False),
    "B1": (None, "paper_b1", "", False),
    "B2": (None, "paper_b2", "--b2", False),
    "B3": (None, "paper_b3", "--feedback", True),
    "B4": ("b4", "paper", "", False),
    "B5a": ("alg1", "paper_nopolice", "", False),
    "B5b": ("alg1", "paper_noeos", "", False),
    "B6": ("b6", "paper", "", False),
    "B7": (None, "b8", "--feedback", True),
    "B8": (None, "paper_tdma", "--feedback", True),
    "B9": (None, "paper_b1", "--per-topic /tmp/fdt-run/dtp.json", False),
}


def spec_of(tmp_path, method, **kw):
    s = {"kind": "fleet", "name": "m-" + method, "M": 2, "Ts": 0.125, "seed": 3, "steps": 8, "lead_s": 2.5,
         "dtp": "dtp.json", "scenario": "emu/scenarios/S1-m2.json", "sync": False, "run_dir": str(tmp_path / method),
         "method": method, "as": {"U": 0.8, "C_ref": 250000.0}, "outer": {"every": 480, "window": 480},
         "monitor": {"backend": "py"}}
    s.update(kw)
    return s


@pytest.mark.parametrize("method", sorted(EXPECT))
def test_method_configuration(tmp_path, method):
    policy, mode, gsopt, fb = EXPECT[method]
    R, calls = fake_session(tmp_path)
    res = fleet.run_fleet(R, spec_of(tmp_path, method), types.SimpleNamespace(stack=None))
    assert res["status"] == "ok", res.get("error")
    assert res["method"] == method and res["plugin_mode"] == mode
    ssh = [c for c in calls if c[0] == "ssh"]
    prep_gs = next(c[2] for c in ssh if c[1] == 0 and "echo prepared" in c[2])
    gs_line = next(l for l in prep_gs.splitlines() if "fleetdt.gsconf gs" in l)
    for opt in ("--b2", "--per-topic", "--feedback"):
        assert (opt in gs_line) == (opt in gsopt), (opt, gs_line)
    if gsopt:
        assert gsopt in gs_line
    for h in (1, 2):
        prep = next(c[2] for c in ssh if c[1] == h and "echo prepared" in c[2])
        assert "--mode %s " % mode in prep and "grep -q '^plugin_opt_mode %s$'" % mode in prep
    started = [c[2] for c in ssh if "setsid nohup" in c[2]]
    as_cmds = [c for c in started if "/as.log" in c]
    fb_cmds = [c for c in started if "/fb.log" in c]
    assert len(fb_cmds) == (1 if fb else 0)
    if policy is None:
        assert not as_cmds and res["commands"]["gs"]["as"] is None
    else:
        assert len(as_cmds) == 1
        a = as_cmds[0]
        assert ("--policy b6" in a) == (policy == "b6")
        if policy == "b4":
            assert "--alpha 0.0 --C0 250000.0" in a and "--outer-dta" not in a
        else:
            assert "--alpha 0.0" not in a and "--outer-every 480" in a
    # every method keeps the DTI program (M(t) and D_i(t) of the DTA) and the layered source
    assert sum("/dti.log" in c for c in started) == 2 and sum("fdt_source.py" in c for c in started) == 2
    nb = {"B2": 2 * 4, "B9": 2 * (1 + 22)}.get(method, 2 * 2)
    assert res["bridges"]["n"] == nb
    wait = next(c[2] for c in ssh if "wait_bridges.py" in c[2])
    assert "--n %d " % nb in wait


def test_method_names_and_checks(tmp_path):
    assert fleet.paper_method({"method": "b5A"})[0] == "B5a"
    assert fleet.paper_method({})[0] == "fleetdt"
    assert fleet.paper_method({"plugin_mode": "paper_noeos"})[0] == "B5b"
    with pytest.raises(SystemExit, match="does not match"):
        fleet.paper_method({"method": "B1", "plugin_mode": "paper"})
    with pytest.raises(SystemExit, match="one of"):
        fleet.paper_method({"method": "B10a"})
    with pytest.raises(SystemExit, match="C_ref"):
        fleet.paper_method({"method": "B4", "as": {"U": 0.7}})
    with pytest.raises(SystemExit, match="needs"):
        fleet.paper_method({"plugin_mode": "b8"})
    f = fleet.paper_feedback_cmd({"method": "B7", "dtp": {"Ts": 0.125}, "feedback": {"U": 0.6, "m": 4}})
    assert "fleetdt.feedback" in f and "--U 0.6" in f and "--m 4" in f
    f = fleet.paper_feedback_cmd({"method": "B7", "dtp": {"Ts": 0.125}})
    assert "--m" not in f.split() and fleet.paper_feedback_cmd({"method": "B6", "dtp": {"Ts": 0.125}}) is None


def test_paper_labels_of_table_s3():
    """The labels are those of Table S3: B6 the per-vessel tables of IPACT, B7 the DCC-like rate control,
    B8 the equal slots of phase 3, B9 one data bridge per uplink topic; the label B10 of the code before the
    paper labels fails with its paper label."""
    assert set(fleet.PAPER_METHODS) == {"fleetdt", "B1", "B2", "B3", "B4", "B5a", "B5b", "B6", "B7", "B8", "B9"}
    assert fleet.PAPER_METHODS["B6"] == ("b6", "paper", "single", False)
    assert fleet.PAPER_METHODS["B7"] == (None, "b8", "single", True)
    assert fleet.PAPER_METHODS["B8"] == (None, "paper_tdma", "single", True)
    assert fleet.PAPER_METHODS["B9"] == (None, "paper_b1", "per_topic", False)
    for old in ("B10", "b10"):
        with pytest.raises(SystemExit, match="now B9"):
            fleet.paper_method({"method": old})
    dtp = DTP(topics=["a"], Ts=1.0, T1=0.25, T2=0.25)
    with pytest.raises(ValueError, match="now b6"):
        AS(dtp, epoch=0.0, policy="b7")
    r = subprocess.run([sys.executable, "-m", "fleetdt.as_service", "--dtp", "x.json", "--policy", "b7"],
                       cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert r.returncode == 2 and "now b6" in r.stderr
    sys.path.insert(0, str(ROOT / "emu" / "experiments"))
    import conformance
    with pytest.raises(SystemExit, match="now B9"):
        conformance.method_key("B10")
    assert conformance.method_key("b6") == "B6" and "B6" in conformance.TABLE_METHODS
    assert conformance.method_key("b9") == "B9" and "B7" not in conformance.TABLE_METHODS


def test_qbar_reset_bridges():
    """BRIDGE_STATE_RE takes the single data bridge b<i>-data, the bridge of every method with the AS; the
    per-class (B2) and per-topic (B9) bridges belong to methods without the AS, so no Qbar_i depends on them."""
    assert BRIDGE_STATE_RE.match("fleet/bridge/b3-data/state").group(1) == "3"
    assert BRIDGE_STATE_RE.match("$SYS/broker/connection/b12-data/state").group(1) == "12"
    for name in ("b1-data-ef", "b1-data-state", "b1-ctl"):
        assert not BRIDGE_STATE_RE.match("fleet/bridge/%s/state" % name)
    for k, (policy, mode, bridges, fb) in fleet.PAPER_METHODS.items():
        if bridges != "single":
            assert policy is None, k


def test_old_core_tdma_reads_the_layered_dtp_v2():
    """The tdma mode of the older core (as2 stack) loads dtp_v2.json (L = 22); the paper stack runs B8 with
    paper_tdma, because the older core uses the 1-based clock, JSON tables and logs and the QoS of the DTP v2."""
    from fleetdt.as2.dtp import DTP as DTP2
    d = DTP2.from_dict(json.loads((ROOT / "dtp_v2.json").read_text()))
    assert d.L == 22
    assert fleet.METHODS["tdma"] == ("tdma", "tdma")


# --------------------------------------------------------------------------- B6 in the AS


def test_b6_redistributes_the_unused_grant():
    D = {1: [475, 1187, 9971, 6920], 2: [100, 100, 30000, 30000], 3: [100, 100, 30000, 30000]}
    order, needed = [1, 2, 3, 4], [1]
    b = 30400.0
    p_by, g = admission_b6(D, order, needed, b)
    assert p_by[1] == [2, 1, 1, 1] and g[1] == sum(D[1])          # fully admitted: keeps what it uses
    excess = b - sum(D[1])
    assert g[2] == pytest.approx(b + excess / 2) and g[3] == pytest.approx(b + excess / 2)
    assert p_by[2] == p_by[3] == [2, 1, 1, 0]
    assert sum(g.values()) == pytest.approx(3 * b)                  # the grants sum to m b = max(0, U K_hat - Q)
    # Algorithm 1 with the same b gives one table for the fleet
    assert admission(D, order, needed, b) == [2, 1, 1, 0]
    # every vessel fully admitted or none: nothing moves
    p_by, g = admission_b6({1: [10, 10], 2: [20, 20]}, [1, 2], [], 100.0)
    assert g == {1: 100.0, 2: 100.0} and p_by == {1: [1, 1], 2: [1, 1]}


def test_as_policy_b6_publishes_one_table_per_vessel():
    dtp = DTP(topics=["a", "b", "c", "d"], Ts=1.0, T1=0.25, T2=0.25, alpha=0.5, U=1.0, C0=121600.0,
              order=["a", "b", "c", "d"], needed=["a"])
    a = AS(dtp, epoch=0.0, policy="b6")
    a.on_demand(1, 0, [475, 1187, 9971, 6920], at=0.01)
    a.on_demand(2, 0, [100, 100, 30000, 30000], at=0.01)
    dec = a.close_phase1(0)
    assert dec["b"] == pytest.approx(30400.0) and dec["p"] is None
    assert dec["p_by"][1] == [2, 1, 1, 1] and dec["p_by"][2] == [2, 1, 1, 0]
    assert sum(dec["grants"].values()) == pytest.approx(dec["cap"]) == pytest.approx(dec["K_hat"])   # U = 1, Q = 0
    with pytest.raises(ValueError):
        AS(dtp, epoch=0.0, policy="x")


def test_ve_takes_the_table_of_each_vessel():
    dtp = DTP(topics=["a", "b"], Ts=1.0, T1=0.25, T2=0.25, c_b=1.0, c_m=0.0)
    asm = Assembler(dtp, StepClock(1.0, 0.0, now=lambda: 0.0))
    for i in (1, 2, 3):
        asm.on_message("boat/b%d/demand" % i, encode_demand(0, [10, 10]), at=0.1)
    asm.on_message("boat/b1/levels", encode_levels(0, [1, 1]), at=0.3)
    asm.on_message("boat/b2/levels", encode_levels(0, [2, 0]), at=0.3)
    g = asm._finish(0)
    assert [g.edge(1, l).p for l in (1, 2)] == [1, 1] and [g.edge(2, l).p for l in (1, 2)] == [2, 0]
    assert [g.edge(3, l).p for l in (1, 2)] == [1, 1]     # without its own table: the first table of the step


# --------------------------------------------------------------------------- feedback and bridges


def test_feedback_payload_and_core():
    p = encode_feedback(7, 123456.5, 0.6, 4, [3, 1, 2])
    assert len(p) == 18 + 3 and decode_feedback(p) == (7, 123456.5, pytest.approx(0.6), 4, [3, 1, 2])
    assert decode_feedback(encode_feedback(7, None, 0.6, 4, [1]))[1] is None
    dtp = DTP.from_dict(json.loads((ROOT / "dtp.json").read_text()))
    c = FeedbackCore(dtp, U=0.5, epoch=100.0)
    assert c.m_fixed is None and c.sigma == list(range(1, 23))
    pay, rec = c.close(3)
    assert c.close(3) is None                                              # once per step
    assert rec["m"] == 0 and rec["M"] == [] and rec["y"] is None
    rev = list(reversed(dtp.topics))
    assert c.on_order(rev) and not c.on_order(rev[:-1])
    t, y, U, m, sig = decode_feedback(c.close(4)[0])
    assert t == 4 and sig == list(range(22, 0, -1)) and y is None and U == pytest.approx(0.5)


def test_feedback_carries_y_of_the_last_step_and_m_of_the_step():
    """The feedback of step t (at start(t) + T1) carries y(t - 1) when it arrived before start(t) + T1 and
    m(t) = |M(t)|, the vessels whose demand table of step t arrived before start(t) + T1 (the rule of the AS)."""
    dtp = DTP(topics=["a", "b"], Ts=1.0, T1=0.25, T2=0.25, U=0.8, vessels=[1, 2, 3])
    c = FeedbackCore(dtp, epoch=0.0)
    assert c.deadline(5) == 5.25
    assert c.on_throughput(4, 9000.0, at=5.001)
    assert c.on_demand(1, 5, 2, at=5.002) and c.on_demand(3, 5, 2, at=5.2)
    assert not c.on_demand(2, 5, 2, at=5.25)               # at start(t) + T1: outside M(t)
    assert not c.on_demand(2, 5, 3, at=5.01)               # L differs from the DTP
    assert not c.on_demand(2, 7, 2, at=5.01)               # more than one step ahead
    assert c.on_demand(2, 6, 2, at=5.9)                    # a table of the next step counts in M(6)
    pay, rec = c.close(5)
    assert rec["M"] == [1, 3] and rec["m"] == 2 and rec["y"] == 9000.0 and rec["y_t"] == 4
    assert decode_feedback(pay)[:4] == (5, 9000.0, pytest.approx(0.8), 2)
    assert not c.on_throughput(5, 7000.0, at=6.25)         # y(5) after start(6) + T1: late
    assert not c.on_demand(1, 5, 2, at=5.1)                # step 5 is closed
    pay, rec = c.close(6)
    assert rec["M"] == [2] and rec["y"] is None and decode_feedback(pay)[1] is None
    # --m fixes m
    f = FeedbackCore(dtp, m=4, epoch=0.0)
    f.on_demand(1, 0, 2, at=0.01)
    assert f.close(0)[1]["m"] == 4


def test_bridges_of_b2_and_b9():
    names = [t["name"] for t in json.loads((ROOT / "dtp.json").read_text())["topics"]]
    conf = gs_conf({1: "10.0.0.1", 2: "10.0.0.2"}, per_topic=names, feedback=True)
    conns = re.findall(r"^connection (\S+)$", conf, re.M)
    assert len(conns) == bridge_count(2, per_topic=names) == 2 * 23
    for i in (1, 2):
        for n in names:
            blk = conf.split("connection b%d-data-%s\n" % (i, n))[1].split("\nconnection ")[0]
            assert "address 10.0.0.%d:1884" % i in blk and "topic boat/b%d/tx/%s in 0" % (i, n) in blk
    assert conf.count("topic fleet/feedback out 0") == 2
    b2 = gs_conf({1: "10.0.0.1"}, b2=True)
    assert len(re.findall(r"^connection ", b2, re.M)) == bridge_count(1, b2=True) == 4
    v = vessel_conf(1, "/x.json", mode="paper_b2")
    assert "plugin_opt_mode paper_b2" in v
    assert "plugin_opt_data_ports 1884,1885,1886" in v and "listener 1885" in v and "listener 1886" in v
    with pytest.raises(ValueError):
        gs_conf({1: "a"}, b2=True, per_topic=names)


def test_b2_classes_map_to_the_access_categories():
    """B2 (Table S3): the class ef of the topic state crosses in AC_VO, af41 (other telemetry) in AC_VI, be
    (video stripes) in AC_BE. Each class has its port (gsconf B2_PORTS); system/nftables-fdt.nft sets the
    skb->priority 256 + UP and the DSCP by port in both directions, and the emulator maps the DSCP to the same UP
    (emu/scenarios/gen.py PRIO). UP -> AC of IEEE 802.11 (Table 10-1): 0 BE, 5 VI, 6 VO."""
    from fleetdt.gsconf import B2_PORTS
    sys.path.insert(0, str(ROOT / "emu" / "scenarios"))
    import gen
    ac = {0: "AC_BE", 4: "AC_VI", 5: "AC_VI", 6: "AC_VO", 7: "AC_VO"}
    nft = (ROOT / "system" / "nftables-fdt.nft").read_text()
    defs = dict(re.findall(r"^define (\S+) = (\S+)$", nft, re.M))
    want = {"be": "AC_BE", "af41": "AC_VI", "ef": "AC_VO"}
    dscp = {"af41": 34, "ef": 46}
    for cls, port in B2_PORTS.items():
        if cls == "be":
            assert port == 1884 and "1884" not in nft.split("table inet")[1]   # the data port stays unmarked
            continue
        var = {p: k for k, p in defs.items()}[str(port)]                     # FDT_B2_<CLS>_PORT
        prio = defs[var.replace("_PORT", "_PRIO")]
        up = int(prio.split(":")[1], 16) - 0x100
        assert ac[up] == want[cls], (cls, prio)
        for d in ("sport", "dport"):
            assert "tcp %s $%s meta priority set $%s" % (d, var, var.replace("_PORT", "_PRIO")) in nft
            assert "meta nfproto ipv4 tcp %s $%s ip dscp set %s" % (d, var, cls) in nft
        assert ac[gen.PRIO["dscp"][str(dscp[cls])]] == want[cls]


# --------------------------------------------------------------------------- end to end on the local broker

RUN_S = float(os.environ.get("FDT_BASELINE_SECONDS", "4"))
C_B, C_M = tps.C_B, tps.C_M
TOPICS = [{"name": "state", "rate": 8, "bound": 48}, {"name": "imu", "rate": 32, "bound": 48},
          {"name": "video", "rate": 64, "bound": 1000, "jitter": 0}, {"name": "bulk", "rate": 16, "bound": 2000}]
NAMES = [t["name"] for t in TOPICS]
IDX = {n: k for k, n in enumerate(NAMES)}
RATE = {"state": 8, "imu": 32, "video": 64, "bulk": 16}
SIZE = {"state": 48, "imu": 48, "video": 1000, "bulk": 2000}
T3 = tps.TS - tps.T1 - tps.T2

e2e = pytest.mark.skipif(not (tps.BROKER.exists() and tps.PLUGIN.exists() and tps.DTI.exists()),
                         reason="needs Mosquitto 2.1 (MOSQ), fdt_regulator.so and fdt_dti (make)")


def cost(n):
    return C_B * (n - 6 + 6) + C_M          # n: payload bytes with the step header


class Sources(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.c = tps.client("fdt-bl-src-%d" % os.getpid())
        self.c.connect("127.0.0.1", port, 10)
        self.c.loop_start()
        self.stop_ev = threading.Event()
        self.start()

    def run(self):
        t0 = time.time()
        nxt = {n: t0 + 0.0013 * k for k, n in enumerate(NAMES)}
        while not self.stop_ev.is_set():
            now = time.time()
            for n in NAMES:
                while nxt[n] <= now:
                    self.c.publish("boat/b1/raw/%s" % n, bytes(SIZE[n]), qos=0)
                    nxt[n] += 1.0 / RATE[n]
            time.sleep(0.0005)

    def stop(self):
        self.stop_ev.set()
        self.join(2)
        self.c.loop_stop()
        self.c.disconnect()


class GS(object):
    def __init__(self, port):
        self.lock = threading.Lock()
        self.tx, self.reg, self.levels, self.bridges = [], {}, {}, {}
        self.c = tps.client("fdt-bl-gs-%d" % os.getpid())
        self.c.on_connect = lambda c, u, f, rc, props=None: c.subscribe(
            [("boat/b1/tx/#", 2), ("boat/b1/regulator", 0), ("boat/+/levels", 1), ("fleet/bridge/+/state", 0)])
        self.c.on_message = self.on_message
        self.c.connect("127.0.0.1", port, 10)
        self.c.loop_start()

    def on_message(self, c, u, msg):
        at = time.time()
        with self.lock:
            if msg.topic.startswith("boat/b1/tx/"):
                t, s = decode_step_header(msg.payload)
                self.tx.append((at, msg.topic[len("boat/b1/tx/"):], msg.qos, t, s, len(msg.payload)))
            elif msg.topic == "boat/b1/regulator":
                o = json.loads(msg.payload)
                self.reg[int(o["t"])] = o
            elif msg.topic.endswith("/levels"):
                t, p = decode_levels(msg.payload)
                self.levels[(int(msg.topic.split("/")[1][1:]), t)] = p
            else:
                self.bridges[msg.topic.split("/")[2]] = msg.payload

    def stop(self):
        self.c.loop_stop()
        self.c.disconnect()


def bring_up(tmp, mode, bridges="single", feedback=False, vessels=(1, 2), C0=300000):
    epoch = float(math.floor(time.time()) - 20)
    dtp = {"epoch": epoch, "Ts": tps.TS, "T1": tps.T1, "T2": tps.T2, "alpha": 0.125, "U": 0.8, "C0": C0,
           "c_b": C_B, "c_m": C_M, "R_PHY": 24000000, "h_W": 150, "vessels": list(vessels), "topics": TOPICS,
           "order": NAMES, "needed": ["state"]}
    (tmp / "dtp.json").write_text(json.dumps(dtp))
    ctl, data, gsp = tps.free_port(), tps.free_port(), tps.free_port()
    vconf = "listener %d 127.0.0.1\nlistener %d 127.0.0.1\n" % (ctl, data) + tps.BASE.format(log=tmp / "vessel.log")
    vconf += ("plugin %s\nplugin_opt_dtp %s\nplugin_opt_vessel 1\nplugin_opt_mode %s\nplugin_opt_data_port %d\n"
              "plugin_opt_log %s\nplugin_opt_stats_s 1\nplugin_opt_demand external\nplugin_opt_jitter_ms 2\n") % (
        tps.PLUGIN, tmp / "dtp.json", mode, data, tmp / "reg-b1.jsonl")
    gconf = "listener %d 127.0.0.1\n" % gsp + tps.BASE.format(log=tmp / "gs.log")
    gconf += tps.BRIDGE.format(name="b1-ctl", port=ctl)
    gconf += "topic boat/b1/demand in 1\ntopic boat/b1/regulator in 0\ntopic boat/b1/levels out 1\n"
    if feedback:
        gconf += "topic fleet/feedback out 1\n"
    names = ["b1-ctl"]
    if bridges == "per_topic":
        for n in NAMES:
            gconf += tps.BRIDGE.format(name="b1-data-%s" % n, port=data) + "topic boat/b1/tx/%s in 0\n" % n
            names.append("b1-data-%s" % n)
    else:
        gconf += tps.BRIDGE.format(name="b1-data", port=data) + "topic boat/b1/tx/# in 0\n"
        names.append("b1-data")
    (tmp / "vessel.conf").write_text(vconf)
    (tmp / "gs.conf").write_text(gconf)
    procs = [subprocess.Popen([str(tps.BROKER), "-c", str(tmp / "vessel.conf")])]
    tps.wait_port(ctl)
    procs.append(subprocess.Popen([str(tps.DTI), "--dtp", str(tmp / "dtp.json"), "--vessel", "1", "--broker",
                                   "127.0.0.1:%d" % ctl, "--jitter-ms", "2", "--prio", "0"]))
    procs.append(subprocess.Popen([str(tps.BROKER), "-c", str(tmp / "gs.conf")]))
    tps.wait_port(gsp)
    gs = GS(gsp)
    end = time.time() + 10
    while time.time() < end and not all(gs.bridges.get(b) == b"1" for b in names):
        time.sleep(0.05)
    assert all(gs.bridges.get(b) == b"1" for b in names), gs.bridges
    return {"tmp": tmp, "epoch": epoch, "ctl": ctl, "gs_port": gsp, "procs": procs, "gs": gs, "dtp": dtp}


def py(tmp, *args):
    env = dict(os.environ, PYTHONPATH=str(ROOT))
    return subprocess.Popen([sys.executable] + list(args), env=env, cwd=str(ROOT), stderr=subprocess.PIPE)


def tear_down(st, extra=()):
    st["gs"].stop()
    for p in list(extra)[::-1] + st["procs"][::-1]:
        tps._kill(p)


class YPublisher(threading.Thread):
    """y(t) = Y on fleet/throughput 1 ms after the end of each step (the channel monitor of the GS)"""

    def __init__(self, port, epoch, Y):
        super().__init__(daemon=True)
        self.epoch, self.Y = epoch, Y
        self.c = tps.client("fdt-bl-y-%d" % os.getpid())
        self.c.connect("127.0.0.1", port, 10)
        self.c.loop_start()
        self.stop_ev = threading.Event()
        self.start()

    def run(self):
        t = math.floor((time.time() - self.epoch) / tps.TS)
        while not self.stop_ev.is_set():
            at = self.epoch + (t + 1) * tps.TS + 0.001
            if self.stop_ev.wait(max(0.0, at - time.time())):
                break
            self.c.publish("fleet/throughput", json.dumps({"t": t, "y": self.Y}))
            t += 1

    def stop(self):
        self.stop_ev.set()
        self.join(2)
        self.c.loop_stop()


def by_step(gs):
    tx = defaultdict(list)
    for at, name, qos, t, s, n in gs.tx:
        tx[t].append((at, name, qos, s, n))
    return tx


def log_reasons(tmp):
    out = defaultdict(set)
    p = tmp / "reg-b1.jsonl"
    for line in (p.read_text().splitlines() if p.exists() else []):
        o = json.loads(line)
        out[o["reason"]].add((o["t"], o["topic"]))
    return out


@e2e
def test_b9_per_topic_bridges_paper_b1(tmp_path):
    st = bring_up(tmp_path, "paper_b1", bridges="per_topic")
    src = Sources(st["ctl"])
    time.sleep(RUN_S)
    src.stop()
    time.sleep(0.5)
    tear_down(st)
    gs, epoch = st["gs"], st["epoch"]
    tx = by_step(gs)
    assert {name for at, name, qos, t, s, n in gs.tx} == set(NAMES)
    assert {qos for at, name, qos, t, s, n in gs.tx} == {0}   # the QoS 0 data bridges carry the QoS 1 publications of the baseline with QoS 0
    checked = 0
    for t, msgs in tx.items():
        for at, name, qos, s, n in msgs:
            # forwarded on arrival with the step of that instant
            assert epoch + t * tps.TS - 0.005 <= at < epoch + (t + 1) * tps.TS + 0.03, (t, name, at - epoch - t * tps.TS)
        reg = gs.reg.get(t)
        if reg is not None and t < max(tx) - 1:
            for name in NAMES:
                got = sorted(s for at, nm, q, s, n in msgs if nm == name)
                assert got == list(range(reg["fwd"][IDX[name]])), (t, name)
                assert reg["fwd"][IDX[name]] == reg["rx"][IDX[name]] and reg["held"][IDX[name]] == 0
            assert reg["mode"] == "paper_b1" and not reg["have_p"]
            checked += 1
    assert checked >= 10
    assert not log_reasons(tmp_path)


@e2e
def test_b3_strict_priority_in_sigma(tmp_path):
    st = bring_up(tmp_path, "paper_b3", feedback=True)
    fb = py(tmp_path, "-m", "fleetdt.feedback", "--dtp", str(tmp_path / "dtp.json"), "--broker",
            "127.0.0.1:%d" % st["gs_port"], "--log", str(tmp_path / "fb.jsonl"))
    oc = tps.client("fdt-bl-order-%d" % os.getpid())
    oc.connect("127.0.0.1", st["gs_port"], 10)
    oc.loop_start()
    time.sleep(1.0)
    sigma = ["bulk", "video", "imu", "state"]
    oc.publish("fleet/order", json.dumps({"order": sigma, "needed": ["bulk"]}), qos=1)
    src = Sources(st["ctl"])
    time.sleep(RUN_S)
    src.stop()
    time.sleep(0.5)
    oc.loop_stop()
    tear_down(st, [fb])
    gs = st["gs"]
    tx = by_step(gs)
    rank = {n: k for k, n in enumerate(sigma)}
    want = [IDX[n] + 1 for n in sigma]
    checked = 0
    for t, reg in sorted(gs.reg.items()):
        if reg.get("sigma") != want or t not in tx or t >= max(tx) - 1:
            continue
        order = [rank[name] for at, name, qos, s, n in sorted(tx[t])]
        assert order == sorted(order), (t, order)              # topic by topic in sigma_t
        assert all(q == 0 for at, name, q, s, n in tx[t])   # QoS 0 data bridge
        assert reg["fwd"] == reg["rx"] and sum(reg["held"]) == 0
        checked += 1
    assert checked >= 10, sorted(gs.reg)[:5]


@e2e
def test_b7_budget_from_y(tmp_path):
    st = bring_up(tmp_path, "b8", feedback=True)
    fb = py(tmp_path, "-m", "fleetdt.feedback", "--dtp", str(tmp_path / "dtp.json"), "--broker",
            "127.0.0.1:%d" % st["gs_port"], "--U", "0.5", "--m", "1", "--log", str(tmp_path / "fb.jsonl"))
    Y = 40000.0
    yp = YPublisher(st["gs_port"], st["epoch"], Y)
    src = Sources(st["ctl"])
    time.sleep(RUN_S)
    src.stop()
    yp.stop()
    time.sleep(0.5)
    tear_down(st, [fb])
    gs = st["gs"]
    tx = by_step(gs)
    B = 0.5 * Y * T3 / tps.TS / 1
    checked = 0
    held_steps = 0
    for t, reg in sorted(gs.reg.items()):
        if abs((reg.get("y") or 0) - Y) > 1 or t >= max(tx or [0]) - 1:
            continue
        assert abs(reg["budget"] - B) < 1, reg
        sent = sum(cost(n) for at, name, q, s, n in tx.get(t, []))
        assert sent <= B + 1e-6, (t, sent, B)
        assert reg["cost"] and sum(reg["cost"]) <= B + 1
        # strict priority: nothing of a later topic in sigma (the DTP order) after a held message
        held = [reg["held"][IDX[n]] for n in NAMES]
        fwd = [reg["fwd"][IDX[n]] for n in NAMES]
        first = next((k for k, h in enumerate(held) if h), None)
        if first is not None:
            held_steps += 1
            assert sum(fwd[first + 1:]) == 0, reg
        checked += 1
    assert checked >= 10 and held_steps >= 5
    assert any(name == "bulk" for t, name in log_reasons(tmp_path)["budget"])


@e2e
def test_b8_paper_tdma_slot(tmp_path):
    st = bring_up(tmp_path, "paper_tdma", feedback=True, vessels=(1, 2))
    src = Sources(st["ctl"])
    time.sleep(RUN_S)
    src.stop()
    time.sleep(0.5)
    tear_down(st)
    gs, epoch = st["gs"], st["epoch"]
    tx = by_step(gs)
    checked = 0
    for t, msgs in tx.items():
        reg = gs.reg.get(t)
        if reg is None or t >= max(tx) - 1:
            continue
        v = epoch + t * tps.TS + tps.T1 + tps.T2
        s0, s1 = reg["slot"]
        assert s0 == int(round(v * 1e6)) and s1 == s0 + int(round(T3 * 1e6)) // 2
        for at, name, q, s, n in msgs:
            assert v - 0.001 <= at < s1 / 1e6 + 0.015, (t, name, at - v)   # vessel 1: first half of phase 3
            assert q == 0   # QoS 0 data bridge
        checked += 1
    assert checked >= 10


@e2e
def test_b6_tables_per_vessel(tmp_path):
    """Vessel 1 (regulator in mode paper, demand of fdt_dti) and the demand table of a heavy vessel 2
    published at the GS: with policy b6 the light vessel 1 gets a table that admits bulk, vessel 2 gets
    the excess of vessel 1 and a table that holds bulk, and the regulator of vessel 1 forwards bulk under
    its own table (Algorithm 1 would hold bulk for both)."""
    st = bring_up(tmp_path, "paper", vessels=(1, 2), C0=100000)
    a = py(tmp_path, "-m", "fleetdt.as_service", "--dtp", str(tmp_path / "dtp.json"), "--broker",
           "127.0.0.1:%d" % st["gs_port"], "--log", str(tmp_path / "as.jsonl"), "--policy", "b6",
           "--alpha", "0", "--U", "0.8")
    v2 = tps.client("fdt-bl-v2-%d" % os.getpid())
    v2.connect("127.0.0.1", st["gs_port"], 10)
    v2.loop_start()
    stop = threading.Event()
    epoch = st["epoch"]

    def demand2():
        t = math.floor((time.time() - epoch) / tps.TS) + 1
        while not stop.is_set():
            at = epoch + t * tps.TS + 0.002
            if stop.wait(max(0.0, at - time.time())):
                break
            v2.publish("boat/b2/demand", encode_demand(t, [100, 100, 30000, 30000]), qos=1)
            t += 1
    th = threading.Thread(target=demand2, daemon=True)
    th.start()
    src = Sources(st["ctl"])
    time.sleep(RUN_S)
    src.stop()
    stop.set()
    th.join(2)
    v2.loop_stop()
    time.sleep(0.5)
    a.send_signal(signal.SIGINT)
    a.wait(10)
    tear_down(st)
    gs = st["gs"]
    tx = by_step(gs)
    dec = {}
    for line in (tmp_path / "as.jsonl").read_text().splitlines():
        o = json.loads(line)
        if o.get("published") and o.get("p_by") and o["M"] == [1, 2]:
            dec[o["t"]] = o
    checked = 0
    for t, o in dec.items():
        p1, p2 = o["p_by"]["1"], o["p_by"]["2"]
        if gs.levels.get((1, t)) is None or t >= max(tx or [0]) - 1 or t not in gs.reg:
            continue
        assert gs.levels[(1, t)] == p1 and gs.levels.get((2, t)) == p2
        assert sum(o["grants"].values()) == pytest.approx(o["cap"], rel=1e-9)
        reg = gs.reg[t]
        if min(reg["d"]) == 0:
            continue
        assert p1[IDX["bulk"]] == 1 and p2[IDX["bulk"]] == 0, (p1, p2, o["grants"])
        assert reg["p"] == p1
        got = [name for at, name, q, s, n in tx.get(t, [])]
        assert "bulk" in got                                      # vessel 1 forwards under its own table
        checked += 1
    assert checked >= 8, (len(dec), sorted(gs.reg)[:3])
