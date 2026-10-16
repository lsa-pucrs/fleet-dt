"""Integration of the paper stack in the emulator harness and its GS programs: the layered DTP of the
as2 campaign and its stack, the per-vessel DTP and the layered video source of emu/fleet.py, the jitter
allowance shared by the regulator and the DTI program, r_il with the cost of d_il, the cost bound of a
DTP without c_b and c_m, the SUBACK before step 0 and the outer loop in a child process of the AS, and
the documented analysis commands."""
import base64
import json
import math
import os
import random
import re
import sys
import threading
import time
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "emu"))
sys.path.insert(0, str(ROOT / "emu" / "experiments"))
sys.path.insert(0, str(ROOT / "vessel" / "publishers"))

import fleet  # noqa: E402
import fdt_source  # noqa: E402
import video_layers as vl  # noqa: E402
from fleetdt import outer  # noqa: E402
from fleetdt.as_model import DTP, publish_header  # noqa: E402
from fleetdt.as_service import Service  # noqa: E402
from fleetdt.mock import FleetSim  # noqa: E402
from fleetdt.model import encode_demand, encode_step_header  # noqa: E402
from fleetdt.ve.collector import Assembler, StepClock  # noqa: E402
from test_fleet_paper import fake_session  # noqa: E402

LAYERS = ["video_%d" % j for j in range(1, 17)]


def root_dtp():
    return json.loads((ROOT / "dtp.json").read_text())


def paper_spec(tmp_path, **kw):
    s = {"kind": "fleet", "name": "harness", "M": 2, "Ts": 0.125, "seed": 5, "steps": 8, "lead_s": 2.5,
         "dtp": "dtp.json", "scenario": "emu/scenarios/S1-m2.json", "sync": True, "run_dir": str(tmp_path / "run"),
         "as": {"U": 0.8}, "monitor": {"backend": "py"}}
    s.update(kw)
    return s


def run(tmp_path, **kw):
    R, calls = fake_session(tmp_path)
    res = fleet.run_fleet(R, paper_spec(tmp_path, **kw), types.SimpleNamespace(stack=None))
    assert res["status"] == "ok", res.get("error")
    ssh = [c for c in calls if c[0] == "ssh"]
    return res, calls, ssh


# --------------------------------------------------------------------------- item 1


def test_dtp_base_has_the_layers_of_dtp_v2():
    base = json.loads((ROOT / "emu" / "experiments" / "dtp_base.json").read_text())
    v2 = json.loads((ROOT / "dtp_v2.json").read_text())
    assert base == v2
    assert [t["name"] for t in base["topics"]][6:] == LAYERS


def test_matrix_runs_the_as2_stack_with_the_layers():
    import matrix
    s = matrix.spec("SAT", "b1", "S1", 4, 0.125, None, 9101, 40, sat=True)
    assert s["stack"] == "as2"
    lay = [t for t in s["dtp"]["topics"] if t["name"] in LAYERS]
    # stripe layers: every layer topic carries one stripe of every frame; n_j from the share of its stripe
    assert len(lay) == 16 and all(abs(t["rate"] - matrix.SAT_RATE) < 1e-6 and t["s"] == 1400 for t in lay)
    w = vl.load_profile(matrix.profile_key(matrix.SAT_TRACE))
    mx = max(matrix.trace(matrix.SAT_TRACE))
    assert [t["n"] for t in lay] == [math.ceil(mx * w[vl.stripe_of_layer(j)] / 1392.0) for j in range(1, 17)]
    from fleetdt.as2.dtp import DTP as DTP2
    assert DTP2.from_dict(s["dtp"]).L == 22
    # rho: the frame rate solves sum_i (c_streams + r Ts c_frame,i) = rho kappa T_W with per-vessel scales
    cal = {"sat": {"S1-m4": {"kappa": 2.0, "R_up": 24e6, "R_down": 24e6}}}
    rs = matrix.rho_setting(cal, "S1", 4, 0.125, 1.5)
    tot = sum(rs["c_streams"] + rs["rate"] * 0.125 * c for c in rs["c_frame_vessels"])
    assert tot == pytest.approx(rs["target"], rel=1e-3)
    assert len(set(rs["c_frame_vessels"])) == 4          # the size_scale of each vessel


def test_runner_passes_the_as2_stack(tmp_path):
    import runner
    s = runner.run_spec({"name": "x", "as": {"init_scores_file": "init/S1-eq.json"}}, "/r", "/m")
    assert s["stack"] == "as2" and s["sync"] is False and s["run_dir"] == "/r"
    assert s["as"]["init_scores_file"] == "/m/init/S1-eq.json"
    assert runner.run_spec({"stack": "paper"}, "/r", "/m")["stack"] == "paper"


# --------------------------------------------------------------------------- items 2 and 3


def _b64_file(cmd, name):
    m = re.search(r"echo (\S+) \| base64 -d > /tmp/fdt-run/%s" % re.escape(name), cmd)
    return json.loads(base64.b64decode(m.group(1))) if m else None


def test_per_vessel_dtp_and_layered_source(tmp_path):
    res, calls, ssh = run(tmp_path)
    d = root_dtp()
    bounds = {}
    for h in (1, 2):
        prep = next(c[2] for c in ssh if c[1] == h and "echo prepared" in c[2])
        vd = _b64_file(prep, "dtp_vessel.json")
        assert vd == vl.resolve_vessel(dict(d, vessels=[1, 2]), h)
        assert json.load(open(tmp_path / "run" / ("dtp_b%d.json" % h))) == vd
        assert "--dtp /tmp/fdt-run/dtp_vessel.json" in prep
        bounds[h] = [t["bound"] for t in vd["topics"] if t["name"] in LAYERS]
    assert bounds[1] != bounds[2]                          # the demand tables differ between vessels
    assert set(res["dtp_vessels_sha256"]) == {"1", "2"}
    started = [c[2] for c in ssh if "setsid nohup" in c[2]]
    src = [c for c in started if "src.log" in c]
    assert len(src) == 2 and all("vessel/publishers/fdt_source.py" in c and "--t0" in c for c in src)
    dti = [c for c in started if "dti.log" in c]
    assert all("--dtp /tmp/fdt-run/dtp_vessel.json" in c for c in dti)


def _jitters(ssh):
    preps = [c[2] for c in ssh if c[1] > 0 and "echo prepared" in c[2]]
    reg = [float(re.search(r"plugin_opt_jitter_ms ([0-9.]+)", p).group(1)) for p in preps]
    dti = [float(re.search(r"--jitter-ms ([0-9.]+)", c[2]).group(1)) for c in ssh
           if "setsid nohup" in c[2] and "dti.log" in c[2]]
    return reg, dti


@pytest.mark.parametrize("opts,want", [({}, 2.0), ({"plugin_opts": {"jitter_ms": 3.5}}, 3.5),
                                       ({"dti": {"jitter_ms": 1.0}}, 1.0)])
def test_regulator_and_dti_jitter_match(tmp_path, opts, want):
    res, calls, ssh = run(tmp_path, **opts)
    reg, dti = _jitters(ssh)
    assert reg == [want, want] and dti == [want, want] and res["jitter_ms"] == want
    # plugin_opt_jitter_ms appears once (not again through plugin_opts)
    prep = next(c[2] for c in ssh if c[1] == 1 and "echo prepared" in c[2])
    assert prep.count("plugin_opt_jitter_ms") == 1


def test_conflicting_jitter_is_refused(tmp_path):
    R, _ = fake_session(tmp_path)
    with pytest.raises(SystemExit, match="jitter"):
        fleet.run_fleet(R, paper_spec(tmp_path, plugin_opts={"jitter_ms": 2}, dti={"jitter_ms": 4}),
                        types.SimpleNamespace(stack=None))


def test_gsconf_writes_the_jitter_of_the_regulator():
    from fleetdt.gsconf import vessel_conf
    c = vessel_conf(1, "/x/dtp.json", mode="paper", demand="external", jitter_ms=2.5, reg_log="/x/reg.jsonl")
    assert "plugin_opt_jitter_ms 2.5" in c and "plugin_opt_demand external" in c and "plugin_opt_log /x/reg.jsonl" in c


def test_source_reads_the_paper_dtp(tmp_path):
    d = vl.resolve_vessel(root_dtp(), 2)
    p = tmp_path / "dtp_vessel.json"
    p.write_text(json.dumps(d))
    dtp = fdt_source.load_dtp(str(p))
    tps = {t.name: t for t in dtp["topics"]}
    assert tps["state"].kind == "stream" and tps["state"].n == 1 and tps["state"].s == 48
    lay = [tps[n] for n in LAYERS]
    raw = {t["name"]: t for t in d["topics"]}
    assert all(t.kind == "output" and t.n == vl.msgs_per_stripe(d, t.name) and t.s == raw[t.name]["bound"] and
               t.layers == 16 for t in lay)
    assert [t.layer for t in lay] == list(range(1, 17)) and all(abs(t.rate - 16.0) < 1e-12 for t in lay)
    # replay of the default trace: each frame within the frame cap of the vessel, on its layer topic
    a = types.SimpleNamespace(dtp=str(p), vessel=2, topics=set(LAYERS), stream_size="max", median=None, sigma=0.5,
                              size_trace=str(ROOT / fleet.PAPER_TRACE), load=1.0, load_mode="size", phase="zero",
                              duration=None, seed=3, t0=None, log=None, video_scale=0.9, fps_scale=1.0)
    src = fdt_source.Source(a, dtp, random.Random(1))
    for tp in lay[:4]:
        for j in range(3):
            msgs, size, trunc, k = src.messages(tp, j, 0)
            assert len(msgs) <= tp.n and all(len(m) <= tp.s for m in msgs) and not trunc
            assert k == j and tp.stripe == vl.stripe_of_layer(tp.layer)   # output j: stripe of frame j


def test_layer_topics_claim_one_frame():
    """dtp.json declares "jitter": 0 for the layer topics: the jitter allowance of the telemetry does not
    add a message to the F m_j messages of the stripes of a step."""
    d = root_dtp()
    for n in LAYERS:
        assert vl.channel_demand(d, n, jitter_s=0.002) == vl.channel_demand(d, n)
    assert vl.channel_demand(d, "imu", jitter_s=0.002) > vl.channel_demand(d, "imu")


# --------------------------------------------------------------------------- item 4


def test_r_il_charges_the_cost_of_d_il():
    d = root_dtp()
    d["epoch"] = 0.0
    dtp = DTP.from_dict(d)
    asm = Assembler(dtp, StepClock(dtp.Ts, 0.0, now=lambda: 0.0))
    l = dtp.index("imu")
    # 5 imu messages of the bound (n = ceil(32 (Ts + 2 ms)) = 5): r = d of the DTI formula
    asm.on_message("boat/b1/demand", encode_demand(3, [0] * dtp.L), at=3 * 0.125 + 0.001)
    for s in range(5):
        asm.on_message("boat/b1/tx/imu", encode_step_header(3, s) + bytes(48), at=3 * 0.125 + 0.05)
    g = asm._finish(3)
    e = g.edge(1, l)
    want = math.ceil(dtp.c_b * 5 * (48 + 6) + dtp.c_m * 5 - 1e-9)
    assert e.n == 5 and e.x == 5 * 48
    assert abs(e.r - want) <= 1 and e.r == round(dtp.message_bytes(l, 240, 5))


DTP2 = {"epoch": 1767225600.0, "Ts": 0.125, "T1": 0.020, "T2": 0.010, "tcp": {"mss": 1448, "h_T": 80},
        "mqtt": {"h_A": 4}, "vessels": [1, 2, 3, 4],
        "topics": [{"name": "state", "rate": 8, "jitter": 0.002, "n": 1, "s": 48, "kind": "stream", "qos": 2},
                   {"name": "video", "rate": 15, "jitter": 0.010, "n": 16, "s": 1400, "kind": "output", "qos": 0}],
        "required": ["state"]}


def test_dtp_v2_without_cost_takes_the_paper_bound():
    """the values that tests/c/test_reg_paper.c checks for fdt_paper.c on the same DTP"""
    d = DTP.from_dict(DTP2)
    assert d.c_b == pytest.approx(1 + 160 / 1448)
    assert publish_header(4, "state", 48) == 22
    assert d.topic_c_m(1) == pytest.approx(d.c_b * 22 + 160 + 12)
    assert d.specs[1].rate == 240 and d.specs[1].bound == 1400
    assert d.message_bytes(1, 48) == pytest.approx(d.c_b * 54 + d.topic_c_m(1))
    # a file with c_b and c_m keeps them
    k = DTP.from_dict(dict(DTP2, c_b=1.06, c_m=180))
    assert k.c_b == 1.06 and k.topic_c_m(2) == 180


# --------------------------------------------------------------------------- item 5


class _FakeClient:
    """paho stand-in: CONNACK at once, SUBACK `delay` seconds later from another thread."""

    def __init__(self, svc, delay):
        self.svc, self.delay = svc, delay
        self.suback_at = None

    def connect(self, host, port, keepalive=30):
        pass

    def loop_start(self):
        self.svc._on_connect(self, None, None, 0)

    def subscribe(self, subs):
        def ack():
            time.sleep(self.delay)
            self.suback_at = time.monotonic()
            self.svc._on_subscribe(self, None, 7, [0] * len(subs))
        threading.Thread(target=ack, daemon=True).start()
        return 0, 7

    def loop_stop(self):
        pass

    def disconnect(self):
        pass


def test_connect_waits_for_the_suback():
    dtp = DTP.from_dict(root_dtp())
    svc = Service(dtp, epoch=0.0)
    svc.client = _FakeClient(svc, 0.3)
    t0 = time.monotonic()
    svc.connect(timeout=5.0)
    assert svc.subscribed.is_set() and svc.client.suback_at is not None
    assert time.monotonic() - t0 >= 0.29
    svc2 = Service(dtp, epoch=0.0)
    svc2.client = _FakeClient(svc2, 10.0)
    with pytest.raises(TimeoutError, match="SUBACK"):
        svc2.connect(timeout=0.3)


def _dta(path, steps=600, seed=4):
    m = FleetSim(M=2, seed=seed)
    with open(path, "w") as f:
        for g in m.graphs(steps):
            f.write(json.dumps(g.to_dict()) + "\n")


def test_outer_loop_runs_in_a_child_process(tmp_path):
    dta = tmp_path / "dta.jsonl"
    _dta(dta)
    dtp = DTP.from_dict(root_dtp())
    svc = Service(dtp, epoch=0.0, outer_dta=str(dta), outer_every=10, outer_window=400, outer_target=0.95,
                  log_path=str(tmp_path / "as.jsonl"))
    svc._outer_tick(0)
    assert svc._outer_busy and svc._outer_pid not in (None, os.getpid())
    end = time.time() + 60
    while svc._outer_busy and time.time() < end:
        time.sleep(0.05)
        svc._outer_poll()
    assert not svc._outer_busy
    svc._outer_tick(1)                                     # the next step applies the result
    want = outer.outer_loop(outer.load_dta(str(dta)), target=0.95, window=400)
    assert svc.core.alpha == pytest.approx(want["alpha_star"], rel=1e-12)
    assert svc.core.U == want["U_star"] or (want["U_star"] is None and svc.core.U == dtp.U)
    rec = [json.loads(l) for l in open(tmp_path / "as.jsonl")]
    assert rec and rec[-1]["event"] == "outer" and rec[-1]["t"] == 1
    # a DTA that cannot be read: a warning, no change, the next run can start
    svc.outer_dta = str(tmp_path / "missing.jsonl")
    svc._outer_tick(10)
    while svc._outer_busy and time.time() < end:
        time.sleep(0.05)
        svc._outer_poll()
    a0 = svc.core.alpha
    svc._outer_tick(11)
    assert svc.core.alpha == a0 and not svc._outer_busy
    svc._outer_tick(20)
    svc.stop()                                             # stops a running child
    assert svc._outer_pid is None


# --------------------------------------------------------------------------- items 6 and 7


def test_documented_analysis_commands_name_the_criterion():
    for doc in ("README.md", "RUNBOOK.md"):
        lines = [l for l in (ROOT / doc).read_text().splitlines() if "-m fleetdt.analysis" in l]
        assert lines and all("--criterion" in l for l in lines), doc


def test_gs_wording_of_repeated_and_late_messages():
    src = (ROOT / "fleetdt" / "model.py").read_text()
    assert "discard" not in src and "recorded in the log of the operation and excluded from the step" in src
