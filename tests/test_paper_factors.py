"""Factors of the experiments of the paper (Section VI-B and VI-C, Table S6): the E0 outage, the R7
rate adaptation and the E5 regime sequence of emu/scenarios/gen.py, channel 6 of the paper scenarios,
the E4 overload and the vessel without video (vessel/publishers/fdt_source.py, video_layers.py), the R8
and R9 settings of the brokers (fleetdt/gsconf.py) and of the guest routes (emu/vm/guest/fdt-netsetup.py),
the spec fields of emu/fleet.py (outage, ablation, vessels, overload, brokers, guest_tcp, catchup_us,
session, the source jitter), the sessions of emu/experiments/runner.py and the Q specs. No emulator, VM or
broker runs here."""
import importlib.util
import json
import math
import os
import subprocess
import sys
import types

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in ("emu/scenarios", "emu/experiments", "emu", "vessel/publishers", "tests", ""):
    sys.path.insert(0, os.path.join(ROOT, p))

import fdt_source  # noqa: E402
import fleet  # noqa: E402
import gen  # noqa: E402
import qualify  # noqa: E402
import runner  # noqa: E402
import video_layers as vl  # noqa: E402
from fleetdt.gsconf import gs_conf, vessel_conf  # noqa: E402
from test_fleet_paper import fake_session  # noqa: E402

LAYERS = ["video_%d" % j for j in range(1, 17)]


def root_dtp(M=4):
    d = json.load(open(os.path.join(ROOT, "dtp.json")))
    d["vessels"] = list(range(1, M + 1))
    return d


def netsetup():
    spec = importlib.util.spec_from_file_location("fdt_netsetup", os.path.join(ROOT, "emu", "vm", "guest",
                                                                               "fdt-netsetup.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ------------------------------------------------------------------------------------------------ gen.py

def test_paper_scenarios_on_channel_6():
    assert gen.channel_hz(1) == 2.412e9 and gen.channel_hz(6) == 2.437e9
    with pytest.raises(ValueError):
        gen.channel_hz(14)
    for sid in gen.PAPER_SCENARIOS:
        sc = gen.make_paper(sid, 4, 3)
        assert sc["channel_number"] == 6
        if sid in ("S5", "S7"):
            assert sc["channel"]["frequency_hz"] == 2.437e9
    # the scenarios of the earlier campaign keep the ns-3 default channel
    assert "channel_number" not in gen.make("S5", 2, 1)


@pytest.mark.parametrize("dur", [1, 3, 5])
def test_outage_modifier(dur):
    sc = gen.make_paper("S1", 4, 21)
    o = gen.with_outage(sc, dur)
    assert sc["channel"]["loss"] == [] and "outage" not in sc          # the input is not changed
    lo, hi = gen.PAPER_PARAMS["outage_start_range_s"]
    rec = o["outage"]
    assert rec["target"] == "v1" and rec["peer"] == "gs" and rec["dur_s"] == dur and rec["loss_db"] == 200.0
    assert lo <= rec["start_s"] <= hi and lo >= 30.0                  # after the warm-up of 30 s
    assert o["channel"]["loss"] == [["gs", "v1", 200.0, rec["start_s"], round(rec["start_s"] + dur, 6)]]
    # seeded: the same for the same seed, M and duration; another seed moves it
    assert gen.with_outage(sc, dur)["outage"] == rec
    assert gen.with_outage(gen.make_paper("S1", 4, 22), dur)["outage"]["start_s"] != rec["start_s"]
    # an explicit start and target; the other entries of the scenario stay
    s3 = gen.with_outage(gen.make_paper("S3", 2, 5), dur, start_s=40.0, target="v2")
    assert s3["channel"]["loss"][-1] == ["gs", "v2", 200.0, 40.0, 40.0 + dur]
    assert len(s3["channel"]["loss"]) == 3 + 1                         # jam to gs, v1, v2 and the outage


def test_outage_refusals():
    with pytest.raises(ValueError, match="matrix"):
        gen.with_outage(gen.make_paper("S5", 2, 1), 3)
    with pytest.raises(ValueError):
        gen.with_outage(gen.make_paper("S1", 2, 1), 3, target="v3")
    with pytest.raises(ValueError):
        gen.with_outage(gen.make_paper("S1", 2, 1), 0)


def test_rate_adaptation_modifier():
    sc = gen.make_paper("S3", 4, 9)
    r = gen.with_rate_adaptation(sc)
    assert r["standard"] == "11a" and r["rate"] == {"manager": "minstrel", "non_unicast": "OfdmRate6Mbps"}
    assert r["basic_modes"] == ["OfdmRate6Mbps", "OfdmRate12Mbps", "OfdmRate24Mbps"]
    assert "channel_number" not in r and r["ablation"] == "R7" and r["phy"] == "spectrum"
    assert r["jammer"] == sc["jammer"] and r["channel"] == sc["channel"]
    assert sc["rate"]["manager"] == "constant"                          # the input is not changed
    with pytest.raises(ValueError, match="DSSS"):
        gen.with_rate_adaptation(gen.make_paper("S2", 4, 9))
    with pytest.raises(ValueError, match="matrix"):
        gen.with_rate_adaptation(gen.make_paper("S7", 4, 9))


@pytest.mark.parametrize("M", [2, 4])
def test_regime_sequence(M):
    sc = gen.make_paper_sequence(M, 31)
    assert sc["regimes"] == [{"t0": 0.0, "scenario": "S1"}, {"t0": 130.0, "scenario": "S3"},
                             {"t0": 260.0, "scenario": "S1"}]
    assert sc["phy"] == "spectrum" and sc["scenario_id"] == "SEQ"
    s3 = gen.make_paper("S3", M, 31, 130.0)
    # the injector of S3 at -50 dBm from every node, bursts only inside the S3 regime
    assert sorted(map(tuple, sc["channel"]["loss"])) == sorted(map(tuple, s3["channel"]["loss"]))
    sch = sc["jammer"]["schedule"]
    assert sch and all(130.0 <= a < b <= 260.0 for a, b, *_ in sch)
    assert [[round(a - 130.0, 3), round(b - 130.0, 3)] + x for a, b, *x in sch] == \
        [[a, min(b, 130.0)] + x for a, b, *x in s3["jammer"]["schedule"] if a < 130.0]
    # everything else is S1 of the same seed
    s1 = gen.make_paper("S1", M, 31, 420.0)
    for k in ("rate", "basic_modes", "edca", "mac_queue", "channel_number", "tx_power_dbm", "cca_ed_dbm"):
        assert sc[k] == s1[k], k
    # regime length as a parameter; a sequence without S3 has no injector node
    sc2 = gen.make_paper_sequence(M, 31, ("S1", "S3", "S1"), regime_s=60.0)
    assert [r["t0"] for r in sc2["regimes"]] == [0.0, 60.0, 120.0]
    assert all(60.0 <= a < b <= 120.0 for a, b, *_ in sc2["jammer"]["schedule"])
    sc3 = gen.make_paper_sequence(M, 31, ("S1",))
    assert "jammer" not in sc3 and all("jam" not in e[:2] for e in sc3["channel"]["loss"])
    with pytest.raises(ValueError):
        gen.make_paper_sequence(M, 31, ("S1", "S2"))


# ------------------------------------------------------------------------------------------------ sources

def test_vessel_without_video_declares_no_video_demand():
    d = root_dtp()
    d["video_layers"]["vessels"]["4"] = dict(d["video_layers"]["vessels"]["4"], video=False)
    v4, v3 = vl.resolve_vessel(d, 4), vl.resolve_vessel(d, 3)
    for t in v4["topics"]:
        if t["name"] in LAYERS:
            assert t["rate"] == 0.0 and vl.channel_demand(v4, t["name"]) == 0
        else:
            assert t == next(x for x in v3["topics"] if x["name"] == t["name"])
    assert all(t["rate"] > 0 for t in v3["topics"])
    # the source publishes no layer topic of vessel 4 (rate 0), and the telemetry as before
    tops = {t.name: t for t in (fdt_source.v1_topic(t, v4, 0.002) for t in v4["topics"])}
    assert all(tops[n].rate == 0 for n in LAYERS) and tops["imu"].rate == 32 and tops["imu"].jitter == 0.002
    assert vl.vessel_demand(v4, 0.002) < vl.vessel_demand(v3, 0.002)


def test_overload_steps():
    s = vl.overload_steps(7, 1200, 0.1, 240)
    assert len(s) == 96 and s == sorted(set(s)) and min(s) >= 240 and max(s) < 1200
    assert vl.overload_steps(7, 1200, 0.1, 240) == s and vl.overload_steps(8, 1200, 0.1, 240) != s
    assert vl.overload_steps(7, 100, 0.0, 10) == [] and vl.overload_steps(7, 100, 1.0, 10) == list(range(10, 100))
    with pytest.raises(ValueError):
        vl.overload_steps(7, 100, 1.5, 10)


def source(tmp_path, dtp, vessel=1, basis="cap", factor=1.2, seed=5, t0=None):
    p = tmp_path / ("dtp_b%d.json" % vessel)
    p.write_text(json.dumps(vl.resolve_vessel(dtp, vessel)))
    d = fdt_source.load_dtp(str(p), 0.002)
    t0 = fleet.paper_start(dtp, 1000) if t0 is None else t0
    a = types.SimpleNamespace(seed=seed, vessel=vessel, size_trace=os.path.join(ROOT, fleet.PAPER_TRACE), log=None,
                              median=None, sigma=0.5, load_mode="size", load=1.0, video_scale=1.0, fps_scale=1.0,
                              stream_size="max", t0=t0, phase="random", topics=None, overload_factor=factor,
                              overload_fraction=0.1, overload_seed="11", overload_steps=400,
                              overload_warm_steps=40, overload_basis=basis)
    return fdt_source.Source(a, d, None), d


def test_overload_of_the_video_source(tmp_path):
    dtp = root_dtp()
    src, d = source(tmp_path, dtp)
    S0 = 1000
    over = set(vl.overload_steps("11", 400, 0.1, 40))
    assert src.over["S0"] == S0 and src.over["steps"] == over
    tp = next(t for t in d["topics"] if t.name == "video_3")
    tel = next(t for t in d["topics"] if t.name == "state")
    Ts, T1, T2 = d["Ts"], d["T1"], d["T2"]
    cap = tp.n * (tp.s - 8)

    def at(r):        # a nominal release inside collection step S0 + r: v(t - 1) + Ts / 2
        return d["epoch"] + (S0 + r - 1) * Ts + T1 + T2 + Ts / 2
    r_over, r_norm = min(over), min(set(range(40, 400)) - over)
    assert src.overload(tp, at(r_over)) == 1.2 and src.overload(tp, at(r_norm)) == 1.0
    assert src.overload(tel, at(r_over)) == 1.0                        # only the layer topics
    assert all(src.overload(tp, at(r)) == 1.0 for r in range(0, 40))  # no overload in the warm-up
    msgs, size, trunc, _ = src.messages(tp, 77, 0, 1.2)
    assert size == math.ceil(1.2 * cap) and not trunc and len(msgs) == math.ceil(size / (tp.s - 8))
    # the stripes of an overloaded step exceed the declared demand of the topic by the factor
    vd = json.loads((tmp_path / "dtp_b1.json").read_text())
    F = vl.frames_per_step(float(vd["video_layers"]["fps"]), Ts)
    cost = F * sum(vl.message_cost(vd, len(m), "video_3") for m in msgs)
    dem = vl.channel_demand(vd, "video_3")
    assert 1.15 * dem < cost < 1.25 * dem
    msgs1, size1, trunc1, _ = src.messages(tp, 77, 0, 1.0)
    assert size1 <= cap and sum(vl.message_cost(vd, len(m), "video_3") for m in msgs1) * F <= dem
    # basis size: the replayed size times the factor
    src2, _ = source(tmp_path, dtp, basis="size")
    size2 = src2.messages(tp, 77, 0, 1.2)[1]
    assert size2 == min(math.ceil(1.2 * cap), max(1, int(round(src2.messages(tp, 77, 0, 1.0)[1] * 1.2))))


def test_source_cli_overload_needs_t0():
    r = subprocess.run([sys.executable, os.path.join(ROOT, "vessel", "publishers", "fdt_source.py"), "--dtp",
                        os.path.join(ROOT, "dtp.json"), "--vessel", "1", "--overload-factor", "1.2"],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=60)
    assert r.returncode == 2 and "--t0" in r.stdout


# ------------------------------------------------------------------------------------------------ brokers, routes

def test_gsconf_ablation_settings():
    for c in (gs_conf({1: "10.0.0.1"}), vessel_conf(1, "/d.json")):
        assert "set_tcp_nodelay true" in c and "max_inflight_messages 0" in c
    for c in (gs_conf({1: "10.0.0.1"}, nodelay=False, max_inflight=20),
              vessel_conf(1, "/d.json", nodelay=False, max_inflight=20)):
        assert "set_tcp_nodelay false" in c and "max_inflight_messages 20" in c and "set_tcp_nodelay true" not in c
    out = subprocess.run([sys.executable, "-m", "fleetdt.gsconf", "vessel", "--vessel", "2", "--nagle",
                          "--max-inflight", "20"], cwd=ROOT, stdout=subprocess.PIPE, text=True, check=True).stdout
    assert "set_tcp_nodelay false" in out and "max_inflight_messages 20" in out
    # patch 0002 applies TCP_NODELAY to the bridge sockets only when set_tcp_nodelay is true
    patch = open(os.path.join(ROOT, "mosquitto", "patches", "0002-bridge-apply-set_tcp_nodelay.patch")).read()
    assert "if(!db.config->set_tcp_nodelay" in patch


def test_netsetup_tcp_profiles(monkeypatch):
    ns = netsetup()
    c = {"index": 1, "hosts": {"0": {"ip": "10.10.0.1", "mac": "m0"}, "1": {"ip": "10.10.0.11", "mac": "m1"}},
         "route_opts": "quickack 1 rto_min 20ms", "sysctl": {"net.ipv4.tcp_slow_start_after_idle": "0"}}
    assert ns.route_opts(c) == ["quickack", "1", "rto_min", "20ms"]
    assert ns.route_opts(c, quickack=0) == ["rto_min", "20ms"]
    assert ns.route_opts(c, quickack=1, rto_min_ms=0) == ["quickack", "1"]
    assert ns.sysctls(c, 1) == {"net.ipv4.tcp_slow_start_after_idle": "1"}
    cmds = []
    monkeypatch.setattr(ns, "sh", lambda *a, **k: cmds.append(a) or "")
    monkeypatch.setattr(ns, "iface_by_mac", lambda mac: "ens4")
    ns.tcp_only(c, quickack=0, rto_min_ms=20, slow_start_after_idle=0)
    assert cmds[0] == ("ip", "route", "replace", "10.10.0.0/24", "dev", "ens4", "proto", "kernel", "scope", "link",
                       "src", "10.10.0.11", "rto_min", "20ms")
    assert cmds[1] == ("sysctl", "-q", "-w", "net.ipv4.tcp_slow_start_after_idle=0") and len(cmds) == 2


# ------------------------------------------------------------------------------------------------ fleet.py

def spec(**kw):
    s = {"kind": "fleet", "stack": "paper", "name": "factors", "M": 2, "Ts": 0.125, "seed": 5, "steps": 400,
         "dtp": root_dtp(2), "scenario": gen.make_paper("S3", 2, 5), "as": {"U": 0.8}}
    s.update(kw)
    return s


def test_emulator_catchup_threshold():
    R = types.SimpleNamespace(CNT_PORT=9650)
    s = spec()
    mon = fleet.paper_monitor(s)
    assert "--catchupUs=100" in fleet.paper_emu_args(R, s, mon)
    assert "--catchupUs=250" in fleet.paper_emu_args(R, dict(s, catchup_us=250), mon)
    a = fleet.paper_emu_args(R, dict(s, emu_args=["--catchupUs=500"]), mon)
    assert a[-1] == "--catchupUs=500" and sum(x.startswith("--catchupUs=") for x in a) == 1
    with pytest.raises(SystemExit):
        fleet.paper_emu_args(R, dict(s, catchup_us=250, emu_args=["--catchupUs=500"]), mon)


def test_source_jitter_and_per_vessel_topics():
    s = spec()
    def jit(x):
        c = fleet.paper_source_cmd(x, 1, 0.0, 1.0).split()
        return c[c.index("--jitter-ms") + 1]
    assert jit(s) == "2" and jit(dict(s, plugin_opts={"jitter_ms": 3.5})) == "3.5"
    assert jit(dict(s, source={"jitter_ms": 0})) == "0"
    s2 = dict(s, source={"topics": ["state", "imu"]}, vessels={"2": {"topics": ["state"]}})
    assert "--topics state,imu" in fleet.paper_source_cmd(s2, 1, 0.0, 1.0)
    assert fleet.paper_source_cmd(s2, 2, 0.0, 1.0).endswith("--topics state")
    with pytest.raises(SystemExit):
        fleet.paper_vessels(dict(s, vessels={"2": {"topics": ["nope"]}}), 2)
    with pytest.raises(SystemExit):
        fleet.paper_vessels(dict(s, vessels={"3": {"video": False}}), 2)
    with pytest.raises(SystemExit):
        fleet.paper_vessels(dict(s, vessels={"1": {"colour": 1}}), 2)


def test_overload_arguments_only_for_the_overloaded_vessel():
    s = spec(overload={"vessel": 1})
    o = fleet.paper_overload(s)
    assert o == dict(fleet.PAPER_OVERLOAD, seed=5, warm_steps=240)
    c1, c2 = fleet.paper_source_cmd(s, 1, 0.0, 1.0), fleet.paper_source_cmd(s, 2, 0.0, 1.0)
    assert ("--overload-factor 1.2 --overload-fraction 0.1 --overload-seed 5 --overload-steps 400 "
            "--overload-warm-steps 240 --overload-basis cap") in c1
    assert "--overload" not in c2
    with pytest.raises(SystemExit):
        fleet.paper_overload(spec(overload={"vessel": 1, "fraction": 2}))


def test_ablation_settings():
    assert fleet.paper_brokers(spec()) == {"nodelay": True, "max_inflight": 0}
    assert fleet.paper_brokers(spec(ablation="R8")) == {"nodelay": False, "max_inflight": 0}
    assert fleet.paper_brokers(spec(ablation="R9")) == {"nodelay": True, "max_inflight": 20}
    assert fleet.paper_guest_tcp(spec())["quickack"] and not fleet.paper_guest_tcp(spec(ablation="R8"))["quickack"]
    assert "--quickack 1 --rto-min-ms 20 --slow-start-after-idle 0" in fleet.paper_tcp_cmd(spec())
    assert "--quickack 0 --rto-min-ms 20 --slow-start-after-idle 0" in fleet.paper_tcp_cmd(spec(ablation="R8"))
    assert fleet.paper_broker_opts(spec(ablation="R8")) == " --nagle"
    assert fleet.paper_broker_opts(spec(ablation="R9")) == " --max-inflight 20"
    sc = fleet.paper_scenario(spec(ablation="R7"))
    assert sc["standard"] == "11a" and sc["rate"]["manager"] == "minstrel"
    assert fleet.paper_scenario(spec(ablation="R8")) == spec()["scenario"]
    with pytest.raises(SystemExit):
        fleet.paper_ablation(spec(ablation="R6"))
    with pytest.raises(SystemExit):
        fleet.paper_scenario(spec(ablation="R7", scenario=gen.make_paper("S2", 2, 5)))


def test_outage_field():
    s = spec(scenario=gen.make_paper("S1", 2, 5), outage={"target": "v1", "dur_s": 3})
    sc = fleet.paper_scenario(s)
    assert sc["outage"] == gen.with_outage(gen.make_paper("S1", 2, 5), 3)["outage"]
    # a scenario that carries the same outage stays; another outage is refused
    assert fleet.paper_scenario(dict(s, scenario=sc)) == sc
    with pytest.raises(SystemExit):
        fleet.paper_scenario(dict(s, scenario=sc, outage={"dur_s": 5}))
    with pytest.raises(SystemExit):
        fleet.paper_scenario(dict(s, outage={"target": "v1"}))


def test_session_fields():
    assert fleet.paper_session(spec()) == {"cpu_ratio": 0.25, "clock_inject": True, "probe_period_s": 0.125}
    bare = fleet.paper_session(spec(session=fleet.BARE_SESSION))
    assert bare == {"cpu_ratio": None, "clock_inject": False, "probe_period_s": 0.1}
    st = {"cpu_ratio": 0.25, "clock_inject": True, "probe_period": 0.125}
    assert fleet.check_session(spec(session={}), st)["cpu_ratio"] == 0.25
    assert fleet.check_session(spec(), {}) is None                    # no "session": no check
    with pytest.raises(SystemExit, match="cpu_ratio"):
        fleet.check_session(spec(session={}), dict(st, cpu_ratio=None))
    with pytest.raises(SystemExit, match="probe_period_s"):
        fleet.check_session(spec(session={}), dict(st, probe_period=0.1))
    # the clock injector is on by default in a session of the paper campaign
    assert fleet.paper_clock(spec(session={})) == {"inject": True}
    assert fleet.paper_clock(spec(session={"clock_inject": False})) is None
    assert fleet.paper_clock(spec()) is None


def _b64(cmd, name):
    import base64
    import re
    m = re.search(r"echo (\S+) \| base64 -d > /tmp/fdt-run/%s" % re.escape(name), cmd)
    return json.loads(base64.b64decode(m.group(1)))


def test_run_with_the_factors(tmp_path):
    """an R8 run of the fairness cell with the overload and the session of the campaign, on a fake session"""
    R, calls = fake_session(tmp_path, M=4)
    st = R.load_state()
    st.update(cpu_ratio=0.25, clock_inject=True, probe_period=0.125, vm_pids={})
    s = spec(M=4, dtp=root_dtp(4), scenario=gen.make_paper("S1", 4, 5), ablation="R8",
             overload={"vessel": 1, "warm_s": 0.5, "fraction": 0.25}, vessels=fleet.FAIRNESS_VESSELS,
             outage={"dur_s": 1}, session={}, run_dir=str(tmp_path / "run"), monitor={"backend": "py"}, sync=False,
             lead_s=2.5, steps=16)
    calls_ssh = R.ssh

    def ssh(h, cmd, timeout=None, **kw):
        r = calls_ssh(h, cmd, timeout, **kw)
        if fleet.CLOCK_SCHEDULE in cmd and "tee" in cmd:
            r.stdout = "installed\n"
        return r
    R.ssh = ssh
    res = fleet.run_fleet(R, s, types.SimpleNamespace(stack=None))
    assert res["status"] == "ok", res.get("error")
    ssh_calls = [c for c in calls if c[0] == "ssh"]
    preps = {c[1]: c[2] for c in ssh_calls if "echo prepared" in c[2]}
    assert "--nagle" in preps[0] and all("--nagle" in preps[h] for h in range(1, 5))
    assert all("fdt-netsetup.py --tcp-only --quickack 0" in preps[h] for h in range(5))
    v4, v1 = _b64(preps[4], "dtp_vessel.json"), _b64(preps[1], "dtp_vessel.json")
    assert all(t["rate"] == 0 for t in v4["topics"] if t["name"] in LAYERS)
    assert v4["video_layers"]["vessels"]["4"]["video"] is False
    assert all(_b64(preps[h], "dtp_vessel.json")["video_layers"]["vessels"][str(h)]["size_scale"] == 1.0
               for h in (1, 2, 3))
    assert [t for t in v1["topics"] if t["name"] in LAYERS] == \
        [t for t in _b64(preps[2], "dtp_vessel.json")["topics"] if t["name"] in LAYERS]   # same sources
    src = {c[1]: c[2] for c in ssh_calls if "src.log" in c[2]}
    assert "--overload-factor 1.2" in src[1] and all("--overload" not in src[h] for h in (2, 3, 4))
    rel = vl.overload_steps(5, 16, 0.25, 4)
    assert len(rel) == 3 and res["overload"]["steps"] == [res["S0"] + r for r in rel]
    assert res["ablation"] == "R8" and res["brokers"]["nodelay"] is False and res["guest_tcp"]["quickack"] is False
    assert res["session"] == {"cpu_ratio": 0.25, "clock_inject": True, "probe_period_s": 0.125}
    assert res["clock"]["inject"] is True                              # the injector of the campaign
    sc = json.load(open(tmp_path / "run" / "scenario.json"))
    assert sc["outage"]["dur_s"] == 1.0 and res["outage"] == sc["outage"]
    extra = next(c[1]["extra"] for c in calls if c[0] == "emu")
    assert "--catchupUs=100" in extra


# ------------------------------------------------------------------------------------------------ runner, Q, run.py

def test_runner_sessions():
    s = runner.run_spec(spec(), "/r", "/m")
    assert s["session"] == {"cpu_ratio": 0.25, "clock_inject": True, "probe_period_s": 0.125}
    key = runner.session_key(s)
    assert key == (2, 0.25, True, 0.125)
    up = runner.up_args(key, 100)
    assert up[:2] == ["up", "--scenario"] and up[2].endswith("emu/scenarios/S1-m2.json")
    assert "--cpu-ratio" in up and up[up.index("--cpu-ratio") + 1] == "0.25" and "--clock-inject" in up
    assert up[up.index("--probe-period") + 1] == "0.125"
    bare = runner.session_key(runner.run_spec(spec(session=fleet.BARE_SESSION), "/r", "/m"))
    assert bare == (2, None, False, 0.1)
    ub = runner.up_args(bare, 101)
    assert "--cpu-ratio" not in ub and "--clock-inject" not in ub and ub[ub.index("--probe-period") + 1] == "0.1"
    # the as2 stack runs on the session of run.py up without options
    as2 = runner.run_spec({"name": "x", "M": 4, "Ts": 0.125}, "/r", "/m")
    assert "session" not in as2 and runner.session_key(as2) == (4, None, False, 0.1)


def test_q_specs_carry_the_session(tmp_path):
    man = qualify.write_manifest(str(tmp_path / "a"), steps=40)
    s = json.load(open(tmp_path / "a" / man["runs"][0]["spec"]))
    assert s["session"] == {"cpu_ratio": 0.25, "clock_inject": True, "probe_period_s": 0.125}
    man = qualify.write_manifest(str(tmp_path / "b"), steps=40, bare_session=True)
    s = json.load(open(tmp_path / "b" / man["runs"][0]["spec"]))
    assert s["session"] == fleet.BARE_SESSION


def test_run_py_probe_period_option():
    r = subprocess.run([sys.executable, os.path.join(ROOT, "emu", "run.py"), "up", "--help"],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=60)
    assert r.returncode == 0 and "--probe-period" in r.stdout
