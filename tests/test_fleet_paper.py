"""Paper stack of the emulator (emu/fleet.py --stack paper): the JMCS stub, the orchestration of a run
with a fake session (commands, order, metadata), and the GS side of the stack on a local broker (AS,
outer loop, VE, JMCS stub, channel monitor adapter, v1 DTI programs)."""
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
import types

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "emu"))

import fleet  # noqa: E402
import jmcs_stub  # noqa: E402
from fleetdt.as_model import DTP  # noqa: E402
from fleetdt.as_service import AS  # noqa: E402

# the DTP of the paper with one video topic (dtp.json before the layered video); the tests below do not
# depend on the current dtp.json, except test_jmcs_current_dtp
PAPER_DTP = {
    "epoch": None, "Ts": 0.125, "T1": 0.020, "T2": 0.010, "alpha": 0.125, "U": 0.8, "C0": 300000,
    "c_b": 1.06, "c_m": 180, "R_PHY": 24000000, "h_W": 150, "vessels": [1, 2],
    "topics": [{"name": "state", "rate": 8, "bound": 48}, {"name": "imu", "rate": 32, "bound": 48},
               {"name": "gps", "rate": 8, "bound": 80}, {"name": "baro", "rate": 8, "bound": 32},
               {"name": "power", "rate": 8, "bound": 40}, {"name": "mag", "rate": 16, "bound": 40},
               {"name": "video", "rate": 30, "bound": 14000}],
    "order": ["state", "imu", "gps", "video", "baro", "power", "mag"],
    "needed": ["state", "imu", "gps"]}


# ------------------------------------------------------------------ JMCS stub
def test_jmcs_modes_follow_the_design():
    m = jmcs_stub.build_modes(PAPER_DTP)
    assert m["survey"]["needed"] == ["state", "imu", "gps", "video"]
    assert m["survey"]["order"] == ["state", "imu", "gps", "video", "baro", "power", "mag"]
    assert m["transit"]["needed"] == ["state", "imu", "gps"]
    assert m["transit"]["order"] == ["state", "imu", "gps", "baro", "power", "mag", "video"]
    core = AS(DTP.from_dict(PAPER_DTP), epoch=0.0, now=lambda: 0.0)
    for mode in ("survey", "transit"):
        o = json.loads(jmcs_stub.payload(5, mode, m))
        assert core.on_order(o["order"], o["needed"])


def test_jmcs_current_dtp():
    """dtp.json of the repository: in survey mode every video topic is needed, in transit none."""
    d = json.load(open(os.path.join(ROOT, "dtp.json")))
    m = jmcs_stub.build_modes(d)
    names = [t["name"] for t in d["topics"]]
    video = [n for n in names if n.startswith("video")]
    assert video
    assert set(video) <= set(m["survey"]["needed"]) and not set(video) & set(m["transit"]["needed"])
    core = AS(DTP.from_dict(d), epoch=0.0, now=lambda: 0.0)
    for mode in ("survey", "transit"):
        assert sorted(m[mode]["order"]) == sorted(names)
        assert m[mode]["order"][:len(m[mode]["needed"])] == m[mode]["needed"]
        assert core.on_order(m[mode]["order"], m[mode]["needed"])


def test_jmcs_layered_video():
    d = dict(PAPER_DTP, topics=PAPER_DTP["topics"][:6] + [{"name": "video%d" % k, "rate": 30, "bound": 900}
                                                           for k in range(16)],
             order=["state", "imu", "gps"] + ["video%d" % k for k in range(16)] + ["baro", "power", "mag"])
    m = jmcs_stub.build_modes(d)
    assert m["survey"]["needed"] == ["state", "imu", "gps"] + ["video%d" % k for k in range(16)]
    assert m["transit"]["order"][-16:] == ["video%d" % k for k in range(16)]
    assert sorted(m["transit"]["order"]) == sorted(t["name"] for t in d["topics"])


def test_jmcs_switches_every_10_s():
    P = int(round(10.0 / 0.125))
    S0 = 1000
    modes = [jmcs_stub.mode_of(t, S0, P) for t in range(S0 - 5, S0 + 4 * P)]
    assert modes[:5] == ["survey"] * 5
    run = modes[5:]
    assert run[:P] == ["survey"] * P and run[P:2 * P] == ["transit"] * P and run[2 * P:3 * P] == ["survey"] * P
    assert jmcs_stub.mode_of(S0, S0, P, first="transit") == "transit"


# ------------------------------------------------------------------ orchestration with a fake session
class FakeR(types.SimpleNamespace):
    pass


def fake_session(tmp_path, M=2):
    calls = []
    state = {"M": M, "scenario": "idle.json", "clocks": {}, "procs": [], "vm_pids": {}}

    class Res:
        def __init__(self, out="", rc=0):
            self.stdout, self.returncode = out, rc

    def ssh(h, cmd, timeout=None, **kw):
        calls.append(("ssh", h, cmd))
        if "guest-patches.sha256" in cmd and cmd.startswith("cat"):
            return Res("\n".join("%s  %s" % (v, k) for k, v in fleet.repo_patches(ROOT).items()))
        if "kill -0" in cmd:
            return Res("alive")
        return Res("prepared\n" if "echo prepared" in cmd else "ok")

    def parallel(fn, items):
        return {x: fn(x) for x in items}

    R = FakeR(SLOT=0, EMU=str(tmp_path), ROOT=ROOT, SESS=str(tmp_path / "sess"), MQTT_FWD=2883, CNT_PORT=9650,
              sys=sys, load_state=lambda: state, save_state=lambda st: None,
              cmd_sync=lambda a: calls.append(("sync",)), parallel=parallel, ssh=ssh,
              host_name=lambda h: "gs" if h == 0 else "v%d" % h, host_ip=lambda h: "10.10.0.%d" % (1 if h == 0 else 10 + h),
              rsync_to=lambda h, s, d, *a: calls.append(("rsync_to", h, s, d)),
              rsync_from=lambda h, s, d: (calls.append(("rsync_from", h)), Res())[1],
              start_daemon=lambda cmd, logp: (calls.append(("daemon", cmd)), 4242 + len(calls))[1],
              kill_pid=lambda pid, marker, *a, **k: calls.append(("kill", marker)),
              stop_emulator=lambda pid: calls.append(("stop_emu",)),
              start_emulator=lambda scen, outdir, M, **kw: (calls.append(("emu", kw)), 999)[1],
              cpu_snapshot=lambda st: {"t": time.time()}, cpu_report=lambda a, b: {},
              log=lambda *a: None, slice_csv=lambda *a: None, slice_jsonl=lambda *a: None)
    return R, calls


def paper_spec(**kw):
    s = {"kind": "fleet", "name": "smoke", "M": 2, "Ts": 0.125, "seed": 7, "steps": 8, "lead_s": 2.5,
         "dtp": PAPER_DTP, "scenario": "emu/scenarios/S1-m2.json", "sync": True,
         "as": {"U": 0.8}, "outer": {"every": 480, "window": 480, "target": 0.99},
         "monitor": {"backend": "py", "jitter_us": 200, "sigma": 0.02}}
    s.update(kw)
    return s


def test_paper_run_orchestration(tmp_path):
    R, calls = fake_session(tmp_path)
    spec = paper_spec(run_dir=str(tmp_path / "run"))
    res = fleet.run_fleet(R, spec, types.SimpleNamespace(stack=None))
    assert res["status"] == "ok", res.get("error")
    assert res["stack"] == "paper"
    rev = res["revision"]
    head = subprocess.run(["git", "-C", ROOT, "rev-parse", "HEAD"], stdout=subprocess.PIPE, text=True).stdout.strip()
    if head:
        assert rev["commit"] == head
    assert "fleetdt/as_service.py" in rev["stack_files"] and "vessel/dti/fdt_dti.c" in rev["stack_files"]
    saved = json.load(open(tmp_path / "run" / "result.json"))
    assert saved["revision"]["commit"] == rev["commit"] and saved["S0"] == res["S0"]
    ssh = [c for c in calls if c[0] == "ssh"]
    prep_v = next(c[2] for c in ssh if c[1] == 1 and "echo prepared" in c[2])
    assert "--mode paper" in prep_v and "plugin_opt_log /tmp/fdt-run/regulator.jsonl" in prep_v
    assert "build/fdt_dti" in prep_v and "plugin_opt_demand external" in prep_v
    started = [c[2] for c in ssh if "setsid nohup" in c[2]]
    joined = "\n".join(started)
    for needle in ("fleetdt.as_service", "--outer-dta /tmp/fdt-run/dta.jsonl", "--outer-every 480",
                   "fleetdt.ve live", "emu/jmcs_stub.py", "--s0 %d" % res["S0"], "build/fdt_dti",
                   "vessel/publishers/fdt_source.py", "--size-trace " + fleet.PAPER_TRACE):
        assert needle in joined, needle
    # the layered video publisher is the default source; the vessels read their own DTP
    assert "raw_load.py" not in joined and "--dtp /tmp/fdt-run/dtp_vessel.json" in joined
    assert "fleetdt.as2" not in joined
    # the AS starts after the VE and the JMCS stub; DTI and sources on both vessels
    order = [n for c in started for n in ("ve.log", "jmcs.log", "as.log") if "/%s " % n in c]
    assert order == ["ve.log", "jmcs.log", "as.log"]
    assert sum("dti.log" in c for c in started) == 2 and sum("src.log" in c for c in started) == 2
    # channel monitor: adapter before the emulator, counter export on the emulator
    kinds = [c[0] for c in calls]
    i_ad = next(i for i, c in enumerate(calls) if c[0] == "daemon" and "fleetdt.monitor_adapter" in c[1])
    i_emu = kinds.index("emu")
    assert i_ad < i_emu
    ad = calls[i_ad][1]
    assert "--udp" in ad and "127.0.0.1:9650" in ad and "--sigma" in ad and ad[ad.index("--sigma") + 1] == "0.02"
    extra = calls[i_emu][1]["extra"]
    assert extra[0] == "--cnt=125000:500:200" and "--cntDst=127.0.0.1:9650" in extra
    assert calls[i_emu][1]["run_start_us"] == int(round(fleet.paper_start(PAPER_DTP, res["S0"]) * 1e6))
    assert ("kill", "monitor_adapter") in calls


def test_stack_option(tmp_path, monkeypatch):
    seen = []
    monkeypatch.setattr(fleet, "run_fleet_as2", lambda R, s, a: seen.append("as2") or {"status": "ok"})
    monkeypatch.setattr(fleet, "run_fleet_paper", lambda R, s, a: seen.append("paper") or {"status": "ok"})
    fleet.run_fleet(None, {}, types.SimpleNamespace(stack=None))
    fleet.run_fleet(None, {"stack": "as2"}, types.SimpleNamespace(stack=None))
    fleet.run_fleet(None, {"stack": "as2"}, types.SimpleNamespace(stack="paper"))
    fleet.run_fleet(None, {}, types.SimpleNamespace(stack="as2"))
    assert seen == ["paper", "as2", "paper", "as2"]


def test_paper_rejects_older_dtp(tmp_path):
    R, _ = fake_session(tmp_path)
    v2 = {"Ts": 0.125, "vessels": [1, 2], "topics": [{"name": "state", "rate": 8, "s": 48, "kind": "stream"}]}
    with pytest.raises(SystemExit, match="--stack as2"):
        fleet.run_fleet(R, paper_spec(dtp=v2), types.SimpleNamespace(stack=None))


def test_pinned_commit(tmp_path):
    R, _ = fake_session(tmp_path)
    with pytest.raises(SystemExit, match="stack_commit"):
        fleet.run_fleet(R, paper_spec(stack_commit="0000000"), types.SimpleNamespace(stack=None))


def test_paper_step_clock():
    assert fleet.paper_start(PAPER_DTP, 0) == fleet.DEFAULT_EPOCH
    t = fleet.paper_step_of(PAPER_DTP, fleet.DEFAULT_EPOCH + 10 * 0.125 + 0.001)
    assert t == 10
    core = AS(DTP.from_dict(PAPER_DTP), epoch=fleet.DEFAULT_EPOCH)
    assert core.start(t) == fleet.paper_start(PAPER_DTP, t)


# ------------------------------------------------------------------ GS side on a local broker
def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _mosq_prefix():
    for p in (os.environ.get("MOSQ"), "/opt/mosquitto-2.1", "/opt/mosquitto-2.1.2-fleetdt"):
        if p and os.path.exists(os.path.join(p, "include", "mosquitto.h")):
            return p
    return None


def _build_dti(tmp):
    mq = _mosq_prefix()
    if not mq or not shutil.which("cc"):
        return None
    out = os.path.join(tmp, "fdt_dti")
    r = subprocess.run(["cc", "-O2", "-std=gnu11", "-pthread", "-I" + os.path.join(ROOT, "common"),
                        "-I" + os.path.join(mq, "include"), "-o", out, os.path.join(ROOT, "vessel", "dti", "fdt_dti.c"),
                        os.path.join(ROOT, "common", "fdt_paper.c"), os.path.join(ROOT, "common", "fdt_common.c"),
                        "-L" + os.path.join(mq, "lib"),
                        "-Wl,-rpath," + os.path.join(mq, "lib"), "-lmosquitto", "-lcjson", "-lm"],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    return out if r.returncode == 0 else None


def test_gs_stack_on_local_broker(tmp_path):
    """AS + VE + JMCS stub + adapter (+ two v1 DTI programs) on one broker: the AS smooths the y(t) of the
    adapter by eq. (2), admits from the demand of the DTIs and follows the orders of the stub."""
    mosq = shutil.which("mosquitto") or ("/usr/sbin/mosquitto" if os.path.exists("/usr/sbin/mosquitto") else None)
    if not mosq:
        pytest.skip("no mosquitto broker")
    pytest.importorskip("paho.mqtt.client")
    pytest.importorskip("aiohttp")
    dti = _build_dti(str(tmp_path))
    if not dti:
        pytest.skip("fdt_dti does not build here (libmosquitto headers)")
    port = _free_port()
    (tmp_path / "m.conf").write_text("listener %d 127.0.0.1\nallow_anonymous true\nset_tcp_nodelay true\n"
                                     "max_inflight_messages 0\n" % port)
    dtp = dict(PAPER_DTP, epoch=None)
    dtp_path = tmp_path / "dtp.json"
    dtp_path.write_text(json.dumps(dtp))
    epoch = fleet.paper_epoch(dtp)
    Ts = dtp["Ts"]
    S0 = fleet.paper_step_of(dtp, time.time() + 1.0) + 1
    env = dict(os.environ, PYTHONPATH=ROOT)
    br = "127.0.0.1:%d" % port
    procs = []

    def start(cmd, name):
        f = open(tmp_path / (name + ".log"), "w")
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, cwd=ROOT,
                             stdin=subprocess.PIPE if name == "adapter" else subprocess.DEVNULL, text=True)
        procs.append(p)
        return p
    start([mosq, "-c", str(tmp_path / "m.conf")], "broker")
    try:
        time.sleep(0.4)
        start([sys.executable, "-m", "fleetdt.ve", "live", "--broker", br, "--dtp", str(dtp_path),
               "--dta", str(tmp_path / "dta.jsonl"), "--epoch", repr(epoch), "--port", str(_free_port())], "ve")
        start([sys.executable, os.path.join(ROOT, "emu", "jmcs_stub.py"), "--dtp", str(dtp_path), "--broker", br,
               "--s0", str(S0), "--period-s", "1.0", "--log", str(tmp_path / "order.jsonl")], "jmcs")
        start([sys.executable, "-m", "fleetdt.as_service", "--dtp", str(dtp_path), "--broker", br,
               "--epoch", repr(epoch), "--log", str(tmp_path / "as.jsonl"), "--alpha", "0.5", "-v"], "as")
        for i in (1, 2):
            start([dti, "--dtp", str(dtp_path), "--vessel", str(i), "--broker", br, "--idle", "0", "--prio", "0"],
                  "dti%d" % i)
        ad = start([sys.executable, "-m", "fleetdt.monitor_adapter", "--dtp", str(dtp_path), "--stdin",
                    "--broker", br, "--backend", "py", "--log", str(tmp_path / "y.jsonl")], "adapter")
        # counter reads at start(t) + 0.5 ms: busy 40 ms per step, 30 kB received in 20 frames
        t = math.floor((time.time() - epoch) / Ts) + 1
        k = 0
        while t < S0 + 24:
            at = epoch + t * Ts + 0.0005
            time.sleep(max(0.0, at - time.time()))
            rec = {"at": float("%.6f" % at), "survey": 1, "time_ms": 125 * k, "busy_ms": 40 * k + (k % 3),
                   "rx_bytes": 30000 * k, "tx_bytes": 0, "rx_packets": 20 * k, "tx_packets": 0, "tx_failed": 0,
                   "stations": 2}
            ad.stdin.write(json.dumps(rec) + "\n")
            ad.stdin.flush()
            t += 1
            k += 1
        time.sleep(0.6)
    finally:
        for p in reversed(procs):
            p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
    ys = {json.loads(l)["t"]: json.loads(l)["y"] for l in open(tmp_path / "y.jsonl")}
    recs = [json.loads(l) for l in open(tmp_path / "as.jsonl")]
    by_t = {r["t"]: r for r in recs if "C_hat" in r}
    assert len(ys) >= 15, open(tmp_path / "adapter.log").read()
    checked = 0
    for tt in sorted(by_t):
        if tt - 1 in by_t and tt - 1 in ys:
            want = 0.5 * by_t[tt - 1]["C_hat"] + 0.5 * ys[tt - 1]          # eq. (2) with alpha = 0.5
            assert by_t[tt]["C_hat"] == pytest.approx(want, rel=1e-9)
            checked += 1
    assert checked >= 10
    run = [by_t[tt] for tt in sorted(by_t) if tt >= S0 + 2]
    assert run and all(r["m"] == 2 and r["published"] for r in run), run[:3]
    modes = {r["t"]: r["needed"] for r in run}
    P = 8
    for tt, needed in modes.items():
        m = jmcs_stub.mode_of(tt, S0, P)
        names = [PAPER_DTP["topics"][l - 1]["name"] for l in needed]
        assert ("video" in names) == (m == "survey"), (tt, m, names)
    dta = [json.loads(l) for l in open(tmp_path / "dta.jsonl")]
    assert any(g.get("y") is not None for g in dta)


def test_example_experiment(tmp_path):
    """emu/experiments/paper-smoke-m2.json (DTP and scenario given as repository paths)."""
    R, calls = fake_session(tmp_path)
    spec = json.load(open(os.path.join(ROOT, "emu", "experiments", "paper-smoke-m2.json")))
    spec.update(steps=8, lead_s=2.5, run_dir=str(tmp_path / "run"), monitor=dict(spec["monitor"], backend="py"))
    res = fleet.run_fleet(R, spec, types.SimpleNamespace(stack=None))
    assert res["status"] == "ok", res.get("error")
    assert res["plugin_mode"] == "paper" and res["demand"].startswith("external")
    dtp = json.load(open(tmp_path / "run" / "dtp.json"))
    assert dtp["vessels"] == [1, 2] and len(dtp["topics"]) == len(json.load(open(os.path.join(ROOT, "dtp.json")))["topics"])
