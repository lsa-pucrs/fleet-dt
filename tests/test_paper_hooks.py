"""Hooks of the paper stack in the emulator harness (emu/fleet.py, emu/run.py, emu/experiments/runner.py):
the vessel sampler of SIOCOUTQ, the phase lag and lag abort of the emulator with the exit of an aborted
emulator, the copy of the emulator summary, the E1 clock schedules, the CPU profile of the vessel VMs, the
subset replays of E2, the regulator step log, the pcap of the AP, the video check of the layered source and
the post-processing of paper-stack runs (emu/analysis/paper_post.py)."""
import base64
import json
import os
import re
import sys
import types

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "emu"))
sys.path.insert(0, os.path.join(ROOT, "emu", "experiments"))
sys.path.insert(0, os.path.join(ROOT, "emu", "analysis"))
sys.path.insert(0, os.path.join(ROOT, "tests"))

import fleet  # noqa: E402
import run as runpy_  # noqa: E402
from test_fleet_paper import fake_session  # noqa: E402


def spec(tmp_path, **kw):
    s = {"kind": "fleet", "name": "hooks", "M": 2, "Ts": 0.125, "seed": 5, "steps": 8, "lead_s": 2.5,
         "dtp": "dtp.json", "scenario": "emu/scenarios/S1-m2.json", "sync": True, "run_dir": str(tmp_path / "run"),
         "as": {"U": 0.8}, "monitor": {"backend": "py"}}
    s.update(kw)
    return s


def session(tmp_path, summary=None, **st):
    R, calls = fake_session(tmp_path)
    state = R.load_state()
    state.update(st)
    if not state.get("vm_pids"):
        state["vm_pids"] = {"gs": 999991, "v1": 999992, "v2": 999993}

    def start_emulator(scen, outdir, M, **kw):
        calls.append(("emu", kw))
        os.makedirs(outdir, exist_ok=True)
        if summary is not None:
            with open(os.path.join(outdir, "summary.json"), "w") as f:
                json.dump(summary, f)
        return 999
    R.start_emulator = start_emulator
    ssh0 = R.ssh

    def ssh(h, cmd, timeout=None, **kw):
        r = ssh0(h, cmd, timeout, **kw)
        if fleet.CLOCK_SCHEDULE in cmd and "tee" in cmd:
            r.stdout = "installed\n"
        return r
    R.ssh = ssh
    return R, calls


def run(R, s):
    return fleet.run_fleet(R, s, types.SimpleNamespace(stack=None))


def ssh_cmds(calls, h=None):
    return [c[2] for c in calls if c[0] == "ssh" and (h is None or c[1] == h)]


# --------------------------------------------------------------------------- sampler, step log, emulator options

def test_vessel_sampler_and_step_log(tmp_path):
    R, calls = session(tmp_path)
    res = run(R, spec(tmp_path))
    assert res["status"] == "ok", res.get("error")
    prep = next(c for c in ssh_cmds(calls, 1) if "echo prepared" in c)
    assert "cc -O2 -Wall -o build/fdt_outq emu/tools/outq_sampler.c -lm" in prep
    assert "build/fdt_outq > /tmp/fdt-run/build.sha256" in prep
    assert "plugin_opt_step_log /tmp/fdt-run/regsteps.jsonl" in prep
    started = [c for c in ssh_cmds(calls) if "setsid nohup" in c]
    outq = [c for c in started if "/outq.log" in c]
    assert len(outq) == 2
    m = re.search(r"build/fdt_outq --epoch (\S+) --ts-us 125000 --t1-us 20000 --t2-us 10000 --port 1884 --port 1883 "
                  r"--out /tmp/fdt-run/outq.jsonl --start-step (\d+) --steps 8", outq[0])
    assert m and int(m.group(2)) == res["S0"] and float(m.group(1)) == fleet.paper_epoch(json.load(open(
        os.path.join(ROOT, "dtp.json"))))
    # after the bridges, before the sources
    i_wait = next(i for i, c in enumerate(calls) if c[0] == "ssh" and "wait_bridges.py" in c[2])
    i_outq = next(i for i, c in enumerate(calls) if c[0] == "ssh" and "/outq.log" in c[2])
    i_src = next(i for i, c in enumerate(calls) if c[0] == "ssh" and "/src.log" in c[2])
    assert i_wait < i_outq < i_src
    kills = [c for c in ssh_cmds(calls) if "kill -TERM" in c]
    assert any("outq.pid" in c for c in kills)
    assert res["commands"]["vessel_1"]["outq"].startswith("cd ~/fleet-dt && exec build/fdt_outq")


def test_b2_samples_its_three_data_bridges():
    c = fleet.paper_outq_cmd({"dtp": {"Ts": 0.125, "T1": 0.02, "T2": 0.01, "epoch": None}}, 100, 50, "b2")
    assert "--port 1884 --port 1885 --port 1886 --port 1883" in c


def test_emulator_options(tmp_path):
    R, calls = session(tmp_path)
    res = run(R, spec(tmp_path, pcap=True))
    extra = next(c[1]["extra"] for c in calls if c[0] == "emu")
    assert "--phaseLag=125000:20000:10000" in extra and "--abortLagUs=1000" in extra
    assert "--stepBytes=125000:30000" in extra and "--pcap=1" in extra
    assert res["abort_lag_us"] == 1000 and res["pcap"] is True
    R, calls = session(tmp_path)
    run(R, spec(tmp_path, abort_lag_us=0, emu_args=["--catchupUs=500"]))
    extra = next(c[1]["extra"] for c in calls if c[0] == "emu")
    assert "--abortLagUs=0" in extra and extra[-1] == "--catchupUs=500" and "--pcap=1" not in extra
    R, _ = session(tmp_path)
    with pytest.raises(SystemExit, match="sets these options"):
        run(R, spec(tmp_path, emu_args=["--phaseLag=1:2:3"]))


def test_summary_fields_are_copied(tmp_path):
    summ = {"lag_us": {"p99": 50}, "phase_lag": {"on": True, "aborted": False, "max_us": 300},
            "qdisc": {"v1": {"sent": 5}}, "conf": {"gs": {"basic0": "x"}}, "pcap": {"on": False}, "other": 1}
    R, calls = session(tmp_path, summary=summ)
    res = run(R, spec(tmp_path))
    es = res["emu_summary"]
    assert es["phase_lag"] == summ["phase_lag"] and es["qdisc"] == summ["qdisc"] and es["conf"] == summ["conf"]
    assert "other" not in es and res["status"] == "ok"


def test_lag_abort_fails_the_run(tmp_path):
    summ = {"lag_us": {"p99": 50}, "phase_lag": {"on": True, "aborted": True, "abort_step": 3}}
    R, calls = session(tmp_path, summary=summ)
    R.emulator_status = lambda pid: 3 if pid == 999 else None
    res = run(R, spec(tmp_path))
    assert res["status"] == "failed" and res["emu_exit"] == 3 and "status 3" in res["error"]
    # an emulator that reports the abort only in its summary also fails the run
    R, calls = session(tmp_path, summary=summ)
    res = run(R, spec(tmp_path))
    assert res["status"] == "failed" and "phase_lag.aborted" in res["error"]


def test_run_py_keeps_the_exit_status_of_the_emulator(monkeypatch):
    class P:
        pid = 4321

        def __init__(self):
            self.rc = None

        def poll(self):
            return self.rc
    p = P()
    monkeypatch.setitem(runpy_._EMU_PROCS, p.pid, p)
    assert runpy_.emulator_status(4321) is None
    p.rc = 3
    assert runpy_.emulator_status(4321) == 3
    assert runpy_.emulator_status(1) is None


# --------------------------------------------------------------------------- E1 clock

def test_clock_inject_needs_the_session_option(tmp_path):
    R, _ = session(tmp_path)
    with pytest.raises(SystemExit, match="--clock-inject"):
        run(R, spec(tmp_path, clock={"inject": True}))
    R, _ = session(tmp_path, clock_inject=True)
    with pytest.raises(SystemExit, match="unknown keys"):
        run(R, spec(tmp_path, clock={"inject": True, "ramp": 3}))


def test_clock_schedules_installed_collected_and_removed(tmp_path):
    clocks = {"0": {"offset_ms": 0, "ppm": 0}, "1": {"offset_ms": 3, "ppm": 12.5}, "2": {"offset_ms": -4, "ppm": -7.0}}
    R, calls = session(tmp_path, clock_inject=True, clocks=clocks)
    res = run(R, spec(tmp_path, clock={"inject": True, "ramp_ppm": 10, "loss_n": 1, "loss_s": [1, 2], "warm_s": 0}))
    assert res["status"] == "ok", res.get("error")
    import fdt_clock_schedule as CS
    for h, ppm in ((1, 12.5), (2, -7.0)):
        inst = next(c for c in ssh_cmds(calls, h) if fleet.CLOCK_SCHEDULE in c and "tee" in c)
        sc = json.loads(base64.b64decode(re.search(r"echo (\S+) \| base64 -d", inst).group(1)))
        assert sc["ppm0"] == ppm and sc["t0_unix"] == res["t0"] and sc["params"]["ramp_ppm"] == 10
        assert sc == json.load(open(tmp_path / "run" / ("clock_schedule_v%d.json" % h)))
        assert CS.Schedule(sc).ppm(0.0) == ppm and len(sc["ntp_loss"]) == 1
        coll = [c for c in ssh_cmds(calls, h) if fleet.CLOCK_LOG in c]
        assert coll and "clockerr.jsonl" in coll[0] and "sudo rm -f %s" % fleet.CLOCK_SCHEDULE in coll[0]
        # installed after S0 is known, before the emulator of the run starts
        i_inst = next(i for i, c in enumerate(calls) if c[0] == "ssh" and c[1] == h and "tee" in c[2])
        assert i_inst < next(i for i, c in enumerate(calls) if c[0] == "emu")
    assert not [c for c in ssh_cmds(calls, 0) if fleet.CLOCK_SCHEDULE in c]
    assert res["clock"]["inject"] and set(res["clock"]["sha256"]) == {"v1", "v2"}


def test_emu_host_json_names_the_schedule():
    h = runpy_.emu_host_json(1, 2, {"offset_ms": 1, "ppm": 3}, clock_inject=True)
    assert h["clock"] == {"offset_ms": 1, "ppm": 3, "schedule": runpy_.CLOCK_SCHEDULE}
    assert "schedule" not in runpy_.emu_host_json(0, 2, {"offset_ms": 0, "ppm": 0}, clock_inject=True)["clock"]
    assert "schedule" not in runpy_.emu_host_json(1, 2, {"offset_ms": 1, "ppm": 3})["clock"]


# --------------------------------------------------------------------------- CPU profile

def test_qemu_command_and_cpu_profile(tmp_path):
    cmd = runpy_.qemu_cmd(1, "fdt-emu-v1", "/d.qcow2", "/s.iso", "hostfwd=x", "/q.pid")
    assert cmd[cmd.index("-cpu") + 1] == "host,+invtsc"
    v = runpy_.vm_launch_cmd(1, cmd, 0.25)
    assert v[:3] == ["taskset", "-c", runpy_.VM_CPUS] and "systemd-run" in v and "CPUQuota=25%" in v
    assert v[v.index("--") + 1:] == cmd
    assert runpy_.vm_launch_cmd(0, cmd, 0.25) == runpy_.pinned(cmd, runpy_.VM_CPUS)   # the GS keeps its CPU
    assert runpy_.vm_launch_cmd(1, cmd, None) == runpy_.pinned(cmd, runpy_.VM_CPUS)   # off by default


def test_cpu_status_before_and_after_the_run(tmp_path, monkeypatch):
    sys.path.insert(0, os.path.join(ROOT, "emu", "vm"))
    import cpu_profile
    seen = []
    monkeypatch.setattr(cpu_profile, "status", lambda pid: seen.append(pid) or {"pid": pid, "cpu.stat": {"nr_throttled": len(seen)}})
    R, calls = session(tmp_path, cpu_ratio=0.25)
    res = run(R, spec(tmp_path))
    cp = res["cpu_profile"]
    assert cp["ratio"] == 0.25 and set(cp["before"]) == {"gs", "v1", "v2"} and set(cp["after"]) == {"gs", "v1", "v2"}
    assert cp["before"]["v1"]["cpu.stat"]["nr_throttled"] < cp["after"]["v1"]["cpu.stat"]["nr_throttled"]
    saved = json.load(open(tmp_path / "run" / "result.json"))
    assert saved["cpu_profile"]["after"]["v2"]["pid"] == 999993


# --------------------------------------------------------------------------- E2 replays, video check

def test_replay_subsets_after_the_run(tmp_path, monkeypatch):
    seen = []
    monkeypatch.setattr(fleet, "run_replay_subsets", lambda R, rd, jobs=4: seen.append((rd, jobs)) or {"rc": 0})
    R, _ = session(tmp_path)
    res = run(R, spec(tmp_path))
    assert not seen and "replay_subsets" not in res
    R, _ = session(tmp_path)
    res = run(R, spec(tmp_path, replay_subsets=True))
    assert seen == [(str(tmp_path / "run"), 4)] and res["replay_subsets"] == {"rc": 0}


def test_replay_command(tmp_path, monkeypatch):
    got = {}

    class Res:
        returncode = 0
    monkeypatch.setattr(fleet.subprocess, "run", lambda cmd, **kw: got.setdefault("cmd", cmd) and Res())
    R = types.SimpleNamespace(sys=sys, ROOT=ROOT, pinned=runpy_.pinned, AUX_CPUS="16-23")
    r = fleet.run_replay_subsets(R, str(tmp_path), 4)
    assert got["cmd"][:3] == ["taskset", "-c", "16-23"]
    assert got["cmd"][-3:] == [str(tmp_path), "--jobs", "4"]
    assert os.path.join("emu", "replay_subsets.py") in " ".join(got["cmd"]) and r["rc"] == 0


def test_video_check(tmp_path):
    R, _ = session(tmp_path)
    res = run(R, spec(tmp_path))
    v = res["video"]
    assert v["trace"] == fleet.PAPER_TRACE and v["profile"] == v["trace_profile"] == "q90-s1"
    assert not v["truncates"] and 0.85 < v["mean_over_cap"] < 0.95
    R, _ = session(tmp_path)
    with pytest.raises(SystemExit, match="truncate"):
        run(R, spec(tmp_path, source={"kind": "fdt_source", "scale": 1.2}))
    R, _ = session(tmp_path)
    with pytest.raises(SystemExit, match="profile"):
        run(R, spec(tmp_path, source={"kind": "fdt_source", "trace": "emu/traces/water-sbs-q50-s0p5.txt"}))
    R, _ = session(tmp_path)
    res = run(R, spec(tmp_path, source={"kind": "fdt_source", "scale": 1.2, "allow_trunc": True}))
    assert res["video"]["truncates"]


# --------------------------------------------------------------------------- post-processing

def test_runner_routes_paper_runs_to_paper_post():
    import runner
    cmds = runner.postprocess_cmds("/r", "paper")
    assert len(cmds) == 1 and cmds[0][1].endswith(os.path.join("emu", "analysis", "paper_post.py"))
    old = runner.postprocess_cmds("/r", "as2")
    assert [os.path.basename(c[1]) for c in old] == ["gtstep.py", "metrics.py"]


def test_paper_post_on_a_synthetic_run(tmp_path):
    import paper_post
    import test_conformance as TC
    rd = TC.build_run(str(tmp_path / "run"))
    with open(os.path.join(rd, "emu", "summary.json"), "w") as f:
        json.dump({"lag_us": {"p99": 80.0, "max": 1500.0}, "phase_lag": {"aborted": False}}, f)
    with open(os.path.join(rd, "emu", "steplag.csv"), "w") as f:
        f.write("k,max1_us,max2_us,max3_us,lag_p3_us,lag_end_us,catchup,n\n")
        for k in range(TC.STEPS):
            # step 5: lag above 1 ms; step 7: the lag changes by 0.2 ms in phase 3 (catch-up, 0.1 ms rule)
            f.write("%d,%.1f,100.0,100.0,50.0,%.1f,0,700\n" % (k, 1500.0 if k == 5 else 100.0,
                                                              250.0 if k == 7 else 50.0))
    assert paper_post.main([rd, "--warm", "2"]) == 0
    out = json.load(open(os.path.join(rd, "metrics.json")))
    assert out["stack"] == "paper" and out["lag_flagged"] == 1 and out["lag_steps"] == [TC.S0 + 5]
    assert out["catchup_steps"] == [TC.S0 + 7] and out["rt_valid"] is False and out["criterion"] == "count"
    assert out["n_steps"] == TC.STEPS - 2                    # flagged steps stay in the metrics
    m = out["metrics"]
    assert m["steps"] == out["n_steps"] and 0 <= m["gamma"]["value"] <= 1 and m["fill"]["n"] > 0
    assert m["lag_steps"] == 1 and m["catchup_steps"] == 1
    sr = m["series"]
    assert len(sr["t"]) == out["n_steps"] and sr["lag"][3] == 1.0 and sr["catchup"][5] == 1.0
    # the sources of BDR_i come from the regulator step log and the vessel log
    assert out["sources"]["regsteps"] and m["BDR_den"] == "source"
    assert set(out["use"]) <= set(TC.NAMES) and all(0 < v["r_over_d"] <= 1.0 for v in out["use"].values())
    import runner
    bad, info = runner.lag_verdict(rd)        # the runner reads steps_over_1ms of paper_post
    assert info["steps_over_1ms"] == pytest.approx(1 / (TC.STEPS - 2))
    assert paper_post.main([rd, "--warm", "2", "--catchup-us", "300"]) == 0
    assert json.load(open(os.path.join(rd, "metrics.json")))["catchup_steps"] == []
