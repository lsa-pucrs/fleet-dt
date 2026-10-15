"""Fleet-DT runs in the emulator (paper Section VI): one run = the whole stack of SPEC_v2.md on the VMs of a
session of emu/run.py, the emulator with the ground truth, and the collection of every log.

A fleet experiment is a JSON object with "kind": "fleet" (emu/experiments/matrix.py writes them):

  {"kind": "fleet", "name": "E1-fleetdt-S1-m4-r1.5-ts125-s1001", "run_dir": "/abs/dir" (optional),
   "exp": "E1", "method": "fleetdt", "predictor": "eq", "scenario_id": "S1", "M": 4, "Ts": 0.125, "rho": 1.5,
   "seed": 1001, "steps": 1200,
   "dtp": {... DTP v2 of the run ...},
   "scenario": {... emulator scenario of the run (seeded) ...},
   "source": {"trace": "emu/traces/zed-sbs-q70.txt", "scale": 1.0},
   "as": {"z0": 1.0, "init_scores_file": "/abs/init/S1-eq.json" (or "init_scores": {"scores": [...], "b_hat": b}),
          "pred_gain": 0.25, "cal_kappa": 0.7},
   "opctl": {"period": 80},
   "lead_s": 15, "sync": true}

Method -> (plugin mode, AS mode): fleetdt, slotted, b1, b2, b3, b5 and tdma use the same name on both
sides; b4 runs the plugin in mode fleetdt and the AS in mode b4 with the constant factor "as.z0"; the
predictors of E2 run Fleet-DT with "predictor" persist, es or calmean.

Timeline of a run: the VMs get the DTP, the broker configurations and the nftables rules (the vessels
build the plugin from the synchronized repository); the run picks its first step S0 at least lead_s
ahead; the emulator restarts with the ground truth and the scenario time 0 at start(S0); the brokers
start (vessel brokers and the GS broker under SCHED_FIFO 50), the bridges connect, then the AS (SCHED_FIFO
40, steps S0 .. S0 + steps, so the record of step S0 + steps closes the last step), the operation
control stub and the sources (seeded release instants relative to start(S0)) start; the run waits until
the AS has run its last step, stops everything, and collects /tmp/fdt-run/ of every VM into
RUN/vms/<host>/, the monitor capture, the emulator outputs (gt.bin, lag.txt, summary.json), the clock
probe samples and the monitor-bridge statistics of the run interval, and writes RUN/result.json.
"""
import base64
import hashlib
import json
import math
import os
import shlex
import shutil
import subprocess
import time

DEFAULT_EPOCH = 1767225600.0
METHODS = {
    "fleetdt": ("fleetdt", "fleetdt"),
    "slotted": ("slotted", "slotted"),
    "b1": ("b1", "b1"),
    "b2": ("b2", "b2"),
    "b3": ("b3", "b3"),
    "b4": ("fleetdt", "b4"),
    "b5": ("b5", "b5"),
    "tdma": ("tdma", "tdma"),
}
RUN = "/tmp/fdt-run"
MONITOR_LAG = 0.0025
_TREE = None


def b64(obj):
    return base64.b64encode(json.dumps(obj, separators=(",", ":")).encode()).decode()


def epoch_of(dtp):
    e = dtp.get("epoch")
    return DEFAULT_EPOCH if e is None else float(e)


def step_start(dtp, t):
    return epoch_of(dtp) + (t - 1) * float(dtp["Ts"])


def step_of(dtp, at):
    return int(math.floor((at - epoch_of(dtp)) / float(dtp["Ts"]))) + 1


def tree_digest(root):
    """sha256 over the relative paths and contents of the repository files (without build products and
    caches): identifies the exact code of a run also when the copy has no .git"""
    global _TREE
    if _TREE is not None:
        return _TREE
    skip = {".git", "build", "build-armhf", "__pycache__", ".pytest_cache", "dta"}
    h = hashlib.sha256()
    n = 0
    for dp, dn, fn in os.walk(root):
        dn[:] = sorted(d for d in dn if d not in skip)
        for f in sorted(fn):
            if f.endswith(".pyc"):
                continue
            p = os.path.join(dp, f)
            rel = os.path.relpath(p, root)
            with open(p, "rb") as fh:
                h.update(rel.encode() + b"\0" + hashlib.sha256(fh.read()).digest())
            n += 1
    _TREE = {"sha256": h.hexdigest(), "files": n}
    return _TREE


def revision(root):
    rev = {}
    try:
        with open(os.path.join(root, ".fdt-revision.json")) as f:
            rev = json.load(f)
    except (OSError, ValueError):
        rev = {"commit": None}
    rev["tree"] = tree_digest(root)
    return rev


def repo_patches(root):
    out = {}
    for p in sorted(os.listdir(os.path.join(root, "mosquitto", "patches"))):
        if p.endswith(".patch"):
            with open(os.path.join(root, "mosquitto", "patches", p), "rb") as f:
                out[p] = hashlib.sha256(f.read()).hexdigest()
    return out


def parse_sha_list(txt):
    out = {}
    for line in txt.splitlines():
        v = line.split()
        if len(v) == 2:
            out[v[1].lstrip("*")] = v[0]
    return out


def prepare_cmd(R, spec, h, M):
    """bash script of the preparation of host h"""
    pmode, amode = METHODS[spec["method"]]
    lines = ["set -e", "rm -rf %s" % RUN, "mkdir -p %s" % RUN, "cd ~/fleet-dt",
             "echo %s | base64 -d > %s/dtp.json" % (b64(spec["dtp"]), RUN),
             "sudo nft delete table inet fleetdt 2>/dev/null || true",
             "sudo nft -f system/nftables-fdt.nft",
             "sudo nft list ruleset > %s/nft.txt" % RUN,
             "cat /opt/mosquitto-2.1/patches.sha256 > %s/guest-patches.sha256" % RUN,
             "/opt/mosquitto-2.1/sbin/mosquitto -h 2>&1 | head -1 > %s/mosquitto-version.txt || true" % RUN,
             "chronyc -n tracking > %s/chrony-before.txt 2>&1 || true" % RUN]
    if h == 0:
        vs = " ".join("--vessel %d=%s" % (i, R.host_ip(i)) for i in range(1, M + 1))
        lines += ["python3 -m fleetdt.gsconf gs %s%s > %s/m.conf" % (vs, " --b2" if pmode == "b2" else "", RUN),
                  "echo 'log_dest file %s/mosquitto.log' >> %s/m.conf" % (RUN, RUN)]
    else:
        lines += ["make -s build/fdt_regulator.so MOSQ=/opt/mosquitto-2.1 > %s/make.log 2>&1" % RUN,
                  "sha256sum build/fdt_regulator.so > %s/plugin.sha256" % RUN,
                  ("python3 -m fleetdt.gsconf vessel --vessel %d --dtp %s/dtp.json --plugin $HOME/fleet-dt/build/"
                   "fdt_regulator.so --mode %s --log-dest 'file %s/mosquitto.log'%s > %s/m.conf") % (
                      h, RUN, pmode, RUN, "".join(" --p1-%s-us %d" % (k, int(spec["p1_%s_us" % k]))
                                                  for k in ("spacing", "guard") if spec.get("p1_%s_us" % k) is not None),
                      RUN)]
    lines.append("echo prepared")
    return "\n".join(lines)


def bg(R, h, cmd, name, rt=None):
    """starts cmd in the background on host h (own session; PID file RUN/name.pid); rt: SCHED_FIFO priority"""
    if rt:
        inner = "%s & p=$!; sudo chrt -f -p %d $p > /dev/null; wait $p" % (cmd, rt)
    else:
        inner = cmd
    r = R.ssh(h, "cd ~; setsid nohup bash -c %s > %s/%s.log 2>&1 < /dev/null & echo $! > %s/%s.pid" % (
        shlex.quote(inner), RUN, name, RUN, name), timeout=30)
    if r.returncode != 0:
        raise RuntimeError("%s: start of %s failed: %s" % (R.host_name(h), name, r.stdout))


def as_cmd(spec, S0, steps):
    pmode, amode = METHODS[spec["method"]]
    a = spec.get("as") or {}
    c = ["python3", "-m", "fleetdt.as2.service", "--dtp", RUN + "/dtp.json", "--broker", "127.0.0.1:1883",
         "--mode", amode, "--log", RUN + "/dta.jsonl", "--first", str(S0), "--steps", str(steps + 1),
         "--predictor", spec.get("predictor", "eq"), "--monitor-lag", repr(MONITOR_LAG)]
    if a.get("init_scores") is not None or a.get("init_scores_file"):
        c += ["--init-scores", RUN + "/init_scores.json"]
    if amode == "b4":
        c += ["--z0", repr(float(a["z0"]))]
    if a.get("pred_gain") is not None:
        c += ["--pred-gain", repr(float(a["pred_gain"]))]
    if a.get("cal_kappa") is not None:
        c += ["--cal-kappa", repr(float(a["cal_kappa"]))]
    c += [str(x) for x in a.get("extra", [])]
    return "cd ~/fleet-dt && exec " + " ".join(shlex.quote(x) for x in c)


def run_fleet(R, spec, a):
    st = R.load_state()
    if not st:
        raise SystemExit("not up")
    M = st["M"]
    if int(spec["M"]) != M:
        raise SystemExit("experiment M=%d but session %d has M=%d" % (spec["M"], R.SLOT, M))
    if spec["method"] not in METHODS:
        raise SystemExit("unknown method %s" % spec["method"])
    dtp = spec["dtp"]
    if len(dtp["vessels"]) != M or float(dtp["Ts"]) != float(spec["Ts"]):
        raise SystemExit("the DTP of the experiment does not match M or Ts")
    name = spec.get("name", "fleet")
    rd = spec.get("run_dir") or os.path.join(R.EMU, "runs", time.strftime("%Y%m%d-%H%M%S") + "-" + name)
    if os.path.isdir(rd):
        shutil.rmtree(rd)
    os.makedirs(rd)
    with open(os.path.join(rd, "experiment.json"), "w") as f:
        json.dump(spec, f, indent=1)
    with open(os.path.join(rd, "dtp.json"), "w") as f:
        json.dump(dtp, f, indent=1)
    scen = os.path.join(rd, "scenario.json")
    with open(scen, "w") as f:
        json.dump(spec["scenario"], f, indent=1)
    res = {"name": name, "slot": R.SLOT, "M": M, "method": spec["method"], "seed": spec.get("seed"),
           "t_start": time.time(), "status": "started", "revision": revision(R.ROOT), "clocks": st.get("clocks"),
           "clock_seed": st.get("clock_seed")}

    def save():
        with open(os.path.join(rd, "result.json"), "w") as f:
            json.dump(res, f, indent=1)
    save()
    hosts = list(range(M + 1))
    Ts = float(dtp["Ts"])
    steps = int(spec.get("steps", 1200))
    cap = None
    started = False
    try:
        if spec.get("sync", True):
            R.cmd_sync(a)
        # preparation of every host
        out = R.parallel(lambda h: R.ssh(h, prepare_cmd(R, spec, h, M), timeout=300), hosts)
        for h, r in out.items():
            if isinstance(r, Exception) or r.returncode != 0 or "prepared" not in r.stdout:
                raise RuntimeError("preparation of %s failed: %s" % (R.host_name(h), r if isinstance(r, Exception)
                                                                     else r.stdout[-2000:]))
        # initial scores of the AS (too large for a command line): copied as a file
        a_ = spec.get("as") or {}
        init = a_.get("init_scores")
        if init is None and a_.get("init_scores_file"):
            with open(a_["init_scores_file"]) as f:
                init = json.load(f)
        if init is not None:
            p = os.path.join(rd, "init_scores.json")
            with open(p, "w") as f:
                json.dump(init, f)
            R.rsync_to(0, p, RUN + "/init_scores.json")
        # the guests must run exactly the patches of the repository
        want = repo_patches(R.ROOT)
        g = R.ssh(0, "cat %s/guest-patches.sha256" % RUN, timeout=20).stdout
        got = parse_sha_list(g)
        res["patches"] = {"repo": want, "guest": got, "equal": want == got}
        if want != got:
            raise RuntimeError("the guest Mosquitto was built with other patches: %s vs %s" % (got, want))
        # first step of the run and the emulator with the ground truth
        lead = float(spec.get("lead_s", 15))
        S0 = step_of(dtp, time.time() + lead) + 1
        t0 = step_start(dtp, S0)
        t_end = step_start(dtp, S0 + steps + 1) + dtp["T1"] + 1.5
        res.update({"S0": S0, "t0": t0, "steps": steps, "t_end_plan": t_end})
        save()
        R.stop_emulator(st.get("emu_pid"))
        # extra monitor batches MONITOR_LAG after the end of Q(t) (start + T1) and of W(t) (end - eps - d_rx)
        # (Section V-B): a record joins the batch in which its exchange ends, so the lag lets the exchanges
        # that cross these instants complete (a 1500-byte frame at 6 Mb/s with its ACK: 2.1 ms)
        flush = "--flush=%d:%d,%d" % (round(Ts * 1e6), round((dtp["T1"] + MONITOR_LAG) * 1e6),
                                      round((Ts - dtp["eps"] - dtp["d_rx"] + MONITOR_LAG) * 1e6))
        epid = R.start_emulator(scen, os.path.join(rd, "emu"), M, gt=True, rt=spec.get("rt", "HardLimit"),
                                extra=[flush] + list(spec.get("emu_args", [])), run_start_us=int(round(t0 * 1e6)))
        st["emu_pid"] = epid
        st["procs"].append([epid, "fdt-emu-optimized"])
        R.save_state(st)
        res["t_emu"] = time.time()
        cpu0 = R.cpu_snapshot(st)
        cap = R.start_daemon([R.sys.executable, os.path.join(R.ROOT, "emu", "tools", "mon_capture.py"),
                              "--broker", "127.0.0.1:%d" % R.MQTT_FWD, "--out", os.path.join(rd, "monitor.jsonl")],
                             os.path.join(rd, "mon_capture.log"))
        started = True
        # brokers: vessels first, so that the bridges of the GS connect at once
        for h in range(1, M + 1):
            bg(R, h, "/opt/mosquitto-2.1/sbin/mosquitto -c %s/m.conf" % RUN, "mosq", rt=50)
        bg(R, 0, "/opt/mosquitto-2.1/sbin/mosquitto -c %s/m.conf" % RUN, "mosq", rt=50)
        nb = M * (4 if METHODS[spec["method"]][0] == "b2" else 2)
        r = R.ssh(0, "cd ~/fleet-dt && python3 emu/tools/wait_bridges.py --n %d --timeout 30" % nb, timeout=40)
        res["bridges"] = r.stdout.strip()[-500:]
        if r.returncode != 0:
            raise RuntimeError("bridges not connected: %s" % r.stdout)
        # AS, operation control stub, sources
        bg(R, 0, as_cmd(spec, S0, steps), "as", rt=40)
        op = spec.get("opctl") or {}
        period = int(op.get("period", round(10.0 / Ts)))
        bg(R, 0, ("cd ~/fleet-dt && exec python3 -m fleetdt.as2.opctl --dtp %s/dtp.json --broker 127.0.0.1:1883 "
                  "--period %d --t0-step %d --first %d --steps %d --seed %d --log %s/order.jsonl") % (
            RUN, period, S0, S0 - 16, steps + 40, int(spec.get("seed", 1)), RUN), "opctl")
        src = spec.get("source") or {}
        dur = t_end - time.time() + 3.0
        for h in range(1, M + 1):
            c = ["python3", "fleet-dt/vessel/publishers/fdt_source.py", "--dtp", RUN + "/dtp.json", "--vessel", str(h),
                 "--t0", "%.6f" % t0, "--seed", str(int(spec.get("seed", 1))), "--log", RUN + "/src.jsonl",
                 "--duration", "%.1f" % dur]
            if src.get("trace"):
                c += ["--size-trace", "fleet-dt/" + src["trace"]]
            if src.get("median"):
                c += ["--median", repr(float(src["median"])), "--sigma", repr(float(src.get("sigma", 0.5)))]
            if src.get("scale", 1.0) != 1.0:
                c += ["--load", repr(float(src["scale"])), "--load-mode", "size"]
            bg(R, h, "cd ~ && exec " + " ".join(shlex.quote(x) for x in c), "src")
        res["t_started"] = time.time()
        res["start_margin_s"] = round(t0 - time.time(), 3)
        save()
        if t0 - time.time() < 1.0:
            raise RuntimeError("start too late: %.2f s before start(S0)" % (t0 - time.time()))
        # wait for the end of the run, watching the AS
        while time.time() < t_end:
            time.sleep(min(5.0, max(0.0, t_end - time.time())))
            r = R.ssh(0, "kill -0 $(cat %s/as.pid) 2>/dev/null && echo alive || echo dead" % RUN, timeout=20)
            if "dead" in r.stdout and time.time() < t_end - 2.0:
                raise RuntimeError("the AS stopped before the end of the run")
        res["status"] = "ok"
    except (RuntimeError, OSError, subprocess.SubprocessError) as e:
        res["status"] = "failed"
        res["error"] = str(e)[-4000:]
        R.log("run failed: %s" % e)
    finally:
        res["t_end"] = time.time()
        if started:
            res["cpu_cores"] = R.cpu_report(cpu0, R.cpu_snapshot(st))
        # stop sources, stub, AS, then the brokers (the plugin writes its statistics on exit)
        for proc in ("src", "opctl", "as"):
            R.parallel(lambda h: R.ssh(h, "[ -f {r}/{n}.pid ] && kill -TERM -- -$(cat {r}/{n}.pid) 2>/dev/null; true".format(
                r=RUN, n=proc), timeout=20), hosts)
        time.sleep(0.5)
        R.parallel(lambda h: R.ssh(h, "[ -f {r}/mosq.pid ] && kill -TERM -- -$(cat {r}/mosq.pid) 2>/dev/null; sleep 0.5; "
                                      "chronyc -n tracking > {r}/chrony-after.txt 2>&1; "
                                      "chronyc -n sourcestats >> {r}/chrony-after.txt 2>&1; true".format(r=RUN),
                                   timeout=30), hosts)
        time.sleep(0.5)
        if cap:
            R.kill_pid(cap, "mon_capture.py")
        R.stop_emulator(st.get("emu_pid"))
        for h in hosts:
            r = R.rsync_from(h, RUN + "/", os.path.join(rd, "vms", R.host_name(h)) + "/")
            if r.returncode != 0:
                res.setdefault("collect_errors", []).append("%s: %s" % (R.host_name(h), r.stdout[-300:]))
        R.slice_csv(os.path.join(R.SESS, "clock.csv"), os.path.join(rd, "clock.csv"), res["t_start"] - 60, res["t_end"])
        R.slice_jsonl(os.path.join(R.SESS, "bridge-stats.jsonl"), os.path.join(rd, "bridge-stats.jsonl"),
                      res["t_start"], res["t_end"])
        try:
            with open(os.path.join(rd, "emu", "summary.json")) as f:
                res["emu_summary"] = {k: v for k, v in json.load(f).items() if k in ("lag_us", "lag_us_all", "monitor",
                                                                                     "shift_s", "origin_ns")}
        except (OSError, ValueError):
            pass
        save()
        epid = R.start_emulator(st["scenario"], os.path.join(R.SESS, "emu-idle"), M, gt=False)
        st["emu_pid"] = epid
        st["procs"].append([epid, "fdt-emu-optimized"])
        R.save_state(st)
    R.log("run %s: %s (%s)" % (name, res["status"], rd))
    print(rd)
    return res
