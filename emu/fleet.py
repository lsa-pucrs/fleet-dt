"""Fleet-DT runs in the emulator (paper Section VI): one run = the whole stack on the VMs of a session of
emu/run.py, the emulator with the ground truth, and the collection of every log.

Two stacks (emu/run.py run EXP.json --stack paper|as2; default: the "stack" of the experiment, else paper):

  paper  the model of the paper (Sections III to V): fleetdt.as_service (AS, Algorithm 1) with the outer
         loop fleetdt.outer, the VE collector (fleetdt.ve live, the DTA file), the v1 DTI program
         vessel/dti/fdt_dti.c, the JMCS stub emu/jmcs_stub.py (fleet/order, survey/transit every 10 s),
         the vessel brokers with the regulator plugin in "plugin_opt_mode paper", and the channel
         monitor of R6: the emulator exports the nl80211 counters of the AP (fdt-emu --cnt) and
         fleetdt.monitor_adapter computes y(t) by the logic of fdt_chmon.c and publishes it on
         fleet/throughput. See run_fleet_paper() and emu/README.md.
  as2    the older model of SPEC_v2.md (fleetdt.as2.service, fleetdt.as2.opctl), described below.

Every run records the git commit and a digest of the files of the stack in RUN/result.json
("revision", "stack_files"); "stack_commit" in the experiment refuses a run whose checkout is not that
commit or has changes in the files of the stack.

The rest of this docstring describes the as2 stack.

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
import glob
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
    # addendum: B6, IPACT with limited service; the regulator of the slotted variant releases R at once and
    # the other admitted units inside the slot of the vessel
    "b6": ("slotted", "ipact"),
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
    g = git_revision(root)   # the git commit of the checkout, when it is one
    if g.get("commit"):
        rev.update(g)
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
    """One fleet experiment on the stack of --stack (or of spec["stack"]; default paper)."""
    stack = getattr(a, "stack", None) or spec.get("stack") or "paper"
    if stack == "as2":
        return run_fleet_as2(R, spec, a)
    if stack == "paper":
        return run_fleet_paper(R, spec, a)
    raise SystemExit("unknown stack %s (paper or as2)" % stack)


def run_fleet_as2(R, spec, a):
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


# ================================================================================================
# paper stack

# files whose content defines the paper stack of a run (digest in RUN/result.json "stack_files")
PAPER_FILES = ["fleetdt/__init__.py", "fleetdt/as_service.py", "fleetdt/as_model.py", "fleetdt/model.py",
               "fleetdt/outer.py", "fleetdt/gsconf.py", "fleetdt/monitor_adapter.py", "fleetdt/ve/*.py",
               "vessel/dti/*.c", "vessel/regulator/*.[ch]", "vessel/publishers/*.py", "common/*.[ch]",
               "radio/channel_monitor/fdt_chmon.c", "radio/channel_monitor/build_host.sh", "emu/fleet.py",
               "emu/run.py", "emu/jmcs_stub.py", "emu/monitor_bridge.py", "emu/ns3/*", "Makefile",
               "mosquitto/patches/*.patch", "system/nftables-fdt.nft"]
PAPER_MONITOR = {"offset_us": 500, "jitter_us": 0, "quant": "harvest", "harvest_ms": 0, "sigma": 0.0,
                 "noise_seed": None, "backend": "auto"}


def paper_epoch(dtp):
    e = dtp.get("epoch")
    return DEFAULT_EPOCH if e is None else float(e)


def paper_start(dtp, t):
    """start(t) = epoch + t Ts (0-based steps, as fleetdt.as_service and the plugin in mode paper)."""
    return paper_epoch(dtp) + t * float(dtp["Ts"])


def paper_step_of(dtp, at):
    return int(math.floor((at - paper_epoch(dtp)) / float(dtp["Ts"])))


def stack_files(root, patterns=PAPER_FILES):
    out = {}
    for pat in patterns:
        for p in sorted(glob.glob(os.path.join(root, pat))):
            if os.path.isfile(p):
                with open(p, "rb") as f:
                    out[os.path.relpath(p, root)] = hashlib.sha256(f.read()).hexdigest()
    return out


def git_revision(root, patterns=PAPER_FILES):
    """git commit of the checkout and the files of the stack that differ from it (None without git)."""
    def git(*args):
        r = subprocess.run(["git", "-C", root] + list(args), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                           text=True, timeout=30)
        return r.stdout.strip() if r.returncode == 0 else None
    try:
        commit = git("rev-parse", "HEAD")
    except (OSError, subprocess.SubprocessError):
        commit = None
    if not commit:
        return {"commit": None}
    st = git("status", "--porcelain", "--untracked-files=all", "--", *patterns) or ""
    return {"commit": commit, "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
            "describe": git("describe", "--always", "--dirty"),
            "dirty_stack": sorted(line[3:] for line in st.splitlines() if line.strip()),
            "dirty": bool(git("status", "--porcelain"))}


def paper_revision(root):
    rev = revision(root)
    rev["stack_files"] = stack_files(root)
    return rev


def check_paper_spec(spec, M):
    dtp = spec.get("dtp") or {}
    topics = dtp.get("topics") or []
    if not topics or not all(isinstance(t, dict) and "rate" in t and "bound" in t for t in topics):
        raise SystemExit("--stack paper needs the DTP of the paper (dtp.json: topics with rate and bound); "
                         "for an experiment of the older model use --stack as2")
    if float(dtp["Ts"]) != float(spec["Ts"]):
        raise SystemExit("the DTP of the experiment does not match Ts")
    vessels = dtp.get("vessels") or list(range(1, M + 1))
    if sorted(int(v) for v in vessels) != list(range(1, M + 1)):
        raise SystemExit("the DTP vessels %s do not match the session (M=%d)" % (vessels, M))


def paper_prepare_cmd(R, spec, h, M):
    """bash script of the preparation of host h (paper stack)"""
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
        lines += ["python3 -m fleetdt.gsconf gs %s > %s/m.conf" % (vs, RUN),
                  "echo 'log_dest file %s/mosquitto.log' >> %s/m.conf" % (RUN, RUN)]
    else:
        pm = paper_plugin_mode(spec)
        ext = (spec.get("dti") or {}).get("enabled", True)
        lines += ["make -s build/fdt_regulator.so build/fdt_dti MOSQ=/opt/mosquitto-2.1 > %s/make.log 2>&1" % RUN,
                  "sha256sum build/fdt_regulator.so build/fdt_dti > %s/build.sha256" % RUN,
                  ("python3 -m fleetdt.gsconf vessel --vessel %d --dtp %s/dtp.json --plugin $HOME/fleet-dt/build/"
                   "fdt_regulator.so --mode %s --log-dest 'file %s/mosquitto.log' > %s/m.conf") % (h, RUN, pm, RUN, RUN),
                  # the vessel log of the regulator (the messages of each step that it did not forward)
                  "echo 'plugin_opt_log %s/regulator.jsonl' >> %s/m.conf" % (RUN, RUN),
                  # D_i(t) comes from the v1 DTI program (fdt_dti); the regulator polices to the table it sees
                  "echo 'plugin_opt_demand %s' >> %s/m.conf" % ("external" if ext else "internal", RUN)]
        for k, v in sorted((spec.get("plugin_opts") or {}).items()):
            lines.append("echo %s >> %s/m.conf" % (shlex.quote("plugin_opt_%s %s" % (k, v)), RUN))
        lines.append("grep -q '^plugin_opt_mode %s$' %s/m.conf" % (pm, RUN))
    lines.append("echo prepared")
    return "\n".join(lines)


def paper_plugin_mode(spec):
    """plugin_opt_mode of the vessel brokers: paper, or the ablations paper_nopolice (B5a) and
    paper_noeos (B5b) of the same regulator"""
    pm = spec.get("plugin_mode", "paper")
    if pm not in ("paper", "paper_nopolice", "paper_noeos"):
        raise SystemExit("plugin_mode: paper, paper_nopolice or paper_noeos")
    return pm


def paper_as_cmd(spec):
    """AS of the paper (Algorithm 1) with the outer loop on the DTA (fleetdt.outer)"""
    a = spec.get("as") or {}
    c = ["python3", "-m", "fleetdt.as_service", "--dtp", RUN + "/dtp.json", "--broker", "127.0.0.1:1883",
         "--epoch", repr(paper_epoch(spec["dtp"])), "--log", RUN + "/as.jsonl", "-v"]
    if a.get("U") is not None:
        c += ["--U", repr(float(a["U"]))]
    if a.get("alpha") is not None:
        c += ["--alpha", repr(float(a["alpha"]))]
    o = spec.get("outer") or {}
    if o.get("every"):
        c += ["--outer-dta", RUN + ("/outer_dta.jsonl" if o.get("dta_file") else "/dta.jsonl"),
              "--outer-every", str(int(o["every"])), "--outer-target", repr(float(o.get("target", 0.99)))]
        if o.get("window"):
            c += ["--outer-window", str(int(o["window"]))]
    c += [str(x) for x in a.get("extra", [])]
    return "cd ~/fleet-dt && exec " + " ".join(shlex.quote(x) for x in c)


def paper_ve_cmd(spec):
    c = ["python3", "-m", "fleetdt.ve", "live", "--broker", "127.0.0.1:1883", "--dtp", RUN + "/dtp.json",
         "--dta", RUN + "/dta.jsonl", "--epoch", repr(paper_epoch(spec["dtp"])),
         "--port", str(int((spec.get("ve") or {}).get("port", 8080)))]
    return "cd ~/fleet-dt && exec " + " ".join(shlex.quote(x) for x in c)


def paper_jmcs_cmd(spec, S0):
    j = spec.get("jmcs") or {}
    c = ["python3", "emu/jmcs_stub.py", "--dtp", RUN + "/dtp.json", "--broker", "127.0.0.1:1883",
         "--epoch", repr(paper_epoch(spec["dtp"])), "--s0", str(S0),
         "--period-s", repr(float(j.get("period_s", 10.0))), "--first", j.get("first", "survey"),
         "--offset-ms", repr(float(j.get("offset_ms", 2.0))), "--log", RUN + "/order.jsonl"]
    if j.get("video_prefix"):
        c += ["--video-prefix", j["video_prefix"]]
    if j.get("modes"):
        c += ["--modes", RUN + "/jmcs_modes.json"]
    return "cd ~/fleet-dt && exec " + " ".join(shlex.quote(x) for x in c)


def paper_dti_cmd(spec, h):
    """v1 DTI program: D_i(t) on boat/b<i>/demand at start(t) + offset (SCHED_FIFO through chrt)"""
    d = spec.get("dti") or {}
    off = float(d.get("offset_ms", 1.0)) + (h - 1) * float(d.get("stagger_ms", 0.0))
    c = ["build/fdt_dti", "--dtp", RUN + "/dtp.json", "--vessel", str(h), "--broker", "127.0.0.1:1883",
         "--offset-ms", repr(off), "--idle", str(int(d.get("idle", 8))), "--prio", "0"]
    return "cd ~/fleet-dt && exec " + " ".join(shlex.quote(x) for x in c)


def paper_source_cmd(spec, h, t0, dur):
    src = spec.get("source") or {}
    seed = int(spec.get("seed", 1))
    if src.get("cmd"):
        c = src["cmd"].format(dtp=RUN + "/dtp.json", vessel=h, seed=seed, t0="%.6f" % t0, duration="%.1f" % dur,
                              run=RUN)
        return "cd ~/fleet-dt && exec " + c
    c = ["python3", "vessel/publishers/raw_load.py", "--dtp", RUN + "/dtp.json", "--vessel", str(h),
         "--host", "127.0.0.1", "--port", "1883", "--scale", repr(float(src.get("scale", 1.0))),
         "--size", str(src.get("size", "random")), "--min-frac", repr(float(src.get("min_frac", 0.5))),
         "--seed", str(seed), "--duration", "%.1f" % dur]
    if src.get("topics"):
        c += ["--topics", ",".join(src["topics"])]
    return "cd ~/fleet-dt && exec " + " ".join(shlex.quote(x) for x in c)


def paper_monitor(spec):
    m = dict(PAPER_MONITOR)
    m.update(spec.get("monitor") or {})
    if m["noise_seed"] is None:
        m["noise_seed"] = int(spec.get("seed", 1))
    if m["quant"] not in ("harvest", "cumulative") or m["backend"] not in ("auto", "c", "py"):
        raise SystemExit("monitor: quant harvest|cumulative, backend auto|c|py")
    if float(m["sigma"]) < 0:
        raise SystemExit("monitor: sigma >= 0")
    return m


def paper_emu_args(R, spec, mon):
    Ts_us = int(round(float(spec["dtp"]["Ts"]) * 1e6))
    return ["--cnt=%d:%d:%d" % (Ts_us, int(mon["offset_us"]), int(mon["jitter_us"])),
            "--cntDst=127.0.0.1:%d" % R.CNT_PORT, "--cntQuant=%s" % mon["quant"],
            "--cntHarvestMs=%g" % float(mon["harvest_ms"]),
            "--cntSeed=%d" % int(spec.get("seed", 1))] + list(spec.get("emu_args", []))


def paper_adapter_cmd(R, rd, mon, chmon):
    c = ["env", "PYTHONPATH=" + R.ROOT, R.sys.executable, "-m", "fleetdt.monitor_adapter",
         "--dtp", os.path.join(rd, "dtp.json"), "--udp", "127.0.0.1:%d" % R.CNT_PORT,
         "--broker", "127.0.0.1:%d" % R.MQTT_FWD, "--backend", mon["backend"],
         "--sigma", repr(float(mon["sigma"])), "--seed", str(int(mon["noise_seed"])),
         "--log", os.path.join(rd, "throughput.jsonl"), "--record", os.path.join(rd, "counters_rx.jsonl"), "-v"]
    if chmon:
        c += ["--chmon", chmon]
    return c


def build_chmon_host(R, rd, mon):
    """fdt_chmon for the host (backend c of the adapter); None when it does not build and the backend
    may fall back to the Python port"""
    if mon["backend"] == "py":
        return None
    out = os.path.join(R.ROOT, "build", "fdt_chmon_host")
    r = subprocess.run(["sh", os.path.join(R.ROOT, "radio", "channel_monitor", "build_host.sh"), out],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    with open(os.path.join(rd, "chmon_build.log"), "w") as f:
        f.write(r.stdout)
    if r.returncode != 0:
        if mon["backend"] == "c":
            raise RuntimeError("fdt_chmon_host does not build: %s" % r.stdout[-500:])
        return None
    return out


def adapter_backend(logpath):
    try:
        with open(logpath) as f:
            for line in f:
                if line.startswith("monitor_adapter: backend "):
                    return line.split()[-1]
    except OSError:
        pass
    return None


def run_fleet_paper(R, spec, a):
    st = R.load_state()
    if not st:
        raise SystemExit("not up")
    M = st["M"]
    if int(spec["M"]) != M:
        raise SystemExit("experiment M=%d but session %d has M=%d" % (spec["M"], R.SLOT, M))
    for k in ("dtp", "scenario"):   # a path (relative to the repository) in place of the object
        if isinstance(spec.get(k), str):
            with open(os.path.join(R.ROOT, spec[k])) as f:
                obj = json.load(f)
            if k == "dtp":            # a DTP file of the repository serves any M: its vessels become 1..M
                obj["vessels"] = list(range(1, M + 1))
            spec = dict(spec, **{k: obj})
    check_paper_spec(spec, M)
    dtp = spec["dtp"]
    mon = paper_monitor(spec)
    paper_plugin_mode(spec)
    rev = paper_revision(R.ROOT)
    pin = spec.get("stack_commit")
    if pin and (not rev.get("commit") or not rev["commit"].startswith(pin) or rev.get("dirty_stack")):
        raise SystemExit("stack_commit %s: the checkout is %s with changes in %s" % (
            pin, rev.get("commit"), rev.get("dirty_stack")))
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
    res = {"name": name, "stack": "paper", "slot": R.SLOT, "M": M, "method": spec.get("method", "fleetdt"),
           "seed": spec.get("seed"), "t_start": time.time(), "status": "started", "revision": rev,
           "dtp_sha256": hashlib.sha256(json.dumps(dtp, sort_keys=True).encode()).hexdigest(),
           "monitor": mon, "clocks": st.get("clocks"), "clock_seed": st.get("clock_seed"),
           "epoch": paper_epoch(dtp), "step_clock": "start(t) = epoch + t Ts (0-based)",
           "plugin_mode": paper_plugin_mode(spec),
           "demand": "external (fdt_dti)" if (spec.get("dti") or {}).get("enabled", True) else "internal (plugin)"}

    def save():
        with open(os.path.join(rd, "result.json"), "w") as f:
            json.dump(res, f, indent=1)
    save()
    hosts = list(range(M + 1))
    Ts = float(dtp["Ts"])
    steps = int(spec.get("steps", 1200))
    cap = adp = None
    started = False
    cpu0 = None
    try:
        if spec.get("sync", True):
            R.cmd_sync(a)
        chmon = build_chmon_host(R, rd, mon)
        if chmon:
            with open(chmon, "rb") as f:
                res["chmon_host_sha256"] = hashlib.sha256(f.read()).hexdigest()
        out = R.parallel(lambda h: R.ssh(h, paper_prepare_cmd(R, spec, h, M), timeout=300), hosts)
        for h, r in out.items():
            if isinstance(r, Exception) or r.returncode != 0 or "prepared" not in r.stdout:
                raise RuntimeError("preparation of %s failed: %s" % (R.host_name(h), r if isinstance(r, Exception)
                                                                     else r.stdout[-2000:]))
        # files of the GS that do not fit a command line
        o = spec.get("outer") or {}
        if o.get("dta_file"):
            shutil.copy(o["dta_file"], os.path.join(rd, "outer_dta.jsonl"))
            R.rsync_to(0, os.path.join(rd, "outer_dta.jsonl"), RUN + "/outer_dta.jsonl")
        j = spec.get("jmcs") or {}
        if j.get("modes"):
            p = os.path.join(rd, "jmcs_modes.json")
            with open(p, "w") as f:
                json.dump(j["modes"], f)
            R.rsync_to(0, p, RUN + "/jmcs_modes.json")
        want = repo_patches(R.ROOT)
        got = parse_sha_list(R.ssh(0, "cat %s/guest-patches.sha256" % RUN, timeout=20).stdout)
        res["patches"] = {"repo": want, "guest": got, "equal": want == got}
        if want != got:
            raise RuntimeError("the guest Mosquitto was built with other patches: %s vs %s" % (got, want))
        # first step of the run (0-based clock of the paper stack)
        lead = float(spec.get("lead_s", 15))
        S0 = paper_step_of(dtp, time.time() + lead) + 1
        t0 = paper_start(dtp, S0)
        t_end = paper_start(dtp, S0 + steps) + float(dtp["T1"]) + 1.5
        res.update({"S0": S0, "t0": t0, "steps": steps, "t_end_plan": t_end})
        save()
        # channel monitor: the adapter listens before the emulator sends the first counter read
        adp = R.start_daemon(paper_adapter_cmd(R, rd, mon, chmon), os.path.join(rd, "monitor_adapter.log"))
        R.stop_emulator(st.get("emu_pid"))
        epid = R.start_emulator(scen, os.path.join(rd, "emu"), M, gt=True, rt=spec.get("rt", "HardLimit"),
                                extra=paper_emu_args(R, spec, mon), run_start_us=int(round(t0 * 1e6)))
        st["emu_pid"] = epid
        st["procs"].append([epid, "fdt-emu-optimized"])
        R.save_state(st)
        res["t_emu"] = time.time()
        cpu0 = R.cpu_snapshot(st)
        cap = R.start_daemon([R.sys.executable, os.path.join(R.ROOT, "emu", "tools", "mon_capture.py"),
                              "--broker", "127.0.0.1:%d" % R.MQTT_FWD, "--out", os.path.join(rd, "monitor.jsonl")],
                             os.path.join(rd, "mon_capture.log"))
        started = True
        for h in range(1, M + 1):
            bg(R, h, "/opt/mosquitto-2.1/sbin/mosquitto -c %s/m.conf" % RUN, "mosq", rt=50)
        bg(R, 0, "/opt/mosquitto-2.1/sbin/mosquitto -c %s/m.conf" % RUN, "mosq", rt=50)
        r = R.ssh(0, "cd ~/fleet-dt && python3 emu/tools/wait_bridges.py --n %d --timeout 30" % (2 * M), timeout=40)
        res["bridges"] = r.stdout.strip()[-500:]
        if r.returncode != 0:
            raise RuntimeError("bridges not connected: %s" % r.stdout)
        # GS: VE (writes the DTA), JMCS stub, AS; vessels: DTI program and sources
        bg(R, 0, paper_ve_cmd(spec), "ve")
        bg(R, 0, paper_jmcs_cmd(spec, S0), "jmcs")
        bg(R, 0, paper_as_cmd(spec), "as", rt=40)
        if (spec.get("dti") or {}).get("enabled", True):
            for h in range(1, M + 1):
                bg(R, h, paper_dti_cmd(spec, h), "dti", rt=45)
        dur = t_end - time.time() + 3.0
        for h in range(1, M + 1):
            bg(R, h, paper_source_cmd(spec, h, t0, dur), "src")
        res["commands"] = {"gs": {"as": paper_as_cmd(spec), "ve": paper_ve_cmd(spec),
                                  "jmcs": paper_jmcs_cmd(spec, S0)},
                           "vessel_1": {"dti": paper_dti_cmd(spec, 1), "src": paper_source_cmd(spec, 1, t0, dur)},
                           "adapter": paper_adapter_cmd(R, rd, mon, chmon),
                           "emulator_extra": paper_emu_args(R, spec, mon)}
        res["t_started"] = time.time()
        res["start_margin_s"] = round(t0 - time.time(), 3)
        save()
        if t0 - time.time() < 1.0:
            raise RuntimeError("start too late: %.2f s before start(S0)" % (t0 - time.time()))
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
        if started and cpu0 is not None:
            res["cpu_cores"] = R.cpu_report(cpu0, R.cpu_snapshot(st))
        # sources, DTI, stub, AS, VE; then the brokers (the plugin writes its log and statistics on exit)
        for proc in ("src", "dti", "jmcs", "as", "ve"):
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
        if adp:
            R.kill_pid(adp, "monitor_adapter")
        res["monitor"]["backend_used"] = adapter_backend(os.path.join(rd, "monitor_adapter.log"))
        for h in hosts:
            r = R.rsync_from(h, RUN + "/", os.path.join(rd, "vms", R.host_name(h)) + "/")
            if r.returncode != 0:
                res.setdefault("collect_errors", []).append("%s: %s" % (R.host_name(h), r.stdout[-300:]))
        R.slice_csv(os.path.join(R.SESS, "clock.csv"), os.path.join(rd, "clock.csv"), res["t_start"] - 60, res["t_end"])
        R.slice_jsonl(os.path.join(R.SESS, "bridge-stats.jsonl"), os.path.join(rd, "bridge-stats.jsonl"),
                      res["t_start"], res["t_end"])
        try:
            with open(os.path.join(rd, "emu", "summary.json")) as f:
                res["emu_summary"] = {k: v for k, v in json.load(f).items() if k in (
                    "lag_us", "lag_us_all", "monitor", "nl80211", "shift_s", "origin_ns")}
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
