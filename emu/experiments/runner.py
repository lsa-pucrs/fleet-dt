#!/usr/bin/env python3
"""Executes a manifest of emu/experiments/matrix.py with up to K sessions at once (emu/run.py --slot k).

  python3 emu/experiments/runner.py DIR [--K 3] [--results EMU/results/<name>] [--max-runs N] [--keep-raw]
                                     [--retry 1] [--no-pin]

Blocks (manifests of emu/experiments/campaign.py): a run whose "block" holds a "slot" s runs only on slot
s mod K (K the current number of slots), so the runs of a block share one session; --no-pin ignores it.
An outer-loop variant whose "outer"."calibration_file" is relative reads it from the manifest directory.

Each slot k (0 .. K-1) brings up its own session (GS + M vessel VMs, emulator, monitor bridge, clock
probe) with the M of its next run, copies the repository into the VMs once, and then runs the pending runs
of that M in the order of the manifest; a run that needs another M takes the session down and up again.
After each run the slot post-processes it and compresses gt.bin and monitor.jsonl with zstd (unless
--keep-raw): a run of the as2 stack gets the ground truth per step (emu/analysis/gtstep.py) and the metrics
of the older model (emu/analysis/metrics.py); a run of the paper stack gets the metrics of the paper from
its DTA with fleetdt/analysis.py (emu/analysis/paper_post.py), never the older-model scripts.

Real-time check (Section VI-B): a run is "lagging" when the p99 of the lag of the emulator after its
warm-up exceeds 300 us or more than 1 % of its steps have a lag above 1 ms; a lagging run is queued again
once (--retry), and two lagging runs in a row on the slots lower K by one (the highest slot stops after its
run). The scenarios of --lag-exempt (default S2, whose lag p99 of 0.3-0.42 ms does not fall with K: K = 1
gave 0.415 ms) are judged by the steps above 1 ms alone (the analysis discards these steps) and do not
count for the lowering of K. The manifest (DIR/manifest.json, written atomically) holds the status of every run, so the runner
resumes after an interruption (runs left "running" start again). A file DIR/STOP stops the slots after
their current runs. A run with "max_K": m (cells that meet the lag rule only with at most m sessions)
runs on the slots 0 .. m-1 and only while at most m sessions run, itself included. Resources: each session uses M+1 VMs of 768 MB; a slot starts only while the host
keeps at least 20 GB of available memory after it.

Stack: a spec without "stack" runs on the as2 stack (the specs of matrix.py and of the frozen campaign);
a spec with "stack": "paper" runs on the paper stack.

Session of a paper-stack run (Section VI-B): "session" of the spec over emu/fleet.py PAPER_SESSION, so by
default the vessel VMs run under the CPU quota of 0.25 of a core (run.py up --cpu-ratio 0.25), the clock
injector is on (--clock-inject, and the run gets "clock": {"inject": true}, the default schedule of
fdt_clock_schedule: +-20 ppm ramps and NTP loss episodes) and the clock probe reads once per step
(--probe-period Ts). "session": {"cpu_ratio": null, "clock_inject": false, "probe_period_s": 0.1} gives
the session of run.py up without options (the as2 stack always runs on it). A run whose session options
(or M) differ from those of the session of its slot takes the session down and up again.
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if os.path.join(ROOT, "emu") not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, "emu"))
import fleet  # noqa: E402
RUNPY = os.path.join(ROOT, "emu", "run.py")
EMU = os.path.expanduser(os.environ.get("FDT_EMU", "~/fleetdt-deploy/emu"))
LOCK = threading.Lock()
RESERVE_GB = 20.0


def log(*a):
    print("[runner %s]" % time.strftime("%H:%M:%S"), *a, flush=True)


def mem_available_gb():
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1048576.0
    return 0.0


def mk(r):
    """the largest number of concurrent sessions of a run (manifest field max_K, default unlimited)"""
    return int(r.get("max_K") or 99)


def block_slot(r, K):
    """slot of the block of a run with K slots: the "slot" of its "block" mod K (and below its max_K); None for a
    run without a block slot"""
    b = r.get("block")
    if not isinstance(b, dict) or b.get("slot") is None:
        return None
    return int(b["slot"]) % max(1, min(int(K), mk(r)))


class Manifest:
    def __init__(self, d):
        self.dir = d
        self.path = os.path.join(d, "manifest.json")
        with open(self.path) as f:
            self.m = json.load(f)
        for r in self.m["runs"]:
            if r["status"] == "running":
                r["status"] = "pending"
        self.save()

    def save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.m, f, indent=1)
        os.replace(tmp, self.path)

    def next(self, M=None, slot=0, K=None):
        """(run, None) to start on slot `slot`, or (None, "wait") or (None, "exit").

        A run with "max_K": m starts only on a slot k < m and only while at most m sessions run a run,
        counting itself, and no running run has a smaller max_K. With K (the number of slots), a run whose
        "block" holds a "slot" s (emu/experiments/campaign.py: the session slot of the block) runs only on
        slot s mod K, so the runs of a block share one session. The slot takes the first pending run of
        the manifest that it may take (the runs of the same max_K and of M first): a run that may not
        start yet makes the slot wait, so the restricted runs (at the end of the manifest) are not
        starved; a slot exits when no pending run may ever run on it."""
        with LOCK:
            pend = [r for r in self.m["runs"] if r["status"] == "pending"]
            mine = [r for r in pend if slot < mk(r) and (K is None or block_slot(r, K) in (None, slot))]
            if not mine:
                return None, "exit"
            first = mine[0]
            same = [r for r in mine if mk(r) == mk(first)]
            pick = next((r for r in same if M is not None and r["M"] == M), first)
            running = [r for r in self.m["runs"] if r["status"] == "running"]
            limit = min([mk(pick)] + [mk(r) for r in running])
            if len(running) + 1 > limit:
                return None, "wait"
            pick["status"] = "running"
            pick["t_begin"] = time.time()
            self.save()
            return pick, None

    def update(self, r, **kw):
        with LOCK:
            r.update(kw)
            self.save()


def run_spec(spec, rd, man_dir):
    """The spec of a manifest run as emu/run.py receives it: run directory, no repository sync, the as2
    stack unless the spec names one (the specs of matrix.py, and those of the frozen campaign, which
    predate the "stack" field; emu/run.py would otherwise default to the paper stack), and the initial
    scores file relative to the manifest directory."""
    spec = dict(spec)
    spec["run_dir"] = rd
    spec["sync"] = False
    spec.setdefault("stack", "as2")
    if spec["stack"] == "paper" and spec.get("Ts") is not None:
        # the session options of the run, resolved (the defaults of the paper campaign unless "session" sets
        # them), so that emu/fleet.py checks them against the session and installs the clock schedule
        spec["session"] = fleet.paper_session(spec)
    ref = (spec.get("as") or {}).get("init_scores_file")
    if ref and not os.path.isabs(ref):
        spec["as"] = dict(spec["as"], init_scores_file=os.path.join(man_dir, ref))
    cf = (spec.get("outer") or {}).get("calibration_file")
    if cf and not os.path.isabs(cf):
        # the calibration of an outer-loop variant (E5), relative to the manifest directory
        spec["outer"] = dict(spec["outer"], calibration_file=os.path.join(man_dir, cf))
    return spec


def session_key(spec):
    """(M, cpu_ratio, clock_inject, probe_period_s) of the session that a run (spec of run_spec) needs"""
    if spec.get("stack") == "paper" and spec.get("session") is not None:
        s = spec["session"]
    else:
        s = fleet.BARE_SESSION
    return (int(spec["M"]), s["cpu_ratio"], bool(s["clock_inject"]), float(s["probe_period_s"]))


def up_args(key, clock_seed):
    """options of run.py up for the session key of session_key"""
    M, ratio, inject, period = key
    out = ["up", "--scenario", os.path.join(ROOT, "emu", "scenarios", "S1-m%d.json" % M),
           "--clock-seed", str(clock_seed), "--sync", "--probe-period", repr(period)]
    if ratio is not None:
        out += ["--cpu-ratio", repr(float(ratio))]
    if inject:
        out.append("--clock-inject")
    return out


def run_py(slot, *args, log_path=None, timeout=None):
    cmd = [sys.executable, RUNPY, "--slot", str(slot)] + list(args)
    out = open(log_path, "a") if log_path else subprocess.DEVNULL
    try:
        return subprocess.run(cmd, stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, timeout=timeout).returncode
    finally:
        if log_path:
            out.close()


def postprocess_cmds(rd, stack="as2"):
    """post-processing commands of a run directory: the paper stack has its own metrics (paper_post.py,
    fleetdt/analysis.py); the as2 stack the ground truth per step and the metrics of the older model"""
    if stack == "paper":
        return [[sys.executable, os.path.join(ROOT, "emu", "analysis", "paper_post.py"), rd]]
    return [[sys.executable, os.path.join(ROOT, "emu", "analysis", "gtstep.py"), rd],
            [sys.executable, os.path.join(ROOT, "emu", "analysis", "metrics.py"), rd]]


def postprocess(rd, keep_raw, stack="as2"):
    env = dict(os.environ)
    # the auxiliary CPUs of the sessions (emu/run.py: FDT_AUX_CPUS, default 16-23 of the i7-14700)
    pin = ["taskset", "-c", os.environ.get("FDT_AUX_CPUS") or "16-23", "nice", "-n", "5"]
    rs = [subprocess.run(pin + c, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
          for c in postprocess_cmds(rd, stack)]
    if not keep_raw:
        for p in (os.path.join(rd, "emu", "gt.bin"), os.path.join(rd, "monitor.jsonl")):
            if os.path.exists(p):
                subprocess.run(pin + ["zstd", "-q", "-9", "-T2", "--rm", "-f", p], check=False)
    return all(r.returncode == 0 for r in rs), "".join(r.stdout[-1000:] for r in rs)


def lag_verdict(rd, exempt=False):
    """(lagging, info) from the emulator summary and the metrics of the run"""
    info = {}
    try:
        with open(os.path.join(rd, "emu", "summary.json")) as f:
            s = json.load(f)
        info["p99_us"] = s["lag_us"]["p99"]
        info["max_us"] = s["lag_us"]["max"]
    except (OSError, ValueError, KeyError):
        return True, {"error": "no summary"}
    try:
        with open(os.path.join(rd, "metrics.json")) as f:
            m = json.load(f)
        if "steps_over_1ms" in m:          # paper_post: the steps stay in the metrics with a lag flag
            info["steps_over_1ms"] = m["steps_over_1ms"]
        else:                              # metrics.py of the as2 stack: the lagging steps are left out
            n = max(1, m.get("n_steps", 0) + m.get("lag_discarded", 0))
            info["steps_over_1ms"] = m.get("lag_discarded", 0) / n
    except (OSError, ValueError):
        info["steps_over_1ms"] = None
    steps_bad = (info["steps_over_1ms"] or 0) > 0.01
    bad = steps_bad if exempt else (info["p99_us"] > 300 or steps_bad)
    info["exempt"] = exempt
    return bad, info


class Runner:
    def __init__(self, a):
        self.a = a
        self.man = Manifest(a.dir)
        self.results = a.results or os.path.join(EMU, "results", self.man.m["name"])
        os.makedirs(self.results, exist_ok=True)
        self.K = a.K
        self.lag_streak = 0
        self.done = 0
        self.stop = threading.Event()

    def slot_loop(self, k):
        lp = os.path.join(self.results, "slot%d.log" % k)
        M_up = None
        key_up = None
        try:
            while not self.stop.is_set():
                if k >= self.K or os.path.exists(os.path.join(self.a.dir, "STOP")):
                    break
                if self.a.max_runs and self.done >= self.a.max_runs:
                    break
                r, why = self.man.next(M_up, k, None if self.a.no_pin else self.K)
                if r is None:
                    if why == "exit":
                        break
                    time.sleep(10)
                    continue
                rd = os.path.join(self.results, r["id"])
                with open(os.path.join(self.a.dir, r["spec"])) as f:
                    spec = run_spec(json.load(f), rd, self.a.dir)
                key = session_key(dict(spec, M=r["M"]))
                if key != key_up:
                    if M_up is not None:
                        run_py(k, "down", log_path=lp)
                    need = (r["M"] + 1) * 0.8
                    while mem_available_gb() - need < RESERVE_GB and not self.stop.is_set():
                        log("slot %d: waiting for memory (%.1f GB available)" % (k, mem_available_gb()))
                        time.sleep(30)
                    rc = run_py(k, *up_args(key, 100 + k), log_path=lp, timeout=600)
                    if rc != 0:
                        log("slot %d: up failed (see %s)" % (k, lp))
                        self.man.update(r, status="pending")
                        run_py(k, "down", log_path=lp)
                        M_up = key_up = None
                        time.sleep(30)
                        continue
                    M_up, key_up = r["M"], key
                sp = os.path.join(self.results, ".spec-slot%d.json" % k)
                with open(sp, "w") as f:
                    json.dump(spec, f)
                log("slot %d: %s" % (k, r["id"]))
                t0 = time.time()
                rc = run_py(k, "run", sp, log_path=lp, timeout=int(spec["steps"] * spec["Ts"]) + 600)
                ok_pp, pp = postprocess(rd, self.a.keep_raw, spec.get("stack", "as2")) if os.path.isdir(rd) \
                    else (False, "no run dir")
                exempt = r.get("scenario") in self.a.lag_exempt
                lagging, info = lag_verdict(rd, exempt)
                st = "done" if rc == 0 and ok_pp else "failed"
                if st == "done" and lagging:
                    with LOCK:
                        self.lag_streak += 0 if exempt else 1
                        if self.lag_streak >= 2 and self.K > 1:
                            self.K -= 1
                            log("two lagging runs in a row: K lowered to %d" % self.K)
                            self.lag_streak = 0
                    if r.get("attempts", 0) < self.a.retry:
                        st = "pending"
                elif st == "done":
                    with LOCK:
                        self.lag_streak = 0
                self.man.update(r, status=st, attempts=r.get("attempts", 0) + 1, run_dir=rd, slot=k,
                                t_done=time.time(), seconds=round(time.time() - t0, 1), lag=info,
                                lagging=lagging, post=pp[-300:] if st == "failed" else None)
                with LOCK:
                    self.done += 1
                log("slot %d: %s %s in %.0f s, lag p99 %s us, steps over 1 ms %s" % (
                    k, r["id"], st, time.time() - t0, info.get("p99_us"), info.get("steps_over_1ms")))
        finally:
            if M_up is not None:
                run_py(k, "down", log_path=lp)

    def run(self):
        th = []
        for k in range(self.K):
            t = threading.Thread(target=self.slot_loop, args=(k,))
            t.start()
            th.append(t)
            time.sleep(20)          # stagger the boots
        try:
            for t in th:
                t.join()
        except KeyboardInterrupt:
            log("interrupted: the slots stop after their current runs")
            self.stop.set()
            for t in th:
                t.join()
        c = {}
        for r in self.man.m["runs"]:
            c[r["status"]] = c.get(r["status"], 0) + 1
        log("status: %s" % c)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dir")
    ap.add_argument("--K", type=int, default=3)
    ap.add_argument("--results", default=None)
    ap.add_argument("--max-runs", type=int, default=0)
    ap.add_argument("--keep-raw", action="store_true")
    ap.add_argument("--retry", type=int, default=1)
    ap.add_argument("--lag-exempt", nargs="*", default=["S2"],
                    help="scenarios judged by the steps above 1 ms alone (not by the p99 of the lag)")
    ap.add_argument("--no-pin", action="store_true",
                    help="ignore the session slot of the blocks (block.slot of campaign.py manifests)")
    a = ap.parse_args()
    Runner(a).run()


if __name__ == "__main__":
    main()
