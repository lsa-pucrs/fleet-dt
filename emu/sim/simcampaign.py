#!/usr/bin/env python3
"""Runs the specs of a simulation campaign with emu/sim/simrun.py, K at a time, pinned to CPUS.

  python3 emu/sim/simcampaign.py DIR [--K 2] [--cpus 14,15] [--sim-args '--delAckCount=1'] [--only PATTERN]

DIR holds specs/<run>.json (and init/ for the initial scores); the results go to DIR/runs/<run>/ and the
status to DIR/status.json. A run whose result.json says "ok" is skipped, so the command resumes.
The default CPUs 14 and 15 are the P-core that the emulation campaign leaves free (emu/run.py: the user
keeps 0-7 and 24-27, the emulators use 8-13, the VMs 16-23).
"""
import argparse
import fnmatch
import glob
import json
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
LOCK = threading.Lock()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dir")
    ap.add_argument("--K", type=int, default=2)
    ap.add_argument("--cpus", default="14,15")
    ap.add_argument("--sim-args", default="--delAckCount=1")
    ap.add_argument("--only", default=None, help="fnmatch pattern of run names")
    ap.add_argument("--nice", type=int, default=5)
    a = ap.parse_args()
    specs = sorted(glob.glob(os.path.join(a.dir, "specs", "*.json")))
    if a.only:
        specs = [p for p in specs if fnmatch.fnmatch(os.path.basename(p)[:-5], a.only)]
    st_path = os.path.join(a.dir, "status.json")
    status = {}
    if os.path.exists(st_path):
        with open(st_path) as f:
            status = json.load(f)
    todo = []
    for p in specs:
        name = os.path.basename(p)[:-5]
        rp = os.path.join(a.dir, "runs", name, "result.json")
        try:
            with open(rp) as f:
                if json.load(f).get("status") == "ok":
                    status[name] = "ok"
                    continue
        except (OSError, ValueError):
            pass
        todo.append(p)

    def save():
        with open(st_path + ".tmp", "w") as f:
            json.dump(status, f, indent=1)
        os.replace(st_path + ".tmp", st_path)

    def worker():
        while True:
            with LOCK:
                if not todo:
                    return
                p = todo.pop(0)
                name = os.path.basename(p)[:-5]
                status[name] = "running"
                save()
            out = os.path.join(a.dir, "runs", name)
            os.makedirs(out, exist_ok=True)
            cmd = ["taskset", "-c", a.cpus, "nice", "-n", str(a.nice), sys.executable,
                   os.path.join(HERE, "simrun.py"), p, "--out", out, "--campaign", a.dir, "--sim-args=" + a.sim_args]
            t0 = time.time()
            with open(os.path.join(out, "simrun.log"), "w") as lf:
                rc = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL).returncode
            with LOCK:
                status[name] = "ok" if rc == 0 else "failed"
                save()
            print("%s %s %s %.0f s" % (time.strftime("%H:%M:%S"), name, status[name], time.time() - t0), flush=True)

    print("%d runs to do, %d done" % (len(todo), sum(1 for v in status.values() if v == "ok")), flush=True)
    ths = [threading.Thread(target=worker) for _ in range(a.K)]
    for t in ths:
        t.start()
    for t in ths:
        t.join()
    print("done: %d ok, %d failed" % (sum(1 for v in status.values() if v == "ok"),
                                       sum(1 for v in status.values() if v == "failed")), flush=True)


if __name__ == "__main__":
    main()
