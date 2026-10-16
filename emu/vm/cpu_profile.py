#!/usr/bin/env python3
"""CPU profile of the vessel VMs: cgroup v2 `cpu.max` of the QEMU process of a vessel, calibrated to the
Raspberry Pi 4 of the Jundiá vessels (paper Section VI-B; design item I9).

The bandwidth controller lets the vCPU thread run `quota` microseconds in every `period`, so the vessel
software runs at the fraction r = quota / period of an E-core of the emulation host (the VMs run on the
E-cores 16-23 of the i7-14700). cgroup v2 refuses a quota or a period below 1 ms, so the profile takes
quota = max(1 ms, r period) with the shortest period that allows it (period = 1 ms / r for r < 1): the
throttling stalls the vCPU for at most period - quota at a time (3 ms at r = 0.25), shorter than the
phases of a step (T1 = 20 ms, T2 = 10 ms).

Assumed ratio (until measured): r = 0.25, i.e. one core of the Raspberry Pi 4 (Cortex-A72, 1.5 GHz)
does a quarter of the single-thread work of one Gracemont E-core of the i7-14700 (up to 4.2 GHz). The
published single-thread results that bracket it give 0.19 (Geekbench 5 single core, about 250 against
about 1300) to 0.32 (CoreMark per core, about 7.4k against about 23k); the value is a placeholder
that the calibration below replaces. The Raspberry Pi 4 has 4 cores and the vessel VM 1 vCPU: the
profile reproduces the speed of one process of the vessel software, not the parallelism of the four
cores (conservative for the contention between the broker, the DTI program and the publishers).

Calibration (repeat after any change of the host, the image or the vessel computer):
  1. on the vessel computer (idle, governor performance):  python3 emu/vm/cpu_bench.py --out rpi4.json
  2. in an unthrottled vessel VM on the VM cores:           python3 emu/vm/cpu_bench.py --out vm.json
  3. python3 emu/vm/cpu_profile.py calibrate rpi4.json vm.json   -> ratio r, cpu.max, systemd properties
  4. start a vessel VM with the profile (hook in emu/run.py: `wrap`) and check inside it:
     python3 emu/vm/cpu_bench.py --out vm-throttled.json;
     python3 emu/vm/cpu_profile.py check rpi4.json vm-throttled.json   (|score ratio - 1| <= 0.1)

  python3 emu/vm/cpu_profile.py profile [--ratio 0.25]            cpu.max line and systemd properties
  python3 emu/vm/cpu_profile.py wrap [--ratio 0.25] [--unit NAME] -- CMD ...
                                   the command under `systemd-run --user --scope` with CPUQuota and
                                   CPUQuotaPeriodSec (rootless: needs the cpu controller delegated to
                                   user@UID.service; `check-delegation` tells)
  python3 emu/vm/cpu_profile.py apply PID [--ratio 0.25] [--cgroup /sys/fs/cgroup/fdt/vNAME]
                                   root: create the cgroup, write cpu.max and move PID (QEMU) into it
  python3 emu/vm/cpu_profile.py status PID                        cpu.max and cpu.stat (throttling) of PID
  python3 emu/vm/cpu_profile.py check-delegation
  python3 emu/vm/cpu_profile.py mechanism                         cgroup (cpu delegated to the user) or user

Mechanism "user" (hosts with cgroup v1 or without the cpu controller delegated and without root, e.g.
hydra, Rocky Linux 8): build/fdt_cpucap (emu/vm/cpucap.c) applies the same rule as cpu.max from user
space: it reads the CPU time of each QEMU process (one perf task-clock counter per thread, exact at the
read) every poll interval (100 us), sends SIGSTOP when the CPU time of the current period reaches the
quota and SIGCONT at the next period boundary; quota and period are those of profile(). Its statistics
(nr_throttled, throttled_usec, the overrun of the quota and the lateness of the resume, p50/p99/max) are
in the stats file of the session (status_user()).
"""
import argparse
import json
import math
import os
import shlex
import subprocess
import sys

ASSUMED_RATIO = 0.25
MIN_US = 1000
MAX_PERIOD_US = 1000000


def profile(ratio=ASSUMED_RATIO):
    """(quota_us, period_us) of the ratio with the shortest period that cgroup v2 accepts"""
    if not 0 < ratio <= 1:
        raise ValueError("ratio must be in (0, 1]")
    if ratio >= 1:
        return MIN_US * 100, MIN_US * 100
    period = int(math.ceil(MIN_US / ratio))
    if period > MAX_PERIOD_US:
        raise ValueError("ratio too small for cgroup v2 (period above 1 s)")
    quota = int(round(ratio * period))
    return max(MIN_US, quota), period


def systemd_props(ratio=ASSUMED_RATIO):
    q, p = profile(ratio)
    # CPUQuota is a percentage of one CPU per CPUQuotaPeriodSec
    return ["CPUQuota=%.4g%%" % (100.0 * q / p), "CPUQuotaPeriodSec=%gms" % (p / 1000.0)]


def wrap_cmd(cmd, ratio=ASSUMED_RATIO, unit=None):
    w = ["systemd-run", "--user", "--scope", "--quiet", "--collect"]
    if unit:
        w += ["--unit", unit]
    for p in systemd_props(ratio):
        w += ["-p", p]
    return w + ["--"] + list(cmd)


def ratio_of(rpi, vm):
    """ratio of the benchmark scores (geometric means; per-kernel ratios as a check)"""
    k = {n: rpi["kernels"][n] / vm["kernels"][n] for n in rpi["kernels"] if n in vm["kernels"]}
    return rpi["score"] / vm["score"], k


def cgroup_of(pid):
    with open("/proc/%d/cgroup" % pid) as f:
        for line in f:
            if line.startswith("0::"):
                return "/sys/fs/cgroup" + line.strip()[3:]
    return None


def read(p):
    try:
        with open(p) as f:
            return f.read().strip()
    except OSError:
        return None


def status(pid):
    cg = cgroup_of(pid)
    if cg is None:          # cgroup v1 host: no unified hierarchy
        return {"pid": pid, "cgroup": None, "cpu.max": None, "cpu.stat": {}}
    st = {}
    for line in (read(os.path.join(cg, "cpu.stat")) or "").splitlines():
        k, _, v = line.partition(" ")
        if v.isdigit():
            st[k] = int(v)
    return {"pid": pid, "cgroup": cg, "cpu.max": read(os.path.join(cg, "cpu.max")), "cpu.stat": st}


def apply(pid, ratio, cg):
    q, p = profile(ratio)
    os.makedirs(cg, exist_ok=True)
    parent = os.path.dirname(cg)
    sub = read(os.path.join(parent, "cgroup.subtree_control")) or ""
    if "cpu" not in sub.split():
        with open(os.path.join(parent, "cgroup.subtree_control"), "w") as f:
            f.write("+cpu")
    with open(os.path.join(cg, "cpu.max"), "w") as f:
        f.write("%d %d" % (q, p))
    with open(os.path.join(cg, "cgroup.procs"), "w") as f:
        f.write(str(pid))
    return status(pid)


def mechanism():
    """"cgroup" when the cpu controller is delegated to the user (systemd-run --user --scope with CPUQuota
    works), else "user" (build/fdt_cpucap); env FDT_CPU_CAP overrides"""
    m = os.environ.get("FDT_CPU_CAP")
    if m:
        return m
    try:
        return "cgroup" if check_delegation()["cpu_delegated"] else "user"
    except OSError:
        return "user"


def cap_cmd(binary, pids, ratio=ASSUMED_RATIO, stats=None, poll_us=100):
    """command line of the user-space cap (build/fdt_cpucap) of the processes pids at the ratio"""
    q, p = profile(ratio)
    c = [binary, "--quota-us", str(q), "--period-us", str(p), "--poll-us", str(int(poll_us))]
    if stats:
        c += ["--stats", stats]
    return c + [str(int(x)) for x in pids]


def status_user(stats, pid):
    """statistics of pid in the stats file of fdt_cpucap (None when absent)"""
    try:
        with open(stats) as f:
            d = json.load(f)
    except (OSError, ValueError):
        return None
    r = (d.get("procs") or {}).get(str(pid))
    if r is None:
        return None
    return dict(r, pid=pid, mechanism="user", quota_us=d.get("quota_us"), period_us=d.get("period_us"),
                poll_us=d.get("poll_us"), source=d.get("source"))


def check_delegation():
    uid = os.getuid()
    p = "/sys/fs/cgroup/user.slice/user-%d.slice/user@%d.service/cgroup.controllers" % (uid, uid)
    c = read(p)
    return {"path": p, "controllers": c, "cpu_delegated": bool(c and "cpu" in c.split())}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    pr = sub.add_parser("profile")
    pr.add_argument("--ratio", type=float, default=ASSUMED_RATIO)
    w = sub.add_parser("wrap")
    w.add_argument("--ratio", type=float, default=ASSUMED_RATIO)
    w.add_argument("--unit", default=None)
    w.add_argument("--print", action="store_true", help="print the command instead of running it")
    w.add_argument("command", nargs=argparse.REMAINDER)
    c = sub.add_parser("calibrate")
    c.add_argument("rpi")
    c.add_argument("vm")
    k = sub.add_parser("check")
    k.add_argument("rpi")
    k.add_argument("vm_throttled")
    k.add_argument("--tol", type=float, default=0.1)
    a_ = sub.add_parser("apply")
    a_.add_argument("pid", type=int)
    a_.add_argument("--ratio", type=float, default=ASSUMED_RATIO)
    a_.add_argument("--cgroup", default=None)
    s = sub.add_parser("status")
    s.add_argument("pid", type=int)
    sub.add_parser("check-delegation")
    sub.add_parser("mechanism")
    a = ap.parse_args()
    if a.cmd == "profile":
        q, p = profile(a.ratio)
        print(json.dumps({"ratio": a.ratio, "cpu.max": "%d %d" % (q, p), "systemd": systemd_props(a.ratio),
                          "max_stall_us": p - q}))
    elif a.cmd == "wrap":
        cmd = a.command[1:] if a.command[:1] == ["--"] else a.command
        if not cmd:
            ap.error("wrap: no command")
        full = wrap_cmd(cmd, a.ratio, a.unit)
        if a.print:
            print(" ".join(shlex.quote(x) for x in full))
            return 0
        return subprocess.call(full)
    elif a.cmd in ("calibrate", "check"):
        with open(a.rpi) as f:
            rpi = json.load(f)
        with open(a.vm if a.cmd == "calibrate" else a.vm_throttled) as f:
            vm = json.load(f)
        r, per = ratio_of(rpi, vm)
        if a.cmd == "calibrate":
            q, p = profile(min(1.0, r))
            print(json.dumps({"ratio": r, "per_kernel": per, "cpu.max": "%d %d" % (q, p),
                              "systemd": systemd_props(min(1.0, r))}, indent=1))
        else:
            ok = abs(1.0 / r - 1.0) <= a.tol
            print(json.dumps({"throttled_vm_over_rpi4": 1.0 / r, "ok": ok, "tol": a.tol}))
            return 0 if ok else 1
    elif a.cmd == "apply":
        cg = a.cgroup or "/sys/fs/cgroup/fdt/pid-%d" % a.pid
        print(json.dumps(apply(a.pid, a.ratio, cg)))
    elif a.cmd == "status":
        print(json.dumps(status(a.pid)))
    elif a.cmd == "mechanism":
        print(mechanism())
    elif a.cmd == "check-delegation":
        d = check_delegation()
        print(json.dumps(d))
        return 0 if d["cpu_delegated"] else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
