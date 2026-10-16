#!/usr/bin/env python3
"""chrony configuration of the emulated hosts and the warm-up of the runs (paper R4, Section VI
"Sessions"; design item I9).

  python3 emu/vm/chrony_conf.py render gs|vessel [--gs-ip 10.10.0.1] [--minpoll 0] [--maxpoll 2]
          [--initial-step-ms 0]                     the conf.d/fdt.conf of the host (stdout)
  python3 emu/vm/chrony_conf.py convergence clock.csv [--threshold-us 500] [--hold-s 10] [--t0 UNIX]
          [--host v1 ...]                           time from t0 (default: the first sample of each host)
                                                    until |offset| stays below the threshold for hold-s
  python3 emu/vm/chrony_conf.py warmup clock.csv [--ts 0.125] [--steps 240] [...]
                                                    check that the analysis warm-up (steps x Ts) covers the
                                                    largest measured convergence (exit status 1 if not)

Vessel: `server GS iburst minpoll M maxpoll N`, no makestep (the initial offset of fdt-clockinit, up to
+-50 ms, and every later error are slewed: no step of the clock after the start, as R4 requires); with
--initial-step-ms X > 0, `makestep X/1000 1` allows one step at the first update only (still no step
after the start). GS: the host clock through ptp_kvm (`refclock PHC /dev/ptp0`), `makestep 0.1 3`
(steps only in the first three updates, at boot, before any run), `allow` the data subnet and
`local stratum 10`. The main /etc/chrony/chrony.conf of the image (emu/vm/guest/chrony.conf) includes
this file and logs tracking, measurements and statistics. The rendering has the directives that
emu/run.py writes into the cloud-init seed today (with --initial-step-ms 0).

clock.csv: host_ns,host,offset_ns,rtt_ns (emu/clockprobe/probe.py).
"""
import argparse
import csv
import math
import sys


def render(role, gs_ip="10.10.0.1", minpoll=0, maxpoll=2, subnet="10.10.0.0/24", initial_step_ms=0.0):
    lines = []
    if role == "gs":
        lines += ["# GS: follows the clock of the emulation host (ptp_kvm) and serves the vessels",
                  "refclock PHC /dev/ptp0 poll -2 dpoll -2",
                  "makestep 0.1 3", "allow %s" % subnet, "local stratum 10"]
    elif role == "vessel":
        lines += ["# vessel: follows the GS; no step of the clock after the start (PR4), offsets are slewed",
                  "server %s iburst minpoll %d maxpoll %d" % (gs_ip, minpoll, maxpoll)]
        if initial_step_ms and initial_step_ms > 0:
            lines.append("makestep %g 1" % (initial_step_ms / 1000.0))
    else:
        raise ValueError("role must be gs or vessel")
    return "\n".join(lines) + "\n"


def read_clock(path):
    """{host: [(t_s, offset_us)]} sorted by time"""
    out = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                t = int(r["host_ns"]) / 1e9
                off = int(r["offset_ns"]) / 1e3
            except (KeyError, ValueError):
                continue
            out.setdefault(r["host"], []).append((t, off))
    for v in out.values():
        v.sort()
    return out


def convergence(samples, threshold_us=500.0, hold_s=10.0, t0=None):
    """seconds from t0 (default: the first sample) to the first time after which |offset| stays below the
    threshold for at least hold_s (and up to the end, if the record is shorter); None when never"""
    if not samples:
        return None
    t0 = samples[0][0] if t0 is None else t0
    s = [(t, o) for t, o in samples if t >= t0]
    start = None
    for t, o in s:
        if abs(o) < threshold_us:
            if start is None:
                start = t
        else:
            start = None
        if start is not None and t - start >= hold_s:
            return start - t0
    if start is not None and s and s[-1][0] - start >= min(hold_s, s[-1][0] - s[0][0]) * 0.999:
        return start - t0
    return None


def warmup_ok(conv_s, ts=0.125, steps=240):
    """(ok, warm-up seconds, steps needed): the warm-up of the analysis must be at least the convergence"""
    w = steps * ts
    need = int(math.ceil(conv_s / ts)) if conv_s is not None else None
    return conv_s is not None and w >= conv_s, w, need


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("render")
    r.add_argument("role", choices=("gs", "vessel"))
    r.add_argument("--gs-ip", default="10.10.0.1")
    r.add_argument("--minpoll", type=int, default=0)
    r.add_argument("--maxpoll", type=int, default=2)
    r.add_argument("--subnet", default="10.10.0.0/24")
    r.add_argument("--initial-step-ms", type=float, default=0.0)
    for name in ("convergence", "warmup"):
        c = sub.add_parser(name)
        c.add_argument("csv")
        c.add_argument("--threshold-us", type=float, default=500.0)
        c.add_argument("--hold-s", type=float, default=10.0)
        c.add_argument("--t0", type=float, default=None)
        c.add_argument("--host", nargs="*", default=None)
        if name == "warmup":
            c.add_argument("--ts", type=float, default=0.125)
            c.add_argument("--steps", type=int, default=240)
    a = ap.parse_args()
    if a.cmd == "render":
        sys.stdout.write(render(a.role, a.gs_ip, a.minpoll, a.maxpoll, a.subnet, a.initial_step_ms))
        return 0
    data = read_clock(a.csv)
    hosts = a.host or sorted(h for h in data if h != "gs")
    conv = {h: convergence(data.get(h, []), a.threshold_us, a.hold_s, a.t0) for h in hosts}
    for h in hosts:
        print("%s convergence %s s" % (h, "never" if conv[h] is None else "%.2f" % conv[h]))
    if a.cmd == "warmup":
        worst = None if any(v is None for v in conv.values()) else max(conv.values() or [0.0])
        ok, w, need = warmup_ok(worst, a.ts, a.steps)
        print("warm-up %d steps = %.1f s; largest convergence %s s; %s" % (
            a.steps, w, "never" if worst is None else "%.2f" % worst,
            "ok" if ok else "NOT ok (need at least %s steps)" % need))
        return 0 if ok else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
