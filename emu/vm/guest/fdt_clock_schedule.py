#!/usr/bin/env python3
"""Clock-error schedule of a vessel VM (paper Section VI, E1 and R4; design item I9).

A schedule is a JSON object, generated on the emulation host and read by the injector of the guest
(fdt-clockerr.py), with the times in seconds relative to `t0_unix` (host time of the schedule origin):

  {"t0_unix": T0, "ppm0": f0,
   "freq": [[t_start, t_end, ppm_start, ppm_end], ...],     piecewise linear oscillator error (ppm)
   "ntp_loss": [[t_start, t_end], ...],                       NTP packets dropped in these intervals
   "seed": s, "params": {...}}

The frequency error is that of the oscillator of the vessel computer: positive values make the guest
clock run fast. It holds at the value of the first segment before it and at the value of the last
segment after it. The generator draws holds of U[hold_s] seconds and linear ramps of U[ramp_s] seconds
between the levels f0 - A and f0 + A (A = ramp_ppm, 20 ppm by default: a temperature swing of the crystal
of the vessel computer over minutes), and NTP loss episodes of U[loss_s] seconds, either inside given
windows (the saturated phases of the E1 runs) or anywhere after the warm-up.

  python3 emu/vm/guest/fdt_clock_schedule.py make --seed 7 --duration 900 --ppm0 12.5 --t0 T0 --out s.json
          [--ramp-ppm 20] [--ramp-s 60 300] [--hold-s 30 120] [--loss-n 3] [--loss-s 10 60]
          [--loss-window A B ...] [--warm 60]
  python3 emu/vm/guest/fdt_clock_schedule.py eval s.json T       ppm and NTP state at schedule time T
"""
import argparse
import bisect
import json
import random
import sys

DEFAULTS = {"ramp_ppm": 20.0, "ramp_s": [60.0, 300.0], "hold_s": [30.0, 120.0], "loss_n": 3,
            "loss_s": [10.0, 60.0], "warm_s": 60.0}


def make(seed, duration, ppm0=0.0, t0_unix=0.0, ramp_ppm=20.0, ramp_s=(60.0, 300.0), hold_s=(30.0, 120.0),
         loss_n=3, loss_s=(10.0, 60.0), loss_windows=None, warm_s=60.0):
    """schedule over [0, duration] s (see the module docstring)"""
    if duration <= 0 or ramp_ppm < 0 or ramp_s[0] <= 0 or hold_s[0] < 0 or loss_s[0] <= 0:
        raise ValueError("bad schedule parameters")
    rng = random.Random("clock:%s" % seed)
    levels = (ppm0 - ramp_ppm, ppm0 + ramp_ppm)
    segs = []
    t, f = 0.0, float(ppm0)
    while t < duration:
        h = rng.uniform(*hold_s)
        if h > 0:
            segs.append([round(t, 3), round(t + h, 3), round(f, 4), round(f, 4)])
            t += h
        if t >= duration:
            break
        if f == levels[0]:
            target = levels[1]
        elif f == levels[1]:
            target = levels[0]
        else:
            target = levels[rng.random() < 0.5]
        d = rng.uniform(*ramp_s)
        segs.append([round(t, 3), round(t + d, 3), round(f, 4), round(target, 4)])
        t += d
        f = target
    loss = []
    if loss_windows:
        for a, b in loss_windows:
            if b - a <= 0:
                continue
            L = min(rng.uniform(*loss_s), b - a)
            s = rng.uniform(a, b - L)
            loss.append([round(s, 3), round(s + L, 3)])
    else:
        lo = min(warm_s, duration)
        tries = 0
        while len(loss) < loss_n and tries < 1000:
            tries += 1
            L = rng.uniform(*loss_s)
            if duration - lo <= L:
                break
            s = rng.uniform(lo, duration - L)
            if all(s + L <= a or s >= b for a, b in loss):
                loss.append([round(s, 3), round(s + L, 3)])
        loss.sort()
    return {"t0_unix": float(t0_unix), "ppm0": float(ppm0), "freq": segs, "ntp_loss": loss, "seed": seed,
            "duration": float(duration),
            "params": {"ramp_ppm": ramp_ppm, "ramp_s": list(ramp_s), "hold_s": list(hold_s), "loss_n": loss_n,
                       "loss_s": list(loss_s), "loss_windows": loss_windows, "warm_s": warm_s}}


class Schedule:
    """evaluation of a schedule at time t (s, relative to t0_unix)"""

    def __init__(self, sc):
        self.sc = sc
        self.t0 = float(sc.get("t0_unix", 0.0))
        self.segs = sorted([list(map(float, s)) for s in sc.get("freq", [])])
        self.starts = [s[0] for s in self.segs]
        self.loss = sorted([list(map(float, w)) for w in sc.get("ntp_loss", [])])
        self.ppm0 = float(sc.get("ppm0", 0.0))

    def ppm(self, t):
        if not self.segs:
            return self.ppm0
        if t <= self.segs[0][0]:
            return self.segs[0][2]
        k = bisect.bisect_right(self.starts, t) - 1
        a, b, fa, fb = self.segs[k]
        if t >= b:
            # after the segment: the end value (gaps keep the last value)
            return fb
        return fa + (fb - fa) * (t - a) / (b - a) if b > a else fb

    def ntp_blocked(self, t):
        return any(a <= t < b for a, b in self.loss)

    def at_unix(self, unix):
        t = unix - self.t0
        return self.ppm(t), self.ntp_blocked(t)


def max_slope(sc):
    """largest |d ppm / dt| of the schedule (ppm/s)"""
    m = 0.0
    for a, b, fa, fb in sc["freq"]:
        if b > a:
            m = max(m, abs(fb - fa) / (b - a))
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("make")
    m.add_argument("--seed", default="1")
    m.add_argument("--duration", type=float, required=True)
    m.add_argument("--ppm0", type=float, default=0.0)
    m.add_argument("--t0", type=float, default=0.0, help="host time (unix s) of the schedule origin")
    m.add_argument("--ramp-ppm", type=float, default=DEFAULTS["ramp_ppm"])
    m.add_argument("--ramp-s", type=float, nargs=2, default=DEFAULTS["ramp_s"])
    m.add_argument("--hold-s", type=float, nargs=2, default=DEFAULTS["hold_s"])
    m.add_argument("--loss-n", type=int, default=DEFAULTS["loss_n"])
    m.add_argument("--loss-s", type=float, nargs=2, default=DEFAULTS["loss_s"])
    m.add_argument("--loss-window", type=float, nargs=2, action="append", default=None,
                   help="A B: one NTP loss episode inside [A, B] (repeatable; e.g. the saturated phases)")
    m.add_argument("--warm", type=float, default=DEFAULTS["warm_s"])
    m.add_argument("--out", default="-")
    e = sub.add_parser("eval")
    e.add_argument("schedule")
    e.add_argument("t", type=float)
    a = ap.parse_args()
    if a.cmd == "make":
        sc = make(a.seed, a.duration, a.ppm0, a.t0, a.ramp_ppm, tuple(a.ramp_s), tuple(a.hold_s), a.loss_n,
                  tuple(a.loss_s), a.loss_window, a.warm)
        txt = json.dumps(sc, indent=1)
        if a.out == "-":
            print(txt)
        else:
            with open(a.out, "w") as f:
                f.write(txt + "\n")
        return
    with open(a.schedule) as f:
        s = Schedule(json.load(f))
    print(json.dumps({"t": a.t, "ppm": s.ppm(a.t), "ntp_blocked": s.ntp_blocked(a.t)}))


if __name__ == "__main__":
    sys.exit(main())
