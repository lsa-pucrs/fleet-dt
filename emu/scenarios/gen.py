#!/usr/bin/env python3
"""Channel scenarios S1 to S5 of the paper (Section VI-C) for the emulator, seeded.

  python3 emu/scenarios/gen.py --out emu/scenarios [--M 2 4 8] [--seed 1] [--duration 180]

writes S<k>-m<M>.json (seed 1: the reference files of the repository; the experiment matrix calls make()
with the seed of each run and puts the scenario into the run file). The times of the schedules and of
the waypoints are relative to the start of the first step of the run (emulator option --runStartUs).

| Scenario | Channel |
|---|---|
| S1 clean | every node hears every node (loss 70 dB between any two nodes), ConstantRate 24 Mb/s (ERP-OFDM) |
| S2 external network | S1 plus a second BSS (ap2, sta2) on the same channel; sta2 sends UDP bursts to ap2 (CSMA) with a seeded on/off schedule: on U[1, 6] s at U[2, 8] Mb/s (1400-byte packets), off U[2, 8] s |
| S3 jammer | S1 plus a transmitter without carrier sense (WaveformGenerator, 20 dBm, 70 dB to every node) with a seeded schedule: on U[1, 6] s with period U[0.5, 5] ms and duty cycle U[0.05, 0.3], off U[2, 8] s |
| S4 hidden vessels | S1 with two pairs of vessels that do not hear each other (loss 200 dB: v1-v2 and v3-v4; M = 2: v1-v2 only); every vessel hears the AP |
| S5 mobility | survey mission over a lagoon: AP on the shore at (0, 0, 6 m); each vessel runs a lawnmower pattern (lanes along x, from 100 m to 800 m from the shore, 30 m apart) in its own strip of y in [-400, 400] m at a seeded speed U[1, 2] m/s from a seeded point of its pattern; TwoRayGround (2.412 GHz, antenna heights 6 m and 1.5 m) with Nakagami fading m = 5 (a strong line-of-sight component over water), Minstrel rate adaptation, standard 11a (see emu/README.md: ns-3 Minstrel needs the OFDM rates without DSSS); the PHY rates of the vessels move between 36 and 54 Mb/s with the distance |

The scenario seed also seeds ns-3 (RngSeedManager), so it fixes the backoff draws and the fading.
"""
import argparse
import json
import math
import os
import random

EDCA = {"aifsn_ap": 1, "cwmin_ap": 7, "cwmax_ap": 1023, "aifsn_sta": 3, "cwmin_sta": 15, "cwmax_sta": 1023}
PRIO = {"dscp": {"46": 6, "34": 5}, "tcp_ports": {"1883": 6}}
SCENARIOS = ("S1", "S2", "S3", "S4", "S5")


def base(name, M, seed):
    return {"name": name, "M": M, "seed": int(seed), "run": 1,
            "rate": {"manager": "constant", "data": "ErpOfdmRate24Mbps", "control": "ErpOfdmRate24Mbps",
                     "non_unicast": "ErpOfdmRate6Mbps"},
            "edca": dict(EDCA), "tx_power_dbm": 20, "mac_queue": {"packets": 100, "max_delay_ms": 500},
            "monitor_period_ms": 5, "beacon_tu": 100, "prio": json.loads(json.dumps(PRIO)),
            "channel": {"model": "matrix", "default_loss_db": 70, "loss": []}}


def onoff(rng, duration, on=(1.0, 6.0), off=(2.0, 8.0)):
    """[(t0, t1)] alternating off/on periods over [0, duration), starting with an off period"""
    out, t = [], rng.uniform(*off)
    while t < duration:
        d = rng.uniform(*on)
        out.append((round(t, 3), round(min(duration, t + d), 3)))
        t += d + rng.uniform(*off)
    return out


def lawnmower(rng, i, M, duration, x0=100.0, x1=800.0, ylim=400.0, lane=30.0, z=1.5):
    """waypoints [[t, x, y, z], ...] of vessel i: lanes along x inside strip i of [-ylim, ylim]"""
    w = 2 * ylim / M
    ys = -ylim + (i - 1) * w + lane / 2
    nl = max(1, int(w // lane))
    pts = []
    for k in range(nl):           # boustrophedon over the lanes of the strip, then back
        y = ys + k * lane
        a, b = (x0, x1) if k % 2 == 0 else (x1, x0)
        pts += [(a, y), (b, y)]
    pts += list(reversed(pts))
    seg = [math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1]) for k in range(len(pts) - 1)]
    total = sum(seg)
    v = rng.uniform(1.0, 2.0)
    s = rng.uniform(0, total)       # start point along the closed path
    out, t = [], 0.0
    # walk from arc length s for `duration` seconds (wrapping around)
    k, acc = 0, 0.0
    while acc + seg[k] <= s:
        acc += seg[k]
        k = (k + 1) % len(seg)
    f = (s - acc) / seg[k] if seg[k] > 0 else 0.0
    x = pts[k][0] + f * (pts[k + 1][0] - pts[k][0])
    y = pts[k][1] + f * (pts[k + 1][1] - pts[k][1])
    out.append([0.0, round(x, 2), round(y, 2), z])
    while t < duration:
        nx, ny = pts[k + 1]
        d = math.hypot(nx - x, ny - y)
        t += d / v
        x, y = nx, ny
        out.append([round(t, 3), round(x, 2), round(y, 2), z])
        k = (k + 1) % len(seg)
    return out, v


def make(sid, M, seed, duration=180.0):
    """scenario sid (S1..S5) for M vessels, seeded; times relative to the start of the run"""
    if sid not in SCENARIOS:
        raise ValueError("scenario must be one of %s" % (SCENARIOS,))
    rng = random.Random("%s:%d:%d" % (sid, M, seed))
    sc = base("%s-m%d-s%d" % (sid, M, seed), M, seed)
    sc["scenario_id"] = sid
    if sid == "S2":
        sch = []
        for t0, t1 in onoff(rng, duration):
            sch.append([t0, t1, round(rng.uniform(2e6, 8e6), -3), 1400])
        sc["external_bss"] = {"dir": "up", "data": "ErpOfdmRate24Mbps", "schedule": sch}
    elif sid == "S3":
        sch = []
        for t0, t1 in onoff(rng, duration):
            sch.append([t0, t1, round(rng.uniform(500, 5000)), round(rng.uniform(0.05, 0.3), 3), 20])
        sc["jammer"] = {"schedule": sch}
    elif sid == "S4":
        pairs = [("v1", "v2")] if M < 4 else [("v1", "v2"), ("v3", "v4")]
        sc["channel"]["loss"] = [[a, b, 200] for a, b in pairs]
    elif sid == "S5":
        sc["standard"] = "11a"
        sc["rate"] = {"manager": "minstrel", "non_unicast": "OfdmRate6Mbps"}
        sc["basic_modes"] = ["OfdmRate6Mbps", "OfdmRate12Mbps", "OfdmRate24Mbps"]
        pos = {"gs": [0, 0, 6]}
        speeds = {}
        for i in range(1, M + 1):
            wp, v = lawnmower(rng, i, M, duration)
            pos["v%d" % i] = wp
            speeds["v%d" % i] = round(v, 3)
        sc["channel"] = {"model": "tworay", "frequency_hz": 2.412e9,
                         "nakagami": {"m0": 5, "m1": 5, "m2": 5, "d1": 80, "d2": 200},
                         "positions": pos}
        sc["speeds_mps"] = speeds
    return sc


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--M", type=int, nargs="+", default=[2, 4, 8])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--duration", type=float, default=180.0)
    a = ap.parse_args()
    for M in a.M:
        for sid in SCENARIOS:
            sc = make(sid, M, a.seed, a.duration)
            p = os.path.join(a.out, "%s-m%d.json" % (sid, M))
            with open(p, "w") as f:
                json.dump(sc, f, indent=1)
            print(p)


if __name__ == "__main__":
    main()
