#!/usr/bin/env python3
"""C(t) of a recorded run by offline replays of its scenario with every nonempty set of saturated
vessel radios (paper Section III: "The saturation throughput C(t) > 0 of step t is the smallest number of
bytes of IP packets, with h_W bytes per frame, that the channel carries in step t among the nonempty sets
of saturated radios"; reference of H2 in Section VI).

  python3 emu/replay_subsets.py RUN_DIR [--out RUN_DIR/csub] [--jobs 2] [--size 1500] [--rate-mbps 30]
  python3 emu/replay_subsets.py --scenario S.json --dtp dtp.json --steps N --out DIR [--outq-dir RUN_DIR]

For a run directory of fleet.py (paper stack) the tool reads RUN/scenario.json (the seeded scenario of
the run: interference schedules, trajectories, motion, walk and the ns-3 seed), RUN/dtp.json (Ts, T1, T2,
h_W), RUN/result.json (S0 and steps) and, when present, the vessel sampler logs RUN/vms/v<i>/outq.jsonl
(emu/tools/outq_sampler.c). For each nonempty set S of the M vessels (2^M - 1 sets, 15 at M = 4) it runs
the emulator offline (not in real time, --rt=none --vms=0) on the same scenario with a saturating UDP
source (`--size` bytes of IP packets at `--rate-mbps` per vessel, above the capacity of the channel) on
each vessel of S only, with the scenario time 0 (start of step S0) at simulator time `--warm-s`, and
--stepBytes: per step and vessel, the unique IP packets that the AP receives without error. Then

  C_S(k) = sum_{i in S} (bytes3_i(k) + h_W frames3_i(k)) * Ts / T3       (phase 3 of step k)
  C(k)   = min_S C_S(k)

and, from the sampler logs, the saturation decision of the run in each step (outq.saturation: every
vessel with forwarded bytes keeps unsent bytes in its data bridge through phase 3).

Outputs in OUT: subset-<mask>/ (stepbytes.csv, summary.json, emu.out of each replay), c_t.jsonl (one
line per step: k, t = S0 + k, C, argmin set, C_S of every set, saturated, sat_vessels), replay.json
(configuration, binary, scenario sha256, run times, the share of steps in which each set is the minimum).

The replays share the interference schedules, the trajectories, the walks and the antenna motion of the
run (they are explicit in the scenario); the backoff and fading draws differ between the sets and from
the live run (the random streams of ns-3 follow the events).
"""
import argparse
import concurrent.futures as cf
import csv
import hashlib
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "tools"))
import outq  # noqa: E402

NS3 = os.path.expanduser(os.environ.get("FDT_NS3", "~/fleetdt-deploy/ns3/ns-3.48"))
EMU_BIN = os.environ.get("FDT_EMU_BIN", os.path.join(NS3, "build/scratch/fdt-emu/ns3.48-fdt-emu-optimized"))


def subsets(M):
    """nonempty subsets of 1..M as sorted tuples, in the order of their bit mask"""
    return [tuple(i for i in range(1, M + 1) if mask >> (i - 1) & 1) for mask in range(1, 1 << M)]


def mask_of(s):
    return sum(1 << (i - 1) for i in s)


def replay_cmd(binary, scenario, outdir, S, steps, Ts, T1, T2, warm, size, rate_bps):
    dur = warm + steps * Ts + 0.01
    return [binary, "--scenario=%s" % scenario, "--outdir=%s" % outdir, "--vms=0", "--rt=none",
            "--monitor=none", "--gt=0", "--selftest=%d,%d" % (int(rate_bps), int(size)),
            "--selftestSet=%s" % ",".join(str(i) for i in S), "--shiftS=%.6f" % warm,
            "--duration=%.6f" % dur,
            "--stepBytes=%d:%d" % (int(round(Ts * 1e6)), int(round((T1 + T2) * 1e6)))]


def read_stepbytes(path):
    """{(k, i): (frames3, bytes3, frames, bytes)}"""
    out = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            out[(int(r["k"]), int(r["i"]))] = (int(r["frames3"]), int(r["bytes3"]), int(r["frames"]), int(r["bytes"]))
    return out


def c_subset(sb, S, steps, Ts, T1, T2, hW):
    """C_S(k), k = 0..steps-1 (bytes per Ts, phase 3 scaled by Ts / T3)"""
    T3 = Ts - T1 - T2
    out = []
    for k in range(steps):
        b = 0.0
        for i in S:
            fr3, by3, _, _ = sb.get((k, i), (0, 0, 0, 0))
            b += by3 + hW * fr3
        out.append(b * Ts / T3)
    return out


def combine(cs, steps):
    """per step: (C, argmin set) over the sets; cs = {set: [C_S(k)]}"""
    out = []
    for k in range(steps):
        best = min(cs, key=lambda S: (cs[S][k], len(S), S))
        out.append((cs[best][k], best))
    return out


def run_one(args):
    cmd, od = args
    os.makedirs(od, exist_ok=True)
    t0 = time.time()
    with open(os.path.join(od, "emu.out"), "w") as f:
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    return r.returncode, time.time() - t0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir", nargs="?")
    ap.add_argument("--scenario")
    ap.add_argument("--dtp")
    ap.add_argument("--steps", type=int)
    ap.add_argument("--S0", type=int, default=None, help="step index of scenario time 0 (default: result.json, else 0)")
    ap.add_argument("--outq-dir", default=None, help="run directory with vms/v<i>/outq.jsonl (default: run_dir)")
    ap.add_argument("--out")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--size", type=int, default=1500, help="IP bytes of the saturating packets (MSS 1448 + 52)")
    ap.add_argument("--rate-mbps", type=float, default=30.0, help="offered rate per saturated vessel")
    ap.add_argument("--warm-s", type=float, default=2.0, help="simulator time of the scenario time 0 (association)")
    ap.add_argument("--hW", type=float, default=None, help="h_W (default: the DTP)")
    ap.add_argument("--emu", default=EMU_BIN)
    ap.add_argument("--keep", action="store_true", help="reuse the replays already in OUT")
    a = ap.parse_args()

    rd = a.run_dir
    scen = a.scenario or (os.path.join(rd, "scenario.json") if rd else None)
    dtpp = a.dtp or (os.path.join(rd, "dtp.json") if rd else None)
    if not scen or not dtpp:
        ap.error("give RUN_DIR or --scenario and --dtp")
    with open(scen) as f:
        sc = json.load(f)
    with open(dtpp) as f:
        dtp = json.load(f)
    res = {}
    if rd and os.path.exists(os.path.join(rd, "result.json")):
        with open(os.path.join(rd, "result.json")) as f:
            res = json.load(f)
    steps = a.steps or int(res.get("steps") or 0)
    if steps <= 0:
        ap.error("number of steps unknown (--steps)")
    S0 = a.S0 if a.S0 is not None else int(res.get("S0") or 0)
    M = int(sc["M"])
    Ts, T1, T2 = float(dtp["Ts"]), float(dtp["T1"]), float(dtp["T2"])
    hW = float(a.hW if a.hW is not None else dtp.get("h_W", 0.0))
    out = a.out or os.path.join(rd, "csub")
    os.makedirs(out, exist_ok=True)
    sets = subsets(M)
    jobs = []
    for S in sets:
        od = os.path.join(out, "subset-%d" % mask_of(S))
        if a.keep and os.path.exists(os.path.join(od, "stepbytes.csv")):
            continue
        jobs.append((replay_cmd(a.emu, os.path.abspath(scen), od, S, steps, Ts, T1, T2, a.warm_s, a.size,
                                a.rate_mbps * 1e6), od))
    t0 = time.time()
    times = {}
    with cf.ThreadPoolExecutor(max_workers=max(1, a.jobs)) as ex:
        for (cmd, od), (rc, dt) in zip(jobs, ex.map(run_one, jobs)):
            times[os.path.basename(od)] = round(dt, 1)
            if rc != 0:
                sys.exit("replay failed (%d): %s; see %s/emu.out" % (rc, " ".join(cmd), od))
    cs = {}
    for S in sets:
        sb = read_stepbytes(os.path.join(out, "subset-%d" % mask_of(S), "stepbytes.csv"))
        cs[S] = c_subset(sb, S, steps, Ts, T1, T2, hW)
    comb = combine(cs, steps)
    logs = {}
    src = a.outq_dir or rd
    if src:
        logs = {i: outq.load(p) for i, p in outq.vessel_logs(src).items()}
    sat = outq.saturation(logs) if len(logs) == M else {}
    argmin_share = {}
    with open(os.path.join(out, "c_t.jsonl"), "w") as f:
        for k, (C, S) in enumerate(comb):
            t = S0 + k
            s = sat.get(t)
            argmin_share[mask_of(S)] = argmin_share.get(mask_of(S), 0) + 1
            f.write(json.dumps({"k": k, "t": t, "C": round(C, 1), "argmin": list(S),
                                "C_S": {str(mask_of(X)): round(cs[X][k], 1) for X in sets},
                                "saturated": None if s is None else s["saturated"],
                                "sat_vessels": None if s is None else s["sat_vessels"]}) + "\n")
    with open(scen, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    try:
        with open(a.emu, "rb") as f:
            bsha = hashlib.sha256(f.read()).hexdigest()
    except OSError:
        bsha = None
    rep = {"scenario": os.path.abspath(scen), "scenario_sha256": sha, "emu": a.emu, "emu_sha256": bsha, "M": M,
           "steps": steps, "S0": S0, "Ts": Ts, "T1": T1, "T2": T2, "h_W": hW, "size": a.size,
           "rate_mbps": a.rate_mbps, "warm_s": a.warm_s, "sets": [list(S) for S in sets],
           "replay_s": times, "wall_s": round(time.time() - t0, 1), "outq_vessels": sorted(logs),
           "saturated_steps": sum(1 for v in sat.values() if v["saturated"]) if sat else None,
           "argmin_share": {str(k): v / steps for k, v in sorted(argmin_share.items())},
           "C_mean": sum(c for c, _ in comb) / steps, "C_min": min(c for c, _ in comb),
           "zero_steps": sum(1 for c, _ in comb if c <= 0)}
    with open(os.path.join(out, "replay.json"), "w") as f:
        json.dump(rep, f, indent=1)
    print("C(t): %d steps, mean %.0f B/step (%.2f Mb/s), min %.0f; argmin shares %s; %s" % (
        steps, rep["C_mean"], rep["C_mean"] * 8 / Ts / 1e6, rep["C_min"], rep["argmin_share"],
        "saturated steps %s" % rep["saturated_steps"] if sat else "no sampler logs"))


if __name__ == "__main__":
    main()
