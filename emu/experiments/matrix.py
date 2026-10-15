#!/usr/bin/env python3
"""Experiment matrix of the paper (Section VI, Table exp) as fleet runs of emu/fleet.py.

  python3 emu/experiments/matrix.py sat   --out DIR                   saturation runs (calibration of rho)
  python3 emu/experiments/matrix.py pilot --calib C.json --out DIR    pilot (Section 9 of the task)
  python3 emu/experiments/matrix.py e0    --calib C.json --out DIR    E0 (needs "sat" in C.json)
  python3 emu/experiments/matrix.py eval  --calib C.json --out DIR    E1 to E4 (needs "sat" and "e0")
  python3 emu/experiments/matrix.py b6    --calib C.json --out DIR    E5, addendum: B6 (IPACT) on the cells of E3
  options: --steps 1200 --order-seed 20261015 --only E3 (eval: a subset of the experiments)
           --yans E4 (the cells of these experiments use YansWifiPhy instead of SpectrumWifiPhy; scenarios
           without the jammer only; emu/README.md, "Yans and Spectrum": same kappa and b distributions,
           half the lag in S2)
           --p1-guard-us G (override of the guard g1 = eps + 2 T_tick of the phase 1 offsets i (T1 - g1) / (M + 1))

Each command writes DIR/specs/<id>.json (one fleet experiment per run) and DIR/manifest.json (the runs in
a random order fixed by --order-seed, grouped by M so that a session keeps its VMs; status "pending");
emu/experiments/runner.py executes the manifest.

Table exp (each cell 12 evaluation seeds, 4 calibration seeds; 1200 steps per run):
| Exp | Methods | Scenarios | rho | M | Ts (ms) | Runs |
|---|---|---|---|---|---|---|
| E0 | Fleet-DT | S1-S5 | 1, 1.5, 2 | 4 | 125 | 5 x 3 x 4 = 60 |
| E1 | Fleet-DT | S1-S5 | 1.5 | 4 | 125 | 5 x 12 = 60 |
| E2 | Fleet-DT with eq. (pred), persistence, ES of kappa, calibration mean | S1-S5 | 1.5 | 4 | 125 | 4 x 5 x 12 = 240 |
| E3 | Fleet-DT, slotted, B1, B2, B3, B4, B5, TDMA | S1, S2, S4, S5 | 1, 1.5, 2 | 4 | 125 | 8 x 4 x 3 x 12 = 1152 |
| E4 | Fleet-DT, B1, B3 | S2 | 1.5 | 2 (Ts 125, spectrum); 4 (Ts 62.5, yans) | | 3 x 2 x 12 = 72 |
| E5 (addendum) | B6 (IPACT with limited service) | S1, S2, S4, S5 | 1, 1.5, 2 | 4 | 125 | 4 x 3 x 12 = 144 |

E5 uses the seeds and the run files of the E3 cells (same channel and data), so its runs pair with the E3
runs of the same seed (emu/experiments/addendum/ADDENDUM.md).

A seed fixes the scenario (trajectories, fading and backoff draws of ns-3, interference schedule), the
release instants and sizes of the sources and the schedule of the operation control stub, so every method
of a cell faces the same channel and the same data.

Sources and rho (Section VI-C): the telemetry streams of the DTP and the video topic of the kind output
with the frame sizes of emu/traces/water-sbs-q<Q>-s0p5.txt (scale 0.5: 672x188 side by side). For
rho = 1, 1.5, 2 the quality is 50, 70, 90 and the video rate r solves
  M (c_streams + r Ts c_frame(Q)) = rho kappa_sat T_W,
with c_streams the cost (eq. cost) of the telemetry units of one vessel per step, c_frame(Q) the mean
cost of one frame of the trace, both at the mean PHY rates of the saturation runs, and kappa_sat the mean
kappa(t) of the saturation runs of the scenario and M (B1, video at quality 90 and scale 1 at 15 frames
per second). The DTP of the run declares the video rate r and n = ceil(max frame / (s - 8)).
"""
import argparse
import copy
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "emu", "scenarios"))
import gen as scengen  # noqa: E402
from fleetdt.as2.airtime import Cost, Phy  # noqa: E402
from fleetdt.as2.dtp import DTP, publish_header  # noqa: E402

SCEN = ("S1", "S2", "S3", "S4", "S5")
E3_SCEN = ("S1", "S2", "S4", "S5")
RHOS = (1.0, 1.5, 2.0)
E3_METHODS = ("fleetdt", "slotted", "b1", "b2", "b3", "b4", "b5", "tdma")
PREDICTORS = ("eq", "persist", "es", "calmean")
EVAL_SEEDS = tuple(range(1001, 1013))
CAL_SEEDS = tuple(range(9001, 9005))
SAT_SEEDS = (9101, 9102)
QUALITY = {1.0: 50, 1.5: 70, 2.0: 90}
SCALE = "0p5"
TRACES = "water"     # emu/traces/water-sbs-q<Q>-s<S>.txt (Pohang Canal Dataset, emu/traces/README.md)
SAT_TRACE = "emu/traces/zed-sbs-q90-s1.txt"   # saturation runs of 2026-10-15 (kappa_sat; video far above capacity)
# E4 cells (M, Ts, phy, max_K): only the cells that meet the lag rule (at most 1 % of the steps with a lag
# above 1 ms; emu/README.md, 2026-10-15): M = 2, Ts 125 ms, spectrum, K = 3 (0 %); M = 4, Ts 62.5 ms, yans,
# K = 3 (0.2-0.4 %, 3 runs). M = 8 (yans, K = 1: 100 %) and M = 4 with Ts 250 ms (yans, K = 1: 0.6 % and
# 1.7 %) are evaluated by the what-if prediction alone (WHATIF_ONLY, H6 of emu/analysis/report.py)
E4_CELLS = ((2, 0.125, "spectrum", 3), (4, 0.0625, "yans", 3))
WHATIF_ONLY = ((8, 0.125), (4, 0.25))
SAT_RATE = 15.0
E0_GAMMA = 0.05


def load_json(p):
    with open(p) as f:
        return json.load(f)


def base_dtp():
    return load_json(os.path.join(HERE, "dtp_base.json"))


def trace(path):
    out = []
    with open(os.path.join(ROOT, path)) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                out.append(int(float(line.split()[0])))
    return out


def trace_path(q, scale=SCALE):
    return "emu/traces/%s-sbs-q%d-s%s.txt" % (TRACES, q, scale)


def make_dtp(M, Ts, video_rate, video_n, lam=None, gamma=None):
    d = copy.deepcopy(base_dtp())
    d["vessels"] = list(range(1, M + 1))
    d["Ts"] = Ts
    for tp in d["topics"]:
        if tp["name"] in d["required"]:
            tp["rate"] = round(1.0 / Ts, 6)        # one state per step (Ts = 1 / r_state)
        if tp["kind"] == "output":
            tp["rate"] = round(float(video_rate), 4)
            tp["n"] = int(video_n)
    if lam is not None:
        d["lambda"] = float(lam)
    if gamma is not None:
        d["gamma"] = float(gamma)
    DTP.from_dict(d)       # validation
    return d


def key(sid, M, Ts=0.125):
    return "%s-m%d" % (sid, M) + ("" if abs(Ts - 0.125) < 1e-9 else "-ts%g" % (Ts * 1e3))


def video_n(sizes, s=1400):
    return int(math.ceil(max(sizes) / float(s - 8)))


def costs(d, sizes, Ri, R0):
    """(cost of the telemetry units of one vessel per step, mean cost of one frame) by eq. (cost)"""
    dtp = DTP.from_dict(d)
    c = Cost.from_dtp(dtp)
    cs = 0.0
    for tp in dtp.topics:
        if tp.output:
            continue
        N = tp.rate * dtp.Ts
        cs += c.a(N * tp.n * tp.s, N * tp.n, tp.qos, Ri, R0, tp.h_P, tp.s)
    vt = [tp for tp in dtp.topics if tp.output][0]
    cf = 0.0
    for x in sizes:
        m = int(math.ceil(x / float(vt.s - 8)))
        cf += c.a(x + 8 * m, m, 0, Ri, R0, vt.h_P, vt.s)
    return cs, cf / len(sizes)


def rho_setting(cal, sid, M, Ts, rho):
    """quality, trace, video rate and n that give rho in scenario sid with M vessels and step Ts"""
    sat = cal["sat"].get(key(sid, M)) or cal["sat"][key(sid, M, 0.125)]
    q = QUALITY[float(rho)]
    tp = trace_path(q)
    sizes = trace(tp)
    n = video_n(sizes)
    d = make_dtp(M, Ts, 1.0, n)
    T_W = d["Ts"] - d["T1"] - d["T2"] - 2 * d["eps"] - d["d_rx"]
    cs, cf = costs(d, sizes, sat["R_up"], sat["R_down"])
    target = rho * sat["kappa"] * T_W
    r = (target / M - cs) / (Ts * cf)
    if r <= 0:
        raise SystemExit("rho %g in %s: the telemetry alone exceeds the target" % (rho, key(sid, M, Ts)))
    return {"quality": q, "trace": tp, "rate": round(r, 3), "n": n, "kappa_sat": sat["kappa"], "T_W": T_W,
            "c_streams": cs, "c_frame": cf, "target": target, "R_up": sat["R_up"], "R_down": sat["R_down"]}


def spec(exp, method, sid, M, Ts, rho, seed, steps, cal=None, predictor="eq", sat=False, phy=None):
    sc = scengen.make(sid, M, seed, duration=steps * Ts + 60.0)
    if phy:
        if phy == "yans" and "jammer" in sc:
            raise SystemExit("%s: the jammer needs phy = spectrum" % sid)
        sc["phy"] = phy
    if sat:
        sizes = trace(SAT_TRACE)
        d = make_dtp(M, Ts, SAT_RATE, video_n(sizes))
        src = {"trace": SAT_TRACE}
        rs = {"quality": 90, "scale": 1.0, "rate": SAT_RATE, "trace": SAT_TRACE}
    else:
        rs = rho_setting(cal, sid, M, Ts, rho)
        e0 = (cal or {}).get("e0") if exp != "E0" else None
        # E0 explores with a larger ACI step (gamma 0.05) so that its levels reach their equilibrium and its
        # steps contain failures for the calibration; E1-E4 use the calibrated lambda and gamma
        d = make_dtp(M, Ts, rs["rate"], rs["n"], lam=(e0 or {}).get("lambda"),
                     gamma=(e0 or {}).get("gamma", E0_GAMMA if exp == "E0" else None))
        src = {"trace": rs["trace"]}
    a = {}
    if not sat and exp not in ("E0", "pilot"):
        e0 = cal["e0"]
        # the initial scores (thousands of values) live in one file per scenario and predictor of the
        # campaign (written by main), which the runner resolves and fleet.py copies to the GS
        a["init_scores_file"] = "init/%s-%s.json" % (sid, predictor)
        if method == "b4":
            a["z0"] = e0["z_b4"][sid]
        if predictor == "es":
            a["pred_gain"] = e0["pred_gain"]
        if predictor == "calmean":
            a["cal_kappa"] = e0["cal_kappa"][sid]
    elif method == "b4":
        a["z0"] = (cal or {}).get("e0", {}).get("z_b4", {}).get(sid, 1.0)
    rid = "%s-%s%s-%s-m%d-r%g-ts%g-s%d" % (exp, method, "" if predictor == "eq" else "-" + predictor, sid, M, rho or 0,
                                           Ts * 1e3, seed)
    return {"kind": "fleet", "name": rid, "exp": exp, "method": method, "predictor": predictor, "scenario_id": sid,
            "M": M, "Ts": Ts, "rho": rho, "seed": seed, "steps": steps, "dtp": d, "scenario": sc, "source": src,
            "rho_setting": rs, "as": a, "opctl": {"period": int(round(10.0 / Ts))}, "lead_s": 15, "sync": False}


def cells(cmd, only=None):
    """(exp, method, predictor, scenario, M, Ts, rho, seed) of the runs of a command"""
    out = []
    if cmd == "sat":
        for sid, M in [(s, 4) for s in SCEN] + [("S2", 2), ("S2", 8)]:
            for seed in SAT_SEEDS:
                out.append(("SAT", "b1", "eq", sid, M, 0.125, None, seed))
    elif cmd == "pilot":
        seed = EVAL_SEEDS[0]
        for sid in ("S1", "S2"):
            out.append(("pilot", "fleetdt", "eq", sid, 4, 0.125, 1.5, seed))
        for m in ("b1", "b3", "tdma", "slotted"):
            out.append(("pilot", m, "eq", "S1", 4, 0.125, 1.5, seed))
        for sid in ("S4", "S5"):
            out.append(("pilot", "fleetdt", "eq", sid, 4, 0.125, 1.5, seed))
    elif cmd == "e0":
        for sid in SCEN:
            for rho in RHOS:
                for seed in CAL_SEEDS:
                    out.append(("E0", "fleetdt", "eq", sid, 4, 0.125, rho, seed))
    elif cmd == "b6":
        for seed in EVAL_SEEDS:
            for sid in E3_SCEN:
                for rho in RHOS:
                    out.append(("E5", "b6", "eq", sid, 4, 0.125, rho, seed))
    elif cmd == "eval":
        exps = only or ("E1", "E2", "E3", "E4")
        for seed in EVAL_SEEDS:
            if "E1" in exps:
                for sid in SCEN:
                    out.append(("E1", "fleetdt", "eq", sid, 4, 0.125, 1.5, seed))
            if "E2" in exps:
                for p in PREDICTORS:
                    for sid in SCEN:
                        out.append(("E2", "fleetdt", p, sid, 4, 0.125, 1.5, seed))
            if "E3" in exps:
                for m in E3_METHODS:
                    for sid in E3_SCEN:
                        for rho in RHOS:
                            out.append(("E3", m, "eq", sid, 4, 0.125, rho, seed))
            if "E4" in exps:
                for m in ("fleetdt", "b1", "b3"):
                    for M, Ts, phy, kmax in E4_CELLS:
                        out.append(("E4", m, "eq", "S2", M, Ts, 1.5, seed, {"phy": phy, "max_K": kmax}))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=("sat", "pilot", "e0", "eval", "b6"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--calib", default=None)
    ap.add_argument("--steps", type=int, default=1200)
    ap.add_argument("--order-seed", type=int, default=20261015)
    ap.add_argument("--only", nargs="*", default=None)
    ap.add_argument("--yans", nargs="*", default=[], help="experiments whose cells use phy = yans")
    ap.add_argument("--p1-guard-us", type=int, default=None,
                    help="override of the guard g1 = eps + 2 T_tick of the phase 1 offsets i (T1 - g1) / (M + 1)")
    a = ap.parse_args()
    cal = load_json(a.calib) if a.calib else None
    if a.cmd in ("eval", "b6") and cal and cal.get("e0"):
        os.makedirs(os.path.join(a.out, "init"), exist_ok=True)
        e0 = cal["e0"]
        for sid, preds in e0["init_scores"].items():
            for p, v in preds.items():
                with open(os.path.join(a.out, "init", "%s-%s.json" % (sid, p)), "w") as f:
                    json.dump({"scores": v, "b_hat": e0.get("b_hat0", {}).get(sid, 0.0)}, f)
    if a.cmd != "sat" and not (cal and cal.get("sat")):
        raise SystemExit("%s needs --calib with the saturation results (emu/experiments/calibrate.py sat)" % a.cmd)
    if a.cmd in ("eval", "b6") and not cal.get("e0"):
        raise SystemExit("eval needs the E0 calibration (emu/experiments/calibrate.py e0)")
    steps = 600 if a.cmd == "sat" and a.steps == 1200 else a.steps
    os.makedirs(os.path.join(a.out, "specs"), exist_ok=True)
    runs = []
    for c in cells(a.cmd, a.only):
        exp, m, p, sid, M, Ts, rho, seed = c[:8]
        extra = c[8] if len(c) > 8 else {}
        phy = "yans" if exp in a.yans else extra.get("phy")
        s = spec(exp, m, sid, M, Ts, rho, seed, steps, cal, predictor=p, sat=(a.cmd == "sat"),
                 phy=None if phy == "spectrum" else phy)
        if a.p1_guard_us is not None:
            s["p1_guard_us"] = a.p1_guard_us
        path = os.path.join(a.out, "specs", s["name"] + ".json")
        with open(path, "w") as f:
            json.dump(s, f)
        run = {"id": s["name"], "spec": os.path.relpath(path, a.out), "M": M, "exp": exp, "method": m,
               "predictor": p, "scenario": sid, "rho": rho, "Ts": Ts, "seed": seed, "status": "pending",
               "phy": s["scenario"].get("phy", "spectrum")}
        if extra.get("max_K"):
            run["max_K"] = int(extra["max_K"])
        runs.append(run)
    rng = random.Random(a.order_seed)
    rng.shuffle(runs)
    order = {4: 0, 2: 1, 8: 2}
    # stable: random within each group; the runs with a smaller max_K last (emu/experiments/runner.py)
    runs.sort(key=lambda r: (-int(r.get("max_K") or 99), order.get(r["M"], 9)))
    if a.cmd == "eval" and cal and cal.get("e0"):
        # what-if configurations without a measured counterpart (H6): the DTP of each, from the calibration
        os.makedirs(os.path.join(a.out, "whatif"), exist_ok=True)
        for M, Ts in WHATIF_ONLY:
            rs = rho_setting(cal, "S2", M, Ts, 1.5)
            d = make_dtp(M, Ts, rs["rate"], rs["n"], lam=cal["e0"].get("lambda"), gamma=cal["e0"].get("gamma"))
            with open(os.path.join(a.out, "whatif", "S2-m%d-ts%g.json" % (M, Ts * 1e3)), "w") as f:
                json.dump({"M": M, "Ts": Ts, "scenario": "S2", "rho": 1.5, "dtp": d, "rho_setting": rs}, f, indent=1)
    man = {"name": os.path.basename(os.path.abspath(a.out)), "cmd": a.cmd, "order_seed": a.order_seed,
           "steps": steps, "calib": os.path.abspath(a.calib) if a.calib else None, "runs": runs}
    with open(os.path.join(a.out, "manifest.json"), "w") as f:
        json.dump(man, f, indent=1)
    by = {}
    for r in runs:
        by[r["exp"]] = by.get(r["exp"], 0) + 1
    print("%d runs %s -> %s" % (len(runs), by, a.out))


if __name__ == "__main__":
    main()
