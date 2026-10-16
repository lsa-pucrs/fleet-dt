#!/usr/bin/env python3
"""Validation of the simulation against the emulation (addendum A2, ADDENDUM.md): E6a (simulated) against
E3 and E5 (emulated), cell by cell, and the saturation capacities of E6c against the frozen calibration.

  python3 emu/experiments/addendum/validate_sim.py --sim E6A_RUNS --emu EVAL_RESULTS [--emu B6_RESULTS]
         --sim-cal CAL.json --out DIR

Checks (thresholds of ADDENDUM.md A2, fixed before the E6a runs), per cell (method, scenario) at M = 4 and
rho = 1.5, on the medians over the 12 evaluation seeds:
| Check | Methods | Threshold |
|---|---|---|
| D_N | all | abs(sim - emu) <= 0.05 |
| state miss ratio | all | abs(sim - emu) <= 0.01 |
| mean kappa(t) | all | abs(sim - emu) <= 0.05 |
| f_bar | Fleet-DT, B6 | abs(sim - emu) <= 0.02 |
| nu | Fleet-DT, B6 | abs(sim - emu) <= 0.05 |
| direction | Fleet-DT against B1, B3, B6, TDMA | the sign of the median paired difference (Fleet-DT - baseline) in D_N and in the state miss ratio agrees between simulation and emulation wherever the emulated median difference exceeds 0.01 (D_N) or 0.002 (state miss ratio) in absolute value |
| kappa_sat | S1-m4, S2-m4, S2-m8 | abs(sim - emu) <= 0.03 |
The simulation passes when every check holds; H9 is tested only then.
"""
import argparse
import glob
import json
import os

import numpy as np

KEYS = ("D_N", "state_miss", "mean_kappa", "f_bar", "nu")
THR = {"D_N": 0.05, "state_miss": 0.01, "mean_kappa": 0.05, "f_bar": 0.02, "nu": 0.05}
ADM = ("fleetdt", "b6")
METHODS = ("fleetdt", "b1", "b3", "b6", "tdma")
DIR_THR = {"D_N": 0.01, "state_miss": 0.002}


def val(m, k):
    if k == "f_bar":
        return (m.get("f_bar") or {}).get("value")
    return m.get(k)


def load(dirs, sel):
    out = {}
    for d in dirs:
        for p in glob.glob(os.path.join(d, "*", "metrics.json")):
            m = json.load(open(p))
            if sel(m):
                out[(m["method"], m["scenario"], int(m["seed"]))] = m
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--sim", required=True)
    ap.add_argument("--emu", action="append", required=True)
    ap.add_argument("--sim-cal", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    sel = lambda m: int(m.get("M") or 0) == 4 and float(m.get("rho") or 0) == 1.5 and m["scenario"] in ("S1", "S2") \
        and m["method"] in METHODS
    S = load([a.sim], sel)
    E = load(a.emu, lambda m: sel(m) and m.get("exp") in ("E3", "E5"))
    rows, ok = [], True
    for meth in METHODS:
        for sid in ("S1", "S2"):
            seeds = sorted(s for (mm, ss, s) in S if mm == meth and ss == sid and (mm, ss, s) in E)
            r = {"method": meth, "scenario": sid, "n": len(seeds)}
            for k in KEYS:
                if k in ("f_bar", "nu") and meth not in ADM:
                    continue
                both = [s for s in seeds if val(S[(meth, sid, s)], k) is not None and val(E[(meth, sid, s)], k) is not None]
                xs = [val(S[(meth, sid, s)], k) for s in both]
                xe = [val(E[(meth, sid, s)], k) for s in both]
                if not xs or not xe:
                    r[k] = None
                    ok = False
                    continue
                ms, me = float(np.median(xs)), float(np.median(xe))
                passed = abs(ms - me) <= THR[k]
                r[k] = {"sim": ms, "emu": me, "diff": ms - me, "pass": passed}
                ok = ok and passed
            rows.append(r)
    dirs = []
    for b in ("b1", "b3", "b6", "tdma"):
        for sid in ("S1", "S2"):
            for k in ("D_N", "state_miss"):
                seeds = sorted(s for (mm, ss, s) in S if mm == "fleetdt" and ss == sid and (b, sid, s) in S
                               and ("fleetdt", sid, s) in E and (b, sid, s) in E
                               and None not in (val(S[("fleetdt", sid, s)], k), val(S[(b, sid, s)], k),
                                                val(E[("fleetdt", sid, s)], k), val(E[(b, sid, s)], k)))
                ds = [val(S[("fleetdt", sid, s)], k) - val(S[(b, sid, s)], k) for s in seeds]
                de = [val(E[("fleetdt", sid, s)], k) - val(E[(b, sid, s)], k) for s in seeds]
                if not ds:
                    dirs.append({"baseline": b, "scenario": sid, "metric": k, "pass": False, "n": 0})
                    ok = False
                    continue
                md_s, md_e = float(np.median(ds)), float(np.median(de))
                need = abs(md_e) > DIR_THR[k]
                passed = (not need) or (np.sign(md_s) == np.sign(md_e))
                dirs.append({"baseline": b, "scenario": sid, "metric": k, "sim": md_s, "emu": md_e,
                             "applies": bool(need), "pass": bool(passed), "n": len(seeds)})
                ok = ok and passed
    cal = json.load(open(a.sim_cal))
    sat = []
    for k in ("S1-m4", "S2-m4", "S2-m8"):
        se = (cal.get("sim_sat") or {}).get(k, {}).get("kappa")
        ee = json.load(open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "frozen",
                                         "calibration.json")))["sat"].get(k, {}).get("kappa")
        passed = se is not None and ee is not None and abs(se - ee) <= 0.03
        sat.append({"cell": k, "sim": se, "emu": ee, "pass": passed})
        ok = ok and passed
    res = {"pass": ok, "cells": rows, "direction": dirs, "kappa_sat": sat}
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "validation.json"), "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps({"pass": ok, "failed": [r for r in rows if any(isinstance(v, dict) and not v["pass"] for v in r.values())]
                      + [d for d in dirs if not d["pass"]] + [s for s in sat if not s["pass"]]}, indent=1))


if __name__ == "__main__":
    main()
