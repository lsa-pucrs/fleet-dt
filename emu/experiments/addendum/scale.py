#!/usr/bin/env python3
"""Run files of the simulation experiments of addendum A2 (ADDENDUM.md): the calibration of the
saturation capacity (E6c), the validation against the emulation at M = 4 (E6a), and the scale study
(E6b), for emu/sim/simcampaign.py.

  python3 emu/experiments/addendum/scale.py sat   --out DIR
  python3 emu/experiments/addendum/scale.py calib --sat-runs DIR/runs --out CAL.json
  python3 emu/experiments/addendum/scale.py e6a   --b6 B6_CAMPAIGN --out DIR
  python3 emu/experiments/addendum/scale.py e6b   --calib CAL.json --out DIR
  python3 emu/experiments/addendum/scale.py pilot --calib CAL.json --out DIR

E6c: B1 at saturation (the saturation run files of matrix.py), S1 and S2, M in {4, 8, 16, 32}, the two
saturation seeds, 600 steps; calib writes the frozen calibration with the "sat" entries of the simulation
for M >= 8 (key "sim_sat" holds every simulated value, M = 4 included, for the validation).
E6a: the frozen E3 run files of S1 and S2 at rho = 1.5 of Fleet-DT, B1, B3 and TDMA, and the E5 run files
of B6 for the same cells (12 evaluation seeds each), renamed E6a-<run>.
E6b: Fleet-DT, B1, B3, B6, TDMA; S1, S2; M = 8; the phases of Table param; the 12 evaluation seeds; every
vessel keeps the sources of the M = 4 runs at rho = 1.5, so the offered load grows with M (spec_fixed_load).
E6d (descriptive, in the same command): Fleet-DT and B1, S2, M in {16, 32}, the phases of Table param.
pilot2: the pilot of Fleet-DT with T1 = 40 ms and T2 = 30 ms (phase timing).
pilot: Fleet-DT and B1, S2, M in {8, 16, 32}, seed 7001 (outside the evaluation seeds), 400 steps.
"""
import argparse
import copy
import glob
import json
import os
import shutil
import sys
import tarfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(ROOT, "emu", "experiments"))
import matrix as MX  # noqa: E402

FROZEN = os.path.join(ROOT, "emu", "experiments", "frozen")
SCEN = ("S1", "S2")
MS = (8, 16, 32)          # pilots
E6B_MS = (8,)             # E6b (H9)
E6D_MS = (16, 32)         # E6d (descriptive)
E6_METHODS = ("fleetdt", "b1", "b3", "b6", "tdma")
E6A_METHODS = ("fleetdt", "b1", "b3", "tdma")
PILOT_SEED = 7001


def write(out, s, name=None):
    os.makedirs(os.path.join(out, "specs"), exist_ok=True)
    name = name or s["name"]
    s = dict(s, name=name)
    with open(os.path.join(out, "specs", name + ".json"), "w") as f:
        json.dump(s, f)
    return name


def spec_fixed_load(exp, method, sid, M, seed, steps, cal, T1=None, T2=None):
    """Run file of the scale study: every vessel keeps the sources of the M = 4 runs at rho = 1.5 (video
    quality and rate of the frozen calibration), so the offered load grows with M; rho_effective is the
    offered airtime cost of the fleet over kappa_sat(M) T_W (simulated saturation, E6c)."""
    s4 = MX.spec(exp, method, sid, 4, 0.125, 1.5, seed, steps, cal)
    rs = dict(s4["rho_setting"])
    e0 = cal["e0"]
    s = copy.deepcopy(s4)
    s["M"] = M
    s["scenario"] = MX.scengen.make(sid, M, seed, duration=steps * 0.125 + 60.0)
    s["dtp"] = MX.make_dtp(M, 0.125, rs["rate"], rs["n"], lam=e0.get("lambda"), gamma=e0.get("gamma"))
    if T1 is not None:
        s["dtp"]["T1"] = T1
    if T2 is not None:
        s["dtp"]["T2"] = T2
    sat = cal["sat"][MX.key(sid, M)]
    d = s["dtp"]
    T_W = d["Ts"] - d["T1"] - d["T2"] - 2 * d["eps"] - d["d_rx"]
    sizes = MX.trace(rs["trace"])
    cs, cf = MX.costs(d, sizes, sat["R_up"], sat["R_down"])
    rs.update({"M4_rho": 1.5, "kappa_sat_M": sat["kappa"], "c_streams_M": cs, "c_frame_M": cf,
               "rho_effective": M * (cs + rs["rate"] * d["Ts"] * cf) / (sat["kappa"] * T_W)})
    s["rho_setting"] = rs
    s["name"] = "%s-%s-%s-m%d-r1.5-ts125-s%d" % (exp, method, sid, M, seed)
    return s


def copy_init(out):
    d = os.path.join(out, "init")
    os.makedirs(d, exist_ok=True)
    for p in glob.glob(os.path.join(FROZEN, "init", "*.json")):
        shutil.copy(p, d)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=("sat", "calib", "e6a", "e6b", "pilot", "pilot2"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--calib", default=None)
    ap.add_argument("--sat-runs", default=None)
    ap.add_argument("--b6", default=None, help="campaign directory of matrix.py b6 (E5 run files)")
    a = ap.parse_args()
    cal0 = json.load(open(os.path.join(FROZEN, "calibration.json")))
    if a.cmd == "sat":
        n = 0
        for sid in SCEN:
            for M in (4,) + MS:
                for seed in MX.SAT_SEEDS:
                    write(a.out, MX.spec("SAT", "b1", sid, M, 0.125, None, seed, 600, cal0, sat=True))
                    n += 1
        print("%d saturation run files -> %s" % (n, a.out))
    elif a.cmd == "calib":
        sys.path.insert(0, os.path.join(ROOT, "emu", "experiments"))
        import calibrate as C  # noqa: E402
        tmp = a.out + ".sat.json"
        with open(tmp, "w") as f:
            json.dump({}, f)
        ns = argparse.Namespace(results=a.sat_runs, calib=tmp)
        C.cmd_sat(ns)
        sim = json.load(open(tmp))["sat"]
        cal = copy.deepcopy(cal0)
        cal["sim_sat"] = sim
        for k, v in sim.items():
            if not k.endswith("-m4"):
                cal["sat"][k] = v       # the simulated value replaces an emulated one (S2-m8) for E6b
        cal["note"] = "frozen calibration of E1-E4; sat entries of M >= 8 from the simulation (E6c, addendum A2)"
        with open(a.out, "w") as f:
            json.dump(cal, f, indent=1)
        print(json.dumps({k: round(v["kappa"], 4) for k, v in sim.items()}))
    elif a.cmd == "e6a":
        copy_init(a.out)
        n = 0
        with tarfile.open(os.path.join(FROZEN, "specs.tar.gz")) as t:
            for m in t.getmembers():
                b = os.path.basename(m.name)
                for meth in E6A_METHODS:
                    for sid in SCEN:
                        if b.startswith("E3-%s-%s-m4-r1.5-" % (meth, sid)):
                            s = json.load(t.extractfile(m))
                            s["exp"] = "E6a"
                            write(a.out, s, "E6a-" + s["name"])
                            n += 1
        for p in sorted(glob.glob(os.path.join(a.b6, "specs", "E5-b6-*-m4-r1.5-*.json"))):
            s = json.load(open(p))
            if s["scenario_id"] in SCEN:
                s["exp"] = "E6a"
                write(a.out, s, "E6a-" + s["name"])
                n += 1
        print("%d E6a run files -> %s" % (n, a.out))
    elif a.cmd == "pilot2":
        # phase timing at M = 8, 16, 32 with wide phases (T1 = 40 ms, T2 = 30 ms): the arrival times of the
        # demands and of the tables fix the rule T1(M), T2(M) of E6b
        cal = json.load(open(a.calib))
        copy_init(a.out)
        for M in MS:
            write(a.out, spec_fixed_load("PILOT2", "fleetdt", "S2", M, PILOT_SEED, 400, cal, T1=0.040, T2=0.030))
        print("3 run files -> %s" % a.out)
    elif a.cmd in ("e6b", "pilot"):
        cal = json.load(open(a.calib))
        copy_init(a.out)
        n = 0
        if a.cmd == "pilot":
            for M in MS:
                for meth in ("fleetdt", "b1"):
                    write(a.out, spec_fixed_load("PILOT", meth, "S2", M, PILOT_SEED, 400, cal))
                    n += 1
        else:
            for seed in MX.EVAL_SEEDS:
                for sid in SCEN:
                    for M in E6B_MS:
                        for meth in E6_METHODS:
                            write(a.out, spec_fixed_load("E6b", meth, sid, M, seed, 1200, cal))
                            n += 1
                for M in E6D_MS:
                    for meth in ("fleetdt", "b1"):
                        write(a.out, spec_fixed_load("E6d", meth, "S2", M, seed, 1200, cal))
                        n += 1
        print("%d run files -> %s" % (n, a.out))


if __name__ == "__main__":
    main()
