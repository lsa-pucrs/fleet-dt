#!/usr/bin/env python3
"""Campaign generator of the Fleet-DT paper (Section VI, Table tab:exp; Table S6 of the supplementary
material): the run specs (paper stack, emu/fleet.py) and the manifests (emu/experiments/runner.py,
emu/analysis/hypotheses.py) of E0, Cal and E1 to E5. Q stays in emu/experiments/qualify.py.

  python3 emu/experiments/campaign.py cal  --out DIR [options]
      DIR/E0 and DIR/Cal: E0 (26 runs) and Cal (210 runs); no calibration needed
  python3 emu/experiments/campaign.py eval --calibration calibration.json --out DIR [options]
      DIR/E1 .. DIR/E5: E1 (80), E2 (290), E3 (190), E4 (676) and E5 (60 runs of 390 s), with the constants of
      calibration.json (emu/experiments/calibrate_paper.py compute on the Cal runs)
  python3 emu/experiments/campaign.py count
      runs and run-equivalents (150 s) per experiment, and the total with Q (60) and the reruns (85)
  python3 emu/experiments/campaign.py merge --out RESULTS/manifest.json MANIFEST [MANIFEST ...]
      one manifest for emu/analysis/hypotheses.py from the manifests that the runner executed (Cal, E1..E5):
      "dir" of each run = the run directory that the runner recorded; "calibration" of the eval manifests

Order of the sessions (Section VI-E): Q -> E0 -> Cal -> calibrate_paper.py compute -> E1 .. E5 ->
hypotheses.py. Each experiment gets its own manifest (DIR/<exp>/manifest.json, DIR/<exp>/specs/*.json), so
E0 can pass before Cal starts.

Cells, levels and runs per cell (Table S6; M = 4, Fleet-DT and the rho of Table tab:exp unless stated):
  E0   M in {2, 4} x {S1, S3} x 5; M in {2, 4} x outages of 1, 3, 5 s (S1, vessel 1) x 1        26
  Cal  the 4 cells of E1 and the 10 saturated cells of E2 in {S1, S2, S3, S6, S7} x M x 10 (the saturated
       S1 cells belong to both sets and run twice); idle cells of {S2, S3, S6, S7} x M x 3; S5 saturated x
       10 and S5 idle x 3; pilot of E4: 11 methods in S3 x 3                                      210
  E1   S1 x M in {2, 4} x load {idle, saturated} x 20                                               80
  E2   saturated {S1, S2, S3, S6, S7} x M x 20; saturated S4, S8, S9 x 10; MAC queues {q1000, codel} x
       {S1, S3} x 10; S3 with alpha in {alpha*, 1} x 10                                            290
  E3   U in {0.6, 0.7, 0.8, 0.9, 0.95, 1.0, 1.1, 1.2} x {S1, S3} x 10 at rho = 1.5; ablations R7, R8, R9
       in S3 at U* and rho = 1.5 x 10                                                              190
  E4   S3: 11 methods x rho in {1.5, 2} x 20; M = 2: 5 methods x 20; 4 methods x rho in {0.5, 1} x 12;
       fairness: S5, rho = 1.5, Fleet-DT and B6 x 20 (fleet.FAIRNESS_VESSELS); the video source of
       vessel 1 overloads (fleet.PAPER_OVERLOAD) in every cell at rho >= 1.5                         676
  E5   rho = 1.5; O1 to O5 over the regimes S1, S3, S1 (gen.make_paper_sequence, 130 s each) x 12 runs of
       390 s (3120 steps; 2.6 run-equivalents each)                                          60 (156)
Runs last 150 s (1200 steps); the analysis skips the first 30 s.

Blocks: a block holds one seed (the channel draws of the scenario, the video segment and every other seeded
draw of the run) and one session slot (runner.py runs every run of the block on that slot), with the runs of
its cell, or of the cells whose factor is a within-block factor (the U levels of E3, the ablations of E3,
the MAC queues and the alpha levels of E2), in a random order of the block. The seeds of the calibration
(E0, Cal) and of the evaluation (E1 to E5) come from disjoint ranges. The manifest lists the blocks in a
random order of --order-seed (recorded), grouped by M, and gives the blocks the slots 0 .. K - 1 in turn.

Load and rho:
  idle       the telemetry topics only: every vessel without video ("vessels": {"i": {"video": false}})
  saturated  the video at --sat-fps frames per second (8) with the frame cap of the largest trace frame and
             the source scale 1: the demand tables offer about 2.4 (M = 2) and 4.6 (M = 4) times the C0 of
             dtp.json, more than K(t) in every step (no calibration needed; E0 and Cal run before it)
  rho        video_layers.plan_for_rho with C = C_ref of the calibration for the same M and scenario: the
             frame rate f_v and the common scale of the stripe sizes (frame cap, source scale) for which the
             demand tables of the fleet sum to rho C_ref per step; the E4 pilot of Cal uses the C0 of the DTP
             (--C-nominal) in place of C_ref; E5 uses the mean of C_ref over its regimes (S1, S3, S1); a rho
             below the smallest demand of the DTP (the telemetry and one message per stripe topic and step,
             179800 bytes per step at M = 4: rho 0.5 needs C_ref >= 359600) gets that smallest demand and a
             warning ("rho_infeasible" of the manifest), or an error with --rho-infeasible refuse
Calibrated constants (eval): T1, T2, T_m, B_ctl_bar of M, c_y, and the alpha* and U* of the scenario of the
run (fleetdt.calibration.apply_to_dtp, lookup_outer: the fit without that scenario, "all" for S4, S8, S9);
the U of B4, B6 and B7 from the pilot of E4 ("pilot" of calibration.json, fleetdt.calibration.lookup_pilot);
B4 with "as" C_ref of the cell; E5 with the outer-loop variants of fleet.py (O1 and O3 start from (alpha_0,
U_0) = (1/8, 0.8); O2, O4, O5 read DIR/E5/calibration.json).

Every spec passes the checks that emu/fleet.py applies before a run (fleet.paper_check, the per-vessel DTPs,
the AS and feedback commands, fleetdt.as_model.DTP) when the generator writes it.
"""
import argparse
import copy
import hashlib
import json
import os
import random
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
for _p in (ROOT, os.path.join(ROOT, "emu"), os.path.join(ROOT, "emu", "scenarios"),
           os.path.join(ROOT, "vessel", "publishers")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import fleet  # noqa: E402
import gen  # noqa: E402
import video_layers as V  # noqa: E402
from fleetdt import calibration as CAL  # noqa: E402
from fleetdt.as_model import DTP  # noqa: E402

TS = 0.125
STEPS = 1200                       # 150 s
RUN_S = STEPS * TS
WARM_S = 30.0
E5_REGIMES = ("S1", "S3", "S1")
E5_REGIME_S = 130.0
E5_STEPS = int(round(E5_REGIME_S * len(E5_REGIMES) / TS))   # 390 s
N_W = 480
Q_RUNS = 60
RERUNS = 85
TABLE_S6 = {"Q": 60, "E0": 26, "Cal": 210, "E1": 80, "E2": 290, "E3": 190, "E4": 676, "E5": 156, "reruns": 85}
TOTAL = 1773
PHASES = {"cal": ("E0", "Cal"), "eval": ("E1", "E2", "E3", "E4", "E5")}
EXPERIMENTS = PHASES["cal"] + PHASES["eval"]

MS = (2, 4)
EVAL_SCEN = ("S1", "S2", "S3", "S6", "S7")
DESC_SCEN = ("S4", "S8", "S9")
IDLE_SCEN = ("S2", "S3", "S6", "S7")
QUEUES = ("q1000", "codel")
QUEUE_SCEN = ("S1", "S3")
U_LEVELS = (0.6, 0.7, 0.8, 0.9, 0.95, 1.0, 1.1, 1.2)
ABLATIONS = ("R7", "R8", "R9")
E4_METHODS = ("fleetdt", "B1", "B2", "B3", "B4", "B5a", "B5b", "B6", "B7", "B8", "B9")
E4_RHOS = (1.5, 2.0)
E4_LOW_RHOS = (0.5, 1.0)
FAIR_METHODS = ("fleetdt", "B6")
VARIANTS = ("O1", "O2", "O3", "O4", "O5")
PILOT_BASELINES = ("B4", "B6", "B7")
AS_FAMILY = ("fleetdt", "B5a", "B5b")          # Algorithm 1 with eq. (2): alpha* and U* of the DTP

# settings of the run programs, as in the Q specs (emu/experiments/qualify.py)
DTI = {"enabled": True, "offset_ms": 1.0, "stagger_ms": 0.0, "idle": 8}
JMCS = {"period_s": 10, "first": "survey", "video_prefix": "video"}
MONITOR = {"offset_us": 500, "jitter_us": 1000, "quant": "harvest", "harvest_ms": 0, "sigma": 0.0, "backend": "auto"}
JITTER_S = fleet.PAPER_JITTER_MS / 1e3

DEFAULTS = {
    "seed_e0": 8000, "seed_cal": 9000, "seed_eval": 10000,   # disjoint ranges (Q: 7000 + ...)
    "order_seed": 20261021, "K": 3, "steps": STEPS, "e5_steps": E5_STEPS,
    "sat_fps": 8.0, "e0_load": "saturated", "e0_outage_scenario": "S1",
    "s5_M": 4, "single_M": 4,
    "pilot_rho": 1.5, "C_nominal": None,            # None: the C0 of the base DTP
    "pilot_overload": True, "pilot_replay": False,
    "m2_rho": 1.5, "m2_methods": ("fleetdt", "B1", "B3", "B6", "B7"),
    "low_methods": ("fleetdt", "B1", "B3", "B6"),
    "fairness_overload": True, "e5_cref": "mean",
    "alpha0": 0.125, "U0": 0.8, "dtp": "dtp.json", "trace": fleet.PAPER_TRACE,
    "allow_missing": False, "validate": True, "rho_infeasible": "floor",
}


class Opts(dict):
    __getattr__ = dict.__getitem__


def opts(**kw):
    o = Opts(DEFAULTS)
    for k, v in kw.items():
        if k not in DEFAULTS:
            raise KeyError("unknown option %s" % k)
        if v is not None:
            o[k] = v
    return o


# ------------------------------------------------------------------------------------------------ design


def unit(exp, part, sid, M, method="fleetdt", load=None, rho=None, U=None, alpha=None, queue=None, ablation=None,
         fairness=False, outage=None, overload=False, label=None, steps=None):
    """one run of a block (without seed and slot): the method, the factors and the settings of its spec"""
    return {"exp": exp, "part": part, "sid": sid, "M": int(M), "method": method, "label": label or method,
            "load": load, "rho": rho, "U": U, "alpha": alpha, "queue": queue, "ablation": ablation,
            "fairness": bool(fairness), "outage": outage, "overload": bool(overload), "steps": steps}


def cell_id(u):
    """cell: the factor levels of a run without the method and the block"""
    parts = [u["exp"], u["sid"], "m%d" % u["M"]]
    if u["part"] == "pilot":
        parts.append("pilot")
    if u["load"]:
        parts.append(u["load"])
    if u["rho"] is not None:
        parts.append("r%g" % u["rho"])
    if isinstance(u["U"], float):
        parts.append("U%.2f" % u["U"])
    if u["queue"]:
        parts.append(u["queue"])
    if u["alpha"] is not None and u["part"] == "alpha":
        parts.append("alpha%s" % ("*" if u["alpha"] == "alpha*" else "%g" % u["alpha"]))
    if u["ablation"]:
        parts.append(u["ablation"])
    if u["fairness"]:
        parts.append("fair")
    if u["outage"] is not None:
        parts.append("out%gs" % u["outage"])
    return "-".join(parts)


def design(exp, o):
    """[(part, blocks, units of one block)] of an experiment (Table S6)"""
    ov = lambda rho: rho is not None and rho >= 1.5   # noqa: E731  (E4 overload at rho >= 1.5)
    single = int(o.single_M)
    out = []
    if exp == "E0":
        for M in MS:
            for sid in ("S1", "S3"):
                out.append(("main", 5, [unit(exp, "main", sid, M, load=o.e0_load)]))
        for M in MS:
            for d in (1, 3, 5):
                out.append(("outage", 1, [unit(exp, "outage", o.e0_outage_scenario, M, load=o.e0_load, outage=d)]))
    elif exp == "Cal":
        for M in MS:
            for load in ("idle", "saturated"):
                out.append(("E1", 10, [unit(exp, "E1", "S1", M, load=load)]))
        for sid in EVAL_SCEN:
            for M in MS:
                out.append(("E2", 10, [unit(exp, "E2", sid, M, load="saturated")]))
        for sid in IDLE_SCEN:
            for M in MS:
                out.append(("idle", 3, [unit(exp, "idle", sid, M, load="idle")]))
        out.append(("S5", 10, [unit(exp, "S5", "S5", o.s5_M, load="saturated")]))
        out.append(("S5", 3, [unit(exp, "S5", "S5", o.s5_M, load="idle")]))
        out.append(("pilot", 3, [unit(exp, "pilot", "S3", 4, method=m, rho=float(o.pilot_rho),
                                      overload=o.pilot_overload and ov(o.pilot_rho)) for m in E4_METHODS]))
    elif exp == "E1":
        for M in MS:
            for load in ("idle", "saturated"):
                out.append(("main", 20, [unit(exp, "main", "S1", M, load=load, alpha="alpha*")]))
    elif exp == "E2":
        for sid in EVAL_SCEN:
            for M in MS:
                out.append(("main", 20, [unit(exp, "main", sid, M, load="saturated", alpha="alpha*")]))
        for sid in DESC_SCEN:
            out.append(("descriptive", 10, [unit(exp, "descriptive", sid, single, load="saturated", alpha="alpha*")]))
        for sid in QUEUE_SCEN:
            out.append(("queue", 10, [unit(exp, "queue", sid, single, load="saturated", alpha="alpha*", queue=q)
                                      for q in QUEUES]))
        out.append(("alpha", 10, [unit(exp, "alpha", "S3", single, load="saturated", alpha=a)
                                  for a in ("alpha*", 1.0)]))
    elif exp == "E3":
        for sid in ("S1", "S3"):
            out.append(("U", 10, [unit(exp, "U", sid, single, rho=1.5, U=float(u), alpha="alpha*") for u in U_LEVELS]))
        out.append(("ablation", 10, [unit(exp, "ablation", "S3", single, rho=1.5, U="U*", alpha="alpha*", ablation=a)
                                     for a in ABLATIONS]))
    elif exp == "E4":
        for rho in E4_RHOS:
            out.append(("main", 20, [unit(exp, "main", "S3", single, method=m, rho=rho, overload=ov(rho))
                                     for m in E4_METHODS]))
        r2 = float(o.m2_rho)
        out.append(("M2", 20, [unit(exp, "M2", "S3", 2, method=m, rho=r2, overload=ov(r2)) for m in o.m2_methods]))
        for rho in E4_LOW_RHOS:
            out.append(("low", 12, [unit(exp, "low", "S3", single, method=m, rho=rho, overload=ov(rho))
                                    for m in o.low_methods]))
        out.append(("fairness", 20, [unit(exp, "fairness", "S5", single, method=m, rho=1.5, fairness=True,
                                          overload=o.fairness_overload) for m in FAIR_METHODS]))
    elif exp == "E5":
        out.append(("main", 12, [unit(exp, "main", "SEQ", single, method="fleetdt", label=v, rho=1.5,
                                      steps=int(o.e5_steps)) for v in VARIANTS]))
    else:
        raise ValueError("experiment %s: one of %s" % (exp, ", ".join(EXPERIMENTS)))
    return out


def count(o=None):
    """{exp: {"runs", "run_equiv"}} of the design and the totals with Q and the reruns"""
    o = o or opts()
    res = {}
    for exp in EXPERIMENTS:
        n = eq = 0
        for _, nb, us in design(exp, o):
            for u in us:
                n += nb
                eq += nb * (u["steps"] or o.steps) * TS / RUN_S
        res[exp] = {"runs": n, "run_equiv": round(eq, 6)}
    tot = sum(v["run_equiv"] for v in res.values()) + Q_RUNS + RERUNS
    return {"experiments": res, "Q": Q_RUNS, "reruns": RERUNS, "total": round(tot, 6)}


def blocks(exp, o, seed0):
    """blocks of an experiment in design order: [{"seed", "part", "units"}]; seeds seed0, seed0 + 1, ..."""
    out = []
    s = int(seed0)
    for part, nb, us in design(exp, o):
        for _ in range(nb):
            out.append({"seed": s, "part": part, "units": [dict(u) for u in us]})
            s += 1
    return out


def order_blocks(bl, exp, o):
    """blocks in a random order of the order seed, grouped by M (a session keeps its VMs), slots 0..K-1 in turn,
    the runs of each block in a random order seeded by (order seed, experiment, block seed)"""
    rng = random.Random("%d:%s" % (int(o.order_seed), exp))
    bl = list(bl)
    rng.shuffle(bl)
    bl.sort(key=lambda b: b["units"][0]["M"])
    for k, b in enumerate(bl):
        b["slot"] = k % max(1, int(o.K))
        random.Random("%d:%s:%d" % (int(o.order_seed), exp, b["seed"])).shuffle(b["units"])
    return bl


# ------------------------------------------------------------------------------------------------ specs


def load_json(p):
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def trace_max(path):
    with open(os.path.join(ROOT, path)) as f:
        return max(float(line.split()[0]) for line in f if line.strip() and not line.startswith("#"))


def is_descriptive(u):
    """the rule of Section VI-C: S4, S8, S9, the MAC queues, the ablations and the E4 cells at rho <= 1"""
    return bool(u["sid"] in DESC_SCEN or u["queue"] or u["ablation"] or
                (u["exp"] == "E4" and u["rho"] is not None and u["rho"] <= 1.0))


class Context:
    """inputs of the specs of one phase: the base DTP, the trace, the calibration (eval) and the options"""

    def __init__(self, o, cal=None, cal_path=None):
        self.o = o
        self.cal = cal
        self.cal_path = cal_path
        self.base = load_json(os.path.join(ROOT, o.dtp))
        self.frame_max = trace_max(o.trace)
        self.C_nominal = float(o.C_nominal if o.C_nominal is not None else self.base["C0"])
        self._plans = {}
        self.missing = []
        self.infeasible = []

    # calibration ------------------------------------------------------------------------------------
    def C_ref(self, M, sid):
        v = ((self.cal or {}).get("C_ref") or {}).get(str(M), {}).get(sid)
        if v is None:
            raise SystemExit("calibration %s: no C_ref for M = %d, %s (Cal runs of that cell with csub/c_t.jsonl)"
                             % (self.cal_path, M, sid))
        return float(v)

    def pilot_U(self, method, M):
        U, src = CAL.lookup_pilot(self.cal, method, M, "S3")
        if U is None:
            raise SystemExit("calibration %s: no pilot U of %s (calibrate_paper.py compute with the runs of the E4 "
                             "pilot of Cal)" % (self.cal_path, method))
        return float(U)

    def dtp(self, M, sid):
        d = copy.deepcopy(self.base)
        d["vessels"] = list(range(1, M + 1))
        d.pop("calibration", None)
        if self.cal is None:
            return d
        if str(M) not in (self.cal.get("M") or {}):
            raise SystemExit("calibration %s has no M = %d" % (self.cal_path, M))
        out, kept = CAL.apply_to_dtp(d, self.cal, M, None if sid == "SEQ" else sid)
        need = [k for k in kept if k in ("alpha", "U")] if sid != "SEQ" else []
        out["calibration"]["kept"] = list(kept)
        rec = {"M": M, "scenario": sid, "kept": kept}
        if kept and rec not in self.missing:
            self.missing.append(rec)
        if need and not self.o.allow_missing:
            raise SystemExit("calibration %s: no %s for M = %d, %s (--allow-missing keeps the DTP values)"
                             % (self.cal_path, " and ".join(need), M, sid))
        return out

    # video ------------------------------------------------------------------------------------------
    def plan(self, dtp, M, rho, C):
        """video_layers.plan_for_rho; a rho below the smallest demand of the DTP (the telemetry and one message
        per stripe topic and step: 179800 bytes per step at M = 4) gets that smallest demand (frame rate 8,
        frame cap 1 byte) with "floor": true, or SystemExit with --rho-infeasible refuse"""
        key = (hashlib.sha256(json.dumps(dtp, sort_keys=True).encode()).hexdigest(), M, float(rho), float(C))
        if key not in self._plans:
            try:
                p = V.plan_for_rho(dtp, M, float(rho), float(C), self.frame_max, jitter_s=JITTER_S)
                p["floor"] = False
            except ValueError as e:
                if self.o.rho_infeasible == "refuse":
                    raise SystemExit("%s (C = %.0f bytes per step)" % (e, C))
                d = V.with_video(dtp, fps=8.0, frame_cap=1)
                tot = sum(V.vessel_demand(V.resolve_vessel(d, i), JITTER_S) for i in range(1, M + 1))
                p = {"fps": 8.0, "frame_cap": 1, "scale": 1.0 / self.frame_max, "total": tot, "rho": tot / float(C),
                     "dtp": d, "floor": True}
                self.infeasible.append({"M": M, "rho": float(rho), "C": float(C), "rho_floor": tot / float(C)})
            self._plans[key] = p
        return self._plans[key]

    def video(self, u, dtp):
        """(DTP, source, vessels, record) of the load or rho of a run"""
        M = u["M"]
        src = {"trace": self.o.trace, "scale": 1.0, "load_mode": "size"}
        vessels = {}
        if u["fairness"]:
            vessels = {k: dict(v) for k, v in fleet.FAIRNESS_VESSELS.items()}
        if u["load"] == "idle":
            vessels = {str(i): {"video": False} for i in range(1, M + 1)}
            d = fleet.paper_dtp_vessels(dtp, {i: v for i, v in ((int(k), v) for k, v in vessels.items())})
            rec = {"load": "idle", "demand": sum(V.vessel_demand(V.resolve_vessel(d, i), JITTER_S)
                                                for i in range(1, M + 1))}
            return dtp, src, vessels, rec
        if u["load"] == "saturated":
            d = V.with_video(dtp, fps=float(self.o.sat_fps), frame_cap=int(self.frame_max))
            rec = {"load": "saturated", "fps": float(self.o.sat_fps), "frame_cap": int(self.frame_max), "scale": 1.0,
                   "demand": sum(V.vessel_demand(V.resolve_vessel(d, i), JITTER_S) for i in range(1, M + 1))}
            rec["rho_nominal"] = rec["demand"] / self.C_nominal
            return d, src, vessels, rec
        # rho: f_v and the common scale of the stripe sizes for sum d_il = rho C
        if u["part"] == "pilot":
            C, basis = self.C_nominal, "C0 of the DTP"
        elif u["sid"] == "SEQ":
            Cs = [self.C_ref(M, s) for s in E5_REGIMES]
            C, basis = (sum(Cs) / len(Cs), "mean C_ref of the regimes") if self.o.e5_cref == "mean" else \
                (Cs[0], "C_ref of the first regime")
        else:
            C, basis = self.C_ref(M, u["sid"]), "C_ref"
        dv = fleet.paper_dtp_vessels(dtp, {int(k): v for k, v in vessels.items()}) if vessels else dtp
        p = self.plan(dv, M, u["rho"], C)
        src["scale"] = round(float(p["scale"]), 9)
        rec = {"rho": u["rho"], "C": C, "C_basis": basis, "fps": p["fps"], "frame_cap": p["frame_cap"],
               "scale": src["scale"], "demand": p["total"], "rho_plan": p["rho"], "floor": p["floor"]}
        return p["dtp"], src, vessels, rec


def run_id(u, seed):
    return "%s-%s-s%d" % (cell_id(u), u["label"], seed)


def spec_of(ctx, u, seed, slot):
    """(spec, manifest entry) of one run"""
    o = ctx.o
    exp, M, sid, method = u["exp"], u["M"], u["sid"], u["method"]
    steps = int(u["steps"] or o.steps)
    dur = steps * TS + 30.0
    if sid == "SEQ":
        sc = gen.make_paper_sequence(M, seed, E5_REGIMES, E5_REGIME_S)
        dtp = ctx.dtp(M, E5_REGIMES[0])
    else:
        sc = gen.make_paper(sid, M, seed, dur, u["queue"])
        dtp = ctx.dtp(M, sid)
    dtp, src, vessels, vrec = ctx.video(u, dtp)
    rid = run_id(u, seed)
    spec = {"kind": "fleet", "stack": "paper", "name": rid, "exp": exp, "method": method, "M": M, "Ts": TS,
            "seed": seed, "steps": steps, "lead_s": 15, "dtp": dtp, "scenario": sc, "scenario_id": sid,
            "dti": dict(DTI), "jmcs": dict(JMCS), "monitor": dict(MONITOR), "source": src, "rt": "HardLimit",
            "abort_lag_us": fleet.PAPER_ABORT_LAG_US, "session": fleet.paper_session({"Ts": TS})}
    if vessels:
        spec["vessels"] = vessels
    if exp == "E0":
        spec["pcap"] = True
    if u["outage"] is not None:
        spec["outage"] = {"dur_s": float(u["outage"])}
    if exp == "E2" or (exp == "Cal" and (u["part"] != "pilot" or o.pilot_replay)):
        spec["replay_subsets"] = True
    if u["ablation"]:
        spec["ablation"] = u["ablation"]
    if u["overload"]:
        spec["overload"] = {k: fleet.PAPER_OVERLOAD[k] for k in ("vessel", "factor", "fraction")}
    a, fb = {}, {}
    U = alpha = None
    calibrated = ctx.cal is not None
    # the alpha of the DTP is the alpha* of the calibration (the factor level "alpha*" of the driver)
    star = calibrated and "alpha" not in (dtp.get("calibration") or {}).get("kept", ["alpha"])
    if method in AS_FAMILY and u["label"] not in VARIANTS:
        if isinstance(u["U"], float):
            a["U"] = u["U"]
        if isinstance(u["alpha"], float):
            a["alpha"] = u["alpha"]
        U = a.get("U", dtp.get("U"))
        alpha = a.get("alpha", "alpha*" if star else dtp.get("alpha"))
    elif method == "B4":
        a["C_ref"] = ctx.C_ref(M, sid) if calibrated else ctx.C_nominal
        if calibrated:
            a["U"] = ctx.pilot_U("B4", M)
        U, alpha = a.get("U", dtp.get("U")), 0.0
    elif method == "B6":
        if calibrated:
            a["U"] = ctx.pilot_U("B6", M)
        U = a.get("U", dtp.get("U"))
        alpha = "alpha*" if star else dtp.get("alpha")
    elif method == "B7":
        if calibrated:
            fb["U"] = ctx.pilot_U("B7", M)
        U = fb.get("U", dtp.get("U"))
    if a:
        spec["as"] = a
    if fb:
        spec["feedback"] = fb
    regimes = changes = None
    if u["label"] in VARIANTS:
        v = u["label"]
        ou = {"variant": v, "N_w": N_W, "target": float((ctx.cal or {}).get("params", {}).get("PICP_nom", 0.99))}
        if v in ("O1", "O3"):
            ou.update(alpha0=float(o.alpha0), U0=float(o.U0))
        if v in ("O2", "O4", "O5"):
            ou["calibration_file"] = "calibration.json"
        spec["outer"] = ou
        regimes = list(E5_REGIMES)
        changes = [int(round(r["t0"] / TS)) for r in sc["regimes"][1:]]
    factors = {"M": M, "scenario": sid, "rho": u["rho"], "U": U, "load": u["load"], "alpha": alpha,
               "queue": u["queue"], "ablation": u["ablation"], "fairness": u["fairness"], "regimes": regimes,
               "regime_changes": changes}
    block = {"seed": seed, "slot": slot}
    spec["campaign"] = {"exp": exp, "part": u["part"], "cell": cell_id(u), "block": block, "label": u["label"],
                        "factors": factors, "video": vrec, "outage_s": u["outage"]}
    entry = {"id": rid, "spec": "specs/%s.json" % rid, "exp": exp, "part": u["part"], "cell": cell_id(u),
             "block": block, "method": u["label"], "factors": factors, "descriptive": is_descriptive(u),
             "status": "pending", "M": M, "scenario": sid, "seed": seed, "Ts": TS, "steps": steps,
             "run_equiv": round(steps * TS / RUN_S, 6)}
    return spec, entry


def validate(spec, man_dir=None):
    """the checks of emu/fleet.py before a run (fleet.paper_check), the per-vessel DTPs, the commands of the AS
    and of the fleet feedback, the data bridges and the DTP parser of the AS; SystemExit or ValueError on a
    spec that a run refuses"""
    s = copy.deepcopy(spec)
    cf = (s.get("outer") or {}).get("calibration_file")
    if cf and not os.path.isabs(cf) and man_dir:
        s["outer"]["calibration_file"] = os.path.join(man_dir, cf)
    M = int(s["M"])
    r, chk = fleet.paper_check(s, M, ROOT)
    for h in range(1, M + 1):
        DTP.from_dict(fleet.paper_vessel_dtp(r["dtp"], h))
    DTP.from_dict(r["dtp"])
    fleet.paper_as_cmd(r, 0)
    fleet.paper_feedback_cmd(r)
    fleet.paper_bridges(r, M)
    if r["scenario"].get("M") is not None and int(r["scenario"]["M"]) != M:
        raise SystemExit("%s: the scenario has M = %s" % (spec["name"], r["scenario"]["M"]))
    if chk["outer"] is None and (spec.get("outer") or {}).get("variant"):
        raise SystemExit("%s: outer variant not applied" % spec["name"])
    return chk


def write_experiment(ctx, exp, out_dir, seed0, campaign_name=None):
    """DIR/<exp>/specs/*.json and DIR/<exp>/manifest.json; returns (manifest, next seed)"""
    o = ctx.o
    d = os.path.join(out_dir, exp)
    os.makedirs(os.path.join(d, "specs"), exist_ok=True)
    bl = blocks(exp, o, seed0)
    nxt = seed0 + len(bl)
    ctx.missing = []
    if exp == "E5" and ctx.cal_path:
        shutil.copy(ctx.cal_path, os.path.join(d, "calibration.json"))
    runs = []
    for b in order_blocks(bl, exp, o):
        for u in b["units"]:
            spec, entry = spec_of(ctx, u, b["seed"], b["slot"])
            if o.validate:
                validate(spec, d)
            with open(os.path.join(d, entry["spec"]), "w") as f:
                json.dump(spec, f)
            runs.append(entry)
    name = "%s-%s" % (campaign_name or os.path.basename(os.path.abspath(out_dir)), exp)
    man = {"name": name, "cmd": exp, "campaign": "Fleet-DT paper, Table S6", "order_seed": int(o.order_seed),
           "K": int(o.K), "steps": int(o.e5_steps if exp == "E5" else o.steps), "seeds": [seed0, nxt - 1],
           "options": {k: (list(v) if isinstance(v, tuple) else v) for k, v in o.items()},
           "calibration": manifest_calibration(ctx.cal, ctx.cal_path), "runs": runs,
           "note": "run with emu/experiments/runner.py %s --K %d (the runs of a block share the slot "
                   "block.slot mod K)" % (d, int(o.K))}
    if ctx.missing:
        man["calibration_kept"] = ctx.missing
    inf = [x for x in ctx.infeasible if any(r["factors"]["rho"] == x["rho"] and r["M"] == x["M"] and
                                            r["exp"] == exp for r in runs)]
    if inf:
        man["rho_infeasible"] = inf
    with open(os.path.join(d, "manifest.json"), "w") as f:
        json.dump(man, f, indent=1)
    return man, nxt


def manifest_calibration(cal, path):
    """"calibration" of a manifest, the fields that emu/analysis/hypotheses.py reads (None without calibration)"""
    if not cal:
        return None
    Ms = cal.get("M") or {}
    out = {"source": os.path.abspath(path) if path else None,
           "eps_s": {M: g.get("eps") for M, g in Ms.items()}, "T2_s": {M: g.get("T2") for M, g in Ms.items()},
           "PICP_nom": float((cal.get("params") or {}).get("PICP_nom", 0.99)), "N_w": N_W,
           "C_ref": {"%s/%s" % (M, s): v for M, by in (cal.get("C_ref") or {}).items() for s, v in by.items()},
           "c_y": cal.get("c_y"), "pilot_U": {}}
    for m in PILOT_BASELINES:
        U, src = CAL.lookup_pilot(cal, m, 4, "S3")
        out["pilot_U"][m] = U
    return out


def write_phase(phase, out_dir, o, cal_path=None, only=None, campaign_name=None):
    """the manifests of a phase (cal: E0, Cal; eval: E1 to E5); returns {exp: manifest}"""
    cal = None
    if phase == "eval":
        if not cal_path:
            raise SystemExit("eval needs --calibration (emu/experiments/calibrate_paper.py compute on the Cal runs)")
        cal = load_json(cal_path)
    ctx = Context(o, cal, cal_path)
    exps = PHASES[phase]
    seed = int(o.seed_e0 if phase == "cal" else o.seed_eval)
    check_seed_ranges(o)
    out = {}
    for exp in exps:
        if phase == "cal" and exp == "Cal":
            seed = int(o.seed_cal)
        n = sum(nb for _, nb, _ in design(exp, o))
        if only and exp not in only:
            seed += n
            continue
        man, seed = write_experiment(ctx, exp, out_dir, seed, campaign_name)
        out[exp] = man
    return out


def check_seed_ranges(o):
    """E0 < Cal < evaluation seeds, without overlap"""
    n = {exp: sum(nb for _, nb, _ in design(exp, o)) for exp in EXPERIMENTS}
    r = {"E0": (o.seed_e0, o.seed_e0 + n["E0"]), "Cal": (o.seed_cal, o.seed_cal + n["Cal"]),
         "eval": (o.seed_eval, o.seed_eval + sum(n[e] for e in PHASES["eval"]))}
    ks = sorted(r.values())
    for (a0, a1), (b0, b1) in zip(ks, ks[1:]):
        if b0 < a1:
            raise SystemExit("seed ranges overlap: %s" % r)
    return r


# ------------------------------------------------------------------------------------------------ merge


def merge(manifests, out):
    """one manifest of the runs of several manifests for emu/analysis/hypotheses.py: "dir" = the run directory
    of the runner ("run_dir"), "calibration" of the manifests that carry one (they must agree)"""
    runs, cal, names = [], None, []
    for p in manifests:
        m = load_json(p)
        names.append(m.get("name"))
        if m.get("calibration"):
            if cal is not None and cal != m["calibration"]:
                raise SystemExit("%s: another calibration than the manifests before it" % p)
            cal = m["calibration"]
        for r in m.get("runs") or []:
            r = dict(r)
            if r.get("run_dir") and not r.get("dir"):
                r["dir"] = r["run_dir"]
            runs.append(r)
    ids = [r["id"] for r in runs]
    if len(set(ids)) != len(ids):
        raise SystemExit("run ids repeat across the manifests")
    man = {"name": os.path.basename(os.path.dirname(os.path.abspath(out))) or "campaign", "merged": names,
           "calibration": cal, "runs": runs}
    with open(out, "w") as f:
        json.dump(man, f, indent=1)
    return man


# ------------------------------------------------------------------------------------------------ CLI


def _add_opts(p):
    p.add_argument("--out", required=True)
    p.add_argument("--only", nargs="*", default=None, help="a subset of the experiments of the phase")
    p.add_argument("--order-seed", type=int, default=None)
    p.add_argument("--K", type=int, default=None, help="session slots of the blocks (3)")
    p.add_argument("--steps", type=int, default=None, help="steps of a run (1200 = 150 s)")
    p.add_argument("--seed-e0", type=int, default=None)
    p.add_argument("--seed-cal", type=int, default=None)
    p.add_argument("--seed-eval", type=int, default=None)
    p.add_argument("--dtp", default=None, help="base DTP (dtp.json)")
    p.add_argument("--sat-fps", type=float, default=None, help="frame rate of the saturated load (8)")
    p.add_argument("--s5-M", type=int, default=None, help="M of the S5 cells of Cal (4)")
    p.add_argument("--single-M", type=int, default=None, help="M of the cells with one M (4)")
    p.add_argument("--C-nominal", type=float, default=None, help="C of the E4 pilot and of B4 in Cal (C0 of the DTP)")
    p.add_argument("--pilot-rho", type=float, default=None)
    p.add_argument("--m2-methods", nargs="+", default=None)
    p.add_argument("--low-methods", nargs="+", default=None)
    p.add_argument("--m2-rho", type=float, default=None)
    p.add_argument("--no-fairness-overload", action="store_true")
    p.add_argument("--e5-cref", choices=("mean", "first"), default=None)
    p.add_argument("--allow-missing", action="store_true", help="keep the DTP alpha, U where the calibration has none")
    p.add_argument("--rho-infeasible", choices=("floor", "refuse"), default=None,
                   help="a rho below the smallest demand of the DTP: that demand (floor) or an error")
    p.add_argument("--no-validate", action="store_true")


def _opts_of(a):
    kw = {k: getattr(a, k, None) for k in ("order_seed", "K", "steps", "seed_e0", "seed_cal", "seed_eval", "dtp",
                                             "sat_fps", "s5_M", "single_M", "C_nominal", "pilot_rho", "m2_rho",
                                             "e5_cref", "rho_infeasible")}
    if getattr(a, "m2_methods", None):
        kw["m2_methods"] = tuple(a.m2_methods)
    if getattr(a, "low_methods", None):
        kw["low_methods"] = tuple(a.low_methods)
    if getattr(a, "no_fairness_overload", False):
        kw["fairness_overload"] = False
    if getattr(a, "allow_missing", False):
        kw["allow_missing"] = True
    if getattr(a, "no_validate", False):
        kw["validate"] = False
    return opts(**kw)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("cal", help="E0 and Cal")
    _add_opts(c)
    e = sub.add_parser("eval", help="E1 to E5 with calibration.json")
    _add_opts(e)
    e.add_argument("--calibration", required=True)
    sub.add_parser("count", help="runs per experiment")
    m = sub.add_parser("merge", help="one manifest for hypotheses.py")
    m.add_argument("--out", required=True)
    m.add_argument("manifests", nargs="+")
    a = ap.parse_args(argv)
    if a.cmd == "count":
        r = count()
        for exp, v in r["experiments"].items():
            print("%-4s %4d runs  %7.1f run-equivalents  (Table S6: %d)" % (exp, v["runs"], v["run_equiv"],
                                                                         TABLE_S6[exp]))
        print("Q    %4d runs (emu/experiments/qualify.py); reruns %d (5 %%, a budget)" % (r["Q"], r["reruns"]))
        print("total %.1f run-equivalents (Table S6: %d)" % (r["total"], TOTAL))
        return r
    if a.cmd == "merge":
        man = merge(a.manifests, a.out)
        print("%d runs -> %s" % (len(man["runs"]), a.out))
        return man
    o = _opts_of(a)
    if a.only:
        bad = [x for x in a.only if x not in PHASES[a.cmd]]
        if bad:
            raise SystemExit("--only %s: experiments of %s are %s" % (bad, a.cmd, ", ".join(PHASES[a.cmd])))
    mans = write_phase(a.cmd, a.out, o, getattr(a, "calibration", None), a.only)
    for exp, man in mans.items():
        print("%s: %d runs, seeds %d-%d -> %s" % (exp, len(man["runs"]), man["seeds"][0], man["seeds"][1],
                                                  os.path.join(a.out, exp)))
        for x in man.get("rho_infeasible") or []:
            print("  warning: rho %g at M = %d needs C >= the smallest demand of the DTP; C = %.0f gives rho %.3f "
                  "(telemetry and one message per stripe topic)" % (x["rho"], x["M"], x["C"], x["rho_floor"]))
        for x in man.get("calibration_kept") or []:
            print("  warning: M = %d, %s: not calibrated, kept from the DTP: %s" % (x["M"], x["scenario"],
                                                                                 ", ".join(x["kept"])))
    return mans


if __name__ == "__main__":
    main()
