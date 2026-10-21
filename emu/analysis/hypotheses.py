#!/usr/bin/env python3
"""Hypothesis-level driver of the Fleet-DT paper (Section VI, Table of hypotheses): every component H1a to H6b
from the per-run metrics of a campaign, the intersection-union test (IUT) of each hypothesis over its tested
components and cells, and the fixed sequence H1 -> H2 -> H3 -> H4 -> H5 -> H6 (the first failure ends it;
descriptive components stay out).

  python3 emu/analysis/hypotheses.py CAMPAIGN [--manifest CAMPAIGN/manifest.json] [--out DIR] [--B 2000]
          [--seed 0] [--alpha 0.05] [--burn 50] [--arima-engine auto|statsmodels|css]
          [--dmr-hat-scenarios same|loso] [--keep-invalid]

Inputs: CAMPAIGN/manifest.json (fields in emu/README.md, "Hypothesis driver") and, for each run of the manifest,
CAMPAIGN/<dir>/metrics.json of emu/analysis/paper_post.py (run metrics and per-step series); the backlog
recursion of H3b also reads the DTA of the calibration runs (CAMPAIGN/<dir>/vms/gs/dta.jsonl and dtp.json).
Outputs: DIR/hypotheses.json (every component and cell, the hypotheses and the sequence) and DIR/hypotheses.md
(one row per component: ID, estimate, bound or p-value, threshold, verdict; Table tab:res of Section VII).

Statistics (Section VI-E and Table S7): exact one-sided signed-rank test with Pratt's zeros on the block
differences minus the threshold (fleetdt.analysis.signed_rank_exact); Clopper-Pearson upper limits with the
raw events in n_eff = n / (2 tau_int) trials, tau_int of Madras and Sokal; one-sided 95 % block-bootstrap
bounds (blocks of ceil(2 tau_int) steps inside each run); TOST; IUT; fixed sequence at alpha = 0.05.

Calibration: for each evaluation scenario s and number of vessels M, alpha* in {0.01, ..., 1} minimizes the
one-step MSE of eq. (2) on the calibration DTA (exp "Cal") of the same M without s, and U* is the largest U in
{0.30, ..., 1} whose lower 95 % block-bootstrap bound of PICP(U) on this DTA reaches PICP_nom (fleetdt.outer).
The other predictors of H5 are fitted on the same DTA (fleetdt.predictors). DMR-hat(U) of H3b replays the
calibration runs of the same M and scenario (--dmr-hat-scenarios same, the default; loso: without s) with
alpha*(M, s), U and Q(t) = q_hat(t - 1) (fleetdt.outer.dmr_hat_replay).

Components (threshold of the Table of hypotheses in brackets):
  H1a  Pr{w2(t) > T2 - eps} per M over the E1 runs; CP [< 1e-3]; tau_int of w2(t)
  H1b  Pr{i not in M(t)} over the pairs (t, i), i = 1..M, per M; CP [< 1e-3]; tau_int of the per-step count
       of absent vessels (the arrival times of the demand tables are not logged)
  H1c  max over the E1 runs of max_{i,t} |theta_i(t) - theta_0(t)| after the warm-up, per M [< eps of M]
  H2   PICP(U*) - PICP_C(U*) per E2 cell of {S1, S2, S3, S6, S7} x M; upper BB [< 0.005]
  H3a  failed fleet steps with K_y >= U K_hat >= Q over the E3 cells outside the ablations (catch-up and
       lag steps as failures), steps and groups of 8 steps; CP [< 1e-3]; the stratum Q(t) = 0 is reported
       as descriptive (--test-q0-stratum adds it to the IUT)
  H3b  DMR(U) - DMR-hat(U) per E3 cell with U <= 0.95; upper BB of the difference of two independent
       samples [< 0.005]
  H3c  logistic fit of the step failure on (U K_hat - K_y)^+ in kB over the E3 steps (descriptive)
  H4a  Delta PCR of Fleet-DT against B1, B2, B3, B6, B7, B8, B9 per E4 cell with rho >= 1.5; SR [Delta > 0.05]
  H4b  Delta PCR against B4 per E4 cell with rho in {1.5, 2}; SR [Delta > 0.02]
  H4c  Delta PDR_state of each vessel 2 to 4 against B5a and B5b per E4 cell with rho >= 1.5; SR [> 0.02]
  H4d  Delta of Jain's index of BDR_i of vessels 1 to 3 against B6 per fairness cell; SR [Delta > 0]
  H5   RelMSE of ES with alpha* against each predictor per E2 cell (fit without the scenario of the cell);
       SR of log RelMSE - log 1.05 [RelMSE < 1.05]
  H6a  PICP(U*) of the calibration DTA minus PICP(U*) of each E3 (outside the ablations) and E4 Fleet-DT run;
       TOST [within +-0.005]
  H6b  mean PICP of O3 in the steps N_w + 1 to 2 N_w after each change of regime; SR of PICP - (PICP_nom -
       0.005) [> 0]; Delta u of O3 against O1, O2 and O4 in the same steps; SR [Delta > 0.01]
A cell with rho >= 1.5 enters the IUT of H3 and H4 only when the mean fill ratio max_i a_i(t) / b(t) of its
Fleet-DT steps reaches 0.85 (FILL_MIN). The cells of S4, S8 and S9, the MAC queues, the ablations and the E4
cells at rho <= 1 are descriptive.
"""
import argparse
import json
import math
import os
import sys
from typing import Dict, List, Optional

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)

from fleetdt import analysis as A  # noqa: E402
from fleetdt import outer as O  # noqa: E402
from fleetdt import predictors as P  # noqa: E402
from fleetdt.as_model import DTP  # noqa: E402

FLEETDT = "fleetdt"
EVAL_SCENARIOS = ("S1", "S2", "S3", "S6", "S7")
DESCRIPTIVE_SCENARIOS = ("S4", "S8", "S9")
H4A_METHODS = ("B1", "B2", "B3", "B6", "B7", "B8", "B9")
H4C_METHODS = ("B5a", "B5b")
H4C_VESSELS = (2, 3, 4)
H6B_BASE = ("O1", "O2", "O4")
SKIP_STATUS = ("pending", "running", "failed", "invalid", "aborted", "skipped")
THRESHOLDS = {"H1a": 1e-3, "H1b": 1e-3, "H2": 0.005, "H3a": 1e-3, "H3b": 0.005, "H4a": 0.05, "H4b": 0.02,
              "H4c": 0.02, "H4d": 0.0, "H5": 1.05, "H6a": 0.005, "H6b_picp": 0.005, "H6b_u": 0.01}
COMPONENTS = {"H1": ("H1a", "H1b", "H1c"), "H2": ("H2",), "H3": ("H3a", "H3b", "H3c"),
              "H4": ("H4a", "H4b", "H4c", "H4d"), "H5": ("H5",), "H6": ("H6a", "H6b")}
DESCRIPTIVE_COMPONENTS = ("H3c",)


# --------------------------------------------------------------------------- campaign


def _arr(sr: Dict, k: str) -> np.ndarray:
    v = sr.get(k)
    if v is None:
        return np.full(len(sr.get("t") or []), np.nan)
    return np.asarray([np.nan if x is None else x for x in v], float)


class Run:
    """One run of the manifest with its metrics.json (paper_post)."""

    def __init__(self, camp: str, entry: Dict, order: int):
        self.e = entry
        self.order = order
        self.id = str(entry.get("id"))
        self.dir = os.path.join(camp, entry.get("dir") or self.id)
        mp = entry.get("metrics")
        self.metrics_path = os.path.join(camp, mp) if mp else os.path.join(self.dir, "metrics.json")
        self.exp = entry.get("exp")
        self.method = entry.get("method") or FLEETDT
        self.cell = str(entry.get("cell"))
        self.factors = dict(entry.get("factors") or {})
        for k in ("M", "scenario", "rho", "U"):
            if k not in self.factors and entry.get(k) is not None:
                self.factors[k] = entry[k]
        b = entry.get("block")
        if b is None:
            b = {"seed": entry.get("seed"), "slot": entry.get("slot", 0)}
        self.block = json.dumps(b, sort_keys=True) if isinstance(b, dict) else str(b)
        self._m = None
        self._recs = None

    def f(self, k, default=None):
        return self.factors.get(k, default)

    @property
    def M(self) -> Optional[int]:
        v = self.f("M")
        return int(v) if v is not None else None

    @property
    def scenario(self) -> Optional[str]:
        return self.f("scenario")

    @property
    def out(self) -> Optional[Dict]:
        if self._m is None:
            try:
                with open(self.metrics_path) as fh:
                    self._m = json.load(fh)
            except (OSError, ValueError):
                self._m = {}
        return self._m or None

    @property
    def metrics(self) -> Dict:
        return (self.out or {}).get("metrics") or {}

    @property
    def series(self) -> Dict:
        return self.metrics.get("series") or {"t": []}

    def s(self, k: str) -> np.ndarray:
        return _arr(self.series, k)

    def records(self):
        """the G(t) records of the analysis window (S0 + warm .. S0 + steps - 1) and the DTP of the run"""
        if self._recs is None:
            o = self.out or {}
            dtp = DTP.load(os.path.join(self.dir, "dtp.json"))
            recs = O.load_dta(os.path.join(self.dir, "vms", "gs", "dta.jsonl"))
            S0, warm, steps = int(o.get("S0") or 0), int(o.get("warm_steps") or 0), int(o.get("steps") or 0)
            hi = S0 + steps if steps else float("inf")
            self._recs = ([r for r in recs if S0 + warm <= int(r["t"]) < hi], dtp)
        return self._recs


def is_descriptive(r: Run) -> bool:
    """explicit "descriptive" of the manifest, else the rule of Section VI-C: S4, S8, S9, the MAC queues, the
    ablations, and the E4 cells at rho <= 1"""
    if r.e.get("descriptive") is not None:
        return bool(r.e["descriptive"])
    if r.scenario in DESCRIPTIVE_SCENARIOS or r.f("queue") or r.f("ablation"):
        return True
    rho = r.f("rho")
    return r.exp == "E4" and rho is not None and float(rho) <= 1.0


def load_campaign(camp: str, manifest: Optional[str] = None, keep_invalid: bool = False) -> Dict:
    """Runs of the manifest that completed and have metrics; a run with rt_valid false (paper_post) leaves out
    unless keep_invalid; a block with "block_invalid" in any of its runs leaves out for every method of its
    cell; of the runs with the same (exp, cell, block, method), the last valid one in manifest order stays
    (reruns)."""
    path = manifest or os.path.join(camp, "manifest.json")
    with open(path) as fh:
        man = json.load(fh)
    runs, dropped = [], []
    bad_blocks = set()
    for k, e in enumerate(man.get("runs") or []):
        r = Run(camp, e, k)
        if e.get("block_invalid"):
            bad_blocks.add((r.exp, r.cell, r.block))
        if str(e.get("status") or "done") in SKIP_STATUS:
            dropped.append({"id": r.id, "why": "status %s" % e.get("status")})
            continue
        if r.out is None or not r.metrics:
            dropped.append({"id": r.id, "why": "no metrics"})
            continue
        if not keep_invalid and r.out.get("rt_valid") is False:
            dropped.append({"id": r.id, "why": "not real-time valid"})
            continue
        runs.append(r)
    keep: Dict = {}
    for r in runs:
        if (r.exp, r.cell, r.block) in bad_blocks:
            dropped.append({"id": r.id, "why": "invalid block"})
            continue
        keep[(r.exp, r.cell, r.block, r.method)] = r
    return {"manifest": man, "runs": sorted(keep.values(), key=lambda r: r.order), "dropped": dropped}


def by_cell(runs: List[Run]) -> Dict[str, Dict[str, Dict[str, Run]]]:
    """{cell: {block: {method: run}}}"""
    out: Dict = {}
    for r in runs:
        out.setdefault(r.cell, {}).setdefault(r.block, {})[r.method] = r
    return out


def fill_gate(cell_runs: Dict[str, Dict[str, Run]]) -> Dict:
    """mean fill ratio of the Fleet-DT steps of a cell (steps pooled over its runs) and the gate of Section VI-E
    for cells with rho >= 1.5"""
    num = den = 0.0
    rho = None
    for blk in cell_runs.values():
        for m, r in blk.items():
            rho = r.f("rho", rho)
            if m != FLEETDT:
                continue
            f = r.metrics.get("fill") or {}
            if f.get("mean") is not None and f.get("n"):
                num += float(f["mean"]) * int(f["n"])
                den += int(f["n"])
    mean = num / den if den else float("nan")
    gated = rho is not None and float(rho) >= 1.5
    return {"rho": rho, "fill_mean": mean, "gated": gated,
            "enters": (not gated) or (mean == mean and mean >= A.FILL_MIN)}


# --------------------------------------------------------------------------- calibration


class Calibration:
    """alpha*, U*, PICP of the calibration DTA, the predictors of H5 and DMR-hat, per (M, scenario), from the runs
    of exp "Cal" (cached)."""

    def __init__(self, runs: List[Run], picp_nom: float = 0.99, B: int = 2000, seed: int = 0, conf: float = 0.95,
                 arima_engine: str = "auto", C_ref: Optional[Dict] = None, dmr_hat_scenarios: str = "same"):
        self.cal = [r for r in runs if r.exp == "Cal"]
        self.picp_nom, self.B, self.seed, self.conf = picp_nom, B, seed, conf
        self.arima_engine = arima_engine
        self.C_ref_tab = C_ref or {}
        self.dmr_hat_scenarios = dmr_hat_scenarios
        self._cache: Dict = {}

    def pool(self, M: int, s: Optional[str], loso: bool = True) -> List[Run]:
        return [r for r in self.cal if r.M == M and ((r.scenario != s) if loso else (r.scenario == s))]

    @staticmethod
    def _zs(rr: List[Run]):
        zs, C0s, ys = [], [], []
        for r in rr:
            y = r.s("y")
            cy = float(r.metrics.get("c_y") or 1.0)
            C = r.s("C_hat")
            fin = C[~np.isnan(C)]
            zs.append(cy * y)
            ys.append(y)
            C0s.append(float(fin[0]) if len(fin) else float(np.nanmean(cy * y)))
        return zs, C0s, ys

    def fit(self, M: int, s: str) -> Dict:
        key = ("fit", M, s)
        if key in self._cache:
            return self._cache[key]
        rr = self.pool(M, s)
        out = {"runs": [r.id for r in rr], "M": M, "left_out": s}
        if not rr:
            out.update(alpha_star=None, U_star=None, PICP_cal=None)
            self._cache[key] = out
            return out
        zs, C0s, ys = self._zs(rr)
        a_star, m_star = O.fit_alpha_runs(zs, C0s, refine=False)
        cy = float(rr[0].metrics.get("c_y") or 1.0)
        r3 = float(rr[0].metrics.get("T3m_Ts") or O.T3_TS_DEFAULT)
        bctl = float(rr[0].metrics.get("B_ctl_bar") or 0.0)
        rep = [(y, O.replay(z, [a_star], C0)[:, 0]) for y, z, C0 in zip(ys, zs, C0s)]
        gb = O.gamma_bounds(rep, O.U_GRID, conf=self.conf, B=self.B, seed=self.seed, T3_Ts=r3, c_y=cy, B_ctl_bar=bctl)
        U_star = O.largest_U(gb["gamma_low"], O.U_GRID, self.picp_nom)
        picp = None
        if U_star is not None:
            picp = float(gb["gamma"][int(np.argmin(np.abs(O.U_GRID - U_star)))])
        out.update(alpha_star=a_star, mse=m_star, U_star=U_star, PICP_cal=picp, block=gb["block"], n=gb["n"])
        self._cache[key] = out
        return out

    def C_ref(self, M: int, s: str) -> Optional[float]:
        k = "%d/%s" % (M, s)
        if k in self.C_ref_tab:
            return float(self.C_ref_tab[k])
        rr = self.pool(M, s, loso=False)
        vals = []
        for r in rr:
            r3 = float(r.metrics.get("T3m_Ts") or O.T3_TS_DEFAULT)
            kc = r.s("K_C")
            if np.any(~np.isnan(kc)):
                vals.append(kc[~np.isnan(kc)] / r3)
        if not vals:
            for r in rr:
                z = float(r.metrics.get("c_y") or 1.0) * r.s("y")
                vals.append(z[~np.isnan(z)])
        v = np.concatenate(vals) if vals else np.zeros(0)
        return float(v.mean()) if len(v) else None

    def predictors(self, M: int, s: str) -> Optional[Dict]:
        key = ("pred", M, s)
        if key in self._cache:
            return self._cache[key]
        f = self.fit(M, s)
        if f.get("alpha_star") is None:
            self._cache[key] = None
            return None
        zs, _, _ = self._zs(self.pool(M, s))
        fit = P.fit_all(zs, f["alpha_star"], self.C_ref(M, s), arima_engine=self.arima_engine)
        self._cache[key] = fit
        return fit

    def dmr_hat(self, M: int, s: str, U: float) -> Dict:
        key = ("dmr", M, s, round(float(U), 6))
        if key in self._cache:
            return self._cache[key]
        f = self.fit(M, s)
        rr = self.pool(M, s, loso=(self.dmr_hat_scenarios == "loso"))
        miss = []
        if f.get("alpha_star") is not None:
            for r in rr:
                try:
                    recs, dtp = r.records()
                except (OSError, ValueError, KeyError):
                    continue
                if recs:
                    cy = float(r.metrics.get("c_y") or dtp.c_y)
                    miss.append(O.dmr_hat_replay(recs, dtp, f["alpha_star"], float(U), c_y=cy)["miss"])
        out = {"miss": miss, "runs": [r.id for r in rr], "alpha_star": f.get("alpha_star")}
        self._cache[key] = out
        return out


# --------------------------------------------------------------------------- components


def _comp(cid, cell, test, threshold, **kw) -> Dict:
    d = {"id": cid, "cell": cell, "test": test, "threshold": threshold, "status": "tested", "reject": None}
    d.update(kw)
    return d


def _cp(cid, cell, k, n, tau, alpha):
    ne = A.n_eff(n, tau) if n else 0.0
    up = A.clopper_pearson_upper_eff(k, n, ne, conf=1.0 - alpha) if n else 1.0
    return _comp(cid, cell, "CP", THRESHOLDS[cid], estimate=(k / n) if n else None, bound=up, n=int(n), events=int(k),
                 tau_int=tau, n_eff=ne, reject=bool(n) and up < THRESHOLDS[cid])


def _sr(cid, cell, d, margin, alpha, alternative="greater", threshold=None, **kw):
    """exact one-sided signed-rank test (Pratt zeros) of the block values d minus `margin`; threshold: the value
    that the table reports (default: margin)"""
    d = np.asarray(d, float)
    d = d[~np.isnan(d)]
    th = margin if threshold is None else threshold
    if not len(d):
        return _comp(cid, cell, "SR", th, status="no data", n=0, **kw)
    r = A.signed_rank_exact(d, margin=margin, alternative=alternative)
    return _comp(cid, cell, "SR", th, estimate=A.hodges_lehmann(d), p=r["p"], n=int(len(d)),
                 reject=r["p"] <= alpha, **kw)


def h1(runs: List[Run], cal: Dict, alpha: float) -> List[Dict]:
    out = []
    e1 = [r for r in runs if r.exp == "E1"]
    for M in sorted({r.M for r in e1 if r.M}):
        rr = [r for r in e1 if r.M == M]
        cell = "M=%d" % M
        eps = (cal.get("eps_s") or {}).get(str(M))
        T2 = (cal.get("T2_s") or {}).get(str(M))
        # H1a
        w2s = [r.s("w2") for r in rr]
        n = int(sum(np.sum(~np.isnan(w)) for w in w2s))
        if n == 0 or eps is None or T2 is None:
            out.append(_comp("H1a", cell, "CP", THRESHOLDS["H1a"], status="no data", n=n,
                             note="needs w2(t) (RUN/h1.jsonl) and T2_s, eps_s of the manifest calibration"))
        else:
            k = int(sum(np.sum(w[~np.isnan(w)] > float(T2) - float(eps)) for w in w2s))
            out.append(_cp("H1a", cell, k, n, A.tau_int_runs(w2s), alpha))
        # H1b: pairs (t, i), i = 1..M
        absent = [M - r.s("m") for r in rr]
        n = int(sum(np.sum(~np.isnan(a)) for a in absent)) * M
        k = int(sum(np.nansum(a) for a in absent))
        if n:
            c = _cp("H1b", cell, k, n, A.tau_int_runs(absent), alpha)
            c["note"] = "tau_int of the per-step count of absent vessels"
            out.append(c)
        else:
            out.append(_comp("H1b", cell, "CP", THRESHOLDS["H1b"], status="no data", n=0))
        # H1c
        th = [((r.out or {}).get("clock") or {}).get("theta_max_abs_s") for r in rr]
        th = [float(v) for v in th if v is not None]
        if not th or eps is None:
            out.append(_comp("H1c", cell, "deterministic", eps, status="no data", n=len(th),
                             note="needs RUN/clock.csv and eps_s of the manifest calibration"))
        else:
            mx = max(th)
            out.append(_comp("H1c", cell, "deterministic", float(eps), estimate=mx, bound=mx, n=len(th),
                             reject=mx < float(eps)))
    return out


def _cover(r: Run, U: float) -> np.ndarray:
    Ky, Kh = r.s("K_y"), r.s("K_hat")
    ok = ~(np.isnan(Ky) | np.isnan(Kh))
    return np.where(ok, (Ky >= U * Kh).astype(float), np.nan)


def h2(runs: List[Run], C: Calibration, B: int, seed: int, conf: float) -> List[Dict]:
    out = []
    for cell, blocks in sorted(by_cell([r for r in runs if r.exp == "E2"]).items()):
        rr = [b[FLEETDT] for b in blocks.values() if FLEETDT in b]
        if not rr or rr[0].scenario not in EVAL_SCENARIOS or rr[0].f("alpha") not in (None, "alpha*") \
                or any(is_descriptive(r) for r in rr):
            continue
        M, s = rr[0].M, rr[0].scenario
        Us = C.fit(M, s).get("U_star")
        if Us is None:
            out.append(_comp("H2", cell, "BB", THRESHOLDS["H2"], status="no data", note="no U* (calibration)"))
            continue
        d = []
        for r in rr:
            kc, kh = r.s("K_C"), r.s("K_hat")
            cy = _cover(r, Us)
            ok = ~(np.isnan(cy) | np.isnan(kc))
            if ok.any():
                d.append(cy[ok] - (kc[ok] >= Us * kh[ok]).astype(float))
        if not d:
            out.append(_comp("H2", cell, "BB", THRESHOLDS["H2"], status="no data", note="no K_C(t) (csub/c_t.jsonl)"))
            continue
        bb = A.block_bootstrap_bounds(d, np.mean, conf=conf, B=B, seed=seed)
        out.append(_comp("H2", cell, "BB", THRESHOLDS["H2"], estimate=bb["value"], bound=bb["upper"], n=bb["n"],
                         block=bb["block"], U_star=Us, reject=bb["upper"] < THRESHOLDS["H2"]))
    return out


def _e3(runs: List[Run]):
    """E3 cells outside the ablations, with their fill gate"""
    cells = {}
    for cell, blocks in by_cell([r for r in runs if r.exp == "E3"]).items():
        rr = [b[FLEETDT] for b in blocks.values() if FLEETDT in b]
        if not rr or any(r.f("ablation") for r in rr) or any(is_descriptive(r) for r in rr):
            continue
        cells[cell] = (rr, fill_gate(blocks))
    return cells


def h3(runs: List[Run], C: Calibration, B: int, seed: int, conf: float, alpha: float,
       test_q0: bool = False, group: int = 8) -> List[Dict]:
    out = []
    cells = _e3(runs)
    pooled = [r for rr, g in cells.values() if g["enters"] for r in rr]
    excluded = sorted(c for c, (rr, g) in cells.items() if not g["enters"])
    conds = [A.condition_from_series(r.series) for r in pooled]
    for stratum in ("general", "Q0"):
        for w in (1, group):
            cell = "%s, %s" % ("steps" if w == 1 else "groups of %d" % w, "K_y >= U K_hat >= Q" if stratum == "general"
                                else "stratum Q = 0")
            if not conds:
                out.append(_comp("H3a", cell, "CP", THRESHOLDS["H3a"], status="no data", n=0))
                continue
            h = A.h3a_pool(conds, window=w, stratum=stratum, conf=1.0 - alpha)
            c = _comp("H3a", cell, "CP", THRESHOLDS["H3a"], estimate=h["estimate"], bound=h["upper"], n=h["n_cond"],
                      events=h["failures"], tau_int=h["tau_margin"], n_eff=h["n_eff"], runs=h["runs"],
                      reject=bool(h["n_cond"]) and h["upper"] < THRESHOLDS["H3a"], excluded_cells=excluded)
            if stratum == "Q0" and not test_q0:
                c["status"] = "descriptive"
            out.append(c)
    # H3b
    for cell, (rr, g) in sorted(cells.items()):
        U = rr[0].f("U")
        if U is None or float(U) > 0.95:
            continue
        if not g["enters"]:
            out.append(_comp("H3b", cell, "BB", THRESHOLDS["H3b"], status="excluded (fill)", fill_mean=g["fill_mean"]))
            continue
        dh = C.dmr_hat(rr[0].M, rr[0].scenario, float(U))
        if not dh["miss"]:
            out.append(_comp("H3b", cell, "BB", THRESHOLDS["H3b"], status="no data", note="no calibration DTA"))
            continue
        bb = A.bb_diff_bounds([r.s("miss") for r in rr], dh["miss"], conf=conf, B=B, seed=seed)
        out.append(_comp("H3b", cell, "BB", THRESHOLDS["H3b"], estimate=bb["value"], bound=bb["upper"],
                         n=bb["n_a"], n_cal=bb["n_b"], U=float(U), alpha_star=dh["alpha_star"],
                         reject=bb["upper"] < THRESHOLDS["H3b"]))
    # H3c
    xs, ys = [], []
    for r in pooled:
        Ky, Kh, U = r.s("K_y"), r.s("K_hat"), r.s("U")
        c = A.condition_from_series(r.series)
        ok = ~(np.isnan(Ky) | np.isnan(Kh) | np.isnan(r.s("miss")))
        xs.append(np.maximum(0.0, U[ok] * Kh[ok] - Ky[ok]) / 1000.0)
        ys.append(c["fail"][ok].astype(float))
    if xs:
        lf = A.logistic_fit(np.concatenate(xs), np.concatenate(ys))
        out.append(_comp("H3c", "E3 steps", "logistic", None, status="descriptive", estimate=lf["b1"], p=lf["p_b1"],
                         n=lf["n"], events=lf["events"], fit=lf, unit="per kB of shortfall"))
    else:
        out.append(_comp("H3c", "E3 steps", "logistic", None, status="no data"))
    return out


def _paired(blocks: Dict[str, Dict[str, Run]], method: str, value) -> np.ndarray:
    d = []
    for blk in blocks.values():
        if FLEETDT in blk and method in blk:
            a, b = value(blk[FLEETDT]), value(blk[method])
            if a is not None and b is not None:
                d.append(float(a) - float(b))
    return np.asarray(d, float)


def _pcr(r: Run):
    return (r.metrics.get("PCR") or {}).get("value")


def h4(runs: List[Run], alpha: float) -> List[Dict]:
    out = []
    for cell, blocks in sorted(by_cell([r for r in runs if r.exp == "E4"]).items()):
        any_r = next(iter(next(iter(blocks.values())).values()))
        rho = any_r.f("rho")
        desc = is_descriptive(any_r)
        g = fill_gate(blocks)
        methods = {m for b in blocks.values() for m in b}
        fair = bool(any_r.f("fairness"))

        def add(c):
            if desc:
                c["status"] = "descriptive"
            elif not g["enters"]:
                c["status"] = "excluded (fill)"
                c["fill_mean"] = g["fill_mean"]
            out.append(c)
        if fair:
            if "B6" in methods:
                add(_sr("H4d", cell + " vs B6", _paired(blocks, "B6", lambda r: r.metrics.get("J")),
                        THRESHOLDS["H4d"], alpha))
            continue
        if rho is None or float(rho) < 1.5:
            for m in sorted(methods - {FLEETDT}):
                c = _sr("H4a", cell + " vs " + m, _paired(blocks, m, _pcr), THRESHOLDS["H4a"], alpha)
                c["status"] = "descriptive"
                out.append(c)
            continue
        for m in H4A_METHODS:
            if m in methods:
                add(_sr("H4a", cell + " vs " + m, _paired(blocks, m, _pcr), THRESHOLDS["H4a"], alpha))
        if "B4" in methods and float(rho) in (1.5, 2.0):
            add(_sr("H4b", cell + " vs B4", _paired(blocks, "B4", _pcr), THRESHOLDS["H4b"], alpha))
        for m in H4C_METHODS:
            if m not in methods:
                continue
            for v in H4C_VESSELS:
                val = lambda r, v=v: (r.metrics.get("PDR_state_i") or {}).get(str(v))
                add(_sr("H4c", "%s vs %s, vessel %d" % (cell, m, v), _paired(blocks, m, val), THRESHOLDS["H4c"], alpha))
    return out


def h5(runs: List[Run], C: Calibration, alpha: float, burn: int) -> List[Dict]:
    out = []
    lm = math.log(THRESHOLDS["H5"])
    for cell, blocks in sorted(by_cell([r for r in runs if r.exp == "E2"]).items()):
        rr = [b[FLEETDT] for b in blocks.values() if FLEETDT in b]
        if not rr or rr[0].scenario not in EVAL_SCENARIOS or rr[0].f("alpha") not in (None, "alpha*") \
                or any(is_descriptive(r) for r in rr):
            continue
        fit = C.predictors(rr[0].M, rr[0].scenario)
        if fit is None:
            out.append(_comp("H5", cell, "SR", THRESHOLDS["H5"], status="no data", note="no calibration DTA"))
            continue
        per = {}
        for r in rr:
            z = float(r.metrics.get("c_y") or 1.0) * r.s("y")
            for name, v in P.rel_mse_all(z, fit, burn=burn).items():
                per.setdefault(name, []).append(v["log"])
        for name in P.PREDICTORS:
            if name not in per:
                continue
            lv = np.asarray(per[name], float)
            c = _sr("H5", "%s vs %s" % (cell, name), lv, lm, alpha, alternative="less", threshold=THRESHOLDS["H5"])
            if c.get("estimate") is not None:
                c["estimate"] = math.exp(c["estimate"])        # RelMSE (Hodges-Lehmann of log RelMSE)
            out.append(c)
    return out


def h6(runs: List[Run], C: Calibration, alpha: float, picp_nom: float, N_w: int) -> List[Dict]:
    out = []
    # H6a
    d = []
    for r in runs:
        ok = (r.exp == "E3" and not r.f("ablation") and not is_descriptive(r)) or \
             (r.exp == "E4" and r.method == FLEETDT and not is_descriptive(r) and not r.f("fairness"))
        if not ok or r.M is None:
            continue
        f = C.fit(r.M, r.scenario)
        if f.get("U_star") is None or f.get("PICP_cal") is None:
            continue
        cv = _cover(r, f["U_star"])
        if np.any(~np.isnan(cv)):
            d.append(f["PICP_cal"] - float(np.nanmean(cv)))
    if len(d) >= 2:
        t = A.tost(d, THRESHOLDS["H6a"], alpha=alpha)
        out.append(_comp("H6a", "E3 and E4 runs", "TOST", THRESHOLDS["H6a"], estimate=t["mean"], p=t["p"], n=t["n"],
                         ci=t["ci"], reject=bool(t["equivalent"])))
    else:
        out.append(_comp("H6a", "E3 and E4 runs", "TOST", THRESHOLDS["H6a"], status="no data", n=len(d)))
    # H6b
    for cell, blocks in sorted(by_cell([r for r in runs if r.exp == "E5"]).items()):
        vals, du = [], {m: [] for m in H6B_BASE}
        for blk in blocks.values():
            o3 = blk.get("O3")
            if o3 is None:
                continue
            if not o3.f("regime_changes"):
                continue

            def window(r):
                """steps N_w + 1 to 2 N_w after each change of regime (scenario steps k = t - S0 of the run)"""
                k = r.s("t") - int((r.out or {}).get("S0") or 0)
                sel = np.zeros(len(k), bool)
                for c in r.f("regime_changes") or o3.f("regime_changes"):
                    sel |= (k >= int(c) + N_w) & (k <= int(c) + 2 * N_w - 1)
                return sel
            sel = window(o3)
            cv = o3.s("cover")[sel]
            if np.any(~np.isnan(cv)):
                vals.append(float(np.nanmean(cv)))

            def u(r):
                s = window(r)
                num, den = r.s("u_num")[s], r.s("u_den")[s]
                ok = ~(np.isnan(num) | np.isnan(den))
                return float(num[ok].sum() / den[ok].sum()) if ok.any() and den[ok].sum() > 0 else None
            u3 = u(o3)
            for m in H6B_BASE:
                if m in blk and u3 is not None:
                    um = u(blk[m])
                    if um is not None:
                        du[m].append(u3 - um)
        if not vals:
            out.append(_comp("H6b", cell + ", PICP of O3", "SR", picp_nom - THRESHOLDS["H6b_picp"], status="no data",
                             note="needs O3 runs with factors.regime_changes"))
            continue
        out.append(_sr("H6b", cell + ", PICP of O3", vals, picp_nom - THRESHOLDS["H6b_picp"], alpha))
        for m in H6B_BASE:
            out.append(_sr("H6b", "%s, u of O3 vs %s" % (cell, m), du[m], THRESHOLDS["H6b_u"], alpha))
    return out


# --------------------------------------------------------------------------- aggregation


def _hyp(cid: str) -> str:
    return cid[:2]


def summarize(comps: List[Dict], alpha: float) -> Dict:
    """Rows per component ID (IUT over its tested cells), verdicts per hypothesis and the fixed sequence."""
    rows = []
    for h, ids in COMPONENTS.items():
        for cid in ids:
            cs = [c for c in comps if c["id"] == cid]
            tested = [c for c in cs if c["status"] == "tested"]
            if cid in DESCRIPTIVE_COMPONENTS or (cs and not tested and all(c["status"] == "descriptive" for c in cs)):
                kind = "descriptive"
            elif not tested:
                kind = "no data"
            else:
                kind = "tested"
            worst = None
            if tested:
                fails = [c for c in tested if not c["reject"]]
                pool = fails or tested
                key = (lambda c: c["p"]) if "p" in pool[0] else (lambda c: (c.get("bound") or 0.0))
                worst = max(pool, key=key)
            elif cs:
                worst = cs[0]
            rows.append({"id": cid, "hypothesis": h, "kind": kind, "cells": len(cs), "tested_cells": len(tested),
                         "reject": (all(c["reject"] for c in tested) if kind == "tested" else None),
                         "worst": worst})
    hyp = {}
    for h, ids in COMPONENTS.items():
        rr = [r for r in rows if r["hypothesis"] == h and r["kind"] == "tested"]
        missing = [r["id"] for r in rows if r["hypothesis"] == h and r["kind"] == "no data"]
        hyp[h] = None if (not rr or missing) else all(r["reject"] for r in rr)
    seq = A.fixed_sequence_verdicts(hyp)
    tested_h = {s["id"]: s for s in seq}
    for r in rows:
        s = tested_h[r["hypothesis"]]
        if r["kind"] == "descriptive":
            r["verdict"] = "descriptive"
        elif r["kind"] == "no data":
            r["verdict"] = "no data"
        elif not s["tested"]:
            r["verdict"] = "not tested (sequence ended)"
        else:
            r["verdict"] = "supported" if r["reject"] else "not supported"
    return {"rows": rows, "hypotheses": hyp, "sequence": seq}


def _fmt(v, f="%.4g"):
    return "-" if v is None or (isinstance(v, float) and v != v) else (f % v)


def markdown(summary: Dict) -> str:
    lines = ["| ID | Estimate | Bound or p | Threshold | Verdict |", "|---|---|---|---|---|"]
    for r in summary["rows"]:
        w = r["worst"] or {}
        if "p" in w and w.get("p") is not None:
            bp = "p = %s" % _fmt(w["p"])
        elif w.get("bound") is not None:
            bp = "%s %s" % (w.get("test", ""), _fmt(w["bound"]))
        else:
            bp = "-"
        cell = (" (%s)" % w["cell"]) if w.get("cell") and r["tested_cells"] > 1 else ""
        lines.append("| %s | %s%s | %s | %s | %s |" % (r["id"], _fmt(w.get("estimate")), cell, bp,
                                                      _fmt(w.get("threshold")), r["verdict"]))
    lines.append("")
    lines.append("Fixed sequence: " + ", ".join("%s %s" % (s["id"], "rejected" if s["rejected"] else
                                                          ("not rejected" if s["tested"] else "not tested"))
                                               for s in summary["sequence"]))
    return "\n".join(lines) + "\n"


def _clean(x):
    if isinstance(x, dict):
        return {str(k): _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, np.ndarray):
        return _clean(x.tolist())
    if hasattr(x, "item") and not isinstance(x, (str, bytes)):
        x = x.item()
    if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
        return None
    return x


def run(camp: str, manifest: Optional[str] = None, out_dir: Optional[str] = None, B: int = 2000, seed: int = 0,
        alpha: float = 0.05, burn: int = 50, arima_engine: str = "auto", dmr_hat_scenarios: str = "same",
        keep_invalid: bool = False, test_q0: bool = False) -> Dict:
    c = load_campaign(camp, manifest, keep_invalid)
    man = c["manifest"]
    cal = dict(man.get("calibration") or {})
    picp_nom = float(cal.get("PICP_nom", 0.99))
    N_w = int(cal.get("N_w", 480))
    conf = 1.0 - alpha
    C = Calibration(c["runs"], picp_nom=picp_nom, B=B, seed=seed, conf=conf, arima_engine=arima_engine,
                    C_ref=cal.get("C_ref"), dmr_hat_scenarios=dmr_hat_scenarios)
    runs = [r for r in c["runs"] if r.exp != "Cal"]
    comps = (h1(runs, cal, alpha) + h2(runs, C, B, seed, conf) + h3(runs, C, B, seed, conf, alpha, test_q0) +
             h4(runs, alpha) + h5(runs, C, alpha, burn) + h6(runs, C, alpha, picp_nom, N_w))
    for x in comps:
        x["hypothesis"] = _hyp(x["id"])
    summ = summarize(comps, alpha)
    calib = {"%s/M=%d/without %s" % ("fit", k[1], k[2]): {kk: v for kk, v in val.items() if kk != "runs"}
             for k, val in C._cache.items() if k[0] == "fit"}
    res = {"campaign": camp, "alpha": alpha, "B": B, "seed": seed, "PICP_nom": picp_nom, "N_w": N_w,
           "runs": len(c["runs"]), "dropped": c["dropped"], "calibration": calib, "components": comps,
           "rows": summ["rows"], "hypotheses": summ["hypotheses"], "sequence": summ["sequence"]}
    res = _clean(res)
    od = out_dir or camp
    os.makedirs(od, exist_ok=True)
    with open(os.path.join(od, "hypotheses.json"), "w") as fh:
        json.dump(res, fh, indent=1)
    with open(os.path.join(od, "hypotheses.md"), "w") as fh:
        fh.write(markdown(summ))
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("campaign")
    ap.add_argument("--manifest", default=None)
    ap.add_argument("--out", default=None, help="output directory (default: the campaign directory)")
    ap.add_argument("--B", type=int, default=2000, help="bootstrap resamples")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--alpha", type=float, default=0.05, help="familywise level of the fixed sequence")
    ap.add_argument("--burn", type=int, default=50, help="first step of the squared errors of H5")
    ap.add_argument("--arima-engine", choices=("auto", "statsmodels", "css"), default="auto")
    ap.add_argument("--dmr-hat-scenarios", choices=("same", "loso"), default="same",
                    help="calibration runs of DMR-hat: the scenario of the cell (same) or the others (loso)")
    ap.add_argument("--keep-invalid", action="store_true", help="keep the runs with rt_valid false")
    ap.add_argument("--test-q0-stratum", action="store_true", help="the stratum Q(t) = 0 of H3a enters the IUT")
    a = ap.parse_args(argv)
    res = run(a.campaign, a.manifest, a.out, a.B, a.seed, a.alpha, a.burn, a.arima_engine, a.dmr_hat_scenarios,
              a.keep_invalid, a.test_q0_stratum)
    for s in res["sequence"]:
        print("%s: %s" % (s["id"], "rejected" if s["rejected"] else ("not rejected" if s["tested"] else "not tested")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
