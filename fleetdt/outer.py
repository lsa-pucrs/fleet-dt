"""Outer loop of the AS (contribution (iv) of the Fleet-DT paper): tunes the gain alpha of eq. (2) and the
target utilization U of eq. (3) from the DTA.

Notation (revised model): K_hat(t) = C_hat(t) (T3 - T_m)/Ts is the predicted capacity of phase 3 and
K_y(t) = c_y y(t) (T3 - T_m)/Ts - B_ctl_bar the measured one, with c_y the smallest C(t)/y(t) of a saturation
test and B_ctl_bar the bytes of control traffic outside the data bridges in phase 3 (DTP fields; T3_Ts below is
(T3 - T_m)/Ts).

  (a) alpha_star = argmin of the one-step mean squared error of eq. (2), C_hat(t) = (1 - alpha) C_hat(t-1) +
      alpha c_y y(t-1), replayed on c_y y(t) (grid 0.01..1 step 0.01, then golden-section refinement around
      the best grid point); with several runs, the MSE is pooled over the runs (each run replayed from its
      own C0);
  (b) PICP(U) (prediction interval coverage probability; gamma in earlier versions) = fraction of the
      steps of S (steps with y(t) and C_hat(t)) with K_y(t) >= U K_hat(t), C_hat(t) recomputed causally
      with alpha_star; PICP_low(U) = lower one-sided 95 % bound of PICP(U) by circular block bootstrap
      (blocks of ceil(2 tau_int) steps, tau_int of the slack series K_y(t) - U K_hat(t), blocks inside each
      run, the same resamples for every U of the grid);
      U_star = largest U of the grid 0.30..1.00 with PICP_low(U) >= PICP_nom (C13; "gamma_target")
      (U_star_point: the same rule on the point estimate PICP(U));
  (c) what-if table: for each U, PICP(U), PICP_low(U) and the predicted deadline miss ratio DMR(U) of the
      backlog predictor of Corollary 1 in Lindley form (C5, function backlog):
          q(t) = max(0, q(t-1) + sum_{i in M(t)} a_i(t) - K_y(t)),   q(-1) = 0,
          DMR(U) = fraction of the steps with q(t) > 0   (phi(U) = 1 - DMR(U), the full-delivery fraction).
      a_i(t) are the bytes that Algorithm 1 admitted for vessel i (the AS logs "a" and their sum "A",
      fleetdt/as_service.py; the VE copies "A" into G(t); without "A" the admitted demand of the edges of G(t),
      sum of d_il over p_l > 0, and without edges the cap max(0, U K_hat(t) - Q(t))). c_y y(t) stands for
      C(t): c_y is the calibrated lower bound of C(t)/y(t) (kappa in earlier versions; 1 by default).
      What-if over U (approximation): the admitted bytes of another U are not observed, so the predictor
      scales them with the cap, X_U(t) = f(t) max(0, U K_hat_alpha(t) - Q(t)), f(t) = A(t) / cap(t) the share
      of the logged cap cap(t) = max(0, U(t) K_hat(t) - Q(t)) that Algorithm 1 filled (1 without A),
      K_hat_alpha(t) = C_hat(t) T3_Ts with the C_hat(t) replayed with alpha_star and Q(t) the logged backlog
      (the backlog of another U is not observed either); at the logged U and alpha, X_U(t) = A(t) exactly.
      Larger U admit more topics only in steps of whole topics, so f(t) of other U deviates from the logged
      one by at most the demand of one topic per vessel. Proposition 2 needs K_y(t) >= U K_hat(t) and
      Q(t) = 0, so DMR(U) >= 1 - PICP(U) when f = 1 and Q = 0: a shortfall in step t-1 leaves bytes that
      delay step t.

alpha* and U* per number of vessels M: calibrate_by_M (one DTA per run, M of a run = largest m(t)).

  python -m fleetdt.outer dta.jsonl [--dtp dtp.json] [--target 0.99] [--c-y 1.0] [--B-ctl-bar 0] [--window W]
                                    [--every K] [--json]
  python -m fleetdt.outer run1.jsonl run2.jsonl ... --by-M [--target 0.99] [--c-y 1.0] [--json]

Output keys: PICP, PICP_low, DMR per U (whatif), PICP_nom, c_y, B_ctl_bar; the keys of earlier versions stay
as aliases for one release: gamma (= PICP), gamma_low (= PICP_low), phi (= 1 - DMR), gamma_target
(= PICP_nom), kappa (= c_y).
"""
from __future__ import annotations

import argparse
import json
import math
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

ALPHA_GRID = np.round(np.arange(1, 101) / 100.0, 2)
U_GRID = np.round(np.arange(30, 101) / 100.0, 2)
T3_TS_DEFAULT = 0.095 / 0.125   # T3 / Ts of the DTP of the paper (Ts = 125 ms, T1 = 20 ms, T2 = 10 ms)


def load_dta(path) -> List[Dict]:
    """Reads a DTA file (one G(t) dict per line, as written by fleetdt.ve); skips malformed lines."""
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def series(records: Iterable[Dict]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Steps t, y(t) and the recorded C_hat(t), sorted by t (the last record of a repeated t wins).
    Missing values are NaN."""
    by_t = {}
    for r in records:
        by_t[int(r["t"])] = r
    ts = np.array(sorted(by_t), dtype=np.int64)
    nan = float("nan")
    y = np.array([nan if by_t[t].get("y") is None else float(by_t[t]["y"]) for t in ts.tolist()])
    C = np.array([nan if by_t[t].get("C_hat") is None else float(by_t[t]["C_hat"]) for t in ts.tolist()])
    return ts, y, C


def admitted_bytes(r: Dict) -> float:
    """A(t) = sum_{i in M(t)} a_i(t) of one record: "A" of the AS (fleet/as, G(t)), else the sum of the a_i
    of "a", else the admitted demand of the edges of G(t) (d_il of the vessels of M(t) with p_l > 0); NaN
    when the record holds none of them (no table: nothing admitted is known)."""
    if r.get("A") is not None:
        return float(r["A"])
    if isinstance(r.get("a"), dict) and r["a"]:
        return float(sum(float(v) for v in r["a"].values()))
    edges = r.get("edges") or []
    M = set(r.get("vessels") or [])
    if edges and any(e.get("p") is not None for e in edges):
        return float(sum(float(e.get("d") or 0) for e in edges
                         if (not M or e.get("i") in M) and (e.get("p") or 0) > 0))
    return float("nan")


def backlog_of(r: Dict) -> float:
    """Q(t) of one record: "Q" of the AS or of G(t), else the sum of Q_i(t) of the vertices in M(t) and of
    Qbar_i(t) of the vertices outside M(t), else 0."""
    if r.get("Q") is not None:
        return float(r["Q"])
    M = set(r.get("vessels") or [])
    return float(sum(float((v.get("Q") if v.get("i") in M else v.get("Qbar")) or 0) for v in r.get("vertices") or []))


def fill_series(records: Iterable[Dict], T3_Ts: float) -> np.ndarray:
    """f(t) = A(t) / cap(t) per step sorted by t (series order), cap(t) = max(0, U(t) K_hat(t) - Q(t)) with
    K_hat(t) = C_hat(t) T3_Ts (T3_Ts = (T3 - T_m) / Ts): the share of the cap of step t that Algorithm 1 filled;
    NaN without A(t), U(t) or C_hat(t) or with a zero cap (the predictor then takes f = 1)."""
    by_t = {}
    for r in records:
        by_t[int(r["t"])] = r
    out = []
    for t in sorted(by_t):
        r = by_t[t]
        A, U, C = admitted_bytes(r), r.get("U"), r.get("C_hat")
        cap = max(0.0, float(U) * float(C) * T3_Ts - backlog_of(r)) if U is not None and C is not None \
            else float("nan")
        out.append(A / cap if cap > 0 and A == A else float("nan"))
    return np.asarray(out, dtype=float)


def replay(y: np.ndarray, alphas, C0: float) -> np.ndarray:
    """Eq. (2) replayed on y for each gain: C[k] predicts y[k]; C[0] = C0; a missing y[k] keeps C."""
    a = np.atleast_1d(np.asarray(alphas, dtype=float))
    C = np.empty((len(y), len(a)))
    c = np.full(len(a), float(C0))
    for k, v in enumerate(y):
        C[k] = c
        if not math.isnan(v):
            c = (1.0 - a) * c + a * v
    return C


def mse(y: np.ndarray, alphas, C0: float) -> np.ndarray:
    """One-step MSE of eq. (2) for each gain; the first step (prediction C0 for every gain) is excluded."""
    C = replay(y, alphas, C0)
    ok = ~np.isnan(y)
    ok[:1] = False
    if not ok.any():
        return np.full(C.shape[1], np.nan)
    e = C[ok] - y[ok, None]
    return np.mean(e * e, axis=0)


def golden(f: Callable[[float], float], lo: float, hi: float, tol: float = 1e-5) -> float:
    """Golden-section search of a unimodal f on [lo, hi]."""
    g = (math.sqrt(5.0) - 1.0) / 2.0
    a, b = lo, hi
    c, d = b - g * (b - a), a + g * (b - a)
    fc, fd = f(c), f(d)
    while b - a > tol:
        if fc <= fd:
            b, d, fd = d, c, fc
            c = b - g * (b - a)
            fc = f(c)
        else:
            a, c, fc = c, d, fd
            d = a + g * (b - a)
            fd = f(d)
    return (a + b) / 2.0


def fit_alpha(y: np.ndarray, C0: float, grid: np.ndarray = ALPHA_GRID, refine: bool = True) -> Tuple[float, float]:
    """(a) alpha_star and its MSE."""
    return fit_alpha_runs([y], [C0], grid=grid, refine=refine)


def mse_runs(ys: Sequence[np.ndarray], C0s: Sequence[float], alphas) -> np.ndarray:
    """One-step MSE of eq. (2) pooled over runs: each run is replayed from its own C0 and the squared
    errors of every run (first step of each run excluded) enter one mean."""
    a = np.atleast_1d(np.asarray(alphas, dtype=float))
    sse, cnt = np.zeros(len(a)), 0
    for y, C0 in zip(ys, C0s):
        y = np.asarray(y, dtype=float)
        if len(y) == 0:
            continue
        C = replay(y, a, C0)
        ok = ~np.isnan(y)
        ok[:1] = False
        if ok.any():
            e = C[ok] - y[ok, None]
            sse += np.sum(e * e, axis=0)
            cnt += int(ok.sum())
    return sse / cnt if cnt else np.full(len(a), np.nan)


def fit_alpha_runs(ys: Sequence[np.ndarray], C0s: Sequence[float], grid: np.ndarray = ALPHA_GRID,
                   refine: bool = True) -> Tuple[float, float]:
    """(a) over several runs: alpha_star = argmin of the pooled one-step MSE, and that MSE."""
    m = mse_runs(ys, C0s, grid)
    if np.all(np.isnan(m)):
        return float("nan"), float("nan")
    k = int(np.nanargmin(m))
    best, best_m = float(grid[k]), float(m[k])
    if refine:
        lo, hi = max(float(grid[0]), best - 0.01), min(float(grid[-1]), best + 0.01)
        a = golden(lambda v: float(mse_runs(ys, C0s, [v])[0]), lo, hi)
        ma = float(mse_runs(ys, C0s, [a])[0])
        if ma < best_m:
            best, best_m = a, ma
    return best, best_m


def k_y(y: np.ndarray, T3_Ts: float, c_y: float = 1.0, B_ctl_bar: float = 0.0) -> np.ndarray:
    """K_y(t) = c_y y(t) T3_Ts - B_ctl_bar, the measured capacity of phase 3 (T3_Ts = (T3 - T_m)/Ts)."""
    return c_y * np.asarray(y, dtype=float) * T3_Ts - B_ctl_bar


def coverage(y: np.ndarray, C_hat: np.ndarray, U_grid: np.ndarray = U_GRID, T3_Ts: float = T3_TS_DEFAULT,
             c_y: float = 1.0, B_ctl_bar: float = 0.0) -> np.ndarray:
    """(b) PICP(U) for each U: fraction of the steps with y and C_hat where K_y(t) >= U K_hat(t),
    K_hat(t) = C_hat(t) T3_Ts (with c_y = 1 and B_ctl_bar = 0: y(t) >= U C_hat(t))."""
    ok = ~(np.isnan(y) | np.isnan(C_hat))
    if not ok.any():
        return np.full(len(U_grid), np.nan)
    # K_y >= U K_hat compared in units of C (divided by T3_Ts), exact for B_ctl_bar = 0
    z = c_y * y[ok] - B_ctl_bar / T3_Ts
    return np.mean(z[:, None] >= np.asarray(U_grid)[None, :] * C_hat[ok, None], axis=0)


def largest_U(gamma: np.ndarray, U_grid: np.ndarray, target: float) -> Optional[float]:
    ok = np.where(gamma >= target)[0]
    return float(U_grid[ok[-1]]) if len(ok) else None


# --------------------------------------------------------------------------- (c) backlog what-if


def _c_y(c_y: Optional[float], kappa: Optional[float]) -> float:
    """c_y, with kappa (its name in earlier versions) as a backward-compatible alias."""
    if kappa is not None:
        if c_y is not None and float(c_y) != float(kappa):
            raise ValueError("c_y and kappa (its old name) differ")
        return float(kappa)
    return 1.0 if c_y is None else float(c_y)


def backlog(y: np.ndarray, C_hat: np.ndarray, U_grid=U_GRID, T3_Ts: float = T3_TS_DEFAULT,
            fill: Optional[np.ndarray] = None, c_y: Optional[float] = None,
            kappa: Optional[float] = None, B_ctl_bar: float = 0.0,
            Q: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
    """Backlog predictor of Corollary 1 in Lindley form (C5) on a DTA series, for every U of U_grid.

    Convention at the step boundary: q(t) is the number of bytes of steps 0..t still in the send
    buffers of the data bridges at the end of phase 3 of step t, and these are the bytes in the
    buffers at the start of phase 3 of step t+1 (the drain during phases 1 and 2 of step t+1 is
    neglected, so the prediction is conservative). q(-1) = 0: phase 3 of the first step of the series
    starts with empty buffers. Phase 3 of step t receives X(t) = sum_{i in M(t)} a_i(t) bytes, the
    admitted bytes of Algorithm 1, and the channel carries K_y(t) = c_y y(t) T3_Ts - B_ctl_bar bytes
    (Assumption 2, c_y y in place of C; T3_Ts = (T3 - T_m)/Ts):

        q(t) = max(0, q(t-1) + X(t) - K_y(t)),   X(t) = f(t) max(0, U C_hat(t) T3_Ts - Q(t)).

    fill holds f(t) = A(t) / cap(t) of the logged run (fill_series; NaN or None: f = 1, the cap
    max(0, U K_hat(t) - Q(t)), which bounds the admitted bytes because sum_i a_i(t) <= m(t) b(t)); Q holds the
    logged backlog Q(t) (None: 0). For the logged U, X(t) is the logged A(t); for other U the admitted bytes
    scale with the cap (what-if approximation of the module docstring).

    Step t delivers every message of level 1 or 2 within the step iff q(t) = 0. This is Corollary 1:
    q(t-1) = 0 (empty buffers at the start of phase 3 of step t) and C(t) >= X(t) Ts/T3 give q(t) = 0
    (Proposition 2); the service condition at every step tau <= t gives q(tau) = 0 for every tau <= t;
    and a step whose service condition holds after a step with q(t-1) > 0 can fail. A step without y(t)
    or C_hat(t) leaves q unchanged and is not evaluated.

    The recursion runs step by step (vectorized over U), so q(t) = 0 is exact and does not depend on
    the rounding of a cumulative sum.
    Returns q (n x len(U_grid), bytes) and the mask of the evaluated steps (n,)."""
    cy = _c_y(c_y, kappa)
    y = np.asarray(y, dtype=float)
    C = np.asarray(C_hat, dtype=float)
    U = np.atleast_1d(np.asarray(U_grid, dtype=float))
    f = np.ones(len(y)) if fill is None else np.where(np.isnan(np.asarray(fill, dtype=float)), 1.0,
                                                       np.asarray(fill, dtype=float))
    Qv = np.zeros(len(y)) if Q is None else np.nan_to_num(np.asarray(Q, dtype=float), nan=0.0)
    ok = ~(np.isnan(y) | np.isnan(C))
    q = np.zeros((len(y), len(U)))
    cur = np.zeros(len(U))
    for t in range(len(y)):
        if ok[t]:
            # X(t) - K_y(t) = (f max(0, U C - Q / T3_Ts) - c_y y) T3_Ts + B_ctl_bar; with Q = 0 and B_ctl_bar = 0 the
            # bracket is that of q(t) = max(0, q(t-1) + (f U C - c_y y) T3_Ts), so q(t) = 0 stays exact
            cur = np.maximum(0.0, cur + (f[t] * np.maximum(0.0, U * C[t] - Qv[t] / T3_Ts) - cy * y[t]) * T3_Ts
                             + B_ctl_bar)
        q[t] = cur
    return q, ok


def backlog_admitted(y: np.ndarray, A: np.ndarray, T3_Ts: float = T3_TS_DEFAULT,
                     c_y: Optional[float] = None, kappa: Optional[float] = None,
                     B_ctl_bar: float = 0.0, Q: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray]:
    """The predictor of C5 on the logged run itself (DMR-hat): q_hat(t) = max(0, q_hat(t-1) + A(t) - K_y(t)),
    K_y(t) = c_y y(t) T3_Ts - B_ctl_bar, with the admitted bytes A(t) = sum_i a_i(t) (admitted_bytes). With the
    logged backlog Q(t) of the AS (Q_i(t) over M(t) plus Qbar_i(t) outside M(t)), the backlog carried into
    phase 3 of step t is max(q_hat(t-1), Q(t)): the measured bound replaces a smaller prediction. A step without
    y(t) or A(t) leaves q unchanged and is not evaluated. Returns q (n,) and the mask of the evaluated steps."""
    cy = _c_y(c_y, kappa)
    y, A = np.asarray(y, dtype=float), np.asarray(A, dtype=float)
    Qv = np.zeros(len(y)) if Q is None else np.nan_to_num(np.asarray(Q, dtype=float), nan=0.0)
    ok = ~(np.isnan(y) | np.isnan(A))
    q = np.zeros(len(y))
    cur = 0.0
    for t in range(len(y)):
        if ok[t]:
            cur = max(0.0, max(cur, Qv[t]) + A[t] - (cy * y[t] * T3_Ts - B_ctl_bar))
        q[t] = cur
    return q, ok


def dmr_predicted(y: np.ndarray, C_hat: np.ndarray, U_grid=U_GRID, T3_Ts: float = T3_TS_DEFAULT,
                  fill: Optional[np.ndarray] = None, c_y: Optional[float] = None,
                  kappa: Optional[float] = None, B_ctl_bar: float = 0.0,
                  Q: Optional[np.ndarray] = None) -> np.ndarray:
    """(c) predicted deadline miss ratio DMR(U): fraction of the evaluated steps with q(t) > 0."""
    q, ok = backlog(y, C_hat, U_grid, T3_Ts, fill=fill, c_y=c_y, kappa=kappa, B_ctl_bar=B_ctl_bar, Q=Q)
    if not ok.any():
        return np.full(q.shape[1], np.nan)
    return np.mean(q[ok] > 0.0, axis=0)


def phi_predicted(y: np.ndarray, C_hat: np.ndarray, U_grid=U_GRID, T3_Ts: float = T3_TS_DEFAULT,
                  fill: Optional[np.ndarray] = None, c_y: Optional[float] = None,
                  kappa: Optional[float] = None) -> np.ndarray:
    """Predicted full-delivery fraction phi(U) = 1 - DMR(U) (name of earlier versions, kept as an alias)."""
    return 1.0 - dmr_predicted(y, C_hat, U_grid, T3_Ts, fill=fill, c_y=c_y, kappa=kappa)


def dmr_predicted_runs(runs: Sequence[Tuple], U_grid=U_GRID, T3_Ts: float = T3_TS_DEFAULT,
                       c_y: Optional[float] = None, kappa: Optional[float] = None,
                       B_ctl_bar: float = 0.0) -> np.ndarray:
    """DMR(U) pooled over runs, each run (y, C_hat), (y, C_hat, fill) or (y, C_hat, fill, Q); the recursion
    restarts with q(-1) = 0 in each run."""
    num, den = np.zeros(len(np.atleast_1d(U_grid))), 0
    for run in runs:
        y, C = run[0], run[1]
        f = run[2] if len(run) > 2 else None
        Qr = run[3] if len(run) > 3 else None
        q, ok = backlog(y, C, U_grid, T3_Ts, fill=f, c_y=c_y, kappa=kappa, B_ctl_bar=B_ctl_bar, Q=Qr)
        num += np.sum(q[ok] > 0.0, axis=0)
        den += int(ok.sum())
    return num / den if den else np.full(len(num), np.nan)


def phi_predicted_runs(runs: Sequence[Tuple], U_grid=U_GRID, T3_Ts: float = T3_TS_DEFAULT,
                       c_y: Optional[float] = None, kappa: Optional[float] = None) -> np.ndarray:
    """phi(U) = 1 - DMR(U) pooled over runs (alias of earlier versions)."""
    return 1.0 - dmr_predicted_runs(runs, U_grid, T3_Ts, c_y=c_y, kappa=kappa)


# --------------------------------------------------------------------------- (b) bootstrap bound of gamma


def _block_length(runs: Sequence[Tuple[np.ndarray, np.ndarray]], U_grid) -> Tuple[int, float]:
    """ceil(2 tau_int) of the slack K_y(t) - U K_hat(t) (runs of (K_y, K_hat)), tau_int pooled over the runs,
    largest over U."""
    from .analysis import tau_int_runs
    tau = 0.5
    U = np.atleast_1d(np.asarray(U_grid, dtype=float))
    for u in np.unique(np.r_[U[::10], U[-1]]):
        tau = max(tau, tau_int_runs([Ky - u * Kh for Ky, Kh in runs]))
    return max(1, int(math.ceil(2.0 * tau))), tau


def gamma_bounds(runs: Sequence[Tuple[np.ndarray, np.ndarray]], U_grid=U_GRID, conf: float = 0.95, B: int = 1000,
                 block: Optional[int] = None, seed: int = 0, T3_Ts: float = T3_TS_DEFAULT, c_y: float = 1.0,
                 B_ctl_bar: float = 0.0) -> Dict:
    """PICP(U) (gamma) and its lower one-sided `conf` bound by circular block bootstrap, pooled over runs;
    a step covers U when K_y(t) >= U K_hat(t) (K_y = c_y y T3_Ts - B_ctl_bar, K_hat = C_hat T3_Ts).

    Each run (y, C_hat) contributes its circular blocks of `block` consecutive steps (steps without y
    or C_hat removed); a resample draws ceil(n / block) blocks with replacement from the blocks of every
    run, and the same resamples serve every U, so gamma_low(U) does not increase with U by sampling
    noise alone. Returns {"gamma", "gamma_low", "block", "tau_margin", "n"}."""
    U = np.atleast_1d(np.asarray(U_grid, dtype=float))
    clean = []
    for y, C in runs:
        y, C = np.asarray(y, dtype=float), np.asarray(C, dtype=float)
        ok = ~(np.isnan(y) | np.isnan(C))
        if ok.any():   # K_y / T3_Ts and K_hat / T3_Ts (the comparison and tau_int do not change with the scale)
            clean.append((c_y * y[ok] - B_ctl_bar / T3_Ts, C[ok]))
    if not clean:
        nan = np.full(len(U), np.nan)
        return {"gamma": nan, "gamma_low": nan, "block": 0, "tau_margin": float("nan"), "n": 0}
    if block is None:
        block, tau = _block_length(clean, U)
    else:
        tau = float("nan")
    sums, lens = [], []
    n_tot, hits = 0, np.zeros(len(U))
    for Ky, Kh in clean:
        ind = (Ky[:, None] >= U[None, :] * Kh[:, None]).astype(float)
        n = len(Ky)
        n_tot += n
        hits += ind.sum(axis=0)
        l = min(block, n)
        ext = np.vstack([ind, ind[:l - 1]]) if l > 1 else ind
        cs = np.vstack([np.zeros((1, len(U))), np.cumsum(ext, axis=0)])
        sums.append(cs[l:l + n] - cs[:n])
        lens.append(np.full(n, l, dtype=float))
    S, Ls = np.vstack(sums), np.concatenate(lens)
    gamma = hits / n_tot
    rng = np.random.default_rng(seed)
    nb = int(math.ceil(n_tot / float(block)))
    reps = np.empty((B, len(U)))
    for b in range(B):
        idx = rng.integers(0, len(Ls), nb)
        reps[b] = S[idx].sum(axis=0) / Ls[idx].sum()
    low = np.quantile(reps, 1.0 - conf, axis=0)
    return {"gamma": gamma, "gamma_low": low, "block": int(block), "tau_margin": float(tau), "n": int(n_tot)}


# --------------------------------------------------------------------------- outer loop


def _T3_Ts(records: Sequence[Dict]) -> float:
    """(T3 - T_m)/Ts from the recorded K_hat = C_hat (T3 - T_m)/Ts (median), or the paper value T3/Ts."""
    v = [float(r["K_hat"]) / float(r["C_hat"]) for r in records if r.get("K_hat") is not None and r.get("C_hat")]
    return float(np.median(v)) if v else T3_TS_DEFAULT


def _C0(y: np.ndarray, Crec: np.ndarray) -> float:
    finite = Crec[~np.isnan(Crec)]
    return float(finite[0]) if len(finite) else float(np.nanmean(y))


def backlog_series(records: Iterable[Dict]) -> np.ndarray:
    """Q(t) per step sorted by t (series order), backlog_of each record."""
    by_t = {}
    for r in records:
        by_t[int(r["t"])] = r
    return np.asarray([backlog_of(by_t[t]) for t in sorted(by_t)], dtype=float)


def _calibrate(runs: Sequence[Tuple], C0s: Sequence[float], target: float,
               alpha: Optional[float], U_grid, conf: float, B: int, block: Optional[int], seed: int,
               T3_Ts: float, c_y: float = 1.0, B_ctl_bar: float = 0.0, refine: bool = True) -> Dict:
    """runs: (y, C_hat recorded), (y, C_hat recorded, f) or (y, C_hat recorded, f, Q) per run, f = fill_series
    and Q = backlog_series of the run. Eq. (2) smooths c_y y, so alpha* fits the replay to c_y y. refine False
    keeps alpha* on the grid 0.01..1 (the calibration of Section VI-E; True adds the golden-section refinement)."""
    ys = [r[0] for r in runs]
    zs = [c_y * np.asarray(y, dtype=float) for y in ys]
    fills = [r[2] if len(r) > 2 else None for r in runs]
    Qs = [r[3] if len(r) > 3 else None for r in runs]
    if alpha is None:
        a_star, m_star = fit_alpha_runs(zs, C0s, refine=refine)
    else:
        a_star, m_star = float(alpha), float(mse_runs(zs, C0s, [alpha])[0])
    rep = [(y, replay(z, [a_star], C0)[:, 0]) for y, z, C0 in zip(ys, zs, C0s)]
    gb = gamma_bounds(rep, U_grid, conf=conf, B=B, block=block, seed=seed, T3_Ts=T3_Ts, c_y=c_y,
                      B_ctl_bar=B_ctl_bar)
    dmr = dmr_predicted_runs([(y, C, f, q) for (y, C), f, q in zip(rep, fills, Qs)], U_grid, T3_Ts, c_y=c_y,
                             B_ctl_bar=B_ctl_bar)
    n_fill = sum(int(np.sum(~np.isnan(f))) for f in fills if f is not None)
    return {
        "alpha_star": a_star, "mse": m_star, "rmse": math.sqrt(m_star) if m_star == m_star else None,
        "PICP_nom": target, "gamma_target": target, "c_y": c_y, "kappa": c_y, "B_ctl_bar": B_ctl_bar,
        "n_admitted": n_fill,
        "conf": conf, "block": gb["block"], "tau_margin": gb["tau_margin"], "T3_Ts": T3_Ts,
        "U_star": largest_U(gb["gamma_low"], U_grid, target),
        "U_star_point": largest_U(gb["gamma"], U_grid, target),
        "whatif": [{"U": float(u), "PICP": float(g), "PICP_low": float(gl), "DMR": float(d),
                    "gamma": float(g), "gamma_low": float(gl), "phi": float(1.0 - d)}
                   for u, g, gl, d in zip(U_grid, gb["gamma"], gb["gamma_low"], dmr)],
    }


def outer_loop(records: Sequence[Dict], target: float = 0.99, window: Optional[int] = None,
               alpha: Optional[float] = None, C0: Optional[float] = None,
               U_grid: np.ndarray = U_GRID, conf: float = 0.95, B: int = 1000, block: Optional[int] = None,
               seed: int = 0, T3_Ts: Optional[float] = None, c_y: Optional[float] = None,
               kappa: Optional[float] = None, B_ctl_bar: float = 0.0) -> Dict:
    """Outer loop over the DTA (or its last `window` steps). With `alpha` given, (a) is skipped.
    C0 defaults to the C_hat recorded in the first step considered (the AS value at that step).
    U_star: largest U with the lower `conf` block-bootstrap bound of PICP >= target (PICP_nom; U_star_point:
    the rule on the point estimate). whatif[k]["DMR"]: DMR(U) of the backlog predictor (function backlog)
    with the admitted bytes of the records and c_y (kappa: old name). T3_Ts is (T3 - T_m)/Ts (default: from the
    recorded K_hat / C_hat). The keys used by fleetdt.as_service (U_star, alpha_star, mse, n_y) keep their
    meaning."""
    cy = _c_y(c_y, kappa)
    ts, y, Crec = series(records)
    r = T3_Ts if T3_Ts is not None else _T3_Ts(records)
    f = fill_series(records, r)
    Q = backlog_series(records)
    if window:
        ts, y, Crec, f, Q = ts[-window:], y[-window:], Crec[-window:], f[-window:], Q[-window:]
    if len(ts) == 0:
        raise ValueError("empty DTA")
    if C0 is None:
        C0 = _C0(y, Crec)
    res = {"t_first": int(ts[0]), "t_last": int(ts[-1]), "n_steps": int(len(ts)), "n_y": int(np.sum(~np.isnan(y))),
           "window": window, "C0": C0}
    res.update(_calibrate([(y, Crec, f, Q)], [C0], target, alpha, U_grid, conf, B, block, seed, r, cy, B_ctl_bar))
    return res


def _jsonable(o):
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError("%r is not JSON serializable" % type(o))


def run_job(dta: str, target: float, window: Optional[int], out: str, c_y: float = 1.0, B_ctl_bar: float = 0.0,
            T3_Ts: Optional[float] = None) -> None:
    """Outer loop of fleetdt.as_service in a child process: outer_loop over the DTA file `dta` with the c_y,
    B_ctl_bar and (T3 - T_m)/Ts of the DTP of the AS, written to `out` as {"result": ...} (or {"error": ...}
    when the DTA cannot be read); the file appears complete (written to out.tmp, then renamed)."""
    import os
    try:
        recs = load_dta(dta)
        msg = {"result": outer_loop(recs, target=target, window=window, c_y=c_y, B_ctl_bar=B_ctl_bar,
                                    T3_Ts=T3_Ts) if recs else None}
    except (OSError, ValueError) as e:
        msg = {"error": str(e)}
    with open(out + ".tmp", "w", encoding="utf-8") as f:
        json.dump(msg, f, default=_jsonable)
    os.replace(out + ".tmp", out)


def run_M(records: Sequence[Dict]) -> int:
    """Number of vessels M of a run: the largest m(t) of its steps."""
    return max([len(r.get("vessels") or []) if r.get("vessels") is not None else int(r.get("m") or 0)
                for r in records] or [0])


def calibrate_by_M(runs: Sequence[Sequence[Dict]], target: float = 0.99, alpha: Optional[float] = None,
                   U_grid: np.ndarray = U_GRID, conf: float = 0.95, B: int = 1000, block: Optional[int] = None,
                   seed: int = 0, T3_Ts: Optional[float] = None, c_y: Optional[float] = None,
                   kappa: Optional[float] = None, B_ctl_bar: float = 0.0) -> Dict[int, Dict]:
    """alpha* and U* per number of vessels M (calibration session): the runs (one DTA each) are grouped
    by run_M; alpha* minimizes the MSE pooled over the runs of the group, gamma and its block-bootstrap
    bound pool the steps of the group (blocks inside each run), and DMR(U) restarts the backlog in each
    run. Returns {M: result} with the keys of outer_loop plus "runs" and "n_y"."""
    cy = _c_y(c_y, kappa)
    groups: Dict[int, List[Sequence[Dict]]] = {}
    for recs in runs:
        if recs:
            groups.setdefault(run_M(recs), []).append(recs)
    out = {}
    for M, rr in sorted(groups.items()):
        if M == 0:
            continue
        ser, C0s = [], []
        r = T3_Ts if T3_Ts is not None else _T3_Ts([x for recs in rr for x in recs])
        for recs in rr:
            _, y, Crec = series(recs)
            ser.append((y, Crec, fill_series(recs, r), backlog_series(recs)))
            C0s.append(_C0(y, Crec))
        res = {"M": M, "runs": len(rr), "n_steps": int(sum(len(s[0]) for s in ser)),
               "n_y": int(sum(np.sum(~np.isnan(s[0])) for s in ser))}
        res.update(_calibrate(ser, C0s, target, alpha, U_grid, conf, B, block, seed, r, cy, B_ctl_bar))
        out[M] = res
    return out


def sliding(records: Sequence[Dict], window: int, every: Optional[int] = None, target: float = 0.99,
            B: int = 1000, c_y: Optional[float] = None, B_ctl_bar: float = 0.0,
            T3_Ts: Optional[float] = None) -> List[Dict]:
    """Recomputation over the last `window` steps every `every` steps (variant O3 of the experiments)."""
    recs = sorted(records, key=lambda r: int(r["t"]))
    every = every or window
    out = []
    for end in range(window, len(recs) + 1, every):
        res = outer_loop(recs[end - window:end], target=target, B=B, c_y=c_y, B_ctl_bar=B_ctl_bar, T3_Ts=T3_Ts)
        res.pop("whatif")
        out.append(res)
    return out


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.outer", description="Outer loop of the Fleet-DT AS")
    ap.add_argument("dta", nargs="+", help="DTA file(s) (JSON lines of G(t)); with --by-M one file per run")
    ap.add_argument("--target", type=float, default=0.99, help="nominal coverage PICP_nom")
    ap.add_argument("--dtp", default=None, help="DTP file: c_y, B_ctl_bar and (T3 - T_m)/Ts (the options override)")
    ap.add_argument("--c-y", "--kappa", dest="c_y", type=float, default=None,
                    help="c_y, lower bound of C(t)/y(t) in eq. (2), PICP and the backlog predictor (default: the "
                         "DTP, else 1; --kappa: old name)")
    ap.add_argument("--B-ctl-bar", dest="B_ctl_bar", type=float, default=None,
                    help="bytes of control traffic outside the data bridges in phase 3 (default: the DTP, else 0)")
    ap.add_argument("--window", type=int, default=None, help="last W steps (with --every: sliding windows)")
    ap.add_argument("--every", type=int, default=None, help="sliding recomputation every K steps (needs --window)")
    ap.add_argument("--by-M", action="store_true", help="alpha* and U* per number of vessels M over the runs")
    ap.add_argument("--conf", type=float, default=0.95, help="confidence of the lower bound of gamma")
    ap.add_argument("--B", type=int, default=1000, help="bootstrap resamples")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    r3 = None
    if a.dtp:
        from .as_model import DTP
        d = DTP.load(a.dtp)
        a.c_y = d.c_y if a.c_y is None else a.c_y
        a.B_ctl_bar = d.B_ctl_bar if a.B_ctl_bar is None else a.B_ctl_bar
        r3 = d.T3m / d.Ts
    a.c_y = 1.0 if a.c_y is None else a.c_y
    a.B_ctl_bar = 0.0 if a.B_ctl_bar is None else a.B_ctl_bar
    if a.by_M or len(a.dta) > 1:
        res = calibrate_by_M([load_dta(p) for p in a.dta], target=a.target, conf=a.conf, B=a.B, c_y=a.c_y,
                             B_ctl_bar=a.B_ctl_bar, T3_Ts=r3)
        if a.json:
            print(json.dumps({str(k): v for k, v in res.items()}, indent=1))
            return
        for M, r in res.items():
            print("M = %d: %d runs, %d steps with y; alpha* = %.4f, RMSE = %.1f, U* = %s (point %s), block %d" % (
                M, r["runs"], r["n_y"], r["alpha_star"], r["rmse"] or float("nan"), r["U_star"], r["U_star_point"],
                r["block"]))
        return
    recs = load_dta(a.dta[0])
    if a.every and a.window:
        res = sliding(recs, a.window, a.every, a.target, B=a.B, c_y=a.c_y, B_ctl_bar=a.B_ctl_bar, T3_Ts=r3)
        if a.json:
            print(json.dumps(res, indent=1))
            return
        print("%8s %8s %10s %8s" % ("t_last", "alpha*", "rmse", "U*"))
        for r in res:
            print("%8d %8.4f %10.1f %8s" % (r["t_last"], r["alpha_star"], r["rmse"] or float("nan"), r["U_star"]))
        return
    res = outer_loop(recs, target=a.target, window=a.window, conf=a.conf, B=a.B, c_y=a.c_y, B_ctl_bar=a.B_ctl_bar,
                     T3_Ts=r3)
    if a.json:
        print(json.dumps(res, indent=1))
        return
    print("steps %d..%d (%d, %d with y), C0 = %.1f" % (res["t_first"], res["t_last"], res["n_steps"], res["n_y"], res["C0"]))
    print("alpha* = %.4f   RMSE = %.1f bytes" % (res["alpha_star"], res["rmse"] or float("nan")))
    print("U* = %s for the lower %.0f %% bound of PICP >= %.3f (point estimate: %s), block %d steps, c_y = %g" % (
        res["U_star"], 100 * res["conf"], res["PICP_nom"], res["U_star_point"], res["block"], res["c_y"]))
    print("%6s %8s %9s %8s" % ("U", "PICP", "PICP_lo", "DMR"))
    for w in res["whatif"][::5]:
        print("%6.2f %8.4f %9.4f %8.4f" % (w["U"], w["PICP"], w["PICP_low"], w["DMR"]))


if __name__ == "__main__":
    main()
