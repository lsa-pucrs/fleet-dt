"""Outer loop of the AS (contribution (iv) of the Fleet-DT paper): tunes the gain alpha of eq. (2) and the
target utilization U of eq. (3) from the DTA.

  (a) alpha_star = argmin of the one-step mean squared error of eq. (2) replayed on y(t)
      (grid 0.01..1 step 0.01, then golden-section refinement around the best grid point); with several
      runs, the MSE is pooled over the runs (each run replayed from its own C0);
  (b) gamma(U) = fraction of steps with y(t) >= U C_hat(t), C_hat(t) recomputed causally with alpha_star;
      gamma_low(U) = lower one-sided 95 % bound of gamma(U) by circular block bootstrap (blocks of
      ceil(2 tau_int) steps, tau_int of the margin series y(t) - U C_hat(t), blocks inside each run,
      the same resamples for every U of the grid);
      U_star = largest U of the grid 0.30..1.00 with gamma_low(U) >= gamma_target
      (U_star_point: the same rule on the point estimate gamma(U));
  (c) what-if table: for each U, gamma(U), gamma_low(U) and the predicted full-delivery fraction
      phi(U) of the backlog recursion of Corollary 1 (function backlog):
          q(t) = max(0, q(t-1) + U C_hat(t) T3/Ts - y(t) T3/Ts),   q(-1) = 0,
          phi(U) = fraction of the steps with q(t) = 0.
      y(t) stands for C(t). Proposition 2 needs C(t) >= U C_hat(t) and empty send buffers at the start of
      phase 3, so phi(U) <= gamma(U): a shortfall in step t-1 leaves bytes that delay step t.

alpha* and U* per number of vessels M: calibrate_by_M (one DTA per run, M of a run = largest m(t)).

  python -m fleetdt.outer dta.jsonl [--target 0.99] [--window W] [--every K] [--json]
  python -m fleetdt.outer run1.jsonl run2.jsonl ... --by-M [--target 0.99] [--json]
"""
from __future__ import annotations

import argparse
import json
import math
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

ALPHA_GRID = np.round(np.arange(1, 101) / 100.0, 2)
U_GRID = np.round(np.arange(30, 101) / 100.0, 2)


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


def coverage(y: np.ndarray, C_hat: np.ndarray, U_grid: np.ndarray = U_GRID) -> np.ndarray:
    """(b) gamma(U) for each U: fraction of the steps with y and C_hat where y(t) >= U C_hat(t)."""
    ok = ~(np.isnan(y) | np.isnan(C_hat))
    if not ok.any():
        return np.full(len(U_grid), np.nan)
    return np.mean(y[ok, None] >= np.asarray(U_grid)[None, :] * C_hat[ok, None], axis=0)


def largest_U(gamma: np.ndarray, U_grid: np.ndarray, target: float) -> Optional[float]:
    ok = np.where(gamma >= target)[0]
    return float(U_grid[ok[-1]]) if len(ok) else None


# --------------------------------------------------------------------------- (c) backlog what-if

T3_TS_DEFAULT = 0.095 / 0.125   # T3 / Ts of the DTP of the paper (Ts = 125 ms, T1 = 20 ms, T2 = 10 ms)


def backlog(y: np.ndarray, C_hat: np.ndarray, U_grid=U_GRID, T3_Ts: float = T3_TS_DEFAULT) -> Tuple[np.ndarray, np.ndarray]:
    """Backlog recursion of Corollary 1 on a DTA series, for every U of U_grid.

    Convention at the step boundary: q(t) is the number of bytes of steps 0..t still in the send
    buffers of the data bridges at the end of phase 3 of step t, and these are the bytes in the
    buffers at the start of phase 3 of step t+1 (the drain during phases 1 and 2 of step t+1 is
    neglected, so the prediction is conservative). q(-1) = 0: phase 3 of the first step of the series
    starts with empty buffers. Phase 3 of step t receives X(t) = U C_hat(t) T3/Ts bytes (every vessel
    of M(t) fills its grant, sum_i a_i(t) <= m(t) b(t) = K_hat(t)) and the channel carries y(t) T3/Ts
    bytes (Assumption 2, y in place of C):

        q(t) = max(0, q(t-1) + U C_hat(t) T3/Ts - y(t) T3/Ts).

    Step t delivers every message of level 1 or 2 within the step iff q(t) = 0. This is Corollary 1:
    q(t-1) = 0 (empty buffers at the start of phase 3 of step t) and y(t) >= U C_hat(t) give q(t) = 0
    (Proposition 2); C(tau) >= U C_hat(tau) for every tau <= t gives q(tau) = 0 for every tau <= t; and
    a step with y(t) >= U C_hat(t) after a step with q(t-1) > 0 can fail. A step without y(t) or
    C_hat(t) leaves q unchanged and is not evaluated.

    The recursion runs step by step (vectorized over U), so q(t) = 0 is exact and does not depend on
    the rounding of a cumulative sum.
    Returns q (n x len(U_grid), bytes) and the mask of the evaluated steps (n,)."""
    y = np.asarray(y, dtype=float)
    C = np.asarray(C_hat, dtype=float)
    U = np.atleast_1d(np.asarray(U_grid, dtype=float))
    ok = ~(np.isnan(y) | np.isnan(C))
    q = np.zeros((len(y), len(U)))
    cur = np.zeros(len(U))
    for t in range(len(y)):
        if ok[t]:
            cur = np.maximum(0.0, cur + (U * C[t] - y[t]) * T3_Ts)
        q[t] = cur
    return q, ok


def phi_predicted(y: np.ndarray, C_hat: np.ndarray, U_grid=U_GRID, T3_Ts: float = T3_TS_DEFAULT) -> np.ndarray:
    """(c) predicted full-delivery fraction phi(U): fraction of the evaluated steps with q(t) = 0."""
    q, ok = backlog(y, C_hat, U_grid, T3_Ts)
    if not ok.any():
        return np.full(q.shape[1], np.nan)
    return np.mean(q[ok] <= 0.0, axis=0)


def phi_predicted_runs(runs: Sequence[Tuple[np.ndarray, np.ndarray]], U_grid=U_GRID,
                       T3_Ts: float = T3_TS_DEFAULT) -> np.ndarray:
    """phi(U) pooled over runs; the recursion restarts with q(-1) = 0 in each run."""
    num, den = np.zeros(len(np.atleast_1d(U_grid))), 0
    for y, C in runs:
        q, ok = backlog(y, C, U_grid, T3_Ts)
        num += np.sum(q[ok] <= 0.0, axis=0)
        den += int(ok.sum())
    return num / den if den else np.full(len(num), np.nan)


# --------------------------------------------------------------------------- (b) bootstrap bound of gamma


def _block_length(runs: Sequence[Tuple[np.ndarray, np.ndarray]], U_grid) -> Tuple[int, float]:
    """ceil(2 tau_int) of the margin y(t) - U C_hat(t), tau_int pooled over the runs, largest over U."""
    from .analysis import tau_int_runs
    tau = 0.5
    U = np.atleast_1d(np.asarray(U_grid, dtype=float))
    for u in np.unique(np.r_[U[::10], U[-1]]):
        tau = max(tau, tau_int_runs([y - u * C for y, C in runs]))
    return max(1, int(math.ceil(2.0 * tau))), tau


def gamma_bounds(runs: Sequence[Tuple[np.ndarray, np.ndarray]], U_grid=U_GRID, conf: float = 0.95, B: int = 1000,
                 block: Optional[int] = None, seed: int = 0) -> Dict:
    """gamma(U) and its lower one-sided `conf` bound by circular block bootstrap, pooled over runs.

    Each run (y, C_hat) contributes its circular blocks of `block` consecutive steps (steps without y
    or C_hat removed); a resample draws ceil(n / block) blocks with replacement from the blocks of every
    run, and the same resamples serve every U, so gamma_low(U) does not increase with U by sampling
    noise alone. Returns {"gamma", "gamma_low", "block", "tau_margin", "n"}."""
    U = np.atleast_1d(np.asarray(U_grid, dtype=float))
    clean = []
    for y, C in runs:
        y, C = np.asarray(y, dtype=float), np.asarray(C, dtype=float)
        ok = ~(np.isnan(y) | np.isnan(C))
        if ok.any():
            clean.append((y[ok], C[ok]))
    if not clean:
        nan = np.full(len(U), np.nan)
        return {"gamma": nan, "gamma_low": nan, "block": 0, "tau_margin": float("nan"), "n": 0}
    if block is None:
        block, tau = _block_length(clean, U)
    else:
        tau = float("nan")
    sums, lens = [], []
    n_tot, hits = 0, np.zeros(len(U))
    for y, C in clean:
        ind = (y[:, None] >= U[None, :] * C[:, None]).astype(float)
        n = len(y)
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
    """T3/Ts from the recorded K_hat = U C_hat T3/Ts (median), or the paper value."""
    v = [float(r["K_hat"]) / (float(r["U"]) * float(r["C_hat"])) for r in records
         if r.get("K_hat") is not None and r.get("U") and r.get("C_hat")]
    return float(np.median(v)) if v else T3_TS_DEFAULT


def _C0(y: np.ndarray, Crec: np.ndarray) -> float:
    finite = Crec[~np.isnan(Crec)]
    return float(finite[0]) if len(finite) else float(np.nanmean(y))


def _calibrate(runs: Sequence[Tuple[np.ndarray, np.ndarray]], C0s: Sequence[float], target: float,
               alpha: Optional[float], U_grid, conf: float, B: int, block: Optional[int], seed: int,
               T3_Ts: float) -> Dict:
    ys = [y for y, _ in runs]
    if alpha is None:
        a_star, m_star = fit_alpha_runs(ys, C0s)
    else:
        a_star, m_star = float(alpha), float(mse_runs(ys, C0s, [alpha])[0])
    rep = [(y, replay(y, [a_star], C0)[:, 0]) for y, C0 in zip(ys, C0s)]
    gb = gamma_bounds(rep, U_grid, conf=conf, B=B, block=block, seed=seed)
    phi = phi_predicted_runs(rep, U_grid, T3_Ts)
    return {
        "alpha_star": a_star, "mse": m_star, "rmse": math.sqrt(m_star) if m_star == m_star else None,
        "gamma_target": target, "conf": conf, "block": gb["block"], "tau_margin": gb["tau_margin"], "T3_Ts": T3_Ts,
        "U_star": largest_U(gb["gamma_low"], U_grid, target),
        "U_star_point": largest_U(gb["gamma"], U_grid, target),
        "whatif": [{"U": float(u), "gamma": float(g), "gamma_low": float(gl), "phi": float(f)}
                   for u, g, gl, f in zip(U_grid, gb["gamma"], gb["gamma_low"], phi)],
    }


def outer_loop(records: Sequence[Dict], target: float = 0.99, window: Optional[int] = None,
               alpha: Optional[float] = None, C0: Optional[float] = None,
               U_grid: np.ndarray = U_GRID, conf: float = 0.95, B: int = 1000, block: Optional[int] = None,
               seed: int = 0, T3_Ts: Optional[float] = None) -> Dict:
    """Outer loop over the DTA (or its last `window` steps). With `alpha` given, (a) is skipped.
    C0 defaults to the C_hat recorded in the first step considered (the AS value at that step).
    U_star: largest U with the lower `conf` block-bootstrap bound of gamma >= target (U_star_point:
    the rule on the point estimate). whatif[k]["phi"]: phi(U) of the backlog recursion (function backlog).
    The keys used by fleetdt.as_service (U_star, alpha_star, mse, n_y) keep their meaning."""
    ts, y, Crec = series(records)
    if window:
        ts, y, Crec = ts[-window:], y[-window:], Crec[-window:]
    if len(ts) == 0:
        raise ValueError("empty DTA")
    if C0 is None:
        C0 = _C0(y, Crec)
    r = T3_Ts if T3_Ts is not None else _T3_Ts(records)
    res = {"t_first": int(ts[0]), "t_last": int(ts[-1]), "n_steps": int(len(ts)), "n_y": int(np.sum(~np.isnan(y))),
           "window": window, "C0": C0}
    res.update(_calibrate([(y, Crec)], [C0], target, alpha, U_grid, conf, B, block, seed, r))
    return res


def run_M(records: Sequence[Dict]) -> int:
    """Number of vessels M of a run: the largest m(t) of its steps."""
    return max([len(r.get("vessels") or []) if r.get("vessels") is not None else int(r.get("m") or 0)
                for r in records] or [0])


def calibrate_by_M(runs: Sequence[Sequence[Dict]], target: float = 0.99, alpha: Optional[float] = None,
                   U_grid: np.ndarray = U_GRID, conf: float = 0.95, B: int = 1000, block: Optional[int] = None,
                   seed: int = 0, T3_Ts: Optional[float] = None) -> Dict[int, Dict]:
    """alpha* and U* per number of vessels M (calibration session): the runs (one DTA each) are grouped
    by run_M; alpha* minimizes the MSE pooled over the runs of the group, gamma and its block-bootstrap
    bound pool the steps of the group (blocks inside each run), and phi(U) restarts the backlog in each
    run. Returns {M: result} with the keys of outer_loop plus "runs" and "n_y"."""
    groups: Dict[int, List[Sequence[Dict]]] = {}
    for recs in runs:
        if recs:
            groups.setdefault(run_M(recs), []).append(recs)
    out = {}
    for M, rr in sorted(groups.items()):
        if M == 0:
            continue
        ser, C0s = [], []
        for recs in rr:
            _, y, Crec = series(recs)
            ser.append((y, Crec))
            C0s.append(_C0(y, Crec))
        r = T3_Ts if T3_Ts is not None else _T3_Ts([x for recs in rr for x in recs])
        res = {"M": M, "runs": len(rr), "n_steps": int(sum(len(y) for y, _ in ser)),
               "n_y": int(sum(np.sum(~np.isnan(y)) for y, _ in ser))}
        res.update(_calibrate(ser, C0s, target, alpha, U_grid, conf, B, block, seed, r))
        out[M] = res
    return out


def sliding(records: Sequence[Dict], window: int, every: Optional[int] = None, target: float = 0.99,
            B: int = 1000) -> List[Dict]:
    """Recomputation over the last `window` steps every `every` steps (condition C3 of the experiments)."""
    recs = sorted(records, key=lambda r: int(r["t"]))
    every = every or window
    out = []
    for end in range(window, len(recs) + 1, every):
        res = outer_loop(recs[end - window:end], target=target, B=B)
        res.pop("whatif")
        out.append(res)
    return out


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.outer", description="Outer loop of the Fleet-DT AS")
    ap.add_argument("dta", nargs="+", help="DTA file(s) (JSON lines of G(t)); with --by-M one file per run")
    ap.add_argument("--target", type=float, default=0.99, help="coverage target gamma")
    ap.add_argument("--window", type=int, default=None, help="last W steps (with --every: sliding windows)")
    ap.add_argument("--every", type=int, default=None, help="sliding recomputation every K steps (needs --window)")
    ap.add_argument("--by-M", action="store_true", help="alpha* and U* per number of vessels M over the runs")
    ap.add_argument("--conf", type=float, default=0.95, help="confidence of the lower bound of gamma")
    ap.add_argument("--B", type=int, default=1000, help="bootstrap resamples")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    if a.by_M or len(a.dta) > 1:
        res = calibrate_by_M([load_dta(p) for p in a.dta], target=a.target, conf=a.conf, B=a.B)
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
        res = sliding(recs, a.window, a.every, a.target, B=a.B)
        if a.json:
            print(json.dumps(res, indent=1))
            return
        print("%8s %8s %10s %8s" % ("t_last", "alpha*", "rmse", "U*"))
        for r in res:
            print("%8d %8.4f %10.1f %8s" % (r["t_last"], r["alpha_star"], r["rmse"] or float("nan"), r["U_star"]))
        return
    res = outer_loop(recs, target=a.target, window=a.window, conf=a.conf, B=a.B)
    if a.json:
        print(json.dumps(res, indent=1))
        return
    print("steps %d..%d (%d, %d with y), C0 = %.1f" % (res["t_first"], res["t_last"], res["n_steps"], res["n_y"], res["C0"]))
    print("alpha* = %.4f   RMSE = %.1f bytes" % (res["alpha_star"], res["rmse"] or float("nan")))
    print("U* = %s for the lower %.0f %% bound of gamma >= %.3f (point estimate: %s), block %d steps" % (
        res["U_star"], 100 * res["conf"], res["gamma_target"], res["U_star_point"], res["block"]))
    print("%6s %8s %9s %8s" % ("U", "gamma", "gamma_lo", "phi"))
    for w in res["whatif"][::5]:
        print("%6.2f %8.4f %9.4f %8.4f" % (w["U"], w["gamma"], w["gamma_low"], w["phi"]))


if __name__ == "__main__":
    main()
