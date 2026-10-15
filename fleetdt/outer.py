"""Outer loop of the AS (contribution (iv) of the Fleet-DT paper): tunes the gain alpha of eq. (2) and the
target utilization U of eq. (3) from the DTA.

  (a) alpha_star = argmin of the one-step mean squared error of eq. (2) replayed on y(t)
      (grid 0.01..1 step 0.01, then golden-section refinement around the best grid point);
  (b) gamma(U) = fraction of steps with y(t) >= U C_hat(t), C_hat(t) recomputed with alpha_star;
      U_star = largest U of the grid 0.30..1.00 with gamma(U) >= gamma_target;
  (c) what-if table: for each U, the predicted coverage gamma(U) and the predicted full-delivery
      fraction phi(U) = gamma(U). Proposition 2 makes C(t) >= U C_hat(t) sufficient for the delivery of
      every admitted message, so gamma(U) is a lower bound of phi(U); y(t) stands for C(t).

  python -m fleetdt.outer dta.jsonl [--target 0.99] [--window W] [--every K] [--json]
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
    m = mse(y, grid, C0)
    if np.all(np.isnan(m)):
        return float("nan"), float("nan")
    k = int(np.nanargmin(m))
    best, best_m = float(grid[k]), float(m[k])
    if refine:
        lo, hi = max(float(grid[0]), best - 0.01), min(float(grid[-1]), best + 0.01)
        a = golden(lambda v: float(mse(y, [v], C0)[0]), lo, hi)
        ma = float(mse(y, [a], C0)[0])
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


def outer_loop(records: Sequence[Dict], target: float = 0.99, window: Optional[int] = None,
               alpha: Optional[float] = None, C0: Optional[float] = None,
               U_grid: np.ndarray = U_GRID) -> Dict:
    """Outer loop over the DTA (or its last `window` steps). With `alpha` given, (a) is skipped.
    C0 defaults to the C_hat recorded in the first step considered (the AS value at that step)."""
    ts, y, Crec = series(records)
    if window:
        ts, y, Crec = ts[-window:], y[-window:], Crec[-window:]
    if len(ts) == 0:
        raise ValueError("empty DTA")
    if C0 is None:
        finite = Crec[~np.isnan(Crec)]
        C0 = float(finite[0]) if len(finite) else float(np.nanmean(y))
    if alpha is None:
        a_star, m_star = fit_alpha(y, C0)
    else:
        a_star, m_star = float(alpha), float(mse(y, [alpha], C0)[0])
    C = replay(y, [a_star], C0)[:, 0]
    gam = coverage(y, C, U_grid)
    return {
        "t_first": int(ts[0]), "t_last": int(ts[-1]), "n_steps": int(len(ts)), "n_y": int(np.sum(~np.isnan(y))),
        "window": window, "C0": C0, "alpha_star": a_star, "mse": m_star, "rmse": math.sqrt(m_star) if m_star == m_star else None,
        "gamma_target": target, "U_star": largest_U(gam, U_grid, target),
        "whatif": [{"U": float(u), "gamma": float(g), "phi": float(g)} for u, g in zip(U_grid, gam)],
    }


def sliding(records: Sequence[Dict], window: int, every: Optional[int] = None, target: float = 0.99) -> List[Dict]:
    """Recomputation over the last `window` steps every `every` steps (condition C3 of the experiments)."""
    recs = sorted(records, key=lambda r: int(r["t"]))
    every = every or window
    out = []
    for end in range(window, len(recs) + 1, every):
        res = outer_loop(recs[end - window:end], target=target)
        res.pop("whatif")
        out.append(res)
    return out


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.outer", description="Outer loop of the Fleet-DT AS")
    ap.add_argument("dta", help="DTA file (JSON lines of G(t))")
    ap.add_argument("--target", type=float, default=0.99, help="coverage target gamma")
    ap.add_argument("--window", type=int, default=None, help="last W steps (with --every: sliding windows)")
    ap.add_argument("--every", type=int, default=None, help="sliding recomputation every K steps (needs --window)")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    recs = load_dta(a.dta)
    if a.every and a.window:
        res = sliding(recs, a.window, a.every, a.target)
        if a.json:
            print(json.dumps(res, indent=1))
            return
        print("%8s %8s %10s %8s" % ("t_last", "alpha*", "rmse", "U*"))
        for r in res:
            print("%8d %8.4f %10.1f %8s" % (r["t_last"], r["alpha_star"], r["rmse"] or float("nan"), r["U_star"]))
        return
    res = outer_loop(recs, target=a.target, window=a.window)
    if a.json:
        print(json.dumps(res, indent=1))
        return
    print("steps %d..%d (%d, %d with y), C0 = %.1f" % (res["t_first"], res["t_last"], res["n_steps"], res["n_y"], res["C0"]))
    print("alpha* = %.4f   RMSE = %.1f bytes" % (res["alpha_star"], res["rmse"] or float("nan")))
    print("U* = %s for gamma >= %.3f" % (res["U_star"], res["gamma_target"]))
    print("%6s %8s %8s" % ("U", "gamma", "phi>="))
    for w in res["whatif"][::5]:
        print("%6.2f %8.4f %8.4f" % (w["U"], w["gamma"], w["phi"]))


if __name__ == "__main__":
    main()
