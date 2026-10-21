"""One-step predictors of the series z(t) = c_y y(t) for H5 of the Fleet-DT paper (Table of hypotheses: RelMSE of
simple exponential smoothing with alpha* against each other predictor, in left-out scenarios).

Every predictor returns, for a series z[0..n-1], the one-step predictions p[k] of z[k] from z[0..k-1], with
p[0] = z[0] (no history), so that all predictors share the same first value; the hypothesis driver evaluates the
squared errors from step `burn` on. The parameters are fitted on the calibration DTA (several runs, each its own
series; every fit pools the one-step errors of the runs, each run restarting its recursion):

  es             simple exponential smoothing, p[k] = (1 - a) p[k-1] + a z[k-1] (eq. 2 of the paper; a = alpha*
                 fitted on the grid {0.01, ..., 1} by fleetdt.outer.fit_alpha_runs; TCP: a = 1/8)
  last_value     p[k] = z[k-1] (es with a = 1)
  moving_average p[k] = mean of z[k-w..k-1] (fewer values at the start); w fitted on MA_WINDOWS
  holt           Holt's linear trend: level l and trend b, p[k] = l + b; (a, b) fitted by a grid of 0.05 and a
                 bounded Nelder-Mead refinement
  local_level    local level model z = mu + eps, mu' = mu + eta, by the Kalman filter with a diffuse start
                 (P_0 = DIFFUSE * var(z)); the variances by maximum likelihood (prediction-error decomposition,
                 sigma_eps^2 concentrated out, q = sigma_eta^2 / sigma_eps^2 by a bounded scalar search); the
                 steady state of the filter equals es with a = (-q + sqrt(q^2 + 4 q)) / 2
  arima          ARIMA(p, 1, q) without a constant, p, q <= 2, the order of the smallest AIC; engine "statsmodels"
                 (when installed; default "auto"): SARIMAX maximum likelihood on the concatenation of the
                 per-run differences (each run differenced on its own, so no jump between runs enters the series);
                 engine "css": conditional sum of squares pooled over the runs (residuals start at 0 in each run),
                 minimized by Nelder-Mead with stationarity and invertibility enforced by a penalty,
                 AIC = n log(SSE / n) + 2 (p + q + 1). The predictions of both engines come from the same ARMA
                 filter of the differences (scipy.signal.lfilter, zero initial residuals)
  constant       p[k] = C_ref (the calibration mean of C(t), baseline B4)
"""
from __future__ import annotations

import math
import warnings
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

MA_WINDOWS = tuple(range(2, 65))
DIFFUSE = 1e6
ARIMA_ORDERS = tuple((p, q) for p in range(3) for q in range(3))


def _clean(z) -> np.ndarray:
    z = np.asarray(z, float)
    return z[~np.isnan(z)]


# --------------------------------------------------------------------------- predictors


def es(z: Sequence[float], a: float) -> np.ndarray:
    z = np.asarray(z, float)
    p = np.empty(len(z))
    if not len(z):
        return p
    c = z[0]
    for k in range(len(z)):
        p[k] = c
        c = (1.0 - a) * c + a * z[k]
    return p


def last_value(z: Sequence[float]) -> np.ndarray:
    z = np.asarray(z, float)
    return np.r_[z[:1], z[:-1]] if len(z) else z.copy()


def moving_average(z: Sequence[float], w: int) -> np.ndarray:
    z = np.asarray(z, float)
    if not len(z):
        return z.copy()
    cs = np.r_[0.0, np.cumsum(z)]
    k = np.arange(len(z))
    lo = np.maximum(0, k - int(w))
    cnt = k - lo
    p = np.where(cnt > 0, (cs[k] - cs[lo]) / np.maximum(cnt, 1), z[0])
    return p


def holt(z: Sequence[float], a: float, b: float) -> np.ndarray:
    z = np.asarray(z, float)
    p = np.empty(len(z))
    if not len(z):
        return p
    lev, tr = z[0], 0.0
    for k in range(len(z)):
        p[k] = lev + tr if k else z[0]
        if k == 0:
            continue
        nl = a * z[k] + (1.0 - a) * (lev + tr)
        tr = b * (nl - lev) + (1.0 - b) * tr
        lev = nl
    return p


def _holt_grid(z: np.ndarray, A: np.ndarray, Bv: np.ndarray) -> np.ndarray:
    """SSE of holt for each pair (A[j], Bv[j]) on one run (vectorized over the pairs)."""
    lev = np.full(len(A), z[0])
    tr = np.zeros(len(A))
    sse = np.zeros(len(A))
    for k in range(1, len(z)):
        p = lev + tr
        e = z[k] - p
        sse += e * e
        nl = A * z[k] + (1.0 - A) * p
        tr = Bv * (nl - lev) + (1.0 - Bv) * tr
        lev = nl
    return sse


def local_level(z: Sequence[float], q: float, diffuse: float = DIFFUSE) -> Tuple[np.ndarray, np.ndarray]:
    """Kalman filter of the local level model with signal-to-noise ratio q = sigma_eta^2 / sigma_eps^2 (variances in
    units of sigma_eps^2): one-step predictions and their variances F (in units of sigma_eps^2)."""
    z = np.asarray(z, float)
    n = len(z)
    p, F = np.empty(n), np.empty(n)
    if not n:
        return p, F
    a = z[0]
    P = diffuse
    for k in range(n):
        p[k] = a if k else z[0]
        Fk = P + 1.0
        F[k] = Fk
        if k == 0:              # the first observation sets the level (diffuse start)
            a, P = z[0], 1.0 * P / Fk + q
            continue
        K = P / Fk
        a = a + K * (z[k] - a)
        P = P * (1.0 - K) + q
    return p, F


def arma_filter(w: np.ndarray, phi: Sequence[float], theta: Sequence[float]) -> np.ndarray:
    """Residuals e of the ARMA(p, q) model w_t = sum phi_i w_{t-i} + e_t + sum theta_j e_{t-j} (zero initial
    values)."""
    from scipy.signal import lfilter
    return lfilter(np.r_[1.0, -np.asarray(phi, float)], np.r_[1.0, np.asarray(theta, float)], w)


def arima_predict(z: Sequence[float], phi: Sequence[float], theta: Sequence[float]) -> np.ndarray:
    """One-step predictions of ARIMA(p, 1, q) without a constant: p[k] = z[k-1] + w_hat[k], w = diff(z)."""
    z = np.asarray(z, float)
    if len(z) < 2:
        return z.copy()
    w = np.diff(z)
    e = arma_filter(w, phi, theta)
    return np.r_[z[0], z[:-1] + (w - e)]


# --------------------------------------------------------------------------- fits on several runs


def _sse(zs: Sequence[np.ndarray], pred) -> Tuple[float, int]:
    s, n = 0.0, 0
    for z in zs:
        if len(z) < 2:
            continue
        p = pred(z)
        e = z[1:] - p[1:]
        s += float(e @ e)
        n += len(e)
    return s, n


def fit_ma(zs: Sequence[Sequence[float]], windows: Sequence[int] = MA_WINDOWS) -> Dict:
    zs = [_clean(z) for z in zs]
    best = None
    for w in windows:
        s, n = _sse(zs, lambda z: moving_average(z, w))
        if n and (best is None or s < best[1]):
            best = (int(w), s, n)
    return {"w": best[0], "mse": best[1] / best[2]} if best else {"w": None, "mse": float("nan")}


def fit_holt(zs: Sequence[Sequence[float]], step: float = 0.05) -> Dict:
    from scipy.optimize import minimize
    zs = [_clean(z) for z in zs if len(_clean(z)) >= 3]
    if not zs:
        return {"a": None, "b": None, "mse": float("nan")}
    g = np.round(np.arange(step, 1.0 + 1e-9, step), 6)
    A, Bv = np.meshgrid(g, g, indexing="ij")
    A, Bv = A.ravel(), Bv.ravel()
    sse = sum(_holt_grid(z, A, Bv) for z in zs)
    n = sum(len(z) - 1 for z in zs)
    j = int(np.argmin(sse))

    def f(x):
        a, b = float(x[0]), float(x[1])
        if not (0.0 < a <= 1.0 and 0.0 <= b <= 1.0):
            return float("inf")
        return _sse(zs, lambda z: holt(z, a, b))[0]
    r = minimize(f, [A[j], Bv[j]], method="Nelder-Mead", options={"xatol": 1e-4, "fatol": 1e-9 * sse[j] + 1e-12,
                                                                   "maxiter": 200})
    a, b, s = (float(r.x[0]), float(r.x[1]), float(r.fun)) if r.fun < sse[j] else (float(A[j]), float(Bv[j]),
                                                                                    float(sse[j]))
    return {"a": a, "b": b, "mse": s / n}


def _ll_negloglik(zs: Sequence[np.ndarray], q: float) -> Tuple[float, float, int]:
    """Concentrated negative log-likelihood of the local level model (sigma_eps^2 profiled out); the first
    step of each run (diffuse) is left out. Returns (value, sigma_eps^2, n)."""
    s, logF, n = 0.0, 0.0, 0
    for z in zs:
        p, F = local_level(z, q)
        e = z[1:] - p[1:]
        s += float(np.sum(e * e / F[1:]))
        logF += float(np.sum(np.log(F[1:])))
        n += len(e)
    if n == 0:
        return float("inf"), float("nan"), 0
    s2 = s / n
    return 0.5 * (n * math.log(max(s2, 1e-300)) + logF + n), s2, n


def fit_local_level(zs: Sequence[Sequence[float]]) -> Dict:
    """Maximum likelihood of q = sigma_eta^2 / sigma_eps^2 over log q in [-12, 6] (bounded search), and the
    variances sigma_eps^2 and sigma_eta^2 = q sigma_eps^2."""
    from scipy.optimize import minimize_scalar
    zs = [_clean(z) for z in zs if len(_clean(z)) >= 3]
    if not zs:
        return {"q": None, "sigma2_eps": float("nan"), "sigma2_eta": float("nan"), "alpha_ss": float("nan")}
    grid = np.linspace(-12.0, 6.0, 37)
    vals = [_ll_negloglik(zs, math.exp(v))[0] for v in grid]
    j = int(np.argmin(vals))
    lo, hi = grid[max(0, j - 1)], grid[min(len(grid) - 1, j + 1)]
    r = minimize_scalar(lambda v: _ll_negloglik(zs, math.exp(v))[0], bounds=(lo, hi), method="bounded",
                        options={"xatol": 1e-6})
    v = float(r.x) if r.fun <= vals[j] else float(grid[j])
    q = math.exp(v)
    _, s2, _ = _ll_negloglik(zs, q)
    return {"q": q, "sigma2_eps": s2, "sigma2_eta": q * s2, "alpha_ss": (-q + math.sqrt(q * q + 4.0 * q)) / 2.0}


def _stable(coef: Sequence[float]) -> bool:
    """roots of 1 - c_1 x - ... - c_k x^k outside the unit circle"""
    c = np.asarray(coef, float)
    if not len(c) or not np.any(c):
        return True
    roots = np.roots(np.r_[-c[::-1], 1.0])
    return bool(np.all(np.abs(roots) > 1.0 + 1e-6))


def _css(ws: Sequence[np.ndarray], p: int, q: int) -> Dict:
    from scipy.optimize import minimize
    n = sum(len(w) for w in ws)

    def f(x):
        phi, th = x[:p], x[p:]
        if not _stable(phi) or not _stable(-np.asarray(th)):
            return 1e300
        return float(sum(float(np.sum(arma_filter(w, phi, th) ** 2)) for w in ws))
    x0 = np.zeros(p + q)
    if p + q:
        best = min((minimize(f, s0, method="Nelder-Mead", options={"xatol": 1e-6, "fatol": 1e-10, "maxiter": 2000})
                    for s0 in (x0, np.full(p + q, 0.1), np.full(p + q, -0.1))), key=lambda r: r.fun)
        x, sse = best.x, float(best.fun)
    else:
        x, sse = x0, f(x0)
    aic = n * math.log(max(sse / n, 1e-300)) + 2.0 * (p + q + 1)
    return {"p": p, "q": q, "phi": [float(v) for v in x[:p]], "theta": [float(v) for v in x[p:]], "aic": aic,
            "sigma2": sse / n, "engine": "css"}


def _sm(ws: Sequence[np.ndarray], p: int, q: int) -> Optional[Dict]:
    from statsmodels.tsa.statespace.sarimax import SARIMAX
    w = np.concatenate(ws)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        try:
            r = SARIMAX(w, order=(p, 0, q), trend="n").fit(disp=False)
        except (ValueError, np.linalg.LinAlgError):
            return None
    par = np.asarray(r.params, float)
    return {"p": p, "q": q, "phi": [float(v) for v in par[:p]], "theta": [float(v) for v in par[p:p + q]],
            "aic": float(r.aic), "sigma2": float(par[-1]), "engine": "statsmodels"}


def have_statsmodels() -> bool:
    try:
        import statsmodels  # noqa: F401
        return True
    except ImportError:
        return False


def fit_arima(zs: Sequence[Sequence[float]], orders: Sequence[Tuple[int, int]] = ARIMA_ORDERS,
              engine: str = "auto") -> Dict:
    """ARIMA(p, 1, q) of the smallest AIC over `orders` (module docstring). engine: "auto" (statsmodels when
    installed, else css), "statsmodels" or "css". Returns {"p", "q", "phi", "theta", "aic", "sigma2", "engine",
    "candidates": [(p, q, aic)]}."""
    if engine == "auto":
        engine = "statsmodels" if have_statsmodels() else "css"
    if engine not in ("statsmodels", "css"):
        raise ValueError("engine must be auto, statsmodels or css")
    ws = [np.diff(z) for z in (_clean(z) for z in zs) if len(z) >= 3]
    if not ws:
        return {"p": None, "q": None, "phi": [], "theta": [], "aic": float("nan"), "engine": engine, "candidates": []}
    fits = []
    for p, q in orders:
        r = _sm(ws, p, q) if engine == "statsmodels" else _css(ws, p, q)
        if r is not None and r["aic"] == r["aic"]:
            fits.append(r)
    best = min(fits, key=lambda r: r["aic"])
    best = dict(best, candidates=[(r["p"], r["q"], r["aic"]) for r in fits])
    return best


# --------------------------------------------------------------------------- H5


def fit_all(zs: Sequence[Sequence[float]], alpha_star: float, C_ref: Optional[float],
            arima_engine: str = "auto", ma_windows: Sequence[int] = MA_WINDOWS) -> Dict:
    """Parameters of every predictor of H5 fitted on the calibration series zs (alpha* is given: the outer loop
    fits it on the same series)."""
    return {"es": {"a": float(alpha_star)}, "tcp": {"a": 0.125}, "last_value": {}, "moving_average": fit_ma(zs, ma_windows),
            "holt": fit_holt(zs), "local_level": fit_local_level(zs), "arima": fit_arima(zs, engine=arima_engine),
            "C_ref": {"value": None if C_ref is None else float(C_ref)}}


PREDICTORS = ("last_value", "moving_average", "holt", "local_level", "tcp", "arima", "C_ref")


def predict_all(z: Sequence[float], fit: Dict) -> Dict[str, np.ndarray]:
    """One-step predictions of every predictor of `fit` on the series z (NaN removed beforehand)."""
    z = _clean(z)
    out = {"es": es(z, fit["es"]["a"]), "last_value": last_value(z), "tcp": es(z, fit["tcp"]["a"])}
    if fit["moving_average"].get("w"):
        out["moving_average"] = moving_average(z, fit["moving_average"]["w"])
    if fit["holt"].get("a") is not None:
        out["holt"] = holt(z, fit["holt"]["a"], fit["holt"]["b"])
    if fit["local_level"].get("q") is not None:
        out["local_level"] = local_level(z, fit["local_level"]["q"])[0]
    if fit["arima"].get("p") is not None:
        out["arima"] = arima_predict(z, fit["arima"]["phi"], fit["arima"]["theta"])
    if fit["C_ref"].get("value") is not None:
        out["C_ref"] = np.full(len(z), fit["C_ref"]["value"])
    return out


def rel_mse_all(z: Sequence[float], fit: Dict, burn: int = 50) -> Dict[str, Dict]:
    """RelMSE = MSE_ES / MSE_alt of ES with alpha* against each other predictor on z, over the steps k >= burn.
    Returns {predictor: {"value", "log", "MSE_ES", "MSE_alt", "n"}}."""
    z = _clean(z)
    pr = predict_all(z, fit)
    k0 = max(1, int(burn))
    out = {}
    if len(z) <= k0:
        return out
    e_es = z[k0:] - pr["es"][k0:]
    m_es = float(np.mean(e_es * e_es))
    for name in PREDICTORS:
        if name not in pr:
            continue
        e = z[k0:] - pr[name][k0:]
        m = float(np.mean(e * e))
        v = m_es / m if m > 0 else float("nan")
        out[name] = {"value": v, "log": math.log(v) if v == v and v > 0 else float("nan"), "MSE_ES": m_es,
                     "MSE_alt": m, "n": int(len(e))}
    return out
