"""Metrics of Section VI of the Fleet-DT paper computed from a DTA (JSON lines of G(t)) and the DTP.

Per run:
  MAPE    mean of |y(t) - C_hat(t)| / y(t) over the steps with y(t) > 0 (fidelity, Definition of the paper)
  gamma   fraction of the steps with y(t) >= U C_hat(t)
  delta   fraction of the pairs (t, i), i in M(t) and "state" in A(t) ∩ N(t), with exactly one state
          message received in step t (edge n == 1 and dup == 0)
  phi     fraction of the steps in which every message of level 1 or 2 of the vessels of M(t) was received
  eta     bytes of the N(t) topics received within the step / bytes of the N(t) topics demanded; also per
          vessel, with Jain's fairness index J and the ratio min_i eta_i / max_i eta_i
  mu      late messages / (received + late)
  nu      received channel bytes / sum of y(t) T3 / Ts, over the steps with y(t)
  prefix  prefix completeness: for each rank k of sigma_t, the fraction of the pairs (t, i), i in M(t),
          in which topics sigma_t(1..k) of vessel i arrive complete within step t
  fleet   fleet consistency: for each topic l, the fraction of the steps (with a positive demand of l)
          in which l arrives complete from every vessel of M(t)
  unused  share of K_hat(t) granted to vessels of M(t) with a zero demand table: sum_t z(t) b(t) / sum_t K_hat(t)
  fill    fill ratio f(t) = max_{i in M(t)} a_i(t) / b(t) (a_i of Algorithm 1); a run is valid for E3 and
          E4 when mean f >= 0.85 (FILL_MIN)

The full-reception criterion of an edge is explicit (argument `criterion`, required on the command line):
  count  n >= messages forwarded by the vessel regulator (fwd of the regulator log; n_l = ceil(r_l Ts)
         without the log); for completeness (prefix, fleet) n >= fwd + held, i.e. every message produced
  bytes  r >= d (1 - tol) - 1

Statistics: integrated autocorrelation time (one run or pooled over runs), effective sample size,
circular block bootstrap (two-sided CI and one-sided bounds, blocks inside each run), Clopper-Pearson
upper limit with n_eff, the H4a bound on the fleet steps (and on non-overlapping 8-step windows,
Corollary 1) with n_eff from tau_int of the margin s(t) = kappa y(t) - U C_hat(t), Diebold-Mariano test
(Newey-West variance), one-sided exact Wilcoxon signed-rank test with Pratt's treatment of zeros
(exact conditional distribution, ties included), Holm, the intersection-union test (IUT), the
fixed-sequence gatekeeping H1 -> H2 -> H4 -> H5 -> H3 -> H6, TOST on run-level differences, and the
Hodges-Lehmann estimate.

  python -m fleetdt.analysis dta.jsonl [more.jsonl ...] --dtp dtp.json --criterion bytes|count [--kappa K] [--json]
"""
from __future__ import annotations

import argparse
import json
import math
import warnings
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from .as_model import DTP
from .outer import load_dta

CRITERIA = ("bytes", "count")
FILL_MIN = 0.85
HYPOTHESIS_ORDER = ("H1", "H2", "H4", "H5", "H3", "H6")

# --------------------------------------------------------------------------- autocorrelation


def autocorr(x: np.ndarray) -> np.ndarray:
    """Normalized autocorrelation rho_k, k = 0..n-1 (FFT, biased estimator)."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    x = x - x.mean()
    f = np.fft.rfft(x, n=2 * n)
    acf = np.fft.irfft(f * np.conj(f))[:n]
    return acf / acf[0] if acf[0] > 0 else np.r_[1.0, np.zeros(n - 1)]


def _tau_from_rho(rho: np.ndarray, method: str = "ips", c: float = 5.0) -> float:
    if method == "sokal":
        tau = 0.5
        for W in range(1, len(rho)):
            tau += rho[W]
            if W >= c * tau:
                return max(tau, 0.5)
        return max(tau, 0.5)
    s = 0.0
    for m in range(len(rho) // 2):
        g = rho[2 * m] + rho[2 * m + 1]
        if g <= 0:
            break
        s += g
    return max(s - 0.5, 0.5)


def tau_int(x: Sequence[float], method: str = "ips", c: float = 5.0) -> float:
    """Integrated autocorrelation time tau_int = 1/2 + sum_{k>=1} rho_k (tau_int = 1/2 for i.i.d. data).
    method "ips": initial positive sequence of Geyer (sums of pairs rho_2m + rho_2m+1 while positive);
    method "sokal": smallest window W with W >= c tau_int(W)."""
    x = np.asarray(x, dtype=float)
    x = x[~np.isnan(x)]
    if len(x) < 4 or np.var(x) == 0:
        return 0.5
    return _tau_from_rho(autocorr(x), method, c)


def tau_int_runs(runs: Sequence[Sequence[float]], method: str = "ips", c: float = 5.0) -> float:
    """tau_int pooled over independent runs: each run is centred on its own mean, the lag-k products of
    every run are summed and divided by the total number of steps, rho_k = acov_k / acov_0, and the
    window rule of tau_int applies to that pooled rho. NaN steps are removed inside each run."""
    xs = []
    for x in runs:
        x = np.asarray(x, dtype=float)
        x = x[~np.isnan(x)]
        if len(x) >= 2:
            xs.append(x - x.mean())
    if not xs:
        return 0.5
    n_max = max(len(x) for x in xs)
    acov = np.zeros(n_max)
    for x in xs:
        n = len(x)
        f = np.fft.rfft(x, n=2 * n)
        acov[:n] += np.fft.irfft(f * np.conj(f))[:n]
    if acov[0] <= 0 or sum(len(x) for x in xs) < 4:
        return 0.5
    return _tau_from_rho(acov / acov[0], method, c)


def n_eff(n: int, tau: float) -> float:
    """Effective sample size |S| / (2 tau_int)."""
    return n / (2.0 * tau)


# --------------------------------------------------------------------------- bootstrap


def block_bootstrap_ci(x, stat: Callable[[np.ndarray], float] = np.mean, conf: float = 0.95, B: int = 2000,
                       block: Optional[int] = None, tau: Optional[float] = None, seed: int = 0) -> Tuple[float, float, int]:
    """Percentile CI of stat(x) by circular block bootstrap. x: per-step values (1-D) or rows (2-D, one
    row per step, e.g. numerator and denominator of a ratio). The block length defaults to
    ceil(2 tau_int) of the first column. Returns (low, high, block length)."""
    r = block_bootstrap_bounds([x], stat, conf=conf, B=B, block=block, tau=tau, seed=seed)
    return r["ci"][0], r["ci"][1], r["block"]


def block_bootstrap_bounds(runs, stat: Callable[[np.ndarray], float] = np.mean, conf: float = 0.95, B: int = 2000,
                           block: Optional[int] = None, tau: Optional[float] = None, seed: int = 0) -> Dict:
    """Circular block bootstrap of stat over one or several runs (a 1-D/2-D array, or a list of them).
    Blocks of `block` consecutive steps never cross the end of a run (each run is circular on its own);
    a resample draws ceil(n / block) blocks with replacement from the blocks of every run. The block
    length defaults to ceil(2 tau_int), tau_int pooled over the runs (first column for 2-D rows).
    Returns {"lower": one-sided lower `conf` bound, "upper": one-sided upper `conf` bound,
    "ci": two-sided `conf` percentile interval, "block", "value", "n"}."""
    if isinstance(runs, np.ndarray) or (len(runs) and np.isscalar(runs[0])):
        runs = [runs]
    rs = [np.asarray(x, dtype=float) for x in runs]
    rs = [x for x in rs if len(x)]
    nan = float("nan")
    if not rs:
        return {"lower": nan, "upper": nan, "ci": [nan, nan], "block": 0, "value": nan, "n": 0}
    allx = np.concatenate(rs, axis=0)
    n = len(allx)
    if block is None:
        cols = [x if x.ndim == 1 else x[:, 0] for x in rs]
        block = max(1, int(math.ceil(2.0 * (tau if tau is not None else tau_int_runs(cols)))))
    starts, lens, obase, olen = [], [], [], []
    base = 0
    for x in rs:
        m = len(x)
        starts.append(np.arange(m))
        lens.append(np.full(m, min(block, m)))
        obase.append(np.full(m, base))
        olen.append(np.full(m, m))
        base += m
    starts, lens, obase, olen = (np.concatenate(v) for v in (starts, lens, obase, olen))
    rng = np.random.default_rng(seed)
    nb = int(math.ceil(n / float(block)))
    ar = np.arange(block)
    vals = np.empty(B)
    for b in range(B):
        pick = rng.integers(0, len(starts), nb)
        idx = obase[pick, None] + (starts[pick, None] + ar[None, :]) % olen[pick, None]
        idx = idx[ar[None, :] < lens[pick, None]][:n]
        vals[b] = stat(allx[idx])
    vals = vals[~np.isnan(vals)]
    if len(vals) == 0:
        return {"lower": nan, "upper": nan, "ci": [nan, nan], "block": int(block), "value": float(stat(allx)), "n": n}
    a = (1.0 - conf) / 2.0
    return {"lower": float(np.quantile(vals, 1.0 - conf)), "upper": float(np.quantile(vals, conf)),
            "ci": [float(np.quantile(vals, a)), float(np.quantile(vals, 1.0 - a))], "block": int(block),
            "value": float(stat(allx)), "n": n}


# --------------------------------------------------------------------------- binomial bounds


def clopper_pearson_upper(k: float, n: float, conf: float = 0.95, n_eff: Optional[float] = None) -> float:
    """One-sided Clopper-Pearson upper limit of an event probability after k events in n steps. With
    n_eff, the counts scale to k n_eff / n events in n_eff trials (autocorrelated steps)."""
    from scipy.stats import beta
    if n <= 0:
        return 1.0
    if n_eff is not None and n_eff > 0:
        k, n = k * n_eff / n, n_eff
    if k >= n:
        return 1.0
    return float(beta.ppf(conf, k + 1.0, n - k))


def clopper_pearson_upper_eff(k: float, n: float, n_eff: float, conf: float = 0.95, events: str = "raw") -> float:
    """One-sided Clopper-Pearson upper limit with n_eff effective trials.
    events "raw": the k observed events in n_eff trials (conservative; the pre-registered rule of H1 and
    H4a: an upper limit below 1e-3 needs n_eff >= 2995 with 0 events, 4742 with 1, 6294 with 2);
    events "scaled": k n_eff / n events in n_eff trials (keeps the observed rate k / n)."""
    from scipy.stats import beta
    if n_eff <= 0 or n <= 0:
        return 1.0
    if events not in ("raw", "scaled"):
        raise ValueError("events must be raw or scaled")
    ke = float(k) if events == "raw" else float(k) * n_eff / float(n)
    if ke >= n_eff:
        return 1.0
    return float(beta.ppf(conf, ke + 1.0, n_eff - ke))


# --------------------------------------------------------------------------- tests


def diebold_mariano(loss1: Sequence[float], loss2: Sequence[float], lag: Optional[int] = None) -> Dict:
    """Diebold-Mariano test of equal expected loss; d = loss1 - loss2, long-run variance of Newey-West
    (Bartlett kernel, default lag floor(4 (n/100)^(2/9))). stat < 0 favours series 1. Two-sided p."""
    d = np.asarray(loss1, dtype=float) - np.asarray(loss2, dtype=float)
    d = d[~np.isnan(d)]
    n = len(d)
    if n < 3:
        return {"stat": float("nan"), "p": float("nan"), "n": n, "lag": lag, "mean": float("nan")}
    if lag is None:
        lag = int(math.floor(4.0 * (n / 100.0) ** (2.0 / 9.0)))
    dm = d.mean()
    e = d - dm
    lrv = float(e @ e) / n
    for k in range(1, lag + 1):
        lrv += 2.0 * (1.0 - k / (lag + 1.0)) * float(e[k:] @ e[:-k]) / n
    if lrv <= 0:
        return {"stat": float("nan"), "p": float("nan"), "n": n, "lag": lag, "mean": float(dm)}
    s = dm / math.sqrt(lrv / n)
    return {"stat": float(s), "p": math.erfc(abs(s) / math.sqrt(2.0)), "n": n, "lag": lag, "mean": float(dm)}


def signed_rank_exact(a: Sequence[float], b: Optional[Sequence[float]] = None, margin: float = 0.0,
                      alternative: str = "greater", zero_method: str = "pratt") -> Dict:
    """Exact Wilcoxon signed-rank test of H0: the differences d = a - b - margin are symmetric about 0,
    against "greater" (d tends to be positive: a - b > margin), "less" or "two-sided".

    zero_method "pratt": |d| is ranked with the zeros (midranks for ties), then the zeros are dropped;
    "wilcox": the zeros are dropped before ranking. The p-value comes from the exact permutation
    distribution of T+ = sum of the ranks of the positive d, conditional on the observed (mid)ranks:
    each nonzero d carries its rank with sign + or - with probability 1/2 (2^n' equally likely
    signs, enumerated by convolution on the doubled ranks), so ties and zeros keep the test exact.
    Returns {"T_plus", "p", "n", "n_nonzero", "alternative", "zero_method"}."""
    from scipy.stats import rankdata
    d = np.asarray(a, dtype=float) - (0.0 if b is None else np.asarray(b, dtype=float)) - float(margin)
    d = d[~np.isnan(d)]
    n = len(d)
    if zero_method == "pratt":
        r = rankdata(np.abs(d))
        nz = d != 0
        r, dn = r[nz], d[nz]
    elif zero_method == "wilcox":
        dn = d[d != 0]
        r = rankdata(np.abs(dn))
    else:
        raise ValueError("zero_method must be pratt or wilcox")
    out = {"n": int(n), "n_nonzero": int(len(dn)), "alternative": alternative, "zero_method": zero_method}
    if len(dn) == 0:
        out.update(T_plus=0.0, p=1.0)
        return out
    r2 = np.rint(2.0 * r).astype(np.int64)          # midranks are multiples of 1/2
    t_obs = int(r2[dn > 0].sum())
    total = int(r2.sum())
    dist = np.zeros(total + 1)
    dist[0] = 1.0
    hi = 0
    for v in r2:                                     # P(T2 = s), T2 = 2 T+; values k / 2^n' are exact
        new = 0.5 * dist
        new[v:hi + v + 1] += 0.5 * dist[:hi + 1]
        dist = new
        hi += int(v)
    p_ge = float(dist[t_obs:].sum())
    p_le = float(dist[:t_obs + 1].sum())
    if alternative == "greater":
        p = p_ge
    elif alternative == "less":
        p = p_le
    elif alternative == "two-sided":
        p = min(1.0, 2.0 * min(p_ge, p_le))
    else:
        raise ValueError("alternative must be greater, less or two-sided")
    out.update(T_plus=t_obs / 2.0, p=min(1.0, p))
    return out


def holm(p: Sequence[float]) -> List[float]:
    """Holm step-down adjustment of a family of p-values."""
    p = list(p)
    order = sorted(range(len(p)), key=lambda k: p[k])
    adj = [0.0] * len(p)
    run = 0.0
    for r, k in enumerate(order):
        run = max(run, min(1.0, (len(p) - r) * p[k]))
        adj[k] = run
    return adj


def holm_reject(p: Sequence[float], alpha: float = 0.05) -> List[bool]:
    """Rejections of the Holm procedure at familywise level alpha."""
    return [q <= alpha for q in holm(p)]


def iut(p: Union[Sequence[float], Dict[str, float]], alpha: float = 0.05) -> Dict:
    """Intersection-union test: H0 = union of the component nulls, rejected at level alpha iff every
    component is rejected at level alpha; p_IUT = max_k p_k (no multiplicity adjustment). A component
    p that is NaN (not computable) counts as 1."""
    items = list(p.items()) if isinstance(p, dict) else list(enumerate(p))
    vals = [1.0 if (v is None or v != v) else float(v) for _, v in items]
    pm = max(vals) if vals else 1.0
    worst = items[int(np.argmax(vals))][0] if vals else None
    return {"p": pm, "reject": bool(vals) and pm <= alpha, "k": len(vals), "worst": worst, "alpha": alpha}


def fixed_sequence(p: Dict[str, float], order: Sequence[str] = HYPOTHESIS_ORDER, alpha: float = 0.05) -> List[Dict]:
    """Fixed-sequence gatekeeping: the hypotheses are tested in `order`, each at the full level alpha;
    testing stops at the first hypothesis that is not rejected (or that has no p-value), and the later
    ones are not tested. This controls the familywise error rate at alpha. The p of each hypothesis is
    the p of its intersection-union test over its components. Returns one dict per hypothesis of
    `order`: {"id", "p", "tested", "rejected"}."""
    out, gate = [], True
    for h in order:
        v = p.get(h)
        v = None if v is None or v != v else float(v)
        tested = gate
        rej = bool(tested and v is not None and v <= alpha)
        out.append({"id": h, "p": v, "tested": tested, "rejected": rej})
        gate = gate and rej
    return out


def tost(d: Sequence[float], margin: Union[float, Tuple[float, float]], alpha: float = 0.05) -> Dict:
    """Two one-sided t tests (TOST) of equivalence on run-level differences d (one value per run):
    H0: mean(d) <= low or mean(d) >= high, with (low, high) = (-margin, margin) or the given pair.
    p = max(p_low, p_high); equivalence at level alpha iff p < alpha, which is the same as the
    (1 - 2 alpha) t interval of mean(d) lying inside (low, high). Returns {"mean", "se", "df", "ci",
    "p_low", "p_high", "p", "equivalent", "n"}."""
    from scipy.stats import t as tdist
    low, high = (-abs(float(margin)), abs(float(margin))) if np.isscalar(margin) else (float(margin[0]), float(margin[1]))
    x = np.asarray(d, dtype=float)
    x = x[~np.isnan(x)]
    n = len(x)
    nan = float("nan")
    if n < 2:
        return {"mean": float(x.mean()) if n else nan, "se": nan, "df": n - 1, "ci": [nan, nan], "p_low": nan,
                "p_high": nan, "p": 1.0, "equivalent": False, "n": n, "margin": [low, high]}
    m, se, df = float(x.mean()), float(x.std(ddof=1) / math.sqrt(n)), n - 1
    if se == 0:
        inside = low < m < high
        return {"mean": m, "se": 0.0, "df": df, "ci": [m, m], "p_low": 0.0 if m > low else 1.0,
                "p_high": 0.0 if m < high else 1.0, "p": 0.0 if inside else 1.0, "equivalent": inside, "n": n,
                "margin": [low, high]}
    p_low = float(tdist.sf((m - low) / se, df))
    p_high = float(tdist.cdf((m - high) / se, df))
    q = float(tdist.ppf(1.0 - alpha, df))
    p = max(p_low, p_high)
    return {"mean": m, "se": se, "df": df, "ci": [m - q * se, m + q * se], "p_low": p_low, "p_high": p_high,
            "p": p, "equivalent": p < alpha, "n": n, "margin": [low, high]}


def hodges_lehmann(a: Sequence[float], b: Optional[Sequence[float]] = None) -> float:
    """Hodges-Lehmann estimate of the paired shift: median of the Walsh averages of a - b."""
    d = np.asarray(a, dtype=float) - (0.0 if b is None else np.asarray(b, dtype=float))
    i, j = np.triu_indices(len(d))
    return float(np.median((d[i] + d[j]) / 2.0))


def wilcoxon_holm(pairs: Dict[str, Tuple[Sequence[float], Sequence[float]]], margin: float = 0.0,
                  alternative: str = "greater") -> Dict[str, Dict]:
    """One-sided exact signed-rank test (Pratt zeros, signed_rank_exact) of each paired comparison of
    run-level values (name -> (a, b)) against a - b > margin, the Holm adjustment over the family, the
    intersection-union p over the family ("_iut"), and the Hodges-Lehmann estimate of a - b."""
    out = {}
    for name, (a, b) in pairs.items():
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        r = signed_rank_exact(a, b, margin=margin, alternative=alternative)
        out[name] = {"n": len(a), "p": r["p"], "T_plus": r["T_plus"], "hl": hodges_lehmann(a, b)}
    for name, q in zip(list(out), holm([v["p"] for v in out.values()])):
        out[name]["p_holm"] = q
    out["_iut"] = iut({k: v["p"] for k, v in out.items()})
    return out


def jain(x: Sequence[float]) -> float:
    """Jain's fairness index (sum x)^2 / (n sum x^2)."""
    x = np.asarray([v for v in x if v == v], dtype=float)
    s2 = float(np.sum(x * x))
    return float(np.sum(x) ** 2 / (len(x) * s2)) if len(x) and s2 > 0 else float("nan")


def min_max_ratio(x: Sequence[float]) -> float:
    """min_i x_i / max_i x_i over the finite values (H5d with x = eta_i)."""
    x = np.asarray([v for v in x if v == v], dtype=float)
    return float(x.min() / x.max()) if len(x) and x.max() > 0 else float("nan")


# --------------------------------------------------------------------------- per-step series


def _check_criterion(criterion: Optional[str]) -> str:
    if criterion not in CRITERIA:
        raise ValueError("criterion must be one of %s (got %r)" % (", ".join(CRITERIA), criterion))
    return criterion


def _received_all(e: Dict, criterion: str, tol: float, n_expected: Dict[int, int]) -> bool:
    """Full reception of an admitted edge (phi, Proposition 2): every forwarded message arrived."""
    if criterion == "count":
        want = e.get("fwd")
        if want is None:
            want = n_expected.get(e["l"])
        return want is not None and e.get("n", 0) >= want
    return e.get("r", 0) >= e.get("d", 0) * (1.0 - tol) - 1.0


def _edge_complete(e: Optional[Dict], criterion: str, tol: float, n_expected: Dict[int, int]) -> bool:
    """Complete arrival of an edge within the step, whatever its level (prefix completeness, fleet
    consistency): a topic with zero demand (or no edge) is complete; with "count", n >= the messages the
    source produced (src, else fwd + held of the regulator log, else n_l = ceil(r_l Ts)); with "bytes",
    r >= d (1 - tol) - 1."""
    if e is None or e.get("d", 0) <= 0:
        return True
    if criterion == "count":
        want = e.get("src")
        if want is None and e.get("fwd") is not None and e.get("held") is not None:
            want = int(e["fwd"]) + int(e["held"])
        if want is None:
            want = n_expected.get(e["l"])
        return want is not None and e.get("n", 0) >= want
    return e.get("r", 0) >= e.get("d", 0) * (1.0 - tol) - 1.0


def margin_series(records: Sequence[Dict], dtp: DTP, kappa: float = 1.0) -> np.ndarray:
    """Margin of the condition of Proposition 2, s(t) = kappa y(t) - U C_hat(t), sorted by t (NaN without
    y or C_hat); kappa is the calibrated lower bound of C(t) / y(t)."""
    out = []
    for g in sorted(records, key=lambda r: int(r["t"])):
        y, C = g.get("y"), g.get("C_hat")
        U = g.get("U") if g.get("U") is not None else dtp.U
        out.append(kappa * y - U * C if y is not None and C is not None else np.nan)
    return np.asarray(out, dtype=float)


def step_rows(records: Sequence[Dict], dtp: DTP, criterion: str, tol: float = 1e-6, kappa: float = 1.0) -> Dict:
    """Per-step quantities of the metrics, sorted by t. Ratio metrics keep (numerator, denominator) rows."""
    _check_criterion(criterion)
    recs = sorted(records, key=lambda r: int(r["t"]))
    L = dtp.L
    state = dtp.index("state") if "state" in dtp.topics else None
    n_expected = {k + 1: int(math.ceil(s.rate * dtp.Ts - 1e-9)) for k, s in enumerate(dtp.specs) if s.rate}
    ratio = dtp.T3 / dtp.Ts
    default_order = [dtp.index(n) for n in dtp.order] if dtp.order else list(range(1, L + 1))
    out = {"t": [], "ape": [], "cover": [], "phi": [], "delta": [], "delta_fail": [], "eta": [], "mu": [], "nu": [],
           "margin": [], "eta_i": {}, "eta_rows": [], "prefix": [], "fleet": [], "unused": [], "fill": [],
           "order": []}
    for g in recs:
        t, y, C = int(g["t"]), g.get("y"), g.get("C_hat")
        U = g.get("U") if g.get("U") is not None else dtp.U
        Mlist = sorted(set(g.get("vessels", [])))
        M = set(Mlist)
        N = set(g.get("needed", []))
        sigma = list(g.get("order") or default_order)
        edges = g.get("edges", [])
        by = {(e["i"], e["l"]): e for e in edges}
        out["t"].append(t)
        out["order"].append(sigma)
        out["ape"].append(abs(y - C) / y if y is not None and C is not None and y > 0 else np.nan)
        out["cover"].append(float(y >= U * C) if y is not None and C is not None else np.nan)
        out["margin"].append(kappa * y - U * C if y is not None and C is not None else np.nan)
        inM = [e for e in edges if e["i"] in M]
        adm = [e for e in inM if (e.get("p") or 0) > 0]
        out["phi"].append(float(all(_received_all(e, criterion, tol, n_expected) for e in adm)) if M else np.nan)
        ok = pairs = 0
        if state is not None and state in N:
            for e in adm:
                if e["l"] == state:
                    pairs += 1
                    ok += int(e.get("n", 0) == 1 and e.get("dup", 0) == 0)
        out["delta"].append((ok, pairs))
        out["delta_fail"].append(float(ok < pairs) if pairs else np.nan)
        num = den = 0
        row = {}
        for e in inM:
            if e["l"] in N:
                num += e.get("r", 0)
                den += e.get("d", 0)
                vi = out["eta_i"].setdefault(e["i"], [0, 0])
                vi[0] += e.get("r", 0)
                vi[1] += e.get("d", 0)
                ri = row.setdefault(e["i"], [0, 0])
                ri[0] += e.get("r", 0)
                ri[1] += e.get("d", 0)
        out["eta"].append((num, den))
        out["eta_rows"].append(row)
        late = sum(e.get("late", 0) for e in edges)
        out["mu"].append((late, sum(e.get("n", 0) for e in edges) + late))
        out["nu"].append((sum(e.get("r", 0) for e in edges), y * ratio) if y is not None else (np.nan, np.nan))
        # prefix completeness: length of the complete prefix of sigma_t, per vessel of M(t)
        comp = {i: [_edge_complete(by.get((i, l)), criterion, tol, n_expected) for l in sigma] for i in Mlist}
        pre = {}
        for i in Mlist:
            k = 0
            while k < len(sigma) and comp[i][k]:
                k += 1
            pre[i] = k
        out["prefix"].append(pre)
        # fleet consistency: topic l complete from every vessel of M(t), for the topics with a demand
        fl = {}
        for pos, l in enumerate(sigma):
            if Mlist and any((by.get((i, l)) or {}).get("d", 0) > 0 for i in Mlist):
                fl[l] = all(comp[i][pos] for i in Mlist)
        out["fleet"].append(fl)
        # unused share and fill ratio (Algorithm 1: a_i = sum of d_il over the admitted topics)
        b, K = g.get("b"), g.get("K_hat")
        if Mlist and b is not None and b > 0:
            D0 = [i for i in Mlist if sum((by.get((i, l)) or {}).get("d", 0) for l in range(1, L + 1)) == 0]
            Kt = K if K is not None else b * len(Mlist)
            out["unused"].append((len(D0) * b, Kt))
            a = [sum(e.get("d", 0) for e in inM if e["i"] == i and (e.get("p") or 0) > 0) for i in Mlist]
            out["fill"].append(max(a) / b)
        else:
            out["unused"].append((np.nan, np.nan))
            out["fill"].append(np.nan)
    return out


def _ratio(rows: np.ndarray) -> float:
    s = np.nansum(rows, axis=0)
    return float(s[0] / s[1]) if s[1] > 0 else float("nan")


def _summary(x: np.ndarray, stat, ci: bool, B: int, conf: float, seed: int, event: bool = False,
             tau: Optional[float] = None, fail: Optional[np.ndarray] = None) -> Dict:
    """Value, bootstrap CI, tau_int and n_eff of a per-step series (rows for ratios). With `tau` (tau_int
    of the margin series, for the event metrics), n_eff = n / (2 tau) with the fleet step as the unit.
    With `event`, viol_upper is the Clopper-Pearson upper limit of the per-step violation probability
    (events: steps with a violation, given by `fail` or 1 - x), with n_eff effective steps."""
    if x.ndim == 1:
        keep = ~np.isnan(x)
        x = x[keep]
        per_step = x
    else:
        keep = ~np.isnan(x).any(axis=1) & (x[:, 1] > 0)
        x = x[keep]
        per_step = x[:, 0] / x[:, 1] if len(x) else x[:, 0]
    n = len(x)
    out = {"value": stat(x) if n else float("nan"), "n": n}
    t = (tau if tau is not None else tau_int(per_step)) if n else float("nan")
    out["tau_int"], out["n_eff"] = t, n_eff(n, t) if n else 0.0
    out["tau_source"] = "margin" if tau is not None else "series"
    if ci and n:
        r = block_bootstrap_bounds([x], stat, conf=conf, B=B, tau=t, seed=seed)
        out["ci"], out["block"], out["lower"], out["upper"] = r["ci"], r["block"], r["lower"], r["upper"]
    if event and n:
        if fail is not None:
            f = np.asarray(fail, dtype=float)
            f = f[~np.isnan(f)]
            k, m = float(np.sum(f)), len(f)
        else:
            k, m = float(np.sum(1.0 - x)) if x.ndim == 1 else float(np.sum(x[:, 0] < x[:, 1])), n
        ne = n_eff(m, t) if m else 0.0
        out["violations"] = k
        out["viol_upper"] = clopper_pearson_upper_eff(k, m, ne, conf=conf, events="scaled") if m else 1.0
    return out


def prefix_completeness(s: Dict, L: int) -> Dict:
    """From step_rows: for each rank k = 1..L, the fraction of the pairs (t, i), i in M(t), whose topics
    sigma_t(1..k) all arrived complete; the mean complete-prefix length (topics) and its share of L."""
    lens = [k for pre in s["prefix"] for k in pre.values()]
    if not lens:
        return {"by_rank": [float("nan")] * L, "mean_len": float("nan"), "mean_frac": float("nan"), "pairs": 0}
    a = np.asarray(lens)
    return {"by_rank": [float(np.mean(a >= k)) for k in range(1, L + 1)], "mean_len": float(a.mean()),
            "mean_frac": float(a.mean() / L), "pairs": int(len(a))}


def fleet_consistency(s: Dict, dtp: DTP) -> Dict[str, float]:
    """From step_rows: per topic name, the fraction of the steps (with a positive demand of the topic in
    M(t)) in which the topic arrived complete from every vessel of M(t)."""
    num, den = {}, {}
    for fl in s["fleet"]:
        for l, v in fl.items():
            den[l] = den.get(l, 0) + 1
            num[l] = num.get(l, 0) + int(v)
    return {dtp.topics[l - 1]: num[l] / den[l] for l in sorted(den)}


def _eta_ratio_rows(s: Dict) -> Tuple[np.ndarray, List[int]]:
    vs = sorted(s["eta_i"])
    rows = np.zeros((len(s["eta_rows"]), 2 * len(vs)))
    for k, row in enumerate(s["eta_rows"]):
        for j, i in enumerate(vs):
            if i in row:
                rows[k, 2 * j], rows[k, 2 * j + 1] = row[i]
    return rows, vs


def _eta_minmax(rows: np.ndarray) -> float:
    s = rows.sum(axis=0)
    eta = [s[2 * j] / s[2 * j + 1] for j in range(len(s) // 2) if s[2 * j + 1] > 0]
    return min_max_ratio(eta)


def run_metrics(records: Sequence[Dict], dtp: DTP, *, criterion: str, tol: float = 1e-6, ci: bool = True,
                B: int = 2000, conf: float = 0.95, seed: int = 0, kappa: float = 1.0) -> Dict:
    """Metrics of one run (Section VI) with an explicit full-reception criterion ("bytes" or "count").
    The event metrics (gamma, phi, delta) take tau_int from the margin series s(t) = kappa y(t) - U C_hat(t)
    and the fleet step as the unit (n_eff = steps / (2 tau_int), no factor of messages per step)."""
    _check_criterion(criterion)
    s = step_rows(records, dtp, criterion, tol, kappa)
    mean = lambda z: float(np.mean(z))
    res = {"steps": len(s["t"]), "t_first": s["t"][0] if s["t"] else None, "t_last": s["t"][-1] if s["t"] else None,
           "criterion": criterion, "kappa": kappa}
    mg = np.asarray(s["margin"], float)
    tau_m = tau_int(mg) if np.any(~np.isnan(mg)) else 0.5
    res["tau_margin"] = tau_m
    res["MAPE"] = _summary(np.asarray(s["ape"], float), mean, ci, B, conf, seed)
    res["gamma"] = _summary(np.asarray(s["cover"], float), mean, ci, B, conf, seed, event=True, tau=tau_m)
    res["phi"] = _summary(np.asarray(s["phi"], float), mean, ci, B, conf, seed, event=True, tau=tau_m)
    for k in ("delta", "eta", "mu", "nu"):
        rows = np.asarray(s[k], float).reshape(-1, 2)
        if k == "delta":
            res[k] = _summary(rows, _ratio, ci, B, conf, seed, event=True, tau=tau_m,
                              fail=np.asarray(s["delta_fail"], float))
        else:
            res[k] = _summary(rows, _ratio, ci, B, conf, seed)
    eta_i = {int(i): (v[0] / v[1] if v[1] > 0 else float("nan")) for i, v in sorted(s["eta_i"].items())}
    res["eta_vessel"] = eta_i
    res["J"] = jain(list(eta_i.values()))
    res["eta_minmax"] = {"value": min_max_ratio(list(eta_i.values()))}
    if ci and len(eta_i) >= 2:
        rows, _ = _eta_ratio_rows(s)
        r = block_bootstrap_bounds([rows], _eta_minmax, conf=conf, B=B, tau=tau_m, seed=seed)
        res["eta_minmax"].update(lower=r["lower"], ci=r["ci"], block=r["block"])
    res["prefix"] = prefix_completeness(s, dtp.L)
    res["fleet_consistency"] = fleet_consistency(s, dtp)
    un = np.asarray(s["unused"], float).reshape(-1, 2)
    res["unused"] = {"value": _ratio(un[~np.isnan(un).any(axis=1)]) if len(un) else float("nan")}
    f = np.asarray(s["fill"], float)
    f = f[~np.isnan(f)]
    res["fill"] = {"mean": float(f.mean()) if len(f) else float("nan"), "min": float(f.min()) if len(f) else float("nan"),
                   "n": int(len(f)), "valid": bool(len(f) and f.mean() >= FILL_MIN), "threshold": FILL_MIN}
    return res


def analyze(records: Sequence[Dict], dtp: DTP, criterion: Optional[str] = None, tol: float = 1e-6, ci: bool = True,
            B: int = 2000, conf: float = 0.95, seed: int = 0, kappa: float = 1.0) -> Dict:
    """Compatibility wrapper of run_metrics. The criterion should be given; without it the wrapper warns
    and uses "count", and the result says so (criterion_explicit False)."""
    explicit = criterion is not None
    if not explicit:
        warnings.warn("fleetdt.analysis.analyze without criterion: using 'count'; pass criterion='count' or "
                      "'bytes' (pre-registered choice)", FutureWarning, stacklevel=2)
        criterion = "count"
    res = run_metrics(records, dtp, criterion=criterion, tol=tol, ci=ci, B=B, conf=conf, seed=seed, kappa=kappa)
    res["criterion_explicit"] = explicit
    return res


# --------------------------------------------------------------------------- H4a over pooled runs


def condition_steps(records: Sequence[Dict], dtp: DTP, criterion: str, kappa: float = 1.0, tol: float = 1e-6,
                    lag_fails: bool = True) -> Dict[str, np.ndarray]:
    """Per fleet step of one run (sorted by t): margin s(t) = kappa y(t) - U C_hat(t); cond(t) = s(t) >= 0
    and the send buffers empty at the start of phase 3 (record field "buf_empty", taken as true when
    absent) and m(t) >= 1; fail(t) = not every message of level 1 or 2 of M(t) received within the step
    (criterion), or, with lag_fails, an emulator lag flagged in the record ("lag" true: worst case)."""
    _check_criterion(criterion)
    s = step_rows(records, dtp, criterion, tol, kappa)
    recs = sorted(records, key=lambda r: int(r["t"]))
    mg = np.asarray(s["margin"], float)
    phi = np.asarray(s["phi"], float)
    buf = np.asarray([bool(r.get("buf_empty", True)) for r in recs])
    lag = np.asarray([bool(r.get("lag", False)) for r in recs])
    cond = (~np.isnan(mg)) & (np.nan_to_num(mg, nan=-1.0) >= 0) & buf & ~np.isnan(phi)
    fail = (phi == 0.0) | (lag & bool(lag_fails))
    return {"margin": mg, "cond": cond, "fail": fail, "buf_known": np.asarray(["buf_empty" in r for r in recs])}


def _windows(c: Dict[str, np.ndarray], w: int) -> Dict[str, np.ndarray]:
    """Non-overlapping windows of w steps (Corollary 1): a window meets the condition when every step
    does (buffers empty at the window start are part of the condition of its first step); it fails when
    any step fails; its margin is the smallest margin of its steps."""
    n = (len(c["cond"]) // w) * w
    if n == 0:
        return {"margin": np.zeros(0), "cond": np.zeros(0, bool), "fail": np.zeros(0, bool)}
    mg = c["margin"][:n].reshape(-1, w)
    wm = np.where(np.isnan(mg).any(axis=1), np.nan, np.min(np.nan_to_num(mg, nan=0.0), axis=1))
    return {"margin": wm,
            "cond": c["cond"][:n].reshape(-1, w).all(axis=1), "fail": c["fail"][:n].reshape(-1, w).any(axis=1)}


def h4a_bound(runs: Sequence[Sequence[Dict]], dtp: DTP, criterion: str, kappa: float = 1.0, window: int = 1,
              conf: float = 0.95, tol: float = 1e-6, lag_fails: bool = True) -> Dict:
    """H4a pooled over runs: P(failure of the fleet step | condition of Proposition 2) with window = 1, or
    P(failure of an 8-step window | condition in every step of the window) with window = 8 (Corollary 1,
    non-overlapping windows). tau_int comes from the margin series (window minimum for windows), pooled
    over the runs; n_eff = condition units / (2 tau_int). upper: Clopper-Pearson with the raw events in
    n_eff trials (pre-registered, conservative); upper_scaled: with k n_eff / n events."""
    margins, n_cond, k = [], 0, 0
    buf_known = True
    for recs in runs:
        if not recs:
            continue
        c = condition_steps(recs, dtp, criterion, kappa, tol, lag_fails)
        buf_known = buf_known and bool(c["buf_known"].all())
        if window > 1:
            c = _windows(c, window)
        margins.append(c["margin"])
        n_cond += int(c["cond"].sum())
        k += int((c["cond"] & c["fail"]).sum())
    tau = tau_int_runs(margins) if margins else 0.5
    ne = n_eff(n_cond, tau)
    return {"window": window, "n_cond": n_cond, "failures": k, "tau_margin": tau, "n_eff": ne,
            "upper": clopper_pearson_upper_eff(k, n_cond, ne, conf=conf, events="raw") if n_cond else 1.0,
            "upper_scaled": clopper_pearson_upper_eff(k, n_cond, ne, conf=conf, events="scaled") if n_cond else 1.0,
            "conf": conf, "kappa": kappa, "criterion": criterion, "buffers_known": buf_known, "runs": len(margins)}


# --------------------------------------------------------------------------- report

METRICS = ("MAPE", "gamma", "delta", "phi", "eta", "mu", "nu")


def format_table(name: str, res: Dict) -> str:
    lines = ["%s: steps %s..%s (%d), criterion %s, tau_int of the margin %.2f" % (
        name, res["t_first"], res["t_last"], res["steps"], res["criterion"], res["tau_margin"]),
             "%-6s %10s %10s %10s %8s %9s %11s" % ("metric", "value", "ci_low", "ci_high", "tau_int", "n_eff", "viol_upper")]
    f = lambda v: "%10.5f" % v if v is not None and v == v else "%10s" % "-"
    for k in METRICS:
        m = res[k]
        lo, hi = m.get("ci", [None, None])
        lines.append("%-6s %s %s %s %8.2f %9.1f %11s" % (k, f(m["value"]), f(lo), f(hi), m["tau_int"] if m["n"] else float("nan"),
                                                          m["n_eff"], ("%.5f" % m["viol_upper"]) if "viol_upper" in m else "-"))
    lines.append("eta per vessel: " + ", ".join("b%d %.5f" % (i, v) for i, v in res["eta_vessel"].items()))
    lines.append("Jain J = %.5f" % res["J"] if res["J"] == res["J"] else "Jain J = -")
    mm = res["eta_minmax"]
    lines.append("min/max eta = %s%s" % (f(mm["value"]).strip(), (", lower %.5f" % mm["lower"]) if "lower" in mm else ""))
    pf = res["prefix"]
    lines.append("prefix completeness: mean length %.3f of %d (%.4f); by rank %s" % (
        pf["mean_len"], len(pf["by_rank"]), pf["mean_frac"], " ".join("%.3f" % v for v in pf["by_rank"])))
    lines.append("fleet consistency: " + ", ".join("%s %.4f" % kv for kv in res["fleet_consistency"].items()))
    lines.append("unused share %s; fill ratio mean %s min %s (valid: %s)" % (
        f(res["unused"]["value"]).strip(), f(res["fill"]["mean"]).strip(), f(res["fill"]["min"]).strip(), res["fill"]["valid"]))
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.analysis", description="Section VI metrics of a Fleet-DT DTA")
    ap.add_argument("dta", nargs="+", help="DTA files (JSON lines of G(t)), one per run")
    ap.add_argument("--dtp", required=True, help="DTP file (dtp.json)")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--criterion", choices=CRITERIA, required=True,
                    help="full reception of an admitted edge (pre-registered): n >= messages forwarded by the vessel "
                         "regulator (count; n_l = ceil(r_l Ts) without the regulator log) or r >= d (1 - tol) - 1 (bytes)")
    ap.add_argument("--kappa", type=float, default=1.0, help="kappa of the margin kappa y - U C_hat (calibration minimum of C/y)")
    ap.add_argument("--tol", type=float, default=1e-6)
    ap.add_argument("--B", type=int, default=2000, help="bootstrap resamples")
    ap.add_argument("--conf", type=float, default=0.95)
    ap.add_argument("--no-ci", action="store_true")
    ap.add_argument("--h4a", action="store_true", help="also the pooled H4a bounds (steps and 8-step windows)")
    a = ap.parse_args(argv)
    dtp = DTP.load(a.dtp)
    out = {}
    runs = {}
    for path in a.dta:
        runs[path] = load_dta(path)
        out[path] = run_metrics(runs[path], dtp, criterion=a.criterion, tol=a.tol, ci=not a.no_ci, B=a.B, conf=a.conf,
                                kappa=a.kappa)
    if a.h4a:
        out["_h4a"] = {"steps": h4a_bound(list(runs.values()), dtp, a.criterion, a.kappa, 1, a.conf, a.tol),
                       "windows8": h4a_bound(list(runs.values()), dtp, a.criterion, a.kappa, 8, a.conf, a.tol)}
    if a.json:
        print(json.dumps(out, indent=1))
    else:
        print("\n\n".join(format_table(p, r) for p, r in out.items() if not p.startswith("_")))
        if a.h4a:
            for k, v in out["_h4a"].items():
                print("H4a %s: %d units with the condition, %d failures, tau_int %.2f, n_eff %.0f, upper %.2e" % (
                    k, v["n_cond"], v["failures"], v["tau_margin"], v["n_eff"], v["upper"]))


if __name__ == "__main__":
    main()
