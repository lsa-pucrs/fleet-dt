"""Metrics of Section VI of the Fleet-DT paper computed from a DTA (JSON lines of G(t)) and the DTP.

Per run (names of the paper; the names of earlier versions stay in the JSON output as aliases for one
release: gamma = PICP, delta = PDR_state, eta_vessel = PDR_i, Pi = PCR, nu = u, kappa = c_y; phi keeps the
full-delivery fraction 1 - DMR):
  K_hat(t) = C_hat(t) (T3 - T_m)/Ts (predicted capacity of phase 3) and K_y(t) = c_y y(t) (T3 - T_m)/Ts - B_ctl_bar
  (measured capacity); c_y, T_m and B_ctl_bar come from the DTP (c_y can be overridden).
  MAPE       mean of |c_y y(t) - C_hat(t)| / (c_y y(t)) over the steps with y(t) > 0 (fidelity, Definition of the
             paper)
  PICP       prediction interval coverage probability: fraction of the steps of S (y(t) and C_hat(t)
             defined) with K_y(t) >= U K_hat(t) (C13)
  PICP_C     the same with the ground truth K(t) - B_ctl(t) in place of K_y(t), K(t) = C(t) (T3 - T_m)/Ts (C(t) of
             the subset replays, B_ctl(t) the control bytes outside the data bridges in phase 3), when given
  PDR_state  fraction of the pairs (t, i), i in M(t) and "state" in A(t) ∩ N(t), in which the GS
             applications receive every state message forwarded in step t exactly once within step t
             (C8: n_rx == n_fwd of the regulator log, else n_rx == ceil(r Ts), and no duplicate)
  DMR        deadline miss ratio: fraction of the fleet steps in which some message of level 1 or 2 of a
             vessel of M(t) misses its step (DMR = 1 - phi, phi the full-delivery fraction)
  DMR_hat    DMR predicted by the backlog predictor of C5 (fleetdt.outer.backlog_admitted):
             q_hat(t) = max(0, q_hat(t-1) + sum_i a_i(t) - K_y(t)), DMR_hat = fraction of the steps with q_hat > 0
  eta        bytes of the N(t) topics received within the step / bytes of the N(t) topics demanded
  PDR_i      per vessel i, the byte-weighted delivery ratio of the needed topics of vessel i (eta_i), with
             Jain's fairness index J and the ratio min_i PDR_i / max_i PDR_i (H5d) over vessels 1 to 3 only
             (the vessels with the same sources; vessel 4 has no video in the fairness cells)
  mu         late messages / (received + late)
  u          measured utilization: sum r_il(t) / sum (K_y(t) + B_ctl_bar), over the steps with y(t)
  PCR        prefix completeness ratio (C21): mean over the pairs (t, i), i in M(t), |N(t)| >= 1, of the share
             of N(t) in the longest prefix of sigma_t that vessel i delivers complete within step t
             (denominator |N(t)|); prefix also holds, for each rank k of sigma_t, the fraction of the pairs
             in which topics sigma_t(1..k) arrive complete, and the mean prefix length
  RelMSE     MSE_ES / MSE_alt of the one-step prediction of y(t), C_hat(t) of the run (ES) against the
             persistence y(t - 1) (alt); H3 tests log RelMSE (rel_mse for other alternatives)
  fleet      fleet consistency: for each topic l, the fraction of the steps (with a positive demand of l)
             in which l arrives complete from every vessel of M(t)
  unused     share of the cap m(t) b(t) = max(0, U K_hat(t) - Q(t)) granted to vessels of M(t) with a zero demand
             table: sum_t z(t) b(t) / sum_t m(t) b(t)
  fill       fill ratio f(t) = max_{i in M(t)} a_i(t) / b(t) (a_i of Algorithm 1); a run is valid for E3 and
             E4 when mean f >= 0.85 (FILL_MIN)

The full-reception criterion of an edge is explicit (argument `criterion`, required on the command line):
  count  n >= messages forwarded by the vessel regulator (fwd of the regulator log; n_l = ceil(r_l Ts)
         without the log); for completeness (prefix, fleet) n >= fwd + held, i.e. every message produced
  bytes  r >= d (1 - tol) - 1

Statistics (C20): integrated autocorrelation time tau_int = 1/2 + sum_{k>=1} rho(k) (one run or pooled over
runs), effective sample size n_eff = n / (2 tau_int), circular block bootstrap (two-sided CI and one-sided
bounds, blocks inside each run), Clopper-Pearson upper limit with the raw count k of events in n_eff
trials (for k = 0 it is 1 - 0.05^(1/n_eff) at 95 %; the slack series s(t) = K_y(t) - U K_hat(t) is the
proxy whose tau_int gives n_eff for the event metrics), the H4a bound on the fleet steps with
K_y(t) >= U K_hat(t) and Q(t) = 0 (and on non-overlapping 8-step windows, Corollary 1), Diebold-Mariano test (Newey-West variance), one-sided exact
Wilcoxon signed-rank test with Pratt's treatment of zeros (exact conditional distribution, ties
included), Holm, the intersection-union test (IUT), the fixed-sequence gatekeeping
H1 -> H2 -> H4 -> H5 -> H3 -> H6, TOST on the signed run-level differences, and the Hodges-Lehmann estimate.

  python -m fleetdt.analysis dta.jsonl [more.jsonl ...] --dtp dtp.json --criterion bytes|count [--c-y C] [--json]
  python -m fleetdt.analysis calibrate RUN [RUN ...] [--json]     c_y, T_m and B_ctl_bar from calibration runs

Calibration (calibrate_c_y, calibrate_margins, calibrate_run): c_y = smallest C(t)/y(t) of a saturation test
(C(t) of emu/replay_subsets.py, RUN/csub/c_t.jsonl, y(t) of the channel monitor); T_m = eps + delta_GS with
eps = 2 max (chrony max error) + 1 ms and delta_GS = largest delivery time of the GS broker measured in Q;
B_ctl_bar = largest bytes outside the data bridges in phase 3 of a calibration step. TODO: the runs do not log
the chrony max error, the delivery time of the GS broker nor the control bytes per phase, so calibrate_run
reports T_m and B_ctl_bar as missing and the DTPs keep T_m = 0 and B_ctl_bar = 0 until those measurements exist.
"""
from __future__ import annotations

import argparse
import json
import math
import warnings
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from .as_model import DTP
from .outer import admitted_bytes, backlog_admitted, backlog_of, load_dta

CRITERIA = ("bytes", "count")
FILL_MIN = 0.85
FAIR_VESSELS = (1, 2, 3)   # H5d: vessels with the same sources (vessel 4 has no video in the fairness cells)
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
    """One-sided Clopper-Pearson upper limit with n_eff effective trials (C20), n_eff = n / (2 tau_int).
    events "raw": the k observed events in n_eff trials (conservative; the pre-registered rule of H1 and
    H4a: an upper limit below 1e-3 needs n_eff >= 2995 with 0 events, 4742 with 1, 6294 with 2; for k = 0
    the limit is 1 - (1 - conf)^(1/n_eff), 1 - 0.05^(1/n_eff) at 95 %);
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


def tost(d: Sequence[float], margin: Union[float, Tuple[float, float]], alpha: float = 0.05,
         b: Optional[Sequence[float]] = None) -> Dict:
    """Two one-sided t tests (TOST) of equivalence on the signed run-level differences d (one value per
    run; with b, d = a - b of the paired values a = d and b): H0: mean(d) <= low or mean(d) >= high, with
    (low, high) = (-margin, margin) or the given pair. The differences keep their sign (C20): a test on
    |a - b| would put every difference on one side of 0 and never reject the lower null.
    p = max(p_low, p_high); equivalence at level alpha iff p < alpha, which is the same as the
    (1 - 2 alpha) t interval of mean(d) lying inside (low, high). Returns {"mean", "se", "df", "ci",
    "p_low", "p_high", "p", "equivalent", "n"}."""
    from scipy.stats import t as tdist
    low, high = (-abs(float(margin)), abs(float(margin))) if np.isscalar(margin) else (float(margin[0]), float(margin[1]))
    x = np.asarray(d, dtype=float) - (0.0 if b is None else np.asarray(b, dtype=float))
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
    """min_i x_i / max_i x_i over the finite values (H5d with x = PDR_i)."""
    x = np.asarray([v for v in x if v == v], dtype=float)
    return float(x.min() / x.max()) if len(x) and x.max() > 0 else float("nan")


def rel_mse(y: Sequence[float], pred_es: Sequence[float], pred_alt: Sequence[float]) -> Dict:
    """RelMSE = MSE_ES / MSE_alt (Hyndman and Koehler 2006) of two one-step predictions of y over the steps
    where y and both predictions are defined; H3 tests log RelMSE (< 0: ES better). Returns {"value",
    "log", "MSE_ES", "MSE_alt", "n"}."""
    y, a, b = (np.asarray(v, dtype=float) for v in (y, pred_es, pred_alt))
    ok = ~(np.isnan(y) | np.isnan(a) | np.isnan(b))
    nan = float("nan")
    if not ok.any():
        return {"value": nan, "log": nan, "MSE_ES": nan, "MSE_alt": nan, "n": 0}
    m1, m2 = float(np.mean((a[ok] - y[ok]) ** 2)), float(np.mean((b[ok] - y[ok]) ** 2))
    v = m1 / m2 if m2 > 0 else nan
    return {"value": v, "log": math.log(v) if v == v and v > 0 else nan, "MSE_ES": m1, "MSE_alt": m2,
            "n": int(ok.sum())}


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


def _cy(c_y: Optional[float], kappa: Optional[float], default: float = 1.0) -> float:
    """c_y, with kappa (its name in earlier versions) as a backward-compatible alias; `default` (the c_y of the
    DTP) when neither is given."""
    if kappa is not None:
        if c_y is not None and float(c_y) != float(kappa):
            raise ValueError("c_y and kappa (its old name) differ")
        return float(kappa)
    return float(default) if c_y is None else float(c_y)


def capacities(g: Dict, dtp: DTP, cy: float) -> Tuple[float, float]:
    """(K_y(t), K_hat(t)) of one record: K_y = c_y y (T3 - T_m)/Ts - B_ctl_bar and K_hat = C_hat (T3 - T_m)/Ts
    (NaN without y or C_hat)."""
    y, C = g.get("y"), g.get("C_hat")
    r = dtp.T3m / dtp.Ts
    Ky = cy * float(y) * r - dtp.B_ctl_bar if y is not None else np.nan
    Kh = float(C) * r if C is not None else np.nan
    return Ky, Kh


def slack(g: Dict, dtp: DTP, cy: float, U: float) -> float:
    """K_y(t) - U K_hat(t) of one record, computed as (c_y y - B_ctl_bar / r - U C_hat) r with r = (T3 - T_m)/Ts, so
    the sign is exact at K_y = U K_hat when B_ctl_bar = 0 (NaN without y or C_hat)."""
    y, C = g.get("y"), g.get("C_hat")
    if y is None or C is None:
        return np.nan
    r = dtp.T3m / dtp.Ts
    return (cy * float(y) - dtp.B_ctl_bar / r - U * float(C)) * r


def margin_series(records: Sequence[Dict], dtp: DTP, c_y: Optional[float] = None,
                  kappa: Optional[float] = None) -> np.ndarray:
    """Slack of the condition of Proposition 2, s(t) = K_y(t) - U K_hat(t), sorted by t (NaN without y or
    C_hat); c_y (kappa: old name; default: the c_y of the DTP) is the calibrated lower bound of C(t) / y(t)."""
    cy = _cy(c_y, kappa, dtp.c_y)
    out = []
    for g in sorted(records, key=lambda r: int(r["t"])):
        U = g.get("U") if g.get("U") is not None else dtp.U
        out.append(slack(g, dtp, cy, U))
    return np.asarray(out, dtype=float)


def _state_ok(e: Dict, n_expected: Dict[int, int]) -> bool:
    """C8: the GS applications receive every state message forwarded in step t exactly once within step t:
    n_rx == n_fwd (fwd of the regulator log; without it, n_l = ceil(r_l Ts)) and no duplicate."""
    want = e.get("fwd")
    if want is None:
        want = n_expected.get(e["l"], 1)
    return int(e.get("n", 0)) == int(want) and int(e.get("dup", 0)) == 0


def step_rows(records: Sequence[Dict], dtp: DTP, criterion: str, tol: float = 1e-6, c_y: Optional[float] = None,
              kappa: Optional[float] = None) -> Dict:
    """Per-step quantities of the metrics, sorted by t. Ratio metrics keep (numerator, denominator) rows."""
    _check_criterion(criterion)
    cy = _cy(c_y, kappa, dtp.c_y)
    recs = sorted(records, key=lambda r: int(r["t"]))
    L = dtp.L
    state = dtp.index("state") if "state" in dtp.topics else None
    n_expected = {k + 1: int(math.ceil(s.rate * dtp.Ts - 1e-9)) for k, s in enumerate(dtp.specs) if s.rate}
    default_order = [dtp.index(n) for n in dtp.order] if dtp.order else list(range(1, L + 1))
    out = {"t": [], "ape": [], "cover": [], "phi": [], "delta": [], "delta_fail": [], "eta": [], "mu": [], "nu": [],
           "margin": [], "eta_i": {}, "eta_rows": [], "prefix": [], "pcr": [], "fleet": [], "unused": [], "fill": [],
           "order": [], "A": [], "y": [], "C_hat": [], "K_y": [], "K_hat": [], "Q": [], "U": []}
    for g in recs:
        t, y, C = int(g["t"]), g.get("y"), g.get("C_hat")
        U = g.get("U") if g.get("U") is not None else dtp.U
        Ky, Kh = capacities(g, dtp, cy)
        Mlist = sorted(set(g.get("vessels", [])))
        M = set(Mlist)
        N = set(g.get("needed", []))
        sigma = list(g.get("order") or default_order)
        edges = g.get("edges", [])
        by = {(e["i"], e["l"]): e for e in edges}
        out["t"].append(t)
        out["order"].append(sigma)
        # fidelity: C_hat(t) against c_y y(t); coverage and slack in bytes of phase 3
        out["ape"].append(abs(cy * y - C) / (cy * y) if y is not None and C is not None and y > 0 else np.nan)
        sl = slack(g, dtp, cy, U)
        out["cover"].append(float(sl >= 0) if sl == sl else np.nan)          # K_y(t) >= U K_hat(t)
        out["margin"].append(sl)
        out["y"].append(float(y) if y is not None else np.nan)
        out["C_hat"].append(float(C) if C is not None else np.nan)
        out["K_y"].append(Ky)
        out["K_hat"].append(Kh)
        out["U"].append(float(U))
        out["Q"].append(backlog_of(g))
        out["A"].append(admitted_bytes(g))
        inM = [e for e in edges if e["i"] in M]
        adm = [e for e in inM if (e.get("p") or 0) > 0]
        out["phi"].append(float(all(_received_all(e, criterion, tol, n_expected) for e in adm)) if M else np.nan)
        ok = pairs = 0
        if state is not None and state in N:
            for e in adm:
                if e["l"] == state:
                    pairs += 1
                    ok += int(_state_ok(e, n_expected))
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
        # u = sum r_il(t) / (K_y(t) + B_ctl_bar)
        out["nu"].append((sum(e.get("r", 0) for e in edges), Ky + dtp.B_ctl_bar) if y is not None
                         else (np.nan, np.nan))
        # prefix completeness: length of the complete prefix of sigma_t, per vessel of M(t)
        comp = {i: [_edge_complete(by.get((i, l)), criterion, tol, n_expected) for l in sigma] for i in Mlist}
        pre, pcr = {}, {}
        for i in Mlist:
            k = 0
            while k < len(sigma) and comp[i][k]:
                k += 1
            pre[i] = k
            if N:   # C21: share of N(t) in the complete prefix, denominator |N(t)|
                pcr[i] = len(N.intersection(sigma[:k])) / float(len(N))
        out["prefix"].append(pre)
        out["pcr"].append(pcr)
        # fleet consistency: topic l complete from every vessel of M(t), for the topics with a demand
        fl = {}
        for pos, l in enumerate(sigma):
            if Mlist and any((by.get((i, l)) or {}).get("d", 0) > 0 for i in Mlist):
                fl[l] = all(comp[i][pos] for i in Mlist)
        out["fleet"].append(fl)
        # unused share and fill ratio (Algorithm 1: a_i = sum of d_il over the admitted topics)
        b = g.get("b")
        if Mlist and b is not None and b > 0:
            D0 = [i for i in Mlist if sum((by.get((i, l)) or {}).get("d", 0) for l in range(1, L + 1)) == 0]
            out["unused"].append((len(D0) * b, b * len(Mlist)))   # of the cap m b = max(0, U K_hat - Q)
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
    (events: steps with a violation, given by `fail` or 1 - x) with the raw count in n_eff effective steps
    (viol_upper_scaled: the count scaled to n_eff)."""
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
        # C20: the raw count k in n_eff trials (the rule of H4a); "viol_upper_scaled" keeps k n_eff / n
        out["viol_upper"] = clopper_pearson_upper_eff(k, m, ne, conf=conf, events="raw") if m else 1.0
        out["viol_upper_scaled"] = clopper_pearson_upper_eff(k, m, ne, conf=conf, events="scaled") if m else 1.0
    return out


def prefix_completeness(s: Dict, L: int) -> Dict:
    """From step_rows: PCR (C21), the mean over the pairs (t, i), i in M(t), |N(t)| >= 1, of
    |N(t) ∩ sigma_t(1..k_i(t))| / |N(t)| with k_i(t) the length of the complete prefix ("PCR",
    "pcr_pairs"); for each rank k = 1..L, the fraction of the pairs whose topics sigma_t(1..k) all arrived
    complete; the mean complete-prefix length (topics) and, as in earlier versions, its share of L
    ("mean_frac", denominator L; not the PCR)."""
    lens = [k for pre in s["prefix"] for k in pre.values()]
    pcr = [v for row in s.get("pcr", []) for v in row.values()]
    p = float(np.mean(pcr)) if pcr else float("nan")
    if not lens:
        return {"by_rank": [float("nan")] * L, "mean_len": float("nan"), "mean_frac": float("nan"), "pairs": 0,
                "PCR": p, "pcr_pairs": len(pcr)}
    a = np.asarray(lens)
    return {"by_rank": [float(np.mean(a >= k)) for k in range(1, L + 1)], "mean_len": float(a.mean()),
            "mean_frac": float(a.mean() / L), "pairs": int(len(a)), "PCR": p, "pcr_pairs": len(pcr)}


def fleet_consistency(s: Dict, dtp: DTP) -> Dict[str, float]:
    """From step_rows: per topic name, the fraction of the steps (with a positive demand of the topic in
    M(t)) in which the topic arrived complete from every vessel of M(t)."""
    num, den = {}, {}
    for fl in s["fleet"]:
        for l, v in fl.items():
            den[l] = den.get(l, 0) + 1
            num[l] = num.get(l, 0) + int(v)
    return {dtp.topics[l - 1]: num[l] / den[l] for l in sorted(den)}


def _eta_ratio_rows(s: Dict, vessels: Optional[Sequence[int]] = None) -> Tuple[np.ndarray, List[int]]:
    vs = sorted(i for i in s["eta_i"] if vessels is None or i in vessels)
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


def picp_ground_truth(s: Dict, dtp: DTP, C_gt: Dict[int, float], B_ctl: Optional[Dict[int, float]] = None) -> Dict:
    """PICP_C(U): fraction of the steps of step_rows with K(t) - B_ctl(t) >= U K_hat(t), K(t) = C(t) (T3 - T_m)/Ts,
    C(t) of the ground truth (subset replays) and B_ctl(t) the control bytes outside the data bridges in phase 3
    (0 when not given), over the steps with C(t) and C_hat(t)."""
    r = dtp.T3m / dtp.Ts
    hits = []
    for t, Ch, U in zip(s["t"], s["C_hat"], s["U"]):
        C = C_gt.get(t)
        if C is None or Ch != Ch:
            continue
        # K(t) - B_ctl(t) >= U K_hat(t), compared in units of C (divided by (T3 - T_m)/Ts)
        hits.append(float(float(C) - float((B_ctl or {}).get(t, 0.0)) / r >= U * Ch))
    return {"value": float(np.mean(hits)) if hits else float("nan"), "n": len(hits)}


def run_metrics(records: Sequence[Dict], dtp: DTP, *, criterion: str, tol: float = 1e-6, ci: bool = True,
                B: int = 2000, conf: float = 0.95, seed: int = 0, c_y: Optional[float] = None,
                kappa: Optional[float] = None, C_gt: Optional[Dict[int, float]] = None,
                B_ctl: Optional[Dict[int, float]] = None, fair_vessels: Optional[Sequence[int]] = FAIR_VESSELS) -> Dict:
    """Metrics of one run (Section VI) with an explicit full-reception criterion ("bytes" or "count").
    The event metrics (PICP, DMR, PDR_state) take tau_int from the slack series s(t) = K_y(t) - U K_hat(t)
    and the fleet step as the unit (n_eff = steps / (2 tau_int), no factor of messages per step).
    c_y (default: the DTP; alias kappa, its old name) enters K_y and the MAPE; C_gt (t -> C(t)) and B_ctl
    (t -> B_ctl(t)) give PICP_C; the H5d ratio min_i PDR_i / max_i PDR_i takes the vessels of fair_vessels
    (None: every vessel). The output holds the names of the paper and, for one release, the names of earlier
    versions (module docstring)."""
    _check_criterion(criterion)
    cy = _cy(c_y, kappa, dtp.c_y)
    s = step_rows(records, dtp, criterion, tol, cy)
    mean = lambda z: float(np.mean(z))
    res = {"steps": len(s["t"]), "t_first": s["t"][0] if s["t"] else None, "t_last": s["t"][-1] if s["t"] else None,
           "criterion": criterion, "c_y": cy, "kappa": cy, "T_m": dtp.T_m, "B_ctl_bar": dtp.B_ctl_bar}
    mg = np.asarray(s["margin"], float)
    tau_m = tau_int(mg) if np.any(~np.isnan(mg)) else 0.5
    res["tau_margin"] = tau_m
    res["MAPE"] = _summary(np.asarray(s["ape"], float), mean, ci, B, conf, seed)
    res["PICP"] = res["gamma"] = _summary(np.asarray(s["cover"], float), mean, ci, B, conf, seed, event=True,
                                          tau=tau_m)
    if C_gt:
        res["PICP_C"] = picp_ground_truth(s, dtp, C_gt, B_ctl)
    phi = np.asarray(s["phi"], float)
    miss = np.where(np.isnan(phi), np.nan, 1.0 - phi)
    res["DMR"] = _summary(miss, mean, ci, B, conf, seed, event=True, tau=tau_m, fail=miss)
    res["phi"] = _summary(phi, mean, ci, B, conf, seed, event=True, tau=tau_m)
    for k in ("delta", "eta", "mu", "nu"):
        rows = np.asarray(s[k], float).reshape(-1, 2)
        if k == "delta":
            res[k] = _summary(rows, _ratio, ci, B, conf, seed, event=True, tau=tau_m,
                              fail=np.asarray(s["delta_fail"], float))
        else:
            res[k] = _summary(rows, _ratio, ci, B, conf, seed)
    res["PDR_state"], res["u"] = res["delta"], res["nu"]
    # C5: DMR predicted from the admitted bytes A(t) of the run and K_y(t)
    q, ok = backlog_admitted(np.asarray(s["y"], float), np.asarray(s["A"], float), dtp.T3m / dtp.Ts, c_y=cy,
                             B_ctl_bar=dtp.B_ctl_bar)
    res["DMR_hat"] = {"value": float(np.mean(q[ok] > 0.0)) if ok.any() else float("nan"), "n": int(ok.sum()),
                      "c_y": cy}
    # RelMSE of the ES prediction C_hat(t) of the run against persistence y(t - 1)
    yv = np.asarray(s["y"], float)
    res["RelMSE"] = dict(rel_mse(yv, np.asarray(s["C_hat"], float), np.r_[np.nan, yv[:-1]]), alt="persistence")
    eta_i = {int(i): (v[0] / v[1] if v[1] > 0 else float("nan")) for i, v in sorted(s["eta_i"].items())}
    res["PDR_i"] = res["eta_vessel"] = eta_i
    res["J"] = jain(list(eta_i.values()))
    fair = {i: v for i, v in eta_i.items() if fair_vessels is None or i in fair_vessels}
    res["eta_minmax"] = {"value": min_max_ratio(list(fair.values())), "vessels": sorted(fair)}
    if ci and len(fair) >= 2:
        rows, _ = _eta_ratio_rows(s, None if fair_vessels is None else list(fair_vessels))
        r = block_bootstrap_bounds([rows], _eta_minmax, conf=conf, B=B, tau=tau_m, seed=seed)
        res["eta_minmax"].update(lower=r["lower"], ci=r["ci"], block=r["block"])
    res["PDR_minmax"] = res["eta_minmax"]
    res["prefix"] = prefix_completeness(s, dtp.L)
    res["PCR"] = res["Pi"] = {"value": res["prefix"]["PCR"], "pairs": res["prefix"]["pcr_pairs"]}
    res["fleet_consistency"] = fleet_consistency(s, dtp)
    un = np.asarray(s["unused"], float).reshape(-1, 2)
    res["unused"] = {"value": _ratio(un[~np.isnan(un).any(axis=1)]) if len(un) else float("nan")}
    f = np.asarray(s["fill"], float)
    f = f[~np.isnan(f)]
    res["fill"] = {"mean": float(f.mean()) if len(f) else float("nan"), "min": float(f.min()) if len(f) else float("nan"),
                   "n": int(len(f)), "valid": bool(len(f) and f.mean() >= FILL_MIN), "threshold": FILL_MIN}
    return res


def analyze(records: Sequence[Dict], dtp: DTP, criterion: Optional[str] = None, tol: float = 1e-6, ci: bool = True,
            B: int = 2000, conf: float = 0.95, seed: int = 0, c_y: Optional[float] = None,
            kappa: Optional[float] = None) -> Dict:
    """Compatibility wrapper of run_metrics. The criterion should be given; without it the wrapper warns
    and uses "count", and the result says so (criterion_explicit False)."""
    explicit = criterion is not None
    if not explicit:
        warnings.warn("fleetdt.analysis.analyze without criterion: using 'count'; pass criterion='count' or "
                      "'bytes' (pre-registered choice)", FutureWarning, stacklevel=2)
        criterion = "count"
    res = run_metrics(records, dtp, criterion=criterion, tol=tol, ci=ci, B=B, conf=conf, seed=seed, c_y=c_y,
                      kappa=kappa)
    res["criterion_explicit"] = explicit
    return res


# --------------------------------------------------------------------------- H4a over pooled runs


def condition_steps(records: Sequence[Dict], dtp: DTP, criterion: str, c_y: Optional[float] = None,
                    tol: float = 1e-6, lag_fails: bool = True, kappa: Optional[float] = None) -> Dict[str, np.ndarray]:
    """Per fleet step of one run (sorted by t): slack s(t) = K_y(t) - U K_hat(t); cond(t) = s(t) >= 0 (that is
    K_y(t) >= U K_hat(t)), the backlog Q(t) = 0 of the demand tables of M(t), the send buffers empty at the start
    of phase 3 (record field "buf_empty", taken as true when absent) and m(t) >= 1; fail(t) = not every
    message of level 1 or 2 of M(t) received within the step (criterion), or, with lag_fails, an emulator lag
    flagged in the record ("lag" true: worst case)."""
    _check_criterion(criterion)
    s = step_rows(records, dtp, criterion, tol, _cy(c_y, kappa, dtp.c_y))
    recs = sorted(records, key=lambda r: int(r["t"]))
    mg = np.asarray(s["margin"], float)
    phi = np.asarray(s["phi"], float)
    Q = np.asarray(s["Q"], float)
    buf = np.asarray([bool(r.get("buf_empty", True)) for r in recs])
    lag = np.asarray([bool(r.get("lag", False)) for r in recs])
    cond = (~np.isnan(mg)) & (np.nan_to_num(mg, nan=-1.0) >= 0) & (Q == 0) & buf & ~np.isnan(phi)
    fail = (phi == 0.0) | (lag & bool(lag_fails))
    return {"margin": mg, "cond": cond, "fail": fail, "buf_known": np.asarray(["buf_empty" in r for r in recs]),
            "Q_known": np.asarray([r.get("Q") is not None or bool(r.get("vertices")) for r in recs])}


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


def h4a_bound(runs: Sequence[Sequence[Dict]], dtp: DTP, criterion: str, c_y: Optional[float] = None, window: int = 1,
              conf: float = 0.95, tol: float = 1e-6, lag_fails: bool = True, kappa: Optional[float] = None) -> Dict:
    """H4a pooled over runs: P(failure of the fleet step | condition of Proposition 2: K_y(t) >= U K_hat(t) and
    Q(t) = 0) with window = 1, or P(failure of an 8-step window | condition in every step of the window) with
    window = 8 (Corollary 1, non-overlapping windows). tau_int comes from the slack series K_y(t) - U K_hat(t)
    (window minimum for windows), pooled over the runs; n_eff = condition units / (2 tau_int). upper:
    Clopper-Pearson with the raw events in n_eff trials (pre-registered, conservative; C20); upper_scaled:
    with k n_eff / n events. c_y has the alias kappa (old name); default: the c_y of the DTP."""
    cy = _cy(c_y, kappa, dtp.c_y)
    margins, n_cond, k = [], 0, 0
    buf_known = Q_known = True
    for recs in runs:
        if not recs:
            continue
        c = condition_steps(recs, dtp, criterion, cy, tol, lag_fails)
        buf_known = buf_known and bool(c["buf_known"].all())
        Q_known = Q_known and bool(c["Q_known"].all())
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
            "conf": conf, "c_y": cy, "kappa": cy, "criterion": criterion, "buffers_known": buf_known,
            "backlog_known": Q_known, "runs": len(margins)}


# --------------------------------------------------------------------------- calibration of c_y, T_m, B_ctl_bar


def calibrate_c_y(C_by_t: Dict[int, float], y_by_t: Dict[int, float]) -> Dict:
    """c_y = smallest C(t) / y(t) over the steps of a saturation test with both values and y(t) > 0, capped at 1
    (c_y lies in (0, 1]). Returns {"c_y", "n", "t_min"} (c_y None without a step)."""
    best, t_best, n = None, None, 0
    for t, C in C_by_t.items():
        y = y_by_t.get(t)
        if C is None or y is None or not y > 0 or not C > 0:
            continue
        n += 1
        v = float(C) / float(y)
        if best is None or v < best:
            best, t_best = v, t
    return {"c_y": None if best is None else min(1.0, best), "n": n, "t_min": t_best}


def calibrate_margins(chrony_max_error_s: Optional[Sequence[float]] = None,
                      gs_delivery_s: Optional[Sequence[float]] = None,
                      ctl_bytes_phase3: Optional[Sequence[float]] = None) -> Dict:
    """T_m = eps + delta_GS with eps = 2 max (chrony max error) + 1 ms and delta_GS = largest delivery time of the
    GS broker measured in Q, and B_ctl_bar = largest bytes outside the data bridges in phase 3 of a calibration
    step. A term without measurements stays None and is listed in "todo" (the DTP then keeps its default 0)."""
    todo = []
    eps = 2.0 * max(chrony_max_error_s) + 1e-3 if chrony_max_error_s else None
    dgs = float(max(gs_delivery_s)) if gs_delivery_s else None
    bctl = float(max(ctl_bytes_phase3)) if ctl_bytes_phase3 else None
    if eps is None:
        todo.append("eps: no chrony max error in the calibration logs")
    if dgs is None:
        todo.append("delta_GS: no delivery time of the GS broker measured in Q")
    if bctl is None:
        todo.append("B_ctl_bar: no bytes outside the data bridges per phase 3 in the calibration logs")
    T_m = eps + dgs if eps is not None and dgs is not None else None
    return {"eps": eps, "delta_GS": dgs, "T_m": T_m, "B_ctl_bar": bctl, "todo": todo}


def _jsonl(path: str) -> List[Dict]:
    out = []
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue
    except OSError:
        pass
    return out


def calibrate_run(run_dir: str) -> Dict:
    """Calibration from one saturation run of emu/fleet.py: c_y from RUN/csub/c_t.jsonl (C(t) of
    emu/replay_subsets.py) and y(t) of RUN/throughput.jsonl (else of the DTA RUN/vms/gs/dta.jsonl); T_m and
    B_ctl_bar from the measurements of calibrate_margins when the run holds them (RUN/calib.json with the lists
    "chrony_max_error_s", "gs_delivery_s", "ctl_bytes_phase3"), else None with a TODO."""
    import os
    C = {int(r["t"]): float(r["C"]) for r in _jsonl(os.path.join(run_dir, "csub", "c_t.jsonl"))
         if r.get("C") is not None and r.get("t") is not None}
    ys = _jsonl(os.path.join(run_dir, "throughput.jsonl")) or _jsonl(os.path.join(run_dir, "vms", "gs", "dta.jsonl"))
    y = {int(r["t"]): float(r["y"]) for r in ys if r.get("y") is not None and r.get("t") is not None}
    out = {"run": run_dir}
    out.update(calibrate_c_y(C, y))
    meas = {}
    try:
        with open(os.path.join(run_dir, "calib.json"), encoding="utf-8") as f:
            meas = json.load(f)
    except (OSError, ValueError):
        pass
    out.update(calibrate_margins(meas.get("chrony_max_error_s"), meas.get("gs_delivery_s"),
                                 meas.get("ctl_bytes_phase3")))
    if out["c_y"] is None:
        out["todo"].append("c_y: no C(t) of the subset replays (csub/c_t.jsonl) with y(t)")
    return out


# --------------------------------------------------------------------------- report

METRICS = ("MAPE", "PICP", "PDR_state", "DMR", "eta", "mu", "u")


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
    lines.append("PDR_i: " + ", ".join("b%d %.5f" % (i, v) for i, v in res["PDR_i"].items()))
    lines.append("Jain J = %.5f" % res["J"] if res["J"] == res["J"] else "Jain J = -")
    mm = res["eta_minmax"]
    lines.append("min/max PDR_i = %s%s" % (f(mm["value"]).strip(), (", lower %.5f" % mm["lower"]) if "lower" in mm else ""))
    pf = res["prefix"]
    lines.append("PCR %s (%d pairs); complete prefix: mean length %.3f of %d; by rank %s" % (
        f(pf["PCR"]).strip(), pf["pcr_pairs"], pf["mean_len"], len(pf["by_rank"]),
        " ".join("%.3f" % v for v in pf["by_rank"])))
    lines.append("DMR_hat %s (c_y %g); RelMSE (ES / persistence) %s" % (
        f(res["DMR_hat"]["value"]).strip(), res["c_y"], f(res["RelMSE"]["value"]).strip()))
    lines.append("fleet consistency: " + ", ".join("%s %.4f" % kv for kv in res["fleet_consistency"].items()))
    lines.append("unused share %s; fill ratio mean %s min %s (valid: %s)" % (
        f(res["unused"]["value"]).strip(), f(res["fill"]["mean"]).strip(), f(res["fill"]["min"]).strip(), res["fill"]["valid"]))
    return "\n".join(lines)


def main_calibrate(argv: List[str]) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.analysis calibrate",
                                 description="c_y, T_m and B_ctl_bar of the DTP from calibration runs")
    ap.add_argument("runs", nargs="+", help="run directories of emu/fleet.py (saturation runs with csub/)")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    per = [calibrate_run(r) for r in a.runs]
    cys = [p["c_y"] for p in per if p["c_y"] is not None]
    tms = [p["T_m"] for p in per if p["T_m"] is not None]
    bcs = [p["B_ctl_bar"] for p in per if p["B_ctl_bar"] is not None]
    out = {"c_y": min(cys) if cys else None, "T_m": max(tms) if tms else None,
           "B_ctl_bar": max(bcs) if bcs else None, "runs": per}
    if a.json:
        print(json.dumps(out, indent=1))
        return
    print("c_y = %s, T_m = %s s, B_ctl_bar = %s bytes (None: not measured, the DTP keeps its default)" % (
        out["c_y"], out["T_m"], out["B_ctl_bar"]))
    for p in per:
        for td in p["todo"]:
            print("TODO %s: %s" % (p["run"], td))


def main(argv: Optional[List[str]] = None) -> None:
    import sys
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv and argv[0] == "calibrate":
        main_calibrate(argv[1:])
        return
    ap = argparse.ArgumentParser(prog="fleetdt.analysis", description="Section VI metrics of a Fleet-DT DTA")
    ap.add_argument("dta", nargs="+", help="DTA files (JSON lines of G(t)), one per run")
    ap.add_argument("--dtp", required=True, help="DTP file (dtp.json)")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--criterion", choices=CRITERIA, required=True,
                    help="full reception of an admitted edge (pre-registered): n >= messages forwarded by the vessel "
                         "regulator (count; n_l = ceil(r_l Ts) without the regulator log) or r >= d (1 - tol) - 1 (bytes)")
    ap.add_argument("--c-y", "--kappa", dest="c_y", type=float, default=None,
                    help="c_y of K_y(t) = c_y y (T3 - T_m)/Ts - B_ctl_bar and of the MAPE (calibration minimum of C/y; "
                         "default: the DTP; --kappa: old name)")
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
                                c_y=a.c_y)
    if a.h4a:
        out["_h4a"] = {"steps": h4a_bound(list(runs.values()), dtp, a.criterion, a.c_y, 1, a.conf, a.tol),
                       "windows8": h4a_bound(list(runs.values()), dtp, a.criterion, a.c_y, 8, a.conf, a.tol)}
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
