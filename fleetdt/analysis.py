"""Metrics of Section VI of the Fleet-DT paper computed from a DTA (JSON lines of G(t)) and the DTP.

Per run (names of the paper; the names of earlier versions stay in the JSON output as aliases for one
release: gamma = PICP, delta = PDR_state, eta_vessel = PDR_i = BDR_i, Pi = PCR, nu = u, kappa = c_y; phi keeps
the full-delivery fraction 1 - DMR):
  K_hat(t) = C_hat(t) (T3 - T_m)/Ts (predicted capacity of phase 3) and K_y(t) = c_y y(t) (T3 - T_m)/Ts - B_ctl_bar
  (measured capacity); c_y, T_m and B_ctl_bar come from the DTP (c_y can be overridden).
  MASE       mean absolute scaled error (Hyndman and Koehler 2006): sum of |c_y y(t) - C_hat(t)| divided by the
             sum of c_y |y(t) - y(t-1)| (the error of the last-value predictor, which is (3) with alpha = 1), both
             over the steps t with y(t), y(t-1) and C_hat(t) defined (Definition of Fidelity); the steps with
             y(t) = y(t-1) stay in both sums
  PICP       fraction of the steps of S (y(t) and C_hat(t) defined) with K_y(t) >= U K_hat(t)
  PICP_C     the same with K(t) - B_ctl(t) in place of K_y(t), K(t) = C(t) (T3 - T_m)/Ts (C(t) of the subset
             replays, B_ctl(t) the control bytes outside the data bridges in phase 3), when given; "diff" holds
             PICP - PICP_C on the steps with both and its block-bootstrap bounds (H2)
  PDR_state  fraction of the pairs (t, i), i in M(t) and "state" in A(t) ∩ N(t), in which the GS applications
             receive within step t, exactly once after the deduplication, every message of topic state of
             step t of vessel i: n == the messages that the source produced in step t (src of the regulator
             step log, else fwd + held, else fwd, else ceil(f Ts)); duplicates that the GS removed do not
             fail a pair (Corollary 2); PDR_state_i per vessel (H4c)
  DMR        deadline miss ratio: fraction of the fleet steps in which the GS receives after the step, or
             never, a message that a vessel of M(t) forwards at level 1 or 2 (criterion "count": n < fwd)
  DMR_hat    fraction of the steps with q_hat(t) > 0 of the backlog recursion (eq. lindley with K_y(t)) and the
             final a_i of Algorithm 1 replayed on the records with alpha, U and Q(t) = q_hat(t - 1)
             (fleetdt.outer.dmr_hat_replay; alpha and U: those logged in the records); H3b replays the
             calibration DTA (emu/analysis/hypotheses.py)
  eta        bytes of the N(t) topics received within the step / bytes of the N(t) topics demanded
  BDR_i      byte delivery ratio of vessel i: sum of r_il(t) over l in N(t) and the steps with i in M(t),
             divided by the same sum of c_l(x_il(t), n_il(t)) of the content bytes and messages that the
             sources produced in step t (edge fields src_x and src_n, which emu/analysis/paper_post.py
             takes from the regulator step log and the vessel log; d_il(t) when a record lacks them, and
             "BDR_den" says which); Jain's index J (H4d) and the ratio min_i BDR_i / max_i BDR_i over the vessels
             of fair_vessels (1 to 3: the vessels with the same sources; vessel 4 has no video in the fairness
             cells); J_all over every vessel
  mu         late messages / (received + late)
  u          measured utilization: sum r_il(t) / sum (K_y(t) + B_ctl_bar), over the steps with y(t)
  PCR        prefix completeness ratio: mean over the pairs (t, i), i in M(t), |N(t)| >= 1, of
             min(pi_i(t), |N(t)|) / |N(t)|, pi_i(t) the longest prefix of sigma_t whose topics vessel i delivers
             complete within step t (criterion "count": every message that the sources produced, a topic
             without messages complete); prefix also holds, for each rank k of sigma_t, the fraction of the
             pairs in which topics sigma_t(1..k) arrive complete, and the mean prefix length
  RelMSE     MSE_ES / MSE_alt of the one-step prediction of c_y y(t): C_hat(t) of the run (ES) against the
             last value c_y y(t - 1); H5 compares ES with alpha* against every predictor of fleetdt.predictors
             in left-out scenarios (emu/analysis/hypotheses.py)
  fleet      fleet consistency: for each topic l, the fraction of the steps (with a positive demand of l)
             in which l arrives complete from every vessel of M(t)
  unused     share of the cap m(t) b(t) = max(0, U K_hat(t) - Q(t)) granted to vessels of M(t) with a zero demand
             table: sum_t z(t) b(t) / sum_t m(t) b(t)
  fill       fill ratio f(t) = max_{i in M(t)} a_i(t) / b(t) (a_i of Algorithm 1); a cell with rho >= 1.5 enters
             the IUT of H3 and H4 only when its mean fill ratio reaches 0.85 (FILL_MIN; the hypothesis driver
             applies the rule per cell, "valid" here is the run-level value)
  series     (series=True) the per-step values that the pooled statistics of the hypothesis driver need

The full-reception criterion of an edge is explicit (argument `criterion`; the paper uses "count"):
  count  n >= messages forwarded by the vessel regulator (fwd of the regulator log; n_l = ceil(r_l Ts)
         without the log); for completeness (prefix, fleet) n >= the messages that the sources produced
         (src, else fwd + held)
  bytes  r >= d (1 - tol) - 1

Statistics: integrated autocorrelation time tau_int = 1/2 + sum_{k=1}^{W} rho(k) with the automatic window of
Madras and Sokal (1988), the smallest W with W >= c tau_int(W), c = 5 (method "sokal", the default; "ips":
initial positive sequence of Geyer), one run or pooled over runs; effective sample size n_eff = n / (2 tau_int);
circular block bootstrap (two-sided CI and one-sided bounds, blocks inside each run, blocks of ceil(2 tau_int)
steps), also of the difference of two independent samples (bb_diff_bounds); Clopper-Pearson upper limit with
the raw count k of events in n_eff trials (for k = 0 it is 1 - 0.05^(1/n_eff) at 95 %); the H3a bound on the
failed fleet steps with K_y(t) >= U K_hat(t) >= Q(t) (and on groups of 8 steps; Q(t) = 0 as a stratum; catch-up
and lag steps as failures), tau_int of the slack series K_y(t) - U K_hat(t); logistic regression by Newton
iterations (H3c); Diebold-Mariano test (Newey-West variance); one-sided exact Wilcoxon signed-rank test with
Pratt's treatment of zeros (exact conditional distribution, ties included); Holm; the intersection-union test
(IUT); the fixed sequence H1 -> H2 -> H3 -> H4 -> H5 -> H6 (the first failure ends it); TOST on the signed
run-level differences; the Hodges-Lehmann estimate.

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
from .outer import admitted_bytes, backlog_of, load_dta

CRITERIA = ("bytes", "count")
PAPER_CRITERION = "count"  # reception criterion of DMR, PCR and H3a in the paper (a message arrives late or never)
FILL_MIN = 0.85
FAIR_VESSELS = (1, 2, 3)   # H4d: vessels with the same sources (vessel 4 has no video in the fairness cells)
HYPOTHESIS_ORDER = ("H1", "H2", "H3", "H4", "H5", "H6")   # fixed sequence of Section VI-E
TAU_METHOD = "sokal"       # Madras and Sokal (1988), automatic window with c = 5
TAU_C = 5.0

# --------------------------------------------------------------------------- autocorrelation


def autocorr(x: np.ndarray) -> np.ndarray:
    """Normalized autocorrelation rho_k, k = 0..n-1 (FFT, biased estimator)."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    x = x - x.mean()
    f = np.fft.rfft(x, n=2 * n)
    acf = np.fft.irfft(f * np.conj(f))[:n]
    return acf / acf[0] if acf[0] > 0 else np.r_[1.0, np.zeros(n - 1)]


def _tau_from_rho(rho: np.ndarray, method: str = TAU_METHOD, c: float = TAU_C) -> float:
    if method == "sokal":
        tau = 0.5
        for W in range(1, len(rho)):
            tau += rho[W]
            if W >= c * tau:
                return max(tau, 0.5)
        return max(tau, 0.5)
    if method != "ips":
        raise ValueError("tau_int method must be sokal or ips")
    s = 0.0
    for m in range(len(rho) // 2):
        g = rho[2 * m] + rho[2 * m + 1]
        if g <= 0:
            break
        s += g
    return max(s - 0.5, 0.5)


def tau_int(x: Sequence[float], method: str = TAU_METHOD, c: float = TAU_C) -> float:
    """Integrated autocorrelation time tau_int = 1/2 + sum_{k>=1} rho_k (tau_int = 1/2 for i.i.d. data).
    method "sokal" (default, the paper): sum up to the automatic window of Madras and Sokal, the smallest
    W with W >= c tau_int(W), c = 5; method "ips": initial positive sequence of Geyer (sums of pairs
    rho_2m + rho_2m+1 while positive)."""
    x = np.asarray(x, dtype=float)
    x = x[~np.isnan(x)]
    if len(x) < 4 or np.var(x) == 0:
        return 0.5
    return _tau_from_rho(autocorr(x), method, c)


def tau_int_runs(runs: Sequence[Sequence[float]], method: str = TAU_METHOD, c: float = TAU_C) -> float:
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


def _block_index(lengths: Sequence[int], block: int):
    """starts, lengths, run offsets and run lengths of the circular blocks of runs of the given lengths."""
    starts, lens, obase, olen = [], [], [], []
    base = 0
    for m in lengths:
        starts.append(np.arange(m))
        lens.append(np.full(m, min(block, m)))
        obase.append(np.full(m, base))
        olen.append(np.full(m, m))
        base += m
    return tuple(np.concatenate(v) if v else np.zeros(0, int) for v in (starts, lens, obase, olen))


def _resample_means(rs: List[np.ndarray], block: int, B: int, rng) -> np.ndarray:
    """B circular-block-bootstrap means of the steps of the runs rs (1-D arrays, blocks inside each run)."""
    allx = np.concatenate(rs)
    n = len(allx)
    starts, lens, obase, olen = _block_index([len(x) for x in rs], block)
    nb = int(math.ceil(n / float(block)))
    ar = np.arange(block)
    out = np.empty(B)
    for b in range(B):
        pick = rng.integers(0, len(starts), nb)
        idx = obase[pick, None] + (starts[pick, None] + ar[None, :]) % olen[pick, None]
        idx = idx[ar[None, :] < lens[pick, None]][:n]
        out[b] = float(np.mean(allx[idx]))
    return out


def bb_diff_bounds(runs_a: Sequence[Sequence[float]], runs_b: Sequence[Sequence[float]], conf: float = 0.95,
                   B: int = 2000, block_a: Optional[int] = None, block_b: Optional[int] = None,
                   seed: int = 0) -> Dict:
    """Block-bootstrap bounds of mean(a) - mean(b) for two independent samples of per-step values, each a
    list of runs (H3b: the miss indicators of the evaluation runs against those of the backlog recursion on
    the calibration DTA). Each sample is resampled by its own circular blocks of ceil(2 tau_int) steps
    (tau_int pooled over its runs, NaN steps removed), independently of the other. Returns {"value",
    "lower", "upper" (one-sided `conf` bounds), "ci", "block_a", "block_b", "n_a", "n_b"}."""
    nan = float("nan")
    ra = [np.asarray(x, float)[~np.isnan(np.asarray(x, float))] for x in runs_a]
    rb = [np.asarray(x, float)[~np.isnan(np.asarray(x, float))] for x in runs_b]
    ra, rb = [x for x in ra if len(x)], [x for x in rb if len(x)]
    if not ra or not rb:
        return {"value": nan, "lower": nan, "upper": nan, "ci": [nan, nan], "block_a": 0, "block_b": 0,
                "n_a": int(sum(len(x) for x in ra)), "n_b": int(sum(len(x) for x in rb))}
    ba = block_a or max(1, int(math.ceil(2.0 * tau_int_runs(ra))))
    bb = block_b or max(1, int(math.ceil(2.0 * tau_int_runs(rb))))
    rng = np.random.default_rng(seed)
    d = _resample_means(ra, ba, B, rng) - _resample_means(rb, bb, B, rng)
    a = (1.0 - conf) / 2.0
    return {"value": float(np.mean(np.concatenate(ra)) - np.mean(np.concatenate(rb))),
            "lower": float(np.quantile(d, 1.0 - conf)), "upper": float(np.quantile(d, conf)),
            "ci": [float(np.quantile(d, a)), float(np.quantile(d, 1.0 - a))], "block_a": int(ba), "block_b": int(bb),
            "n_a": int(sum(len(x) for x in ra)), "n_b": int(sum(len(x) for x in rb))}


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
    H3a: an upper limit below 1e-3 needs n_eff >= 2995 with 0 events, 4742 with 1, 6294 with 2; for k = 0
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


def fixed_sequence_verdicts(reject: Dict[str, Optional[bool]], order: Sequence[str] = HYPOTHESIS_ORDER) -> List[Dict]:
    """Fixed sequence on the verdicts of the hypotheses (the IUT of each hypothesis mixes p-values, bootstrap
    bounds and Clopper-Pearson limits, so its outcome is a rejection of the null or not): each hypothesis
    of `order` is tested only when every earlier one was rejected; None (no data) counts as not rejected.
    Returns one dict per hypothesis: {"id", "tested", "rejected"}."""
    out, gate = [], True
    for h in order:
        rej = bool(gate and reject.get(h) is True)
        out.append({"id": h, "tested": gate, "rejected": rej})
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
    """min_i x_i / max_i x_i over the finite values (H4d with x = BDR_i)."""
    x = np.asarray([v for v in x if v == v], dtype=float)
    return float(x.min() / x.max()) if len(x) and x.max() > 0 else float("nan")


def logistic_fit(x: Sequence[float], y: Sequence[float], max_iter: int = 50, tol: float = 1e-10,
                 ridge: float = 0.0) -> Dict:
    """Logistic regression P(y = 1) = 1 / (1 + exp(-(b0 + b1 x))) by Newton iterations (IRLS) on the
    log-likelihood, with an optional ridge penalty on b1 (0: maximum likelihood). Steps with NaN are
    dropped. With no event or no non-event, or a constant x, the fit is not identifiable and b1 is NaN.
    Returns {"b0", "b1", "se_b0", "se_b1", "z_b1", "p_b1" (two-sided Wald), "n", "events", "iter",
    "converged", "loglik"}."""
    from scipy.stats import norm
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = ~(np.isnan(x) | np.isnan(y))
    x, y = x[ok], y[ok]
    n, k = len(x), float(np.sum(y))
    nan = float("nan")
    out = {"n": int(n), "events": int(k), "b0": nan, "b1": nan, "se_b0": nan, "se_b1": nan, "z_b1": nan,
           "p_b1": nan, "iter": 0, "converged": False, "loglik": nan}
    if n < 2 or k == 0 or k == n or np.ptp(x) == 0:
        if n and 0 < k < n:
            out["b0"] = math.log(k / (n - k))
        return out
    X = np.c_[np.ones(n), x]
    b = np.array([math.log(k / (n - k)), 0.0])
    pen = np.diag([0.0, ridge])
    for it in range(1, max_iter + 1):
        eta = np.clip(X @ b, -35.0, 35.0)
        p = 1.0 / (1.0 + np.exp(-eta))
        W = p * (1.0 - p)
        g = X.T @ (y - p) - pen @ b
        H = (X * W[:, None]).T @ X + pen
        try:
            step = np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            break
        b = b + step
        out["iter"] = it
        if float(np.max(np.abs(step))) < tol:
            out["converged"] = True
            break
    eta = np.clip(X @ b, -35.0, 35.0)
    p = 1.0 / (1.0 + np.exp(-eta))
    H = (X * (p * (1.0 - p))[:, None]).T @ X + pen
    try:
        cov = np.linalg.inv(H)
        se = np.sqrt(np.maximum(np.diag(cov), 0.0))
    except np.linalg.LinAlgError:
        se = np.array([nan, nan])
    ll = float(np.sum(y * np.log(np.maximum(p, 1e-300)) + (1 - y) * np.log(np.maximum(1 - p, 1e-300))))
    z = float(b[1] / se[1]) if se[1] > 0 else nan
    out.update(b0=float(b[0]), b1=float(b[1]), se_b0=float(se[0]), se_b1=float(se[1]), z_b1=z,
               p_b1=float(2.0 * norm.sf(abs(z))) if z == z else nan, loglik=ll)
    return out


def rel_mse(y: Sequence[float], pred_es: Sequence[float], pred_alt: Sequence[float]) -> Dict:
    """RelMSE = MSE_ES / MSE_alt of two one-step predictions of y over the steps where y and both predictions
    are defined; H5 tests log RelMSE - log 1.05 < 0. Returns {"value", "log", "MSE_ES", "MSE_alt", "n"}."""
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
    source produced (src_n or src, else fwd + held of the regulator log, else n_l = ceil(r_l Ts)); with
    "bytes", r >= d (1 - tol) - 1."""
    if e is None or e.get("d", 0) <= 0:
        return True
    if criterion == "count":
        want = e.get("src_n") if e.get("src_n") is not None else e.get("src")
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


def produced(e: Dict, n_expected: Dict[int, int]) -> Optional[int]:
    """Messages of step t that the source of the edge produced: src_n (or src) of the regulator step log
    ("rx"), else fwd + held, else fwd, else n_l = ceil(f_l Ts) of the DTP (None without a rate)."""
    for k in ("src_n", "src"):
        if e.get(k) is not None:
            return int(e[k])
    if e.get("fwd") is not None and e.get("held") is not None:
        return int(e["fwd"]) + int(e["held"])
    if e.get("fwd") is not None:
        return int(e["fwd"])
    return n_expected.get(e["l"])


def _state_ok(e: Dict, n_expected: Dict[int, int]) -> bool:
    """Definition of Fidelity: the GS applications receive within step t, exactly once, every message of topic
    state of step t of vessel i. n counts the first copies within the step (the VE deduplicates by vessel,
    topic, step and sequence number), so a pair holds when n equals the messages that the source produced;
    the duplicates that the GS removed do not fail the pair (Corollary 2)."""
    want = produced(e, n_expected)
    if want is None:
        want = 1
    return int(e.get("n", 0)) == int(want)


def bdr_denominator(e: Dict, dtp: DTP) -> Tuple[float, bool]:
    """Denominator term of BDR_i for one edge: c_l(x, n) of the content bytes x and the messages n that the
    sources produced in step t (edge fields src_x and src_n, DTP.message_bytes), or d_il(t) when the edge
    lacks them. Returns (bytes, True when the source counts were used)."""
    if e.get("src_n") is not None and e.get("src_x") is not None:
        n = int(e["src_n"])
        return (float(dtp.message_bytes(int(e["l"]), int(e["src_x"]), n)) if n else 0.0), True
    return float(e.get("d", 0) or 0), False


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
    out = {"t": [], "ase": [], "cover": [], "phi": [], "delta": [], "delta_fail": [], "eta": [], "mu": [], "nu": [],
           "margin": [], "eta_i": {}, "eta_rows": [], "prefix": [], "pcr": [], "fleet": [], "unused": [], "fill": [],
           "order": [], "A": [], "y": [], "C_hat": [], "K_y": [], "K_hat": [], "Q": [], "U": [], "m": [],
           "state_i": [], "catchup": [], "lag": [], "w1": [], "w2": [], "bdr_src": None}
    prev_t, prev_y = None, None
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
        out["m"].append(len(Mlist))
        # emulator flags of the step (emu/analysis/paper_post.py): catch-up and lag above the run limit
        out["catchup"].append(float(bool(g["catchup"])) if g.get("catchup") is not None else np.nan)
        out["lag"].append(float(bool(g["lag"])) if g.get("lag") is not None else np.nan)
        out["w1"].append(float(g["w1"]) if g.get("w1") is not None else np.nan)
        out["w2"].append(float(g["w2"]) if g.get("w2") is not None else np.nan)
        # fidelity: C_hat(t) against c_y y(t); coverage and slack in bytes of phase 3
        if y is not None and C is not None and prev_y is not None and prev_t == t - 1:
            out["ase"].append([abs(cy * y - C), cy * abs(y - prev_y)])     # MASE rows: error, last-value error
        else:
            out["ase"].append([np.nan, np.nan])
        prev_t, prev_y = t, (float(y) if y is not None else None)
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
        st = {}
        if state is not None and state in N:
            for e in adm:
                if e["l"] == state:
                    pairs += 1
                    v = _state_ok(e, n_expected)
                    ok += int(v)
                    st[int(e["i"])] = bool(v)
        out["delta"].append((ok, pairs))
        out["delta_fail"].append(float(ok < pairs) if pairs else np.nan)
        out["state_i"].append(st)
        num = den = 0
        row = {}
        for e in inM:
            if e["l"] in N:
                dd, src = bdr_denominator(e, dtp)
                out["bdr_src"] = src if out["bdr_src"] is None else (out["bdr_src"] and src)
                num += e.get("r", 0)
                den += e.get("d", 0)
                vi = out["eta_i"].setdefault(e["i"], [0, 0])
                vi[0] += e.get("r", 0)
                vi[1] += dd
                ri = row.setdefault(e["i"], [0, 0])
                ri[0] += e.get("r", 0)
                ri[1] += dd
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
            if N:   # min(pi_i(t), |N(t)|) / |N(t)| (the topics of N(t) come first in sigma_t)
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


def _ratio_of_sums(z: np.ndarray) -> float:
    """Sum of the first column divided by the sum of the second (nan for a zero denominator)."""
    den = float(np.sum(z[:, 1]))
    return float(np.sum(z[:, 0])) / den if den > 0 else float("nan")


def _summary(x: np.ndarray, stat, ci: bool, B: int, conf: float, seed: int, event: bool = False,
             tau: Optional[float] = None, fail: Optional[np.ndarray] = None, zero_den: bool = False) -> Dict:
    """Value, bootstrap CI, tau_int and n_eff of a per-step series (rows for ratios). With `tau` (tau_int
    of the margin series, for the event metrics), n_eff = n / (2 tau) with the fleet step as the unit.
    With `event`, viol_upper is the Clopper-Pearson upper limit of the per-step violation probability
    (events: steps with a violation, given by `fail` or 1 - x) with the raw count in n_eff effective steps
    (viol_upper_scaled: the count scaled to n_eff)."""
    if x.ndim == 1:
        keep = ~np.isnan(x)
        x = x[keep]
        per_step = x
    elif zero_den:                       # ratio of sums whose rows may have a zero denominator (MASE)
        keep = ~np.isnan(x).any(axis=1)
        x = x[keep]
        per_step = x[:, 0]
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
        # C20: the raw count k in n_eff trials (the rule of H1 and H3a); "viol_upper_scaled" keeps k n_eff / n
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


def picp_ground_truth(s: Dict, dtp: DTP, C_gt: Dict[int, float], B_ctl: Optional[Dict[int, float]] = None,
                      U: Optional[float] = None, ci: bool = False, B: int = 2000, conf: float = 0.95,
                      seed: int = 0) -> Dict:
    """PICP_C(U): fraction of the steps of step_rows with K(t) - B_ctl(t) >= U K_hat(t), K(t) = C(t) (T3 - T_m)/Ts,
    C(t) of the ground truth (subset replays) and B_ctl(t) the control bytes outside the data bridges in phase 3
    (0 when not given), over the steps with C(t) and C_hat(t). U: the logged U of each step, or the given value
    for every step (U* in H2). "K_C" holds K(t) - B_ctl(t) per step of s (NaN without C(t)), and "diff" the
    difference PICP(U) - PICP_C(U) on the steps with both, with its one-sided block-bootstrap bounds when ci
    (blocks of ceil(2 tau_int) steps of the per-step difference; H2 tests the upper bound < 0.005)."""
    r = dtp.T3m / dtp.Ts
    hits, d, KC = [], [], []
    for t, Ch, Ul, Ky in zip(s["t"], s["C_hat"], s["U"], s["K_y"]):
        C = C_gt.get(t)
        if C is None:
            KC.append(np.nan)
            continue
        kc = float(C) * r - float((B_ctl or {}).get(t, 0.0))
        KC.append(kc)
        if Ch != Ch:
            continue
        u = float(Ul) if U is None else float(U)
        # K(t) - B_ctl(t) >= U K_hat(t), compared in units of C (divided by (T3 - T_m)/Ts)
        hc = float(float(C) - float((B_ctl or {}).get(t, 0.0)) / r >= u * Ch)
        hits.append(hc)
        if Ky == Ky:
            hy = float(Ky / r >= u * Ch)           # K_y(t) >= U K_hat(t)
            d.append(hy - hc)
    out = {"value": float(np.mean(hits)) if hits else float("nan"), "n": len(hits), "K_C": KC}
    dd = np.asarray(d, float)
    diff = {"value": float(dd.mean()) if len(dd) else float("nan"), "n": int(len(dd))}
    if ci and len(dd):
        bb = block_bootstrap_bounds([dd], np.mean, conf=conf, B=B, seed=seed)
        diff.update(lower=bb["lower"], upper=bb["upper"], block=bb["block"])
    out["diff"] = diff
    return out


def _nanlist(x) -> List[Optional[float]]:
    return [None if (v is None or v != v) else float(v) for v in np.asarray(x, float).tolist()]


def run_metrics(records: Sequence[Dict], dtp: DTP, *, criterion: str, tol: float = 1e-6, ci: bool = True,
                B: int = 2000, conf: float = 0.95, seed: int = 0, c_y: Optional[float] = None,
                kappa: Optional[float] = None, C_gt: Optional[Dict[int, float]] = None,
                B_ctl: Optional[Dict[int, float]] = None, fair_vessels: Optional[Sequence[int]] = FAIR_VESSELS,
                series: bool = False) -> Dict:
    """Metrics of one run (Section VI) with an explicit full-reception criterion ("bytes" or "count"; the paper
    uses "count"). The event metrics (PICP, DMR, PDR_state) take tau_int from the slack series
    s(t) = K_y(t) - U K_hat(t) and the fleet step as the unit (n_eff = steps / (2 tau_int)).
    c_y (default: the DTP; alias kappa, its old name) enters K_y and the MASE; C_gt (t -> C(t)) and B_ctl
    (t -> B_ctl(t)) give PICP_C; J and the ratio min_i BDR_i / max_i BDR_i take the vessels of fair_vessels
    (None: every vessel). series: also the per-step values of the hypothesis driver ("series"). The output
    holds the names of the paper and, for one release, the names of earlier versions (module docstring)."""
    from .outer import dmr_hat_replay
    _check_criterion(criterion)
    cy = _cy(c_y, kappa, dtp.c_y)
    s = step_rows(records, dtp, criterion, tol, cy)
    mean = lambda z: float(np.mean(z))
    res = {"steps": len(s["t"]), "t_first": s["t"][0] if s["t"] else None, "t_last": s["t"][-1] if s["t"] else None,
           "criterion": criterion, "c_y": cy, "kappa": cy, "T_m": dtp.T_m, "B_ctl_bar": dtp.B_ctl_bar,
           "T3m_Ts": dtp.T3m / dtp.Ts}
    mg = np.asarray(s["margin"], float)
    tau_m = tau_int(mg) if np.any(~np.isnan(mg)) else 0.5
    res["tau_margin"] = tau_m
    res["MASE"] = _summary(np.asarray(s["ase"], float).reshape(-1, 2), _ratio_of_sums, ci, B, conf, seed,
                           zero_den=True)
    res["PICP"] = res["gamma"] = _summary(np.asarray(s["cover"], float), mean, ci, B, conf, seed, event=True,
                                          tau=tau_m)
    if C_gt:
        res["PICP_C"] = picp_ground_truth(s, dtp, C_gt, B_ctl, ci=ci, B=B, conf=conf, seed=seed)
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
    # PDR_state per vessel (H4c: vessels 2 to 4)
    num_i, den_i = {}, {}
    for st in s["state_i"]:
        for i, v in st.items():
            den_i[i] = den_i.get(i, 0) + 1
            num_i[i] = num_i.get(i, 0) + int(v)
    res["PDR_state_i"] = {int(i): num_i[i] / den_i[i] for i in sorted(den_i)}
    # DMR-hat: backlog recursion with K_y(t) and Algorithm 1 replayed on these records with the logged alpha and U
    recs = sorted(records, key=lambda r: int(r["t"]))
    a_log = next((float(r["alpha"]) for r in recs if r.get("alpha") is not None), dtp.alpha)
    U_log = next((float(r["U"]) for r in recs if r.get("U") is not None), dtp.U)
    dh = dmr_hat_replay(recs, dtp, alpha=a_log, U=U_log, c_y=cy)
    res["DMR_hat"] = {"value": dh["value"], "n": dh["n"], "c_y": cy, "alpha": a_log, "U": U_log,
                      "source": "replay of Algorithm 1 on this DTA"}
    # RelMSE of the ES prediction C_hat(t) of the run against the last value c_y y(t - 1), both against c_y y(t)
    z = cy * np.asarray(s["y"], float)
    res["RelMSE"] = dict(rel_mse(z, np.asarray(s["C_hat"], float), np.r_[np.nan, z[:-1]]), alt="last value",
                         target="c_y y")
    eta_i = {int(i): (v[0] / v[1] if v[1] > 0 else float("nan")) for i, v in sorted(s["eta_i"].items())}
    res["BDR_i"] = res["PDR_i"] = res["eta_vessel"] = eta_i
    res["BDR_den"] = "source" if s["bdr_src"] else "demand"
    fair = {i: v for i, v in eta_i.items() if fair_vessels is None or i in fair_vessels}
    res["J"] = jain(list(fair.values()))
    res["J_vessels"] = sorted(fair)
    res["J_all"] = jain(list(eta_i.values()))
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
    cu = np.asarray(s["catchup"], float)
    lg = np.asarray(s["lag"], float)
    res["catchup_steps"] = int(np.nansum(cu)) if np.any(~np.isnan(cu)) else None
    res["lag_steps"] = int(np.nansum(lg)) if np.any(~np.isnan(lg)) else None
    if series:
        nu = np.asarray(s["nu"], float).reshape(-1, 2)
        res["series"] = {
            "t": [int(v) for v in s["t"]], "y": _nanlist(s["y"]), "C_hat": _nanlist(s["C_hat"]), "U": _nanlist(s["U"]),
            "K_y": _nanlist(s["K_y"]), "K_hat": _nanlist(s["K_hat"]), "Q": _nanlist(s["Q"]), "m": list(s["m"]),
            "miss": _nanlist(miss), "cover": _nanlist(s["cover"]), "fill": _nanlist(s["fill"]),
            "catchup": _nanlist(cu), "lag": _nanlist(lg), "w1": _nanlist(s["w1"]), "w2": _nanlist(s["w2"]),
            "u_num": _nanlist(nu[:, 0]), "u_den": _nanlist(nu[:, 1]), "miss_hat": _nanlist(dh["miss"]),
        }
        if C_gt:
            res["series"]["K_C"] = _nanlist(res["PICP_C"]["K_C"])
    if C_gt:
        res["PICP_C"].pop("K_C", None)
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


# --------------------------------------------------------------------------- H3a over pooled runs

STRATA = ("general", "Q0")


def condition_arrays(K_y, K_hat, U, Q, miss, catchup=None, lag=None, margin=None, buf_empty=None,
                     lag_fails: bool = True, catchup_fails: bool = True) -> Dict[str, np.ndarray]:
    """Per fleet step: slack s(t) = K_y(t) - U K_hat(t) ("margin", given or computed); the condition of
    Proposition 2 as measured, cond(t): K_y(t) >= U K_hat(t) >= Q(t) with m(t) >= 1 (miss defined);
    cond_Q0(t): the stratum Q(t) = 0 of cond(t); fail(t): a message of level 1 or 2 forwarded by a vessel
    of M(t) arrives late or never (miss), or the emulator flags the step as catch-up (catchup_fails) or
    with a lag above the run limit (lag_fails); NaN flags count as no flag. buf_empty (optional) adds the
    empty send buffers at the start of phase 3 to both conditions."""
    Ky, Kh, Uv, Qv, ms = (np.asarray(v, float) for v in (K_y, K_hat, U, Q, miss))
    mg = Ky - Uv * Kh if margin is None else np.asarray(margin, float)
    known = ~(np.isnan(mg) | np.isnan(Kh) | np.isnan(ms))
    ok = known & (np.nan_to_num(mg, nan=-1.0) >= 0) & (Uv * np.nan_to_num(Kh) >= np.nan_to_num(Qv, nan=np.inf))
    if buf_empty is not None:
        ok = ok & np.asarray(buf_empty, bool)
    flag = lambda v: np.zeros(len(ms), bool) if v is None else (np.nan_to_num(np.asarray(v, float), nan=0.0) > 0)
    fail = (np.nan_to_num(ms, nan=0.0) > 0)
    if catchup_fails:
        fail = fail | flag(catchup)
    if lag_fails:
        fail = fail | flag(lag)
    return {"margin": mg, "cond": ok, "cond_Q0": ok & (np.nan_to_num(Qv, nan=1.0) == 0), "fail": fail}


def condition_steps(records: Sequence[Dict], dtp: DTP, criterion: str, c_y: Optional[float] = None,
                    tol: float = 1e-6, lag_fails: bool = True, kappa: Optional[float] = None,
                    catchup_fails: bool = True, require_buf_empty: bool = False) -> Dict[str, np.ndarray]:
    """condition_arrays of one run from its G(t) records (sorted by t): the slack of the records (exact sign),
    Q(t) of the records (Q_i over M(t) plus Qbar_i outside), the miss of the criterion and the step flags
    "catchup" and "lag" that emu/analysis/paper_post.py writes from emu/steplag.csv. require_buf_empty also
    needs the record field "buf_empty" (true when absent) in the condition."""
    _check_criterion(criterion)
    s = step_rows(records, dtp, criterion, tol, _cy(c_y, kappa, dtp.c_y))
    recs = sorted(records, key=lambda r: int(r["t"]))
    phi = np.asarray(s["phi"], float)
    buf = np.asarray([bool(r.get("buf_empty", True)) for r in recs]) if require_buf_empty else None
    c = condition_arrays(s["K_y"], s["K_hat"], s["U"], s["Q"], np.where(np.isnan(phi), np.nan, 1.0 - phi),
                         s["catchup"], s["lag"], margin=s["margin"], buf_empty=buf, lag_fails=lag_fails,
                         catchup_fails=catchup_fails)
    c["buf_known"] = np.asarray(["buf_empty" in r for r in recs])
    c["Q_known"] = np.asarray([r.get("Q") is not None or bool(r.get("vertices")) for r in recs])
    return c


def condition_from_series(sr: Dict, lag_fails: bool = True, catchup_fails: bool = True) -> Dict[str, np.ndarray]:
    """condition_arrays of one run from the "series" of run_metrics (the hypothesis driver)."""
    f = lambda k: np.asarray([np.nan if v is None else v for v in sr.get(k) or [None] * len(sr["t"])], float)
    return condition_arrays(f("K_y"), f("K_hat"), f("U"), f("Q"), f("miss"), f("catchup"), f("lag"),
                            lag_fails=lag_fails, catchup_fails=catchup_fails)


def _windows(c: Dict[str, np.ndarray], w: int) -> Dict[str, np.ndarray]:
    """Non-overlapping groups of w steps from the start of the run: a group meets the condition when every
    step does; it fails when any step fails; its slack is the smallest slack of its steps."""
    n = (len(c["cond"]) // w) * w
    if n == 0:
        return {"margin": np.zeros(0), "cond": np.zeros(0, bool), "cond_Q0": np.zeros(0, bool),
                "fail": np.zeros(0, bool)}
    mg = c["margin"][:n].reshape(-1, w)
    wm = np.where(np.isnan(mg).any(axis=1), np.nan, np.min(np.nan_to_num(mg, nan=0.0), axis=1))
    return {"margin": wm, "cond": c["cond"][:n].reshape(-1, w).all(axis=1),
            "cond_Q0": c["cond_Q0"][:n].reshape(-1, w).all(axis=1), "fail": c["fail"][:n].reshape(-1, w).any(axis=1)}


def h3a_pool(conds: Sequence[Dict[str, np.ndarray]], window: int = 1, stratum: str = "general",
             conf: float = 0.95) -> Dict:
    """H3a pooled over runs from their condition_arrays: P(failure | condition) of the fleet steps (window 1)
    or of the groups of `window` steps, for the stratum "general" (K_y >= U K_hat >= Q) or "Q0" (Q(t) = 0).
    tau_int comes from the slack series (group minimum for groups), pooled over the runs; n_eff = condition
    units / (2 tau_int); upper: Clopper-Pearson with the raw events in n_eff trials (1 - 0.05^(1/n_eff) for
    0 events); upper_scaled: with k n_eff / n events."""
    if stratum not in STRATA:
        raise ValueError("stratum must be one of %s" % (STRATA,))
    key = "cond" if stratum == "general" else "cond_Q0"
    margins, n_cond, k = [], 0, 0
    for c in conds:
        if window > 1:
            c = _windows(c, window)
        margins.append(c["margin"])
        n_cond += int(c[key].sum())
        k += int((c[key] & c["fail"]).sum())
    tau = tau_int_runs(margins) if margins else 0.5
    ne = n_eff(n_cond, tau)
    return {"window": window, "stratum": stratum, "n_cond": n_cond, "failures": k, "tau_margin": tau, "n_eff": ne,
            "estimate": k / n_cond if n_cond else float("nan"),
            "upper": clopper_pearson_upper_eff(k, n_cond, ne, conf=conf, events="raw") if n_cond else 1.0,
            "upper_scaled": clopper_pearson_upper_eff(k, n_cond, ne, conf=conf, events="scaled") if n_cond else 1.0,
            "conf": conf, "runs": len(margins)}


def h3a_bound(runs: Sequence[Sequence[Dict]], dtp: DTP, criterion: str, c_y: Optional[float] = None, window: int = 1,
              conf: float = 0.95, tol: float = 1e-6, lag_fails: bool = True, kappa: Optional[float] = None,
              stratum: str = "general", catchup_fails: bool = True, require_buf_empty: bool = False) -> Dict:
    """H3a pooled over runs of G(t) records (h3a_pool of their condition_steps). c_y has the alias kappa (old
    name); default: the c_y of the DTP."""
    cy = _cy(c_y, kappa, dtp.c_y)
    conds = []
    buf_known = Q_known = True
    for recs in runs:
        if not recs:
            continue
        c = condition_steps(recs, dtp, criterion, cy, tol, lag_fails, catchup_fails=catchup_fails,
                            require_buf_empty=require_buf_empty)
        buf_known = buf_known and bool(c["buf_known"].all())
        Q_known = Q_known and bool(c["Q_known"].all())
        conds.append(c)
    out = h3a_pool(conds, window, stratum, conf)
    out.update(c_y=cy, kappa=cy, criterion=criterion, buffers_known=buf_known, backlog_known=Q_known)
    return out


h4a_bound = h3a_bound   # name of earlier versions (H4a was the guarantee before the renumbering)


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

METRICS = ("MASE", "PICP", "PDR_state", "DMR", "eta", "mu", "u")


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
    lines.append("BDR_i (%s): " % res["BDR_den"] + ", ".join("b%d %.5f" % (i, v) for i, v in res["BDR_i"].items()))
    lines.append(("Jain J (vessels %s) = %.5f" % (res["J_vessels"], res["J"])) if res["J"] == res["J"] else "Jain J = -")
    mm = res["eta_minmax"]
    lines.append("min/max BDR_i = %s%s" % (f(mm["value"]).strip(), (", lower %.5f" % mm["lower"]) if "lower" in mm else ""))
    pf = res["prefix"]
    lines.append("PCR %s (%d pairs); complete prefix: mean length %.3f of %d; by rank %s" % (
        f(pf["PCR"]).strip(), pf["pcr_pairs"], pf["mean_len"], len(pf["by_rank"]),
        " ".join("%.3f" % v for v in pf["by_rank"])))
    lines.append("DMR_hat %s (c_y %g); RelMSE (ES / last value) %s" % (
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
                    help="c_y of K_y(t) = c_y y (T3 - T_m)/Ts - B_ctl_bar and of the MASE (calibration minimum of C/y; "
                         "default: the DTP; --kappa: old name)")
    ap.add_argument("--tol", type=float, default=1e-6)
    ap.add_argument("--B", type=int, default=2000, help="bootstrap resamples")
    ap.add_argument("--conf", type=float, default=0.95)
    ap.add_argument("--no-ci", action="store_true")
    ap.add_argument("--h3a", "--h4a", dest="h3a", action="store_true",
                    help="also the pooled H3a bounds (steps and groups of 8 steps; --h4a: old name)")
    a = ap.parse_args(argv)
    dtp = DTP.load(a.dtp)
    out = {}
    runs = {}
    for path in a.dta:
        runs[path] = load_dta(path)
        out[path] = run_metrics(runs[path], dtp, criterion=a.criterion, tol=a.tol, ci=not a.no_ci, B=a.B, conf=a.conf,
                                c_y=a.c_y)
    if a.h3a:
        out["_h3a"] = {"steps": h3a_bound(list(runs.values()), dtp, a.criterion, a.c_y, 1, a.conf, a.tol),
                       "groups8": h3a_bound(list(runs.values()), dtp, a.criterion, a.c_y, 8, a.conf, a.tol)}
    if a.json:
        print(json.dumps(out, indent=1))
    else:
        print("\n\n".join(format_table(p, r) for p, r in out.items() if not p.startswith("_")))
        if a.h3a:
            for k, v in out["_h3a"].items():
                print("H3a %s: %d units with the condition, %d failures, tau_int %.2f, n_eff %.0f, upper %.2e" % (
                    k, v["n_cond"], v["failures"], v["tau_margin"], v["n_eff"], v["upper"]))


if __name__ == "__main__":
    main()
