"""Statistics of Section VI-E: paired Wilcoxon signed-rank test, Holm correction, Hodges-Lehmann estimate
of the paired difference with its exact 95 % interval (Hollander, Wolfe and Chicken, 2013)."""
import math
from functools import lru_cache

import numpy as np
from scipy import stats


@lru_cache(maxsize=None)
def signed_rank_cdf(n):
    """P(T+ <= c) for c = 0 .. n(n+1)/2 under H0 (no ties), by counting subsets"""
    N = n * (n + 1) // 2
    cnt = np.zeros(N + 1)
    cnt[0] = 1.0
    for k in range(1, n + 1):
        nxt = cnt.copy()
        nxt[k:] += cnt[:-k] if k <= N else 0
        cnt = nxt
    return np.cumsum(cnt) / 2.0 ** n


def wilcoxon(x, y):
    """two-sided paired Wilcoxon signed-rank test of x - y; exact without zeros and ties, else normal
    approximation (scipy). Returns (p, n_nonzero)"""
    d = np.asarray(x, float) - np.asarray(y, float)
    d = d[np.isfinite(d)]
    nz = d[d != 0]
    if len(nz) == 0:
        return 1.0, 0
    ties = len(np.unique(np.abs(nz))) < len(nz)
    method = "exact" if (len(nz) == len(d) and not ties and len(nz) <= 50) else "approx"
    r = stats.wilcoxon(d, zero_method="wilcox", alternative="two-sided", method=method)
    return float(r.pvalue), int(len(nz))


def holm(p):
    """Holm-adjusted p-values (same order as p)"""
    p = np.asarray(p, float)
    m = len(p)
    o = np.argsort(p)
    adj = np.empty(m)
    run = 0.0
    for k, i in enumerate(o):
        run = max(run, min(1.0, (m - k) * p[i]))
        adj[i] = run
    return adj


def hodges_lehmann(x, y, conf=0.95):
    """Hodges-Lehmann estimate of the paired difference x - y (median of the Walsh averages) and the
    exact distribution-free confidence interval from the order statistics of the Walsh averages"""
    d = np.asarray(x, float) - np.asarray(y, float)
    d = d[np.isfinite(d)]
    n = len(d)
    if n == 0:
        return None, (None, None)
    w = np.sort([(d[i] + d[j]) / 2.0 for i in range(n) for j in range(i, n)])
    est = float(np.median(w))
    if n < 2:
        return est, (None, None)
    cdf = signed_rank_cdf(n)
    a2 = (1 - conf) / 2
    c = int(np.max(np.nonzero(cdf <= a2)[0])) if np.any(cdf <= a2) else -1   # largest c with P(T <= c) <= a/2
    N = len(w)
    if c < 0:
        return est, (float(w[0]), float(w[-1]))
    return est, (float(w[c]), float(w[N - c - 1]))         # W_(c+1) and W_(N-c), 1-indexed
