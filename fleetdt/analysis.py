"""Metrics of Section VI of the Fleet-DT paper computed from a DTA (JSON lines of G(t)) and the DTP.

Per run:
  MAPE  mean of |y(t) - C_hat(t)| / y(t) over the steps with y(t) > 0 (Definition 15)
  gamma fraction of the steps with y(t) >= U C_hat(t)
  delta fraction of the pairs (t, i), i in M(t) and "state" in A(t) ∩ N(t), with exactly one state
        message received in step t (edge n == 1 and dup == 0)
  phi   fraction of the steps in which every message of level 1 or 2 of the vessels of M(t) was received
  eta   bytes of the N(t) topics received within the step / bytes of the N(t) topics demanded; also per
        vessel, with Jain's fairness index J over the vessels
  mu    late messages / (received + late)
  nu    received channel bytes / sum of y(t) T3 / Ts, over the steps with y(t)

Statistics: integrated autocorrelation time, effective sample size, circular block bootstrap,
Clopper-Pearson upper limit with n_eff, Diebold-Mariano test (Newey-West variance), Wilcoxon signed-rank
test with Holm correction and the Hodges-Lehmann estimate.

  python -m fleetdt.analysis dta.jsonl [more.jsonl ...] --dtp dtp.json [--json] [--criterion bytes|count]
"""
from __future__ import annotations

import argparse
import json
import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .as_model import DTP
from .outer import load_dta

# --------------------------------------------------------------------------- statistics


def autocorr(x: np.ndarray) -> np.ndarray:
    """Normalized autocorrelation rho_k, k = 0..n-1 (FFT, biased estimator)."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    x = x - x.mean()
    f = np.fft.rfft(x, n=2 * n)
    acf = np.fft.irfft(f * np.conj(f))[:n]
    return acf / acf[0] if acf[0] > 0 else np.r_[1.0, np.zeros(n - 1)]


def tau_int(x: Sequence[float], method: str = "ips", c: float = 5.0) -> float:
    """Integrated autocorrelation time tau_int = 1/2 + sum_{k>=1} rho_k (tau_int = 1/2 for i.i.d. data).
    method "ips": initial positive sequence of Geyer (sums of pairs rho_2m + rho_2m+1 while positive);
    method "sokal": smallest window W with W >= c tau_int(W)."""
    x = np.asarray(x, dtype=float)
    x = x[~np.isnan(x)]
    if len(x) < 4 or np.var(x) == 0:
        return 0.5
    rho = autocorr(x)
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


def n_eff(n: int, tau: float) -> float:
    """Effective sample size |S| / (2 tau_int)."""
    return n / (2.0 * tau)


def block_bootstrap_ci(x, stat: Callable[[np.ndarray], float] = np.mean, conf: float = 0.95, B: int = 2000,
                       block: Optional[int] = None, tau: Optional[float] = None, seed: int = 0) -> Tuple[float, float, int]:
    """Percentile CI of stat(x) by circular block bootstrap. x: per-step values (1-D) or rows (2-D, one
    row per step, e.g. numerator and denominator of a ratio). The block length defaults to
    ceil(2 tau_int) of the first column. Returns (low, high, block length)."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n == 0:
        return float("nan"), float("nan"), 0
    if block is None:
        col = x if x.ndim == 1 else x[:, 0]
        block = max(1, int(math.ceil(2.0 * (tau if tau is not None else tau_int(col)))))
    block = min(block, n)
    rng = np.random.default_rng(seed)
    nb = int(math.ceil(n / block))
    offs = np.arange(block)
    vals = np.empty(B)
    for b in range(B):
        starts = rng.integers(0, n, nb)
        idx = ((starts[:, None] + offs[None, :]) % n).ravel()[:n]
        vals[b] = stat(x[idx])
    vals = vals[~np.isnan(vals)]
    if len(vals) == 0:
        return float("nan"), float("nan"), block
    a = (1.0 - conf) / 2.0
    return float(np.quantile(vals, a)), float(np.quantile(vals, 1.0 - a)), block


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


def hodges_lehmann(a: Sequence[float], b: Optional[Sequence[float]] = None) -> float:
    """Hodges-Lehmann estimate of the paired shift: median of the Walsh averages of a - b."""
    d = np.asarray(a, dtype=float) - (0.0 if b is None else np.asarray(b, dtype=float))
    i, j = np.triu_indices(len(d))
    return float(np.median((d[i] + d[j]) / 2.0))


def wilcoxon_holm(pairs: Dict[str, Tuple[Sequence[float], Sequence[float]]]) -> Dict[str, Dict]:
    """Wilcoxon signed-rank test of each paired comparison of run-level values (name -> (a, b)), Holm
    correction over the family, and the Hodges-Lehmann estimate of a - b."""
    from scipy.stats import wilcoxon
    out = {}
    for name, (a, b) in pairs.items():
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        try:
            p = float(wilcoxon(a, b).pvalue)
        except ValueError:  # every difference is zero
            p = 1.0
        out[name] = {"n": len(a), "p": p, "hl": hodges_lehmann(a, b)}
    for name, q in zip(out, holm([v["p"] for v in out.values()])):
        out[name]["p_holm"] = q
    return out


def jain(x: Sequence[float]) -> float:
    """Jain's fairness index (sum x)^2 / (n sum x^2)."""
    x = np.asarray([v for v in x if v == v], dtype=float)
    s2 = float(np.sum(x * x))
    return float(np.sum(x) ** 2 / (len(x) * s2)) if len(x) and s2 > 0 else float("nan")


# --------------------------------------------------------------------------- per-step series


def _received_all(e: Dict, criterion: str, tol: float, n_expected: Dict[int, int]) -> bool:
    if criterion == "count":
        want = e.get("fwd")
        if want is None:
            want = n_expected.get(e["l"])
        return want is not None and e.get("n", 0) >= want
    return e.get("r", 0) >= e.get("d", 0) * (1.0 - tol) - 1.0


def step_rows(records: Sequence[Dict], dtp: DTP, criterion: str = "count", tol: float = 1e-6) -> Dict:
    """Per-step quantities of the metrics, sorted by t. Ratio metrics keep (numerator, denominator) rows."""
    recs = sorted(records, key=lambda r: int(r["t"]))
    state = dtp.index("state") if "state" in dtp.topics else None
    n_expected = {k + 1: int(math.ceil(s.rate * dtp.Ts)) for k, s in enumerate(dtp.specs) if s.rate}
    ratio = dtp.T3 / dtp.Ts
    out = {"t": [], "ape": [], "cover": [], "phi": [], "delta": [], "eta": [], "mu": [], "nu": [], "eta_i": {}}
    for g in recs:
        t, y, C = int(g["t"]), g.get("y"), g.get("C_hat")
        U = g.get("U") if g.get("U") is not None else dtp.U
        M = set(g.get("vessels", []))
        N = set(g.get("needed", []))
        edges = g.get("edges", [])
        out["t"].append(t)
        out["ape"].append(abs(y - C) / y if y is not None and C is not None and y > 0 else np.nan)
        out["cover"].append(float(y >= U * C) if y is not None and C is not None else np.nan)
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
        num = den = 0
        for e in inM:
            if e["l"] in N:
                num += e.get("r", 0)
                den += e.get("d", 0)
                vi = out["eta_i"].setdefault(e["i"], [0, 0])
                vi[0] += e.get("r", 0)
                vi[1] += e.get("d", 0)
        out["eta"].append((num, den))
        late = sum(e.get("late", 0) for e in edges)
        out["mu"].append((late, sum(e.get("n", 0) for e in edges) + late))
        out["nu"].append((sum(e.get("r", 0) for e in edges), y * ratio) if y is not None else (np.nan, np.nan))
    return out


def _ratio(rows: np.ndarray) -> float:
    s = np.nansum(rows, axis=0)
    return float(s[0] / s[1]) if s[1] > 0 else float("nan")


def _summary(x: np.ndarray, stat, ci: bool, B: int, conf: float, seed: int, event: bool = False) -> Dict:
    """Value, bootstrap CI, tau_int and n_eff of a per-step series (rows for ratios)."""
    if x.ndim == 1:
        x = x[~np.isnan(x)]
        per_step = x
    else:
        x = x[~np.isnan(x).any(axis=1) & (x[:, 1] > 0)]
        per_step = x[:, 0] / x[:, 1] if len(x) else x[:, 0]
    n = len(x)
    out = {"value": stat(x) if n else float("nan"), "n": n}
    tau = tau_int(per_step) if n else float("nan")
    out["tau_int"], out["n_eff"] = tau, n_eff(n, tau) if n else 0.0
    if ci and n:
        lo, hi, blk = block_bootstrap_ci(x, stat, conf=conf, B=B, tau=tau, seed=seed)
        out["ci"], out["block"] = [lo, hi], blk
    if event and n:  # upper limit of the probability of the complementary event (violation)
        k = float(np.sum(1.0 - x)) if x.ndim == 1 else float(np.sum(x[:, 1] - x[:, 0]))
        tot = float(n) if x.ndim == 1 else float(np.sum(x[:, 1]))
        ne = out["n_eff"] * (tot / n)
        out["viol_upper"] = clopper_pearson_upper(k, tot, conf=conf, n_eff=ne)
    return out


def analyze(records: Sequence[Dict], dtp: DTP, criterion: str = "count", tol: float = 1e-6, ci: bool = True,
            B: int = 2000, conf: float = 0.95, seed: int = 0) -> Dict:
    """Metrics of one run (Section VI)."""
    s = step_rows(records, dtp, criterion, tol)
    mean = lambda z: float(np.mean(z))
    res = {"steps": len(s["t"]), "t_first": s["t"][0] if s["t"] else None, "t_last": s["t"][-1] if s["t"] else None,
           "criterion": criterion}
    res["MAPE"] = _summary(np.asarray(s["ape"], float), mean, ci, B, conf, seed)
    res["gamma"] = _summary(np.asarray(s["cover"], float), mean, ci, B, conf, seed, event=True)
    res["phi"] = _summary(np.asarray(s["phi"], float), mean, ci, B, conf, seed, event=True)
    for k in ("delta", "eta", "mu", "nu"):
        rows = np.asarray(s[k], float).reshape(-1, 2)
        res[k] = _summary(rows, _ratio, ci, B, conf, seed, event=(k == "delta"))
    eta_i = {int(i): (v[0] / v[1] if v[1] > 0 else float("nan")) for i, v in sorted(s["eta_i"].items())}
    res["eta_vessel"] = eta_i
    res["J"] = jain(list(eta_i.values()))
    return res


METRICS = ("MAPE", "gamma", "delta", "phi", "eta", "mu", "nu")


def format_table(name: str, res: Dict) -> str:
    lines = ["%s: steps %s..%s (%d), criterion %s" % (name, res["t_first"], res["t_last"], res["steps"], res["criterion"]),
             "%-6s %10s %10s %10s %8s %9s %11s" % ("metric", "value", "ci_low", "ci_high", "tau_int", "n_eff", "viol_upper")]
    f = lambda v: "%10.5f" % v if v is not None and v == v else "%10s" % "-"
    for k in METRICS:
        m = res[k]
        lo, hi = m.get("ci", [None, None])
        lines.append("%-6s %s %s %s %8.2f %9.1f %11s" % (k, f(m["value"]), f(lo), f(hi), m["tau_int"] if m["n"] else float("nan"),
                                                          m["n_eff"], ("%.5f" % m["viol_upper"]) if "viol_upper" in m else "-"))
    lines.append("eta per vessel: " + ", ".join("b%d %.5f" % (i, v) for i, v in res["eta_vessel"].items()))
    lines.append("Jain J = %.5f" % res["J"] if res["J"] == res["J"] else "Jain J = -")
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.analysis", description="Section VI metrics of a Fleet-DT DTA")
    ap.add_argument("dta", nargs="+", help="DTA files (JSON lines of G(t)), one per run")
    ap.add_argument("--dtp", required=True, help="DTP file (dtp.json)")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--criterion", choices=("bytes", "count"), default="count",
                    help="full reception of an admitted edge: n >= messages forwarded by the vessel regulator "
                         "(count; n_l = ceil(r_l Ts) without the regulator log) or r >= d (1 - tol) - 1 (bytes)")
    ap.add_argument("--tol", type=float, default=1e-6)
    ap.add_argument("--B", type=int, default=2000, help="bootstrap resamples")
    ap.add_argument("--conf", type=float, default=0.95)
    ap.add_argument("--no-ci", action="store_true")
    a = ap.parse_args(argv)
    dtp = DTP.load(a.dtp)
    out = {}
    for path in a.dta:
        out[path] = analyze(load_dta(path), dtp, criterion=a.criterion, tol=a.tol, ci=not a.no_ci, B=a.B, conf=a.conf)
    if a.json:
        print(json.dumps(out, indent=1))
    else:
        print("\n\n".join(format_table(p, r) for p, r in out.items()))


if __name__ == "__main__":
    main()
