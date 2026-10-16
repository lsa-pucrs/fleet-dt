"""Tests of the statistics and endpoints of fleetdt.analysis and of the backlog what-if of fleetdt.outer
(design v2 of the software-only evaluation, items I4 and I5)."""
import itertools
import json
import math
import subprocess
import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

from fleetdt import analysis as A
from fleetdt import outer as O
from fleetdt.as_model import DTP

ROOT = Path(__file__).resolve().parents[1]


# --------------------------------------------------------------------------- helpers


def ar1(n, phi, rng, scale=1.0):
    x = np.zeros(n)
    e = rng.normal(0, scale, n)
    for k in range(1, n):
        x[k] = phi * x[k - 1] + e[k]
    return x


def brute_force_p(d, alternative="greater", zero_method="pratt"):
    """Exact p of the signed-rank test by enumeration of the 2^n' sign vectors (reference)."""
    from scipy.stats import rankdata
    d = np.asarray(d, float)
    if zero_method == "pratt":
        r = rankdata(np.abs(d))
        r, dn = r[d != 0], d[d != 0]
    else:
        dn = d[d != 0]
        r = rankdata(np.abs(dn))
    t = r[dn > 0].sum()
    ts = [sum(rr for rr, s in zip(r, signs) if s) for signs in itertools.product([0, 1], repeat=len(r))]
    ts = np.asarray(ts)
    ge, le = np.mean(ts >= t - 1e-9), np.mean(ts <= t + 1e-9)
    return {"greater": ge, "less": le, "two-sided": min(1.0, 2 * min(ge, le))}[alternative]


# --------------------------------------------------------------------------- tau_int and n_eff


def test_tau_int_runs_pools_runs():
    rng = np.random.default_rng(1)
    iid = [rng.normal(size=3000) for _ in range(4)]
    assert abs(A.tau_int_runs(iid) - 0.5) < 0.1
    runs = [ar1(4000, 0.9, rng) for _ in range(5)]
    tau = A.tau_int_runs(runs)
    assert 7.5 < tau < 11.5                                 # AR(1): (1 + 0.9) / (2 (1 - 0.9)) = 9.5
    one = ar1(20000, 0.9, rng)
    assert A.tau_int_runs([one]) == pytest.approx(A.tau_int(one), rel=1e-9)
    # per-run means are removed: a level shift between runs does not inflate tau_int
    shifted = [r + 50.0 * k for k, r in enumerate(runs)]
    assert A.tau_int_runs(shifted) == pytest.approx(tau, rel=1e-9)
    assert A.n_eff(1000, 5.0) == 100.0


def _records(y, C, U=0.8, M=(1, 2), fail_at=(), lag_at=(), buf_empty=None, b=None, edges=None):
    """G(t) dicts with one admitted topic per vessel; a failure leaves the edge of vessel 1 short."""
    recs = []
    for t, (yy, cc) in enumerate(zip(y, C)):
        es = []
        for i in M:
            ok = t not in fail_at or i != M[0]
            es.append({"i": i, "l": 1, "d": 100, "p": 2, "r": 100 if ok else 40, "n": 1 if ok else 0, "dup": 0,
                       "late": 0, "fwd": 1})
        r = {"t": t, "y": float(yy), "C_hat": float(cc), "U": U, "vessels": list(M), "needed": [1], "order": [1],
             "edges": es if edges is None else edges(t), "b": b, "K_hat": None}
        if t in lag_at:
            r["lag"] = True
        if buf_empty is not None:
            r["buf_empty"] = bool(buf_empty[t])
        recs.append(r)
    return recs


def tiny_dtp(L=1):
    return DTP(topics=["state"] + ["x%d" % k for k in range(2, L + 1)], Ts=0.125, T1=0.02, T2=0.01, U=0.8)


def test_event_metrics_take_n_eff_from_the_margin_series():
    rng = np.random.default_rng(2)
    n = 6000
    C = np.full(n, 300_000.0)
    y = 330_000.0 + ar1(n, 0.95, rng, 2000.0)               # margin always positive, strongly correlated
    recs = _records(y, C)
    res = A.run_metrics(recs, tiny_dtp(), criterion="bytes", ci=False)
    tau_m = A.tau_int(y - 0.8 * C)
    assert res["tau_margin"] == pytest.approx(tau_m)
    assert res["tau_margin"] > 5
    for k in ("gamma", "phi"):
        assert res[k]["value"] == 1.0
        assert res[k]["tau_source"] == "margin"
        # fleet step as the unit: n_eff = steps / (2 tau_margin); the indicator alone (constant) gives n
        assert res[k]["n_eff"] == pytest.approx(n / (2 * tau_m))
        assert res[k]["n_eff"] < n / 10
    # no factor of messages per step: the delta pairs (2 vessels per step) do not double n_eff
    assert res["delta"]["n_eff"] == pytest.approx(n / (2 * tau_m))


def test_margin_uses_kappa():
    recs = _records([100.0, 200.0], [100.0, 100.0], U=0.5)
    s = A.margin_series(recs, tiny_dtp(), kappa=0.9)
    assert s.tolist() == pytest.approx([90 - 50, 180 - 50])


# --------------------------------------------------------------------------- exact signed-rank test


@pytest.mark.parametrize("d,p", [
    ([1, 2, 3, 4, 5], 1 / 32),                  # all positive, n = 5: 2^-5
    ([1, 2, -3, 4, 5], 5 / 32),                 # T+ = 12: complements with sum <= 3
    ([1, 2, -2, 4, 5], 4 / 32),                 # tie: ranks 1, 2.5, 2.5, 4, 5; T+ = 12.5
    ([0, 0, 1, 2, -3], 4 / 8),                  # Pratt: ranks 3, 4, 5 after the zeros; T+ = 7
    ([0, 1, 2, 3, 4, 5], 1 / 32),
])
def test_signed_rank_known_small_sample_p_values(d, p):
    r = A.signed_rank_exact(d, alternative="greater")
    assert r["p"] == pytest.approx(p, abs=1e-15)


def test_signed_rank_n20_all_positive_is_two_to_the_minus_20():
    r = A.signed_rank_exact(np.arange(1, 21) * 0.01, alternative="greater")
    assert r["p"] == 2.0 ** -20 and r["T_plus"] == 210


def test_pratt_differs_from_wilcox_with_zeros():
    d = [0, 0, 1, 2, -3]
    assert A.signed_rank_exact(d, zero_method="pratt")["p"] == pytest.approx(0.5)
    assert A.signed_rank_exact(d, zero_method="wilcox")["p"] == pytest.approx(5 / 8)


def test_signed_rank_matches_enumeration_with_ties_and_zeros():
    rng = np.random.default_rng(5)
    for _ in range(30):
        n = int(rng.integers(3, 11))
        d = rng.integers(-3, 5, n).astype(float)           # many ties and zeros
        for alt in ("greater", "less", "two-sided"):
            for zm in ("pratt", "wilcox"):
                got = A.signed_rank_exact(d, alternative=alt, zero_method=zm)["p"]
                assert got == pytest.approx(brute_force_p(d, alt, zm), abs=1e-12), (d, alt, zm)


def test_signed_rank_matches_scipy_without_ties():
    from scipy.stats import wilcoxon
    rng = np.random.default_rng(8)
    for _ in range(20):
        d = rng.normal(0.3, 1.0, 15)
        ref = wilcoxon(d, alternative="greater", method="exact").pvalue
        assert A.signed_rank_exact(d)["p"] == pytest.approx(ref, rel=1e-10)


def test_signed_rank_margin_and_pairs():
    a = np.array([0.30, 0.32, 0.28, 0.35, 0.31, 0.29, 0.33, 0.30])
    b = a - 0.10 + np.array([0.01, -0.01, 0.02, 0.0, 0.01, -0.02, 0.0, 0.01])
    assert A.signed_rank_exact(a, b, margin=0.05)["p"] == 2.0 ** -8
    assert A.signed_rank_exact(a, b, margin=0.2)["p"] == 1.0
    r = A.wilcoxon_holm({"B1": (a, b), "B2": (a, a - 0.06)}, margin=0.05)
    assert r["B1"]["p"] == 2.0 ** -8 and r["_iut"]["p"] == max(r["B1"]["p"], r["B2"]["p"])


# --------------------------------------------------------------------------- multiplicity


def test_holm_and_iut_and_fixed_sequence():
    assert A.holm([0.01, 0.04, 0.03]) == pytest.approx([0.03, 0.06, 0.06])
    assert A.holm_reject([0.01, 0.04, 0.03], 0.05) == [True, False, False]
    r = A.iut({"a": 0.01, "b": 0.049, "c": 0.002})
    assert r["reject"] and r["p"] == 0.049 and r["worst"] == "b"
    assert not A.iut([0.01, 0.06])["reject"]
    assert not A.iut([0.01, float("nan")])["reject"]
    seq = A.fixed_sequence({"H1": 0.01, "H2": 0.02, "H4": 0.20, "H5": 0.001, "H3": 0.0, "H6": 0.0})
    assert [s["id"] for s in seq] == ["H1", "H2", "H4", "H5", "H3", "H6"]
    assert [(s["tested"], s["rejected"]) for s in seq] == [(True, True), (True, True), (True, False),
                                                           (False, False), (False, False), (False, False)]
    seq = A.fixed_sequence({"H1": 0.01, "H2": 0.02, "H4": 0.03, "H5": 0.04, "H3": 0.05, "H6": 0.01})
    assert all(s["rejected"] for s in seq)
    assert [s["tested"] for s in A.fixed_sequence({"H1": 0.01})] == [True, True, False, False, False, False]


def test_tost():
    from scipy.stats import t as tdist
    rng = np.random.default_rng(3)
    d = rng.normal(0.001, 0.002, 20)
    r = A.tost(d, 0.005)
    m, se = d.mean(), d.std(ddof=1) / math.sqrt(20)
    assert r["p_low"] == pytest.approx(tdist.sf((m + 0.005) / se, 19))
    assert r["p_high"] == pytest.approx(tdist.cdf((m - 0.005) / se, 19))
    assert r["equivalent"] and r["ci"][0] > -0.005 and r["ci"][1] < 0.005
    assert r["equivalent"] == (-0.005 < r["ci"][0] and r["ci"][1] < 0.005)
    assert not A.tost(d + 0.006, 0.005)["equivalent"]
    assert not A.tost([0.001], 0.005)["equivalent"]
    assert A.tost(d, (-0.001, 0.01))["margin"] == [-0.001, 0.01]


# --------------------------------------------------------------------------- binomial and bootstrap bounds


def test_clopper_pearson_with_n_eff():
    up = A.clopper_pearson_upper_eff
    assert up(0, 50_000, 2995) < 1e-3 <= up(0, 50_000, 2994)        # numbers of the pre-registration
    assert up(1, 50_000, 4742) < 1e-3 <= up(1, 50_000, 4741)
    assert up(2, 50_000, 6294) < 1e-3 <= up(2, 50_000, 6293)
    assert up(0, 1000, 1000) == pytest.approx(1 - 0.05 ** (1 / 1000), rel=1e-9)
    assert up(10, 1000, 100, events="scaled") == pytest.approx(A.clopper_pearson_upper(10, 1000, n_eff=100))
    assert up(10, 1000, 100, events="raw") > up(10, 1000, 100, events="scaled")
    assert up(5, 0, 10) == 1.0


def test_block_bootstrap_bounds():
    rng = np.random.default_rng(4)
    x = 0.5 + ar1(3000, 0.8, rng, 0.1)
    r = A.block_bootstrap_bounds(x, B=400)
    assert r["ci"][0] <= r["lower"] <= r["value"] <= r["upper"] <= r["ci"][1]
    assert r["block"] >= 2
    lo, hi, blk = A.block_bootstrap_ci(x, B=400)
    assert (lo, hi, blk) == (r["ci"][0], r["ci"][1], r["block"])
    # blocks stay inside their run: two constant runs and block = run length give means 0, 1/2 or 1
    runs = [np.zeros(100), np.ones(100)]
    seen = set()
    stat = lambda z: (seen.add(round(float(np.mean(z)), 6)), float(np.mean(z)))[1]
    A.block_bootstrap_bounds(runs, stat, B=200, block=100)
    assert seen <= {0.0, 0.5, 1.0} and len(seen) == 3
    # 2-D rows (ratio of sums)
    rows = np.c_[rng.integers(0, 10, 500), np.full(500, 10)]
    r2 = A.block_bootstrap_bounds([rows], lambda z: z[:, 0].sum() / z[:, 1].sum(), B=200, block=5)
    assert r2["lower"] < r2["value"] < r2["upper"]


# --------------------------------------------------------------------------- criterion is explicit


def test_criterion_is_required():
    recs = _records([300_000.0] * 10, [300_000.0] * 10)
    with pytest.raises(TypeError):
        A.run_metrics(recs, tiny_dtp())
    with pytest.raises(ValueError):
        A.run_metrics(recs, tiny_dtp(), criterion="messages")
    with pytest.warns(FutureWarning):
        res = A.analyze(recs, tiny_dtp(), ci=False)
    assert res["criterion"] == "count" and res["criterion_explicit"] is False
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        assert A.analyze(recs, tiny_dtp(), criterion="bytes", ci=False)["criterion_explicit"] is True
    out = subprocess.run([sys.executable, "-m", "fleetdt.analysis", "x.jsonl", "--dtp", str(ROOT / "dtp.json")],
                         cwd=ROOT, capture_output=True, text=True)
    assert out.returncode == 2 and "--criterion" in out.stderr


# --------------------------------------------------------------------------- new endpoints


def _endpoint_records():
    """Two steps, vessels 1..3, sigma = (1, 2, 3); vessel 3 has a zero demand table in step 0."""
    def e(i, l, d, p, r, n, fwd=None, held=None):
        return {"i": i, "l": l, "d": d, "p": p, "r": r, "n": n, "dup": 0, "late": 0, "fwd": fwd, "held": held}
    s0 = [e(1, 1, 100, 2, 100, 1, 1, 0), e(1, 2, 300, 1, 300, 3, 3, 0), e(1, 3, 500, 0, 0, 0, 0, 5),
          e(2, 1, 100, 2, 100, 1, 1, 0), e(2, 2, 200, 1, 100, 1, 2, 0), e(2, 3, 400, 0, 0, 0, 0, 4),
          e(3, 1, 0, 2, 0, 0, 0, 0), e(3, 2, 0, 1, 0, 0, 0, 0), e(3, 3, 0, 0, 0, 0, 0, 0)]
    s1 = [e(1, 1, 100, 2, 100, 1, 1, 0), e(1, 2, 300, 1, 300, 3, 3, 0), e(1, 3, 500, 1, 500, 5, 5, 0),
          e(2, 1, 100, 2, 100, 1, 1, 0), e(2, 2, 200, 1, 200, 2, 2, 0), e(2, 3, 400, 1, 400, 4, 4, 0)]
    g0 = {"t": 0, "y": 1000.0, "C_hat": 1000.0, "U": 0.8, "vessels": [1, 2, 3], "needed": [1, 2], "order": [1, 2, 3],
          "b": 500.0, "K_hat": 1500.0, "edges": s0}
    g1 = {"t": 1, "y": 1000.0, "C_hat": 1000.0, "U": 0.8, "vessels": [1, 2], "needed": [1, 2], "order": [1, 2, 3],
          "b": 1000.0, "K_hat": 2000.0, "edges": s1}
    return [g0, g1]


@pytest.mark.parametrize("criterion", ["count", "bytes"])
def test_prefix_fleet_unused_fill_and_fairness(criterion):
    dtp = DTP(topics=["state", "imu", "video"], Ts=0.125, T1=0.02, T2=0.01, U=0.8)
    res = A.run_metrics(_endpoint_records(), dtp, criterion=criterion, ci=False)
    # pairs (t, i): (0,1) prefix 2 (topic 3 held), (0,2) prefix 1 (topic 2 short), (0,3) prefix 3 (zero
    # demand is complete), (1,1) 3, (1,2) 3
    pf = res["prefix"]
    assert pf["pairs"] == 5
    assert pf["by_rank"] == pytest.approx([1.0, 4 / 5, 3 / 5])
    assert pf["mean_len"] == pytest.approx((2 + 1 + 3 + 3 + 3) / 5)
    # fleet consistency: state complete from every vessel in both steps; imu fails in step 0 (vessel 2);
    # video fails in step 0 (held)
    assert res["fleet_consistency"] == {"state": 1.0, "imu": 0.5, "video": 0.5}
    # unused share: vessel 3 (zero demand) in step 0 holds b = 500 of K_hat 1500 + 2000
    assert res["unused"]["value"] == pytest.approx(500 / 3500)
    # fill ratio: step 0: a = (400, 300, 0), b = 500 -> 0.8; step 1: a = (900, 700), b = 1000 -> 0.9
    assert res["fill"]["mean"] == pytest.approx(0.85) and res["fill"]["min"] == pytest.approx(0.8)
    assert res["fill"]["valid"] is True
    # eta over the needed topics: vessel 1 (400+400)/(400+400) = 1; vessel 2 (200+300)/(300+300)
    assert res["eta_vessel"][1] == pytest.approx(1.0)
    assert res["eta_vessel"][2] == pytest.approx(500 / 600)
    assert res["eta_minmax"]["value"] == pytest.approx(500 / 600)
    assert A.min_max_ratio([0.5, 1.0, float("nan")]) == 0.5


def test_fill_validity_rule():
    recs = _endpoint_records()
    recs[1]["b"] = 2000.0                                   # f(1) = 900 / 2000
    dtp = DTP(topics=["state", "imu", "video"], Ts=0.125, T1=0.02, T2=0.01, U=0.8)
    res = A.run_metrics(recs, dtp, criterion="bytes", ci=False)
    assert res["fill"]["mean"] == pytest.approx((0.8 + 0.45) / 2) and res["fill"]["valid"] is False


# --------------------------------------------------------------------------- H4a pooled bound


def test_h4a_steps_and_windows():
    rng = np.random.default_rng(6)
    runs = []
    for k in range(3):
        n = 800
        C = np.full(n, 300_000.0)
        y = 260_000.0 + ar1(n, 0.9, rng, 3000.0)            # margin y - 0.8 C mostly positive
        y[100:110] = 200_000.0                              # condition fails here
        runs.append(_records(y, C, fail_at={105, 300 + k}, lag_at={500} if k == 0 else set()))
    dtp = tiny_dtp()
    r1 = A.h4a_bound(runs, dtp, "count", kappa=1.0, window=1)
    mg = [A.margin_series(rr, dtp, 1.0) for rr in runs]
    cond = sum(int(np.sum(m >= 0)) for m in mg)
    assert r1["n_cond"] == cond
    assert r1["failures"] == 3 + 1                          # steps 300 + k (condition holds) and the lag step
    assert r1["tau_margin"] == pytest.approx(A.tau_int_runs(mg))
    assert r1["n_eff"] == pytest.approx(cond / (2 * r1["tau_margin"]))
    assert r1["upper"] == pytest.approx(A.clopper_pearson_upper_eff(4, cond, r1["n_eff"], events="raw"))
    assert r1["buffers_known"] is False
    r8 = A.h4a_bound(runs, dtp, "count", kappa=1.0, window=8)
    assert r8["n_cond"] == sum(int(np.all((m[:800].reshape(-1, 8)) >= 0, axis=1).sum()) for m in mg)
    assert r8["failures"] == 4                              # windows of 300 + k and of the lag step
    no_lag = A.h4a_bound(runs, dtp, "count", kappa=1.0, window=1, lag_fails=False)
    assert no_lag["failures"] == 3


def test_h4a_requires_empty_buffers():
    n = 40
    y, C = np.full(n, 300_000.0), np.full(n, 300_000.0)
    buf = np.ones(n, bool)
    buf[10:20] = False
    runs = [_records(y, C, buf_empty=buf, fail_at={15})]
    r = A.h4a_bound(runs, tiny_dtp(), "bytes")
    assert r["n_cond"] == 30 and r["failures"] == 0 and r["buffers_known"] is True


# --------------------------------------------------------------------------- outer loop (I4)


def loop_backlog(y, C, U, r):
    q, out = 0.0, []
    for yy, cc in zip(y, C):
        q = max(0.0, q + (U * cc - yy) * r)
        out.append(q)
    return np.asarray(out)


def test_backlog_recursion_and_corollary1():
    y = np.array([100, 100, 70, 100, 100, 100, 80, 120, 100], float)
    C = np.full(len(y), 100.0)
    U = [0.8, 0.9, 1.0]
    q, ok = O.backlog(y, C, U, T3_Ts=0.76)
    for k, u in enumerate(U):
        assert q[:, k] == pytest.approx(loop_backlog(y, C, u, 0.76))
    # U = 0.9: step 2 lacks 20 B (x 0.76); steps 3 and 4 drain 10 B each, so step 3 still fails although
    # y(3) >= U C_hat(3) (bytes left over from step 2), and step 4 is the first with q = 0 again
    assert (q[:, 1] == 0).tolist() == [True, True, False, False, True, True, False, True, True]
    assert O.phi_predicted(y, C, U).tolist() == pytest.approx([8 / 9, 6 / 9, (q[:, 2] == 0).mean()])
    gam = O.coverage(y, C, np.asarray(U))
    assert np.all(O.phi_predicted(y, C, U) <= gam + 1e-12)
    # a missing y leaves q unchanged and is not evaluated
    y2 = y.copy()
    y2[3] = np.nan
    q2, ok2 = O.backlog(y2, C, [0.9])
    assert not ok2[3] and q2[3, 0] == q2[2, 0]


def test_outer_loop_U_star_from_the_lower_bootstrap_bound():
    rng = np.random.default_rng(3)
    level = 300_000 + np.cumsum(rng.normal(0, 3000, 4000))
    y = level + rng.normal(0, 20_000, 4000)
    recs = [{"t": t, "y": float(v), "C_hat": 300_000.0} for t, v in enumerate(y)]
    res = O.outer_loop(recs, target=0.95, B=300)
    w = {x["U"]: x for x in res["whatif"]}
    assert all(x["gamma_low"] <= x["gamma"] + 1e-12 for x in res["whatif"])
    assert all(x["phi"] <= x["gamma"] + 1e-12 for x in res["whatif"])
    assert res["U_star"] <= res["U_star_point"]
    assert w[res["U_star"]]["gamma_low"] >= 0.95
    nxt = round(res["U_star"] + 0.01, 2)
    assert nxt not in w or w[nxt]["gamma_low"] < 0.95
    for k in ("U_star", "alpha_star", "mse", "n_y"):     # keys read by fleetdt.as_service
        assert k in res
    a, C = res["alpha_star"], O.replay(y, [res["alpha_star"]], 300_000.0)[:, 0]
    assert w[0.9]["phi"] == pytest.approx(O.phi_predicted(y, C, [0.9], res["T3_Ts"])[0])


def test_calibrate_by_M():
    rng = np.random.default_rng(9)

    def run(M, scale, n=1500):
        lev = scale * (300_000 + np.cumsum(rng.normal(0, 2000, n)))
        y = lev + rng.normal(0, 15_000 * scale, n)
        return [{"t": t, "y": float(v), "C_hat": scale * 300_000.0, "vessels": list(range(1, M + 1)),
                 "U": 0.8, "K_hat": 0.8 * scale * 300_000.0 * 0.76} for t, v in enumerate(y)]
    runs = [run(2, 1.0), run(2, 1.0), run(4, 0.6), run(4, 0.6), run(4, 0.6)]
    res = O.calibrate_by_M(runs, target=0.95, B=200)
    assert sorted(res) == [2, 4]
    assert res[2]["runs"] == 2 and res[4]["runs"] == 3
    assert res[4]["n_y"] == 4500
    for M in (2, 4):
        assert res[M]["T3_Ts"] == pytest.approx(0.76)
        assert 0 < res[M]["alpha_star"] <= 1 and res[M]["U_star"] is not None
    # pooled MSE: alpha* of a group minimizes the sum over its runs
    ys = [np.array([r["y"] for r in rr]) for rr in runs[:2]]
    m = O.mse_runs(ys, [300_000.0, 300_000.0], [res[2]["alpha_star"], 0.05, 0.9])
    assert m[0] <= m[1] and m[0] <= m[2]
    out = subprocess.run([sys.executable, "-m", "fleetdt.outer", "--help"], cwd=ROOT, capture_output=True, text=True)
    assert out.returncode == 0 and "--by-M" in out.stdout


# --------------------------------------------------------------------------- notation spec (C5, C8, C20, C21, names)


def test_backlog_predictor_with_admitted_bytes_and_c_y():
    """C5: q(t) = max(0, q(t-1) + sum_i a_i(t) - c_y y(t) T3/Ts); DMR = fraction of steps with q > 0. With
    f(t) = A(t) / (U(t) C_hat(t) T3/Ts) the what-if over U reproduces A(t) at the logged U and scales it
    with the cap at other U."""
    r = 0.76
    y = np.array([100, 100, 70, 100, 100, 100, 80, 120, 100], float)
    C = np.full(len(y), 100.0)
    A_ = np.array([60, 70, 75, 68, 60, 76, 70, 50, 65], float)      # admitted bytes, <= U C T3/Ts = 76
    recs = [{"t": t, "y": float(yy), "C_hat": 100.0, "U": 1.0, "A": float(a)} for t, (yy, a) in enumerate(zip(y, A_))]
    f = O.fill_series(recs, r)
    assert f == pytest.approx(A_ / (1.0 * 100.0 * r))
    for cy in (1.0, 0.9):
        want, q = [], 0.0
        for yy, a in zip(y, A_):
            q = max(0.0, q + a - cy * yy * r)
            want.append(q)
        qa, ok = O.backlog_admitted(y, A_, r, c_y=cy)
        assert qa == pytest.approx(want) and ok.all()
        qb, _ = O.backlog(y, C, [1.0], r, fill=f, c_y=cy)          # logged U: X(t) = A(t)
        assert qb[:, 0] == pytest.approx(want)
        assert O.dmr_predicted(y, C, [1.0], r, fill=f, c_y=cy)[0] == pytest.approx(np.mean(np.asarray(want) > 0))
    # c_y < 1 counts less of y as capacity: the backlog can only grow
    assert np.all(O.backlog_admitted(y, A_, r, c_y=0.9)[0] >= O.backlog_admitted(y, A_, r, c_y=1.0)[0])
    # other U: X_U(t) = f(t) U C_hat(t) T3/Ts
    qu, _ = O.backlog(y, C, [0.5], r, fill=f)
    q, want = 0.0, []
    for k, yy in enumerate(y):
        q = max(0.0, q + f[k] * 0.5 * 100.0 * r - yy * r)
        want.append(q)
    assert qu[:, 0] == pytest.approx(want)
    # without A the predictor takes the cap (f = 1) and phi = 1 - DMR
    assert O.phi_predicted(y, C, [0.9], r)[0] == pytest.approx(1 - O.dmr_predicted(y, C, [0.9], r)[0])
    # kappa is an alias of c_y; both with different values are an error
    assert O.backlog_admitted(y, A_, r, kappa=0.9)[0] == pytest.approx(O.backlog_admitted(y, A_, r, c_y=0.9)[0])
    with pytest.raises(ValueError):
        O.backlog(y, C, [1.0], r, c_y=0.9, kappa=0.8)


def test_admitted_bytes_sources():
    """A(t): "A" of the AS, else the sum of "a", else the admitted demand of the edges of G(t)"""
    e = [{"i": 1, "l": 1, "d": 100, "p": 2}, {"i": 1, "l": 2, "d": 50, "p": 0}, {"i": 2, "l": 1, "d": 70, "p": 1},
         {"i": 3, "l": 1, "d": 999, "p": 1}]
    assert O.admitted_bytes({"A": 5, "a": {"1": 1}, "edges": e}) == 5
    assert O.admitted_bytes({"a": {"1": 1, "2": 2}, "edges": e}) == 3
    assert O.admitted_bytes({"edges": e, "vessels": [1, 2]}) == 170
    assert math.isnan(O.admitted_bytes({"edges": [{"i": 1, "l": 1, "d": 100, "p": None}]}))


def test_outer_loop_whatif_names_and_aliases():
    rng = np.random.default_rng(5)
    y = 300_000 + rng.normal(0, 20_000, 2000)
    recs = [{"t": t, "y": float(v), "C_hat": 300_000.0, "U": 0.8, "K_hat": 0.8 * 300_000.0 * 0.76,
             "A": 0.6 * 300_000.0 * 0.76} for t, v in enumerate(y)]
    res = O.outer_loop(recs, target=0.95, B=200, alpha=0.0, c_y=0.95)
    assert res["PICP_nom"] == res["gamma_target"] == 0.95 and res["c_y"] == res["kappa"] == 0.95
    assert res["n_admitted"] == 2000
    for w in res["whatif"]:
        assert w["gamma"] == w["PICP"] and w["gamma_low"] == w["PICP_low"]
        assert w["phi"] == pytest.approx(1 - w["DMR"])
    w = {x["U"]: x for x in res["whatif"]}
    # alpha = 0 keeps C_hat = C0 = 300000: at the logged U = 0.8 the predictor replays A(t) exactly
    qa, ok = O.backlog_admitted(y, np.full(len(y), 0.6 * 300_000.0 * 0.76), 0.76, c_y=0.95)
    assert w[0.8]["DMR"] == pytest.approx(np.mean(qa[ok] > 0))
    assert O.outer_loop(recs, target=0.95, B=50, alpha=0.0, kappa=0.95)["c_y"] == 0.95
    out = subprocess.run([sys.executable, "-m", "fleetdt.outer", "--help"], cwd=ROOT, capture_output=True, text=True)
    assert "--c-y" in out.stdout and "--kappa" in out.stdout


def _state_records(cases):
    """one step per case (n, fwd, dup), vessel 1, state admitted (p 2) and needed"""
    recs = []
    for t, (n, fwd, dup) in enumerate(cases):
        recs.append({"t": t, "y": 100.0, "C_hat": 100.0, "U": 0.8, "vessels": [1], "needed": [1], "order": [1],
                     "edges": [{"i": 1, "l": 1, "d": 100, "p": 2, "r": 100, "n": n, "dup": dup, "late": 0,
                                "fwd": fwd}]})
    return recs


def test_pdr_state_c8():
    """C8: a pair (t, i) counts when the GS receives every state message forwarded in step t exactly once:
    n_rx == n_fwd and no duplicate (two forwarded and two received count; one of two does not)."""
    dtp = DTP(topics=["state"], Ts=0.125, T1=0.02, T2=0.01, U=0.8,
              specs=[__import__("fleetdt.as_model", fromlist=["TopicSpec"]).TopicSpec(name="state", rate=8)])
    recs = _state_records([(2, 2, 0), (1, 2, 0), (1, 1, 1), (1, None, 0), (2, None, 0), (1, 1, 0)])
    res = A.run_metrics(recs, dtp, criterion="count", ci=False)
    assert res["PDR_state"]["value"] == pytest.approx(3 / 6)        # cases 0, 3 (n == ceil(8 Ts) = 1), 5
    assert res["PDR_state"] is res["delta"]


def test_names_aliases_pcr_and_relmse():
    dtp = DTP(topics=["state", "imu", "video"], Ts=0.125, T1=0.02, T2=0.01, U=0.8)
    res = A.run_metrics(_endpoint_records(), dtp, criterion="count", ci=False, c_y=0.9)
    # C21: share of N(t) = {1, 2} in the complete prefix: (0,1) 2/2, (0,2) 1/2, (0,3) 2/2, (1,1) 2/2, (1,2) 2/2
    assert res["PCR"]["value"] == pytest.approx((1 + 0.5 + 1 + 1 + 1) / 5)
    assert res["Pi"] is res["PCR"] and res["PCR"]["pairs"] == 5
    assert res["prefix"]["mean_frac"] == pytest.approx(12 / 15)     # the old ratio (denominator L) stays
    assert res["PICP"] is res["gamma"] and res["u"] is res["nu"] and res["PDR_i"] is res["eta_vessel"]
    assert res["DMR"]["value"] == pytest.approx(1 - res["phi"]["value"])
    assert res["c_y"] == res["kappa"] == 0.9
    # DMR_hat from the admitted demand of the edges: step 0 A = 700 (vessels 1, 2; p > 0), step 1 A = 1600;
    # c_y y T3/Ts = 0.9 * 1000 * 0.76 = 684: q(0) = 16, q(1) = 932
    assert res["DMR_hat"]["value"] == pytest.approx(1.0) and res["DMR_hat"]["n"] == 2
    assert A.margin_series(_endpoint_records(), dtp, kappa=0.9).tolist() == \
        A.margin_series(_endpoint_records(), dtp, c_y=0.9).tolist()
    # RelMSE = MSE_ES / MSE_alt and its log
    r = A.rel_mse([1, 2, 3, 4], [1, 2, 3, 5], [0, 2, 3, 6])
    assert r["value"] == pytest.approx(0.25 / 1.25) and r["log"] == pytest.approx(math.log(0.2)) and r["n"] == 4
    assert res["RelMSE"]["alt"] == "persistence"


def test_c20_n_eff_clopper_pearson_and_signed_tost():
    # n_eff = n / (2 tau_int); tau_int = 1/2 for i.i.d. data gives n_eff = n
    assert A.n_eff(1000, 0.5) == 1000 and A.n_eff(1000, 2.5) == 200
    # Clopper-Pearson with the raw count in n_eff trials: k = 0 gives 1 - 0.05^(1/n_eff), n_eff not integer
    for ne in (37.5, 200.0, 2995.0):
        assert A.clopper_pearson_upper_eff(0, 5000, ne, events="raw") == pytest.approx(1 - 0.05 ** (1 / ne), rel=1e-9)
    assert A.clopper_pearson_upper_eff(0, 1e6, 2995.0) < 1e-3 < A.clopper_pearson_upper_eff(0, 1e6, 2990.0)
    # raw is the conservative rule: with k > 0 events it is not below the scaled count
    assert A.clopper_pearson_upper_eff(3, 6000, 600, events="raw") >= A.clopper_pearson_upper_eff(3, 6000, 600,
                                                                                                    events="scaled")
    # the event metrics report the raw bound (viol_upper) and the scaled one
    rng = np.random.default_rng(4)
    recs = _records(330_000 + ar1(800, 0.9, rng, 2000.0), np.full(800, 300_000.0), fail_at=(5,))
    res = A.run_metrics(recs, tiny_dtp(), criterion="count", ci=False)
    ne = res["DMR"]["n_eff"]
    assert res["DMR"]["violations"] == 1
    assert res["DMR"]["viol_upper"] == pytest.approx(A.clopper_pearson_upper_eff(1, 800, ne, events="raw"))
    assert res["DMR"]["viol_upper_scaled"] == pytest.approx(A.clopper_pearson_upper_eff(1, 800, ne, events="scaled"))
    # TOST on the signed differences: +-0.006 around 0 is equivalent within 0.005; |d| = 0.006 is not
    d = np.array([0.006, -0.006] * 20)
    assert A.tost(d, 0.005)["equivalent"]
    assert not A.tost(np.abs(d), 0.005)["equivalent"]
    a = np.linspace(0.5, 0.6, 40)
    r1, r2 = A.tost(a, 0.005, b=a - d), A.tost(d, 0.005)
    assert r1["equivalent"] == r2["equivalent"] and r1["p"] == pytest.approx(r2["p"], rel=1e-9)
