"""Tests of the predictors of H5 (fleetdt/predictors.py)."""
import math

import numpy as np
import pytest

from fleetdt import outer as O
from fleetdt import predictors as P


def local_level_series(n, q, rng, s_eps=1.0, level=100.0):
    mu = level + np.cumsum(rng.normal(0, math.sqrt(q) * s_eps, n))
    return mu + rng.normal(0, s_eps, n)


def test_simple_predictors():
    z = np.array([10.0, 12.0, 11.0, 15.0, 14.0])
    assert P.last_value(z).tolist() == [10, 10, 12, 11, 15]
    assert P.es(z, 1.0).tolist() == P.last_value(z).tolist()
    # ES equals the replay of eq. (2) of the outer loop from C0 = z[0]
    assert P.es(z, 0.3) == pytest.approx(O.replay(z, [0.3], z[0])[:, 0])
    assert P.moving_average(z, 2).tolist() == pytest.approx([10, 10, 11, 11.5, 13])
    h = P.holt(z, 1.0, 1.0)                   # a = b = 1: level z[k-1], trend z[k-1] - z[k-2] (0 at k = 1)
    assert h.tolist() == pytest.approx([10, 10, 14, 10, 19])
    assert P.arima_predict(z, [], []) == pytest.approx(P.last_value(z))   # ARIMA(0, 1, 0)


def test_local_level_kalman_steady_state_is_es():
    rng = np.random.default_rng(1)
    q = 0.2
    z = local_level_series(3000, q, rng)
    p, F = P.local_level(z, q)
    a_ss = (-q + math.sqrt(q * q + 4 * q)) / 2
    # after the transient, the Kalman gain is the ES gain of the steady state
    pe = P.es(z, a_ss)
    assert np.max(np.abs(p[200:] - pe[200:])) < 1e-6 * np.max(np.abs(z))
    fit = P.fit_local_level([z[:1500], z[1500:]])
    assert 0.1 < fit["q"] < 0.4 and fit["alpha_ss"] == pytest.approx((-fit["q"] + math.sqrt(fit["q"] ** 2 + 4 * fit["q"])) / 2)
    assert fit["sigma2_eps"] == pytest.approx(1.0, rel=0.2)


def test_arima_css_recovers_an_ma1():
    rng = np.random.default_rng(2)
    e = rng.normal(0, 1, 4001)
    w = e[1:] - 0.6 * e[:-1]                  # ARIMA(0, 1, 1) with theta = -0.6
    z = np.r_[0.0, np.cumsum(w)]
    fit = P.fit_arima([z[:2000], z[2000:]], orders=[(0, 0), (0, 1), (1, 0)], engine="css")
    assert (fit["p"], fit["q"]) == (0, 1) and fit["theta"][0] == pytest.approx(-0.6, abs=0.05)
    assert fit["engine"] == "css" and len(fit["candidates"]) == 3
    # ARIMA(0, 1, 1) with theta = -(1 - a) is ES with gain a
    p = P.arima_predict(z, [], [-0.6])
    assert p[50:] == pytest.approx(P.es(z, 0.4)[50:], abs=1e-6 * np.max(np.abs(z)) + 1e-9)
    with pytest.raises(ValueError):
        P.fit_arima([z], engine="other")


@pytest.mark.skipif(not P.have_statsmodels(), reason="statsmodels not installed")
def test_arima_statsmodels_engine():
    rng = np.random.default_rng(3)
    e = rng.normal(0, 1, 1501)
    z = np.r_[0.0, np.cumsum(e[1:] - 0.5 * e[:-1])]
    fit = P.fit_arima([z], orders=[(0, 1), (1, 1)], engine="statsmodels")
    assert fit["engine"] == "statsmodels" and fit["theta"][0] == pytest.approx(-0.5, abs=0.1)


def test_fits_and_relmse():
    rng = np.random.default_rng(4)
    cal = [local_level_series(800, 0.1, rng) for _ in range(3)]
    ma = P.fit_ma(cal, windows=range(2, 20))
    assert 2 <= ma["w"] < 20
    h = P.fit_holt(cal)
    assert 0 < h["a"] <= 1 and 0 <= h["b"] <= 1
    zs = [np.asarray(c) for c in cal]
    a_star, _ = O.fit_alpha_runs(zs, [c[0] for c in zs])
    fit = P.fit_all(cal, a_star, C_ref=100.0, arima_engine="css", ma_windows=range(2, 20))
    z = local_level_series(800, 0.1, rng)
    r = P.rel_mse_all(z, fit, burn=50)
    assert set(r) == set(P.PREDICTORS)
    assert all(v["n"] == 750 for v in r.values())
    assert r["last_value"]["value"] < 1.0 and r["local_level"]["value"] == pytest.approx(1.0, abs=0.05)
    pr = P.predict_all(z, fit)
    e = z[50:] - pr["es"][50:]
    assert r["tcp"]["MSE_ES"] == pytest.approx(float(np.mean(e * e)))
    # alpha* stays on the grid {0.01, ..., 1}
    assert a_star == pytest.approx(round(a_star, 2))
    a_ref, _ = O.fit_alpha_runs(zs, [c[0] for c in zs], refine=True)
    assert O.mse_runs(zs, [c[0] for c in zs], [a_ref])[0] <= O.mse_runs(zs, [c[0] for c in zs], [a_star])[0] + 1e-9
