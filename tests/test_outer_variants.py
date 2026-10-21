"""Outer-loop variants O1 to O5 (fleetdt.outer_variants, the AS of fleetdt.as_service, emu/fleet.py): synthetic
y(t) and decisions, no broker."""
import json
import os
import shlex
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from fleetdt import outer
from fleetdt import outer_variants as OV
from fleetdt.as_model import DTP
from fleetdt.as_service import AS, Service, build_variant, main as as_main, parse_regime

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "emu"))
import fleet  # noqa: E402


def small_dtp(**kw):
    # T3 / Ts = 0.5, C0 = 4000: K_hat(0) = 2000
    base = dict(topics=["a", "b", "c", "d"], Ts=1.0, T1=0.25, T2=0.25, alpha=0.125, U=0.8, C0=4000.0,
                order=["a", "b", "c", "d"], needed=["a", "b"], vessels=[1, 2])
    base.update(kw)
    return DTP(**base)


def calibration(**entries):
    by = entries or {"2": {"S1": {"alpha_star": 0.5, "U_star": 0.8}, "S3": {"alpha_star": 0.2, "U_star": 0.6}}}
    return {"format": OV.CALIBRATION_FORMAT, "PICP_nom": 0.99, "by_M": by}


def run_steps(a, ys, t0=0):
    """Closes steps t0, t0 + 1, ...; y(t) = ys[t - t0] (None: no y) arrives 0.1 s after start(t + 1), before the
    deadline of step t + 1. Returns the decision record (AS.last) of each step."""
    recs = []
    for k, y in enumerate(ys):
        t = t0 + k
        a.close_phase1(t)
        recs.append(dict(a.last))
        if y is not None:
            a.on_throughput(t, y, at=a.start(t + 1) + 0.1)
    return recs


# --------------------------------------------------------------------------- calibration file


def test_calibration_file_format_and_lookup(tmp_path):
    p = tmp_path / "cal.json"
    p.write_text(json.dumps(calibration()))
    cal = OV.load_calibration(p)
    assert OV.calibration_values(cal, 2, "S3") == (0.2, 0.6)
    with pytest.raises(KeyError, match="M = 4"):
        OV.calibration_values(cal, 4, "S1")
    with pytest.raises(KeyError, match="scenario S9"):
        OV.calibration_values(cal, 2, "S9")
    for bad in ({"by_M": calibration()["by_M"]},                                  # no format
                calibration(**{"2": {"S1": {"alpha_star": 0.0, "U_star": 0.8}}}),  # alpha out of (0, 1]
                calibration(**{"2": {"S1": {"alpha_star": 0.5, "U_star": 1.2}}}),  # U outside the grid
                calibration(**{"2": {"S1": {"alpha_star": 0.5, "U_star": None}}}),
                calibration(**{"x": {"S1": {"alpha_star": 0.5, "U_star": 0.8}}})):
        with pytest.raises(ValueError):
            OV.validate_calibration(bad)


def test_regime_change_steps():
    rs = OV.regime_steps([{"t0": 130.0, "scenario": "S3"}, {"t0": 0, "scenario": "S1"}, (260.0, "S1")], 0.125, 1000)
    assert rs == [(1000, "S1"), (2040, "S3"), (3080, "S1")]
    assert OV.regime_steps([(0.3, "S3")], 0.125, 0) == [(2, "S3")]        # the step that contains t0
    assert parse_regime("130.0:S3") == {"t0": 130.0, "scenario": "S3"}
    with pytest.raises(ValueError):
        parse_regime("130")


# --------------------------------------------------------------------------- O1 and O2


def test_o1_holds_the_dtp_values_and_logs_them_per_step():
    dtp = small_dtp()
    a = AS(dtp, 0.0, outer=OV.make_variant("O1", alpha0=dtp.alpha, U0=dtp.U))
    recs = run_steps(a, [4000.0, 0.0, None, 4000.0, 4000.0])
    assert all(r["variant"] == "O1" and r["alpha"] == 0.125 and r["U"] == 0.8 for r in recs)
    assert "outer_update" not in json.dumps(recs)
    # eq. (2) with alpha_0: C_hat(1) = (1 - 1/8) 4000 + 1/8 4000, C_hat(2) = 7/8 C_hat(1) + 0
    assert recs[1]["C_hat"] == pytest.approx(4000.0) and recs[2]["C_hat"] == pytest.approx(3500.0)
    # coverage events of the previous step: K_y(t-1) >= U K_hat(t-1)
    assert recs[0]["cover"] is None
    assert recs[1]["cover"] == [0, 1]           # K_y = 2000 >= 0.8 * 2000
    assert recs[2]["cover"] == [1, 0]           # K_y = 0
    assert recs[3]["cover"] is None             # no y(2)
    assert recs[4]["cover"] == [3, 1]
    with pytest.raises(ValueError):
        AS(dtp, 0.0, U=0.5, outer=OV.make_variant("O1", alpha0=0.125, U0=0.8))


def test_o2_takes_the_calibration_of_m_and_the_scenario():
    a = AS(small_dtp(), 0.0, outer=OV.make_variant("O2", alpha0=0.125, U0=0.8, cal=calibration(), M=2,
                                                   scenario="S3"))
    recs = run_steps(a, [4000.0] * 4)
    assert all(r["alpha"] == 0.2 and r["U"] == 0.6 and r["variant"] == "O2" for r in recs)
    with pytest.raises(ValueError):
        OV.make_variant("O2", alpha0=0.125, U0=0.8)


# --------------------------------------------------------------------------- O3


def test_o3_windows_lag_and_fallbacks():
    v = OV.make_variant("O3", alpha0=0.125, U0=0.8, N_w=480, origin=1000, lag=2)
    assert (v.alpha, v.U) == (0.125, 0.8)                                   # init "dtp": alpha_0, U_0
    due = {t: v.job_due(t) for t in range(900, 2500) if v.job_due(t)}
    assert due == {1481: (1000, 1479), 1961: (1480, 1959), 2441: (1960, 2439)}
    assert v.apply({"alpha_star": 0.37, "U_star": 0.91}) == {"alpha": 0.37, "U": 0.91, "alpha_star": 0.37,
                                                             "U_star": 0.91}
    v.apply({"alpha_star": 0.4, "U_star": None})                            # no U of the grid reaches PICP_nom
    assert (v.alpha, v.U) == (0.4, 0.30)
    k = OV.make_variant("O3", alpha0=0.125, U0=0.8, no_U="keep")
    k.apply({"alpha_star": float("nan"), "U_star": None})
    assert (k.alpha, k.U) == (0.125, 0.8)
    c = OV.make_variant("O3", alpha0=0.125, U0=0.8, init="calibration", cal=calibration(), M=2, scenario="S1")
    assert (c.alpha, c.U) == (0.5, 0.8) and not c.refine
    w = OV.make_variant("O3", alpha0=0.125, U0=0.8, N_w=10)                 # origin: the first step closed
    a = AS(small_dtp(), 0.0, outer=w)
    run_steps(a, [None] * 3, t0=50)
    assert w.origin == 50 and w.job_due(61) == (50, 59)


def synthetic_dta(path, n=300, seed=3):
    """DTA records with y(t) of an AR(1) around 3e5 and the C_hat(t), U(t) of an AS with alpha 0.125."""
    rng = np.random.default_rng(seed)
    y = 3e5 + np.cumsum(rng.normal(0, 4e3, n)) * 0.3 + rng.normal(0, 1.5e4, n)
    C, recs = 3e5, []
    for t in range(n):
        recs.append({"t": 100 + t, "y": float(y[t]), "C_hat": C, "K_hat": C * 0.76, "U": 0.8, "vessels": [1, 2]})
        C = 0.875 * C + 0.125 * float(y[t])
    with open(path, "w") as f:
        for r in recs:
            f.write(json.dumps(r) + "\n")
    return recs


def test_o3_job_uses_the_steps_of_the_window_and_the_alpha_grid(tmp_path):
    dta = tmp_path / "dta.jsonl"
    recs = synthetic_dta(dta)
    out = str(tmp_path / "res.json")
    outer.run_job(str(dta), 0.95, None, out, c_y=1.0, B_ctl_bar=0.0, T3_Ts=0.76, t_first=150, t_last=249, B=200)
    res = json.load(open(out))["result"]
    want = outer.outer_loop(outer.window_records(recs, 150, 249), target=0.95, T3_Ts=0.76, B=200)
    assert res["t_first"] == 150 and res["t_last"] == 249 and res["n_steps"] == 100
    assert res["alpha_star"] == want["alpha_star"] and res["U_star"] == want["U_star"]
    assert res["alpha_star"] in set(outer.ALPHA_GRID.tolist()) and res["refine"] is False
    outer.run_job(str(dta), 0.95, None, out, t_first=5000, t_last=5100)
    assert "error" in json.load(open(out))


def test_alpha_refinement_is_an_explicit_option():
    rng = np.random.default_rng(1)
    y = 1e5 + np.cumsum(rng.normal(0, 2e3, 400))
    a_grid, m_grid = outer.fit_alpha(y, float(y[0]))
    a_ref, m_ref = outer.fit_alpha(y, float(y[0]), refine=True)
    assert a_grid in set(outer.ALPHA_GRID.tolist())
    assert m_ref <= m_grid and abs(a_ref - a_grid) <= 0.01 + 1e-9


def test_o3_in_the_service_applies_the_window_result(tmp_path):
    dta = tmp_path / "dta.jsonl"
    recs = synthetic_dta(dta)
    dtp = DTP.from_dict({"topics": [{"name": "state", "rate": 8, "bound": 100}], "Ts": 0.125, "vessels": [1, 2]})
    v = OV.make_variant("O3", alpha0=dtp.alpha, U0=dtp.U, N_w=100, origin=100, lag=2, target=0.95, B=200)
    svc = Service(dtp, epoch=0.0, outer_dta=str(dta), outer_variant=v, log_path=str(tmp_path / "as.jsonl"))
    svc._outer_tick(199)                                   # window 100..199 ended, lag 2: no job yet
    assert not svc._outer_busy
    svc._outer_tick(201)
    assert svc._outer_busy and svc._job_window == (100, 199)
    end = time.time() + 60
    while svc._outer_busy and time.time() < end:
        time.sleep(0.05)
        svc._outer_poll()
    assert not svc._outer_busy
    svc._outer_tick(202)
    want = outer.outer_loop(outer.window_records(recs, 100, 199), target=0.95, T3_Ts=dtp.T3m / dtp.Ts, B=200)
    assert svc.core.alpha == want["alpha_star"]
    assert svc.core.U == (want["U_star"] if want["U_star"] is not None else 0.30)
    ev = [json.loads(l) for l in open(tmp_path / "as.jsonl")][-1]
    assert ev["event"] == "outer" and ev["variant"] == "O3" and ev["window"] == [100, 199] and ev["t"] == 202
    svc.stop()
    with pytest.raises(ValueError, match="DTA"):
        Service(dtp, epoch=0.0, outer_variant=OV.make_variant("O3", alpha0=0.1, U0=0.8))


# --------------------------------------------------------------------------- O4


def test_o4_adapts_U_from_the_coverage_of_each_window():
    """N_w = 5: window 0 with y(t) = C0 (every step covered) raises U by 0.05, window 1 with y = 0 (no step
    covered) lowers it, window 2 without y keeps it; the new U applies from the first step of the next window."""
    dtp = small_dtp()
    v = OV.make_variant("O4", alpha0=0.125, U0=0.8, cal=calibration(), M=2, scenario="S1", N_w=5, origin=0)
    a = AS(dtp, 0.0, outer=v)
    assert (a.alpha, a.U) == (0.5, 0.8)                                     # alpha*, U*
    recs = run_steps(a, [4000.0] * 5 + [0.0] * 5 + [None] * 5 + [None])
    U = [r["U"] for r in recs]
    assert U[:5] == [0.8] * 5 and U[5:10] == [0.85] * 5 and U[10:] == [0.8] * 6
    assert all(r["alpha"] == 0.5 for r in recs)
    assert recs[5]["outer_update"] == {"window": [0, 4], "PICP": 1.0, "n": 5, "alpha": 0.5, "U": 0.85}
    assert recs[10]["outer_update"]["PICP"] == 0.0 and recs[10]["outer_update"]["window"] == [5, 9]
    assert recs[15]["outer_update"] == {"window": [10, 14], "PICP": None, "n": 0, "alpha": 0.5, "U": 0.8}
    assert sum("outer_update" in r for r in recs) == 3


def test_o4_coverage_uses_K_y_with_c_y_and_B_ctl_bar():
    # K_y = c_y y (T3 - T_m)/Ts - B_ctl_bar = 0.9 * 4000 * 0.5 - 100 = 1700 < U K_hat = 0.9 * 2222.2: not covered
    dtp = small_dtp(c_y=0.9, B_ctl_bar=100.0)
    v = OV.CoverageFeedback(0.5, 0.9, N_w=2, origin=0)
    a = AS(dtp, 0.0, outer=v)
    a.C0 = 4000.0 / 0.9                                                     # C_hat(0) = c_y y
    recs = run_steps(a, [4000.0] * 3)
    assert recs[1]["cover"] == [0, 0] and recs[2]["outer_update"]["U"] == pytest.approx(0.85)
    dtp2 = small_dtp(c_y=0.9, B_ctl_bar=0.0)
    a2 = AS(dtp2, 0.0, outer=OV.CoverageFeedback(0.5, 0.9, N_w=2, origin=0))
    a2.C0 = 3900.0
    assert run_steps(a2, [4000.0] * 3)[1]["cover"] == [0, 1]                # 1800 >= 0.9 * 1950


def test_o4_clamps_U_and_ignores_steps_before_the_origin():
    v = OV.CoverageFeedback(0.5, 1.19, N_w=2, origin=10)
    assert v.step(8, (7, True)) is None and v.step(10, (9, True)) is None   # before the origin: no window
    v.step(11, (10, True))
    assert v.step(12, (11, True))["U"] == 1.2                               # clamp at U_max
    v.U = 0.31
    v.step(13, (12, False))
    assert v.step(14, (13, False))["U"] == 0.30                             # clamp at U_min
    assert OV.CoverageFeedback(0.5, 1.5).U == 1.2


# --------------------------------------------------------------------------- O5


def test_o5_switches_at_the_step_of_each_change():
    regimes = [{"t0": 0.0, "scenario": "S1"}, {"t0": 2.5, "scenario": "S3"}, {"t0": 4.0, "scenario": "S1"}]
    v = OV.make_variant("O5", alpha0=0.125, U0=0.8, cal=calibration(), M=2, regimes=regimes, Ts=1.0, origin=10)
    a = AS(small_dtp(), 0.0, outer=v)
    recs = run_steps(a, [4000.0, 4000.0, 2000.0, 4000.0, 4000.0, 4000.0, 4000.0], t0=9)   # steps 9..15
    by = {r["t"]: r for r in recs}
    assert [(by[t]["regime"], by[t]["alpha"], by[t]["U"]) for t in range(9, 16)] == [
        ("S1", 0.5, 0.8), ("S1", 0.5, 0.8), ("S1", 0.5, 0.8), ("S3", 0.2, 0.6), ("S3", 0.2, 0.6),
        ("S1", 0.5, 0.8), ("S1", 0.5, 0.8)]
    assert [t for t in by if "outer_update" in by[t]] == [12, 14]
    # eq. (2) of the change step uses the alpha* of the new regime
    assert by[12]["C_hat"] == pytest.approx(0.8 * by[11]["C_hat"] + 0.2 * 2000.0)
    with pytest.raises(KeyError):
        OV.make_variant("O5", alpha0=0.1, U0=0.8, cal=calibration(), M=2, regimes=[(0, "S7")])
    with pytest.raises(ValueError):
        OV.make_variant("O5", alpha0=0.1, U0=0.8, cal=calibration(), M=2, regimes=[])


# --------------------------------------------------------------------------- command lines


def test_as_command_line_builds_the_variant(tmp_path):
    dtp = tmp_path / "dtp.json"
    dtp.write_text(json.dumps({"topics": [{"name": "state", "rate": 8, "bound": 100}], "vessels": [1, 2]}))
    cal = tmp_path / "cal.json"
    cal.write_text(json.dumps(calibration()))
    import argparse
    ns = dict(outer_variant="O5", calibration=str(cal), calib_M=None, calib_scenario=None, outer_alpha0=None,
              outer_U0=None, regime=["0:S1", "130:S3", "260:S1"], outer_Nw=480, outer_origin=7,
              outer_target=0.99, outer_init="dtp", outer_lag=2, outer_refine=False, outer_no_U="min", outer_B=1000,
              outer_step=0.05, outer_U_min=0.3, outer_U_max=1.2)
    v = build_variant(argparse.Namespace(**ns), DTP.load(dtp))
    assert isinstance(v, OV.RegimeOracle) and (v.alpha, v.U) == (0.5, 0.8)
    v.step(7, None)
    assert v.changes == [(7, "S1"), (1047, "S3"), (2087, "S1")]
    with pytest.raises(SystemExit):
        as_main(["--dtp", str(dtp), "--outer-variant", "O1", "--U", "0.7"])


PAPER = {"M": 2, "method": "fleetdt", "dtp": {"epoch": None}, "scenario": {"scenario_id": "S3"}}


def spec_with(outer_cfg, **kw):
    s = dict(PAPER, outer=outer_cfg)
    s.update(kw)
    return s


def as_args(spec, S0=None):
    return shlex.split(fleet.paper_as_cmd(spec, S0).split("exec ", 1)[1])


def test_fleet_spec_fields_of_the_variants(tmp_path):
    cal = tmp_path / "cal.json"
    cal.write_text(json.dumps(calibration()))
    c = as_args(spec_with({"variant": "o1"}), S0=1234)
    assert c[c.index("--outer-variant") + 1] == "O1" and c[c.index("--outer-origin") + 1] == "1234"
    assert "--calibration" not in c and "--outer-dta" not in c and "--U" not in c
    c = as_args(spec_with({"variant": "O2", "calibration_file": str(cal)}))
    assert c[c.index("--calibration") + 1] == fleet.CALIBRATION_GS and c[c.index("--calib-M") + 1] == "2"
    assert c[c.index("--calib-scenario") + 1] == "S3"
    c = as_args(spec_with({"variant": "O3", "N_w": 240}))
    assert c[c.index("--outer-dta") + 1] == fleet.RUN + "/dta.jsonl" and c[c.index("--outer-Nw") + 1] == "240"
    assert "--outer-refine" not in c and c[c.index("--outer-no-U") + 1] == "min"
    c = as_args(spec_with({"variant": "O4", "calibration_file": str(cal), "U_max": 1.1}))
    assert c[c.index("--outer-U-max") + 1] == "1.1" and c[c.index("--outer-step") + 1] == "0.05"
    regimes = [{"t0": 0, "scenario": "S1"}, {"t0": 130, "scenario": "S3"}, {"t0": 260, "scenario": "S1"}]
    s5 = spec_with({"variant": "O5", "calibration_file": str(cal)}, scenario={"regimes": regimes})
    c = as_args(s5, S0=50)
    assert [c[i + 1] for i, x in enumerate(c) if x == "--regime"] == ["0.0:S1", "130.0:S3", "260.0:S1"]
    assert fleet.paper_outer(s5)["calib_scenario"] == "S1"                  # first regime
    # the legacy "every" outer loop is unchanged
    c = as_args(dict(PAPER, outer={"every": 480, "window": 480}))
    assert "--outer-every" in c and "--outer-variant" not in c


@pytest.mark.parametrize("bad", [
    {"variant": "O6"},
    {"variant": "O2"},                                                       # no calibration file
    {"variant": "O2", "calibration_file": "CAL", "calib_M": 4},              # no entry for M = 4
    {"variant": "O2", "calibration_file": "CAL", "calib_scenario": "S9"},
    {"variant": "O5", "calibration_file": "CAL"},                            # no regimes
    {"variant": "O1", "calibration_file": "CAL"},
    {"variant": "O3", "every": 480},
    {"variant": "O3", "init": "x"},
])
def test_fleet_refuses_inconsistent_variants(tmp_path, bad):
    cal = tmp_path / "cal.json"
    cal.write_text(json.dumps(calibration()))
    o = {k: (str(cal) if v == "CAL" else v) for k, v in bad.items()}
    with pytest.raises(SystemExit):
        fleet.paper_outer(spec_with(o))


def test_fleet_refuses_a_variant_with_fixed_parameters_or_b4(tmp_path):
    with pytest.raises(SystemExit, match="as.U"):
        fleet.paper_outer(spec_with({"variant": "O1"}, **{"as": {"U": 0.7}}))
    with pytest.raises(SystemExit, match="Algorithm 1"):
        fleet.paper_outer(spec_with({"variant": "O1"}, method="B4", **{"as": {"C_ref": 1e5}}))


def test_calibration_file_of_calibrate_paper_is_accepted():
    """calibration.json of emu/experiments/calibrate_paper.py: (alpha*, U*) from "outer".<M>."left_out", the
    "all" entry for a scenario without its own fit, and no entry with a null U*."""
    from fleetdt import outer_variants as OV
    cal = {"version": 1, "params": {"PICP_nom": 0.99},
           "outer": {"4": {"left_out": {"S1": {"alpha": 0.2, "U": 0.8}, "S3": {"alpha": 0.3, "U": None},
                                        "all": {"alpha": 0.25, "U": 0.75}}}}}
    c = OV.validate_calibration(cal)
    assert OV.calibration_values(c, 4, "S1") == (0.2, 0.8)
    assert OV.calibration_values(c, 4, "S8") == (0.25, 0.75)
    assert "S3" not in c["by_M"]["4"]
    with pytest.raises(KeyError):
        OV.calibration_values(c, 4, "S3")
