"""Calibration helpers of emu/experiments/calibrate.py and the rho settings of matrix.py."""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "emu", "experiments"))
import calibrate as C  # noqa: E402
import matrix as MX  # noqa: E402


def test_replays():
    ser = [{"t": k, "b": 0.1 if k % 2 else 0.0, "eta": 0.8, "kappa": 0.7 + 0.01 * (k % 3)} for k in range(50)]
    kh = C.replay_eq(ser, 1.0)
    assert math.isclose(kh[1], 0.9 * 0.8) and math.isclose(kh[2], 0.8)
    ks = C.replay_es(ser, 1.0)              # persistence: kappa(t - 1)
    assert all(math.isclose(ks[k], ser[k - 1]["kappa"]) for k in range(1, 50))
    assert math.isclose(C.replay_es(ser, 0.5)[1], 0.5 * 0.8 + 0.5 * 0.7)


def test_aci_replay_converges_to_beta():
    rng = random.Random(3)
    z = [rng.gauss(1.0, 0.1) for _ in range(5000)]
    init = [rng.gauss(1.0, 0.1) for _ in range(2000)]
    fb = C.aci_replay(z, init, 0.05, 0.02, 0.5, 240)
    assert abs(fb - 0.05) < 0.01
    # with c < 1 (conservative cost) the level climbs: the same target, other quantile
    fb2 = C.aci_replay(z, init, 0.05, 0.05, 0.5, 240, c=0.9)
    assert abs(fb2 - 0.05) < 0.01


def test_rho_setting_scales_with_kappa():
    cal = {"sat": {"S1-m4": {"kappa": 0.8, "R_up": 24e6, "R_down": 24e6}}}
    r1 = MX.rho_setting(cal, "S1", 4, 0.125, 1.5)
    T_W = r1["T_W"]
    assert math.isclose(4 * (r1["c_streams"] + r1["rate"] * 0.125 * r1["c_frame"]), 1.5 * 0.8 * T_W, rel_tol=1e-3)
    cal["sat"]["S1-m4"]["kappa"] = 0.6
    assert MX.rho_setting(cal, "S1", 4, 0.125, 1.5)["rate"] < r1["rate"]


def test_aci_replay_attainability():
    # narrow scores and c well below 1: even the largest score as Z gives f_bar < beta (unattainable)
    rng = random.Random(5)
    z = [rng.gauss(1.0, 0.02) for _ in range(2000)]
    init = [rng.gauss(1.0, 0.02) for _ in range(1000)]
    assert C.aci_replay(z, init, 0.05, 0.0, 1.0, 240, c=0.8, hold=True) < 0.05
    assert C.aci_replay(z, init, 0.05, 0.05, 0.5, 240, c=0.8) < 0.01
    # wide scores: attainable with the level held at 1
    zw = [rng.gauss(1.0, 0.2) for _ in range(2000)]
    assert C.aci_replay(zw, init, 0.05, 0.0, 1.0, 240, c=0.95, hold=True) >= 0.05
