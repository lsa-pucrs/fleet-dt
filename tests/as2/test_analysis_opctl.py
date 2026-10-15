"""Analysis (Def. fidelity, Theorem 1 check, F_R <= H_R), what-if replay (Def. whatif), the
operation-control schedule, and the AS modes b4 and oracle on the synthetic fleet."""
import json
import random

import numpy as np
import pytest

from fleetdt.as2 import dtp as D
from fleetdt.as2.analysis import aoi, analyze, wilson
from fleetdt.as2.core import ASCore
from fleetdt.as2.opctl import Schedule, goals_payload
from fleetdt.as2.whatif import configure, replay, series

from .fleet import SimFleet


@pytest.fixture(scope="module")
def run():
    d = D.example()
    core = ASCore(d)
    rng = random.Random(4)
    level = [0.6]

    def kappa(t):
        level[0] = min(0.85, max(0.25, level[0] + rng.gauss(0, 0.02)))
        return level[0] * (0.5 if 300 <= t < 360 else 1.0)
    sim = SimFleet(d, core, seed=9, kappa_fn=kappa, monitor=True, b_fn=lambda t: 0.15)
    recs = sim.run(900)
    return d, core, recs


def test_analysis_fidelity(run):
    d, core, recs = run
    res = analyze(json.loads(json.dumps(recs)), d)
    assert res["steps"] == 899
    fb = res["f_bar"]["value"]
    assert fb == sum(core.f_hist) / len(core.f_hist)
    lo, hi = res["f_bar"]["wilson95"]
    assert lo <= fb <= hi
    assert res["theorem1"]["holds_all_prefixes"] and res["theorem1"]["left"] <= res["theorem1"]["right"]
    assert res["F_R"] == core.aci.FR
    # Corollary 2: F_R <= H_R (the fleet fails only when the work exceeds kappa T_W)
    assert res["F_R"] <= res["H_R"]
    assert 0.5 < res["coverage"]["value"] <= 1.0
    assert 0 < res["mae_kappa_hat"] < 0.5
    assert 0 < res["nu"] < 1.5
    assert res["scores"]["n"] > 800
    # per-edge message ratio: the victims of failed steps miss one message
    assert all(0.5 < v <= 1.0 for v in res["edge_msgs"].values() if v is not None)


def test_aoi_and_wilson():
    ev = [(1.0 + 0.125 * k, 0.9 + 0.125 * k) for k in range(80)]
    a = aoi(ev)
    assert a["n"] == 80 and abs(a["peak_mean"] - 0.225) < 1e-9 and abs(a["mean"] - (0.1 + 0.0625)) < 1e-9
    lo, hi = wilson(5, 100)
    assert lo < 0.05 < hi


def test_whatif_replay(run):
    d, core, recs = run
    seq = series(recs)
    assert len(seq) > 800
    same = replay(d, seq)
    assert 0 <= same["f_bar"] <= same["bound"] + 1e-12
    out = [replay(configure(d, M=M), seq) for M in (2, 4, 8, 16)]
    # more vessels: less airtime per output; the efficiency of eq. (eff) falls with M
    assert out[0]["eta"] > out[-1]["eta"]
    assert out[0]["outputs_per_step"] >= out[-1]["outputs_per_step"] - 1e-9
    for o in out:
        assert 0 <= o["f_bar"] <= o["bound"] + 1e-12 and o["nu"] > 0
    slow = replay(configure(d, rate=6e6), seq, {i: (6e6, 6e6) for i in d.vessels})
    assert slow["outputs_per_step"] <= same["outputs_per_step"]
    lowb = replay(configure(d, beta=0.01), seq)
    assert lowb["outputs_per_step"] <= same["outputs_per_step"] + 1e-9


def test_schedule():
    d = D.example()
    s = Schedule(d, period=4, survey_vessels=[2])
    order, needed, mode = s.plan(1)
    assert mode == "survey" and order[0] == "state" and sorted(order) == sorted(t.name for t in d.topics)
    assert "video" in needed["2"] and "video" not in needed["1"]
    assert order.index("video") < order.index("baro")
    order, needed, mode = s.plan(5)
    assert mode == "transit" and order[-1] == "video" and all("video" not in v for v in needed.values())
    assert s.plan(9)[2] == "survey"
    assert len(goals_payload(d, 1, 8, "survey", random.Random(1))) <= d.goals_s
    # the AS accepts the order and extends N_i(t) by R
    core = ASCore(d)
    o, n, _ = s.plan(1)
    core.on_order({"t": 3, "order": o, "needed": n})
    rank, needed = core.order_for(4)
    assert rank[d.names["state"]] == 0 and d.names["video"] in needed[2] and 0 in needed[1]
    with pytest.raises(ValueError):
        core.on_order({"t": 3, "order": ["imu"] + [x for x in o if x != "imu"], "needed": {}})


def test_modes_b4_and_oracle():
    d = D.example()
    oracle = {t: 0.4 for t in range(1, 200)}
    for mode, kw in (("b4", {"z0": 0.7}), ("oracle", {"oracle": oracle})):
        core = ASCore(d, mode=mode, **kw)
        sim = SimFleet(d, core, seed=2, kappa_fn=lambda t: 0.4, monitor=True)
        recs = sim.run(150)
        if mode == "b4":
            assert all(r["Z"] == 0.7 for r in recs)
            assert all(abs(r["K"] - (0.7 * r["kappa_hat"] * d.T_W - sum(r["cB"]))) < 1e-12 for r in recs)
        else:
            assert all(abs(r["Z"] * r["kappa_hat"] - 0.4) < 1e-12 for r in recs)
            # with the true capacity in the budget the fleet never fails after the first steps
            assert sum(r["f_prev"] or 0 for r in recs[5:]) == 0


def test_load_oracle(tmp_path):
    from fleetdt.as2.service import load_oracle
    d = D.example()
    p = tmp_path / "k.jsonl"
    p.write_text('{"t": 3, "kappa": 0.5}\n{"t": 4, "kappa_true": 0.6}\n')
    assert load_oracle(str(p)) == {3: 0.5, 4: 0.6}
    c = tmp_path / "w.csv"
    c.write_text("t_s,kappa,busy,b_ext,n_exch\n0.0,0.4,0.5,0.1,10\n0.125,0.45,0.5,0.1,10\n")
    assert load_oracle(str(c), d, origin=d.start(7)) == {7: 0.4, 8: 0.45}
