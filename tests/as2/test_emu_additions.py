"""Additions for the emulator campaign: own airtime of Def. busy with group-addressed frames and the access
category of the records, the capacity of a slot (slotted variant), the point predictors of E2, the output
records of the acceptor, the table of B3 (sigma_t), and the statistics of Section VI-E."""
import math
import os
import struct
import sys

import numpy as np
import pytest

from fleetdt.as2 import dtp as D
from fleetdt.as2.airtime import Phy
from fleetdt.as2.core import ASCore
from fleetdt.as2.failure import Acceptor
from fleetdt.as2.monitor import MonitorStore

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "emu", "analysis"))
import stats as S  # noqa: E402


def us(x):
    return int(round(x * 1e6))


def test_own_includes_group_frames_of_fleet_and_ac_mask():
    d = D.example()
    m = MonitorStore(Phy(d.phy))
    t = 10
    q0, q1 = us(d.quiet(t)[0]), us(d.quiet(t)[1])
    busy = [[q0 + 1000, q0 + 1058], [q0 + 2000, q0 + 2058], [q0 + 3000, q0 + 3100], [q0 + 4000, q0 + 4500]]
    fr = [[q0 + 1000, q0 + 1058, 3, 0, 0, 6000, 1, 2, -1],      # CF-End of vessel 3: own
          [q0 + 2000, q0 + 2058, 0, 1, 0, 6000, 1, 2, -1],      # CF-End of the AP (group): own
          [q0 + 3000, q0 + 3100, -1, 0, 1400, 24000, 1, 0, 0],  # external BSS: not own
          [q0 + 4000, q0 + 4500, 2, 0, 1000, 24000, 0, 0, 0]]   # failed fleet frame: not own
    m.add({"from_us": q0 - 1000, "to_us": q1 + 1000, "busy": busy, "fr": fr})
    r = m.busy_fraction(q0, q1)
    assert r["T_own"] == 58 + 58
    assert math.isclose(r["b"], (100 + 500) / (q1 - q0))
    # the data bridges are the AC_BE data frames of the vessels once the records carry the AC
    w, we = us(d.window(t)[0]), us(d.window(t)[1])
    m2 = MonitorStore(Phy(d.phy))
    m2.add({"from_us": w - 100, "to_us": us(d.end(t)) + 1000, "busy": [],
            "fr": [[w + 100, w + 400, 1, 0, 1000, 24000, 1, 0, 0],
                   [w + 500, w + 700, 1, 0, 200, 24000, 1, 0, 3]]})     # control bridge (AC_VO)
    c = m2.capacity(w, we, us(d.end(t)))
    assert m2.has_ac and c["e"] == w + 400


def test_slot_capacity():
    d = D.example()
    m = MonitorStore(Phy(d.phy))
    t = 10
    w, end = us(d.window(t)[0]), us(d.end(t))
    a, b = w + 10000, w + 30000
    fr = [[a + 100, a + 600, 2, 0, 1000, 24000, 1, 0, 0], [a + 700, a + 900, 2, 1, 52, 24000, 1, 0, 0],
          [a + 1000, a + 1500, 3, 0, 1000, 24000, 1, 0, 0]]                 # vessel 3 outside the slot of 2
    m.add({"from_us": w - 1000, "to_us": end + 1000, "busy": [], "fr": fr})
    c = m.slot_capacity(a, b, end, 2)
    assert c["e"] == a + 900
    exp = (m.t_succ_rows(np.array([fr[0]])).sum() + m.t_succ_rows(np.array([fr[1]])).sum()) / (900e-6)
    assert math.isclose(c["kappa"], exp)
    # vessel 2 still sends after its slot: it held bytes, e = slot end
    m.add({"from_us": end + 1000, "to_us": end + 2000, "busy": [],
           "fr": [[b + 100, b + 600, 2, 0, 1000, 24000, 1, 0, 0]]})
    assert m.slot_capacity(a, b, end, 2)["e"] == b


@pytest.mark.parametrize("pred", ["persist", "es", "calmean"])
def test_predictors(pred):
    d = D.example()
    core = ASCore(d, predictor=pred, pred_gain=0.5, cal_kappa=0.6)
    close = {"kappa": 0.8, "e": 20000, "w": 0}
    assert core._kappa_hat(0.7, None) == (0.6 if pred != "eq" else 0.7)
    v = core._kappa_hat(0.7, close)
    assert math.isclose(v, {"persist": 0.8, "es": 0.7, "calmean": 0.6}[pred])
    # a closing without a valid busy period (e - w < T_min) keeps the prediction
    assert math.isclose(core._kappa_hat(0.7, {"kappa": 0.1, "e": 1000, "w": 0}), v)
    with pytest.raises(ValueError):
        ASCore(d, predictor="calmean")


def test_acceptor_out_records():
    d = D.example()
    got = []
    acc = Acceptor(d, out_log=got.append)
    lv = [tp.index for tp in d.topics if tp.output][0]
    t = 100
    for k in range(3):
        body = struct.pack("!IH", t, k) + struct.pack("!IHH", 77, k, 3) + b"x" * 100
        acc.on_message(d.topic_string(1, lv), body, d.end(t) - 0.01)
    assert len(got) == 1 and got[0]["complete"] and got[0]["o"] == 77 and got[0]["x"] == 3 * 108
    body = struct.pack("!IH", t, 9) + struct.pack("!IHH", 78, 0, 2) + b"x" * 10
    acc.on_message(d.topic_string(2, lv), body, d.end(t))
    acc.prune(t + 1)
    assert len(got) == 2 and not got[1]["complete"] and got[1]["got"] == 1


def test_b3_table_has_sigma_order():
    d = D.example()
    core = ASCore(d, mode="b3")
    t = 5
    for i in d.vessels:
        dd = [[[tp.n * tp.s, tp.n]] * (2 if tp.output else 1) for tp in d.topics]
        core.on_demand(i, {"t": t, "v": i, "d": dd, "held": [0] * d.L, "B": {}, "rep": None}, d.start(t))
    sigma = ["state", "gps", "imu", "mag", "power", "baro", "video"]
    core.on_order({"t": t, "order": sigma, "needed": {"1": ["state", "video"]}})
    tabs, rec = core.phase2(t)
    assert set(tabs) == set(d.vessels)
    for tb in tabs.values():
        assert [d.topics[l].name for l in tb["o"]] == sigma


def test_hodges_lehmann_interval_and_holm():
    cdf = S.signed_rank_cdf(12)
    assert abs(cdf[13] - 0.021240234375) < 1e-12 and cdf[14] > 0.025
    x = np.arange(12, dtype=float)
    est, (lo, hi) = S.hodges_lehmann(x + 1.0, x)
    assert est == 1.0 and lo == 1.0 and hi == 1.0
    p, n = S.wilcoxon(np.arange(1, 13) + 0.5, np.zeros(12))
    assert n == 12 and math.isclose(p, 2 ** -11)
    assert list(S.holm([0.01, 0.04, 0.03])) == [0.03, 0.06, 0.06]


def test_fleet_prepare_p1_spacing():
    import types
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    sys.path.insert(0, root)
    sys.path.insert(0, os.path.join(root, "emu"))
    import fleet
    R = types.SimpleNamespace(host_ip=lambda i: "10.10.0.%d" % (10 + i))
    line = [ln for ln in fleet.prepare_cmd(R, {"method": "fleetdt", "dtp": {}, "p1_spacing_us": 3200}, 2, 4).splitlines()
            if "gsconf vessel" in ln][0]
    assert line.endswith("--log-dest 'file /tmp/fdt-run/mosquitto.log' --p1-spacing-us 3200 > /tmp/fdt-run/m.conf")
    line = [ln for ln in fleet.prepare_cmd(R, {"method": "fleetdt", "dtp": {}}, 2, 4).splitlines()
            if "gsconf vessel" in ln][0]
    assert "p1-spacing" not in line and line.endswith("mosquitto.log' > /tmp/fdt-run/m.conf")
    line = [ln for ln in fleet.prepare_cmd(R, {"method": "fleetdt", "dtp": {}, "p1_guard_us": 4000}, 2, 4).splitlines()
            if "gsconf vessel" in ln][0]
    assert line.endswith("mosquitto.log' --p1-guard-us 4000 > /tmp/fdt-run/m.conf")
    from fleetdt.gsconf import vessel_conf
    assert "plugin_opt_p1_guard_us 4000" in vessel_conf(1, "d.json", p1_guard_us=4000)


def test_runner_max_k(tmp_path):
    import json as _json
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    sys.path.insert(0, os.path.join(root, "emu", "experiments"))
    import runner as RN
    runs = [{"id": "a", "M": 4, "status": "pending"}, {"id": "b", "M": 2, "status": "pending"},
            {"id": "c", "M": 4, "status": "pending", "max_K": 2}, {"id": "d", "M": 4, "status": "pending", "max_K": 1}]
    (tmp_path / "manifest.json").write_text(_json.dumps({"name": "t", "runs": runs}))
    man = RN.Manifest(str(tmp_path))
    r0, _ = man.next(4, 0)
    r1, _ = man.next(None, 1)
    assert (r0["id"], r1["id"]) == ("a", "b")
    assert man.next(4, 2) == (None, "exit")          # c and d may not run on slot 2
    assert man.next(4, 0) == (None, "wait")          # c (max_K 2) waits: two sessions run
    man.update(r1, status="done")
    r2, _ = man.next(4, 0)
    assert r2["id"] == "c"
    assert man.next(4, 1) == (None, "exit")          # d (max_K 1) may run on slot 0 only
    man.update(r0, status="done")
    assert man.next(4, 0) == (None, "wait")          # d waits for c
    man.update(r2, status="done")
    r3, _ = man.next(4, 0)
    assert r3["id"] == "d"
