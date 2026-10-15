"""Monitor computations (Defs. busy and capacity, Section V-B) on synthetic batches."""
import math

import numpy as np
import pytest

from fleetdt.as2 import dtp as D
from fleetdt.as2.airtime import Phy
from fleetdt.as2.monitor import MonitorStore, intersect, measure, union


def us(x):
    return int(round(x * 1e6))


def test_intervals():
    u = union(np.array([[5, 10], [0, 3], [2, 4], [10, 12], [20, 21], [11, 11]]))
    assert u.tolist() == [[0, 4], [5, 12], [20, 21]]
    v = union(np.array([[3, 6], [11, 25]]))
    assert intersect(u, v).tolist() == [[3, 4], [5, 6], [11, 12], [20, 21]]
    assert measure(u) == 4 + 7 + 1
    rng = np.random.default_rng(1)
    for _ in range(200):
        a = rng.integers(0, 100, size=(10, 1))
        iv = np.hstack([a, a + rng.integers(0, 15, size=(10, 1))])
        mask = np.zeros(130, bool)
        for s, e in iv:
            mask[s:e] = True
        assert measure(union(iv)) == mask.sum()


@pytest.fixture
def setup():
    d = D.example()
    return d, MonitorStore(Phy(d.phy))


def test_busy_fraction(setup):
    d, m = setup
    t = 10
    q0, q1 = us(d.quiet(t)[0]), us(d.quiet(t)[1])
    Q = q1 - q0
    # external busy 3000 us; own exchange 1000 us (frame 600 + SIFS gap 16 + ACK 384: busy except SIFS);
    # beacon 500 us; a failed fleet frame 800 us (counts as external: collision of the fleet)
    own = [q0 + 5000, q0 + 6000]
    busy = [[q0 + 100, q0 + 3100], [own[0], own[0] + 600], [own[0] + 616, own[1]],
            [q0 + 8000, q0 + 8500], [q0 + 10000, q0 + 10800]]
    fr = [[own[0], own[1], 2, 0, 1000, 24000, 1, 0],
          [q0 + 8000, q0 + 8500, 0, 1, 0, 1000, 1, 1],
          [q0 + 10000, q0 + 10800, 3, 0, 1000, 24000, 0, 0]]
    m.add({"from_us": q0 - 1000, "to_us": q0 + 7000, "busy": busy[:3], "fr": fr[:1]})
    m.add({"from_us": q0 + 7000, "to_us": q1 + 1000, "busy": busy[3:], "fr": fr[1:]})
    r = m.busy_fraction(q0, q1)
    assert r["cover"] == 1.0
    assert r["T_own"] == 984 + 500
    assert math.isclose(r["b"], (3000 + 800) / Q)
    # half of Q covered: b over the covered part; below min_cover: no b
    m2 = MonitorStore(m.phy)
    m2.add({"from_us": q0, "to_us": q0 + Q // 2, "busy": [[q0, q0 + 1200]], "fr": []})
    assert math.isclose(m2.busy_fraction(q0, q1)["b"], 1200 / (Q // 2))
    assert m2.busy_fraction(q0, q1, min_cover=0.6)["b"] is None


def test_capacity_cases(setup):
    d, m = setup
    phy = m.phy
    t = 7
    w, we = us(d.window(t)[0]), us(d.window(t)[1])
    end = us(d.end(t))
    ts_up = phy.t_succ(1528, 24e6)
    ts_dn = phy.t_succ(80, 24e6, phy.aifsn_ap)
    dur = 300
    fr = []
    x = w + 1000
    for k in range(20):
        fr.append([x, x + dur, 1 + k % 2, 0, 1528, 24000, 1, 0])
        fr.append([x + dur + 40, x + dur + 140, 1 + k % 2, 1, 80, 24000, 1, 0])
        x += 1000
    last_end = fr[-1][1]
    fr.append([x, x + dur, 3, 0, 1528, 24000, 0, 0])            # failed, after the last success: outside [w, e)
    fr.append([x + 400, x + 500, -1, 0, 500, 24000, 1, 0])     # outside the fleet
    fr.append([x + 600, x + 700, 2, 0, 0, 6000, 1, 2])         # management frame of a vessel
    m.add({"from_us": us(d.start(t)), "to_us": end + 10, "busy": [], "fr": fr})
    # no data bridge holds bytes: e = end of the last successful exchange in W
    r = m.capacity(w, we, end)
    assert r["e"] == last_end and r["n"] == 2
    assert math.isclose(r["T_fleet"], 20 * (ts_up + ts_dn), rel_tol=1e-9)
    assert math.isclose(r["kappa"], 20 * (ts_up + ts_dn) / ((last_end - w) * 1e-6))
    # vessel 2 reports B_i(t+1) > 0: e = end of W
    r2 = m.capacity(w, we, end, held=[2])
    assert r2["e"] == we and r2["hold"] == [2]
    assert math.isclose(r2["kappa"], 20 * (ts_up + ts_dn) / ((we - w) * 1e-6))
    # an uplink frame of a data bridge after the end of W: e = end of W
    m.add({"from_us": end + 10, "to_us": end + 20, "busy": [], "fr": [[we + 100, we + 400, 1, 0, 1528, 24000, 1, 0]]})
    assert m.capacity(w, we, end)["hold"] == [1]
    # rates of the last successful frames
    assert m.rates(1, 6e6) == (24e6, 24e6) and m.rates(4, 6e6) == (6e6, 6e6)


def test_capacity_no_frames_and_missing(setup):
    d, m = setup
    t = 3
    w, we = us(d.window(t)[0]), us(d.window(t)[1])
    end = us(d.end(t))
    assert m.capacity(w, we, end)["e"] is None            # no batch: frames of W(t) missing
    m.add({"from_us": us(d.start(t)), "to_us": end, "busy": [], "fr": []})
    r = m.capacity(w, we, end)
    assert r["e"] == w and r["kappa"] is None             # no exchange in W: e = w, no kappa


def test_straddling_frame_prorated(setup):
    d, m = setup
    t = 4
    w, we = us(d.window(t)[0]), us(d.window(t)[1])
    end = us(d.end(t))
    m.add({"from_us": us(d.start(t)), "to_us": end, "busy": [],
           "fr": [[w - 100, w + 100, 1, 0, 1000, 24000, 1, 0], [w + 1000, w + 1400, 1, 0, 1000, 24000, 1, 0]]})
    r = m.capacity(w, we, end)
    ts = m.phy.t_succ(1000, 24e6)
    assert r["e"] == w + 1400
    assert math.isclose(r["T_fleet"], 1.5 * ts)
