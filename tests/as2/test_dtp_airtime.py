"""DTP v2 loader (SPEC Section 2) and airtime (eq. tsucc, eq. cost)."""
import copy
import math
import random

import pytest

from fleetdt.as2 import dtp as D
from fleetdt.as2.airtime import Cost, Phy, exact_airtime, segments


def test_spec_example_loads():
    d = D.example()
    assert d.L == 7 and d.M == 4 and d.epoch == D.DEFAULT_EPOCH
    assert d.required == [0] and d.order[0] == 0
    assert math.isclose(d.T3, 0.095) and math.isclose(d.T_W, 0.095 - 0.004 - 0.002)
    # h_P = 1 + RL bytes + 2 + len("boat/b4/tx/<name>") + (2 if qos) + 6
    assert d.h_P(0) == 1 + 1 + 2 + len("boat/b4/tx/state") + 2 + 6        # RL = 74 < 128
    assert d.h_P(6) == 1 + 2 + 2 + len("boat/b4/tx/video") + 0 + 6        # RL = 1424 >= 128
    # the largest vessel index sets the topic length
    d2 = D.example(vessels=[1, 12])
    assert d2.h_P(1) == d.h_P(1) + 1
    # step clock: step t >= 1 is [epoch + (t-1) Ts, epoch + t Ts)
    assert d.step_of(d.epoch) == 1 and d.step_of(d.epoch + 0.125) == 2
    w, we = d.window(3)
    assert math.isclose(w, d.start(3) + 0.020 + 0.010 + 0.002) and math.isclose(we, d.end(3) - 0.004)
    q0, q1 = d.quiet(3)
    assert math.isclose(q0, d.window(2)[1]) and math.isclose(q1, d.phase2(3))
    assert d.demand_open(5, d.start(5) - 0.0019) and not d.demand_open(5, d.start(5) - 0.0021)
    assert not d.demand_open(5, d.phase2(5))
    assert d.rot(1, 1) == 0 and d.rot(4, 1) == 3 and d.rot(1, 2) == 3


@pytest.mark.parametrize("change,msg", [
    (lambda x: x["topics"][1].update(qos=2), "qos"),
    (lambda x: x.update(order=["imu", "state", "gps", "video", "baro", "power", "mag"]), "start with"),
    (lambda x: x.update(needed=["imu"]), "needed"),
    (lambda x: x.update(T1=0.12), "T3"),
    (lambda x: x["topics"][0].update(kind="blob"), "kind"),
    (lambda x: x["phy"].update(rate_init=11e6), "OFDM"),
])
def test_dtp_validation(change, msg):
    d = copy.deepcopy(D.SPEC_EXAMPLE)
    change(d)
    with pytest.raises(D.DTPError, match=msg):
        D.DTP.from_dict(d)


def test_erp_ofdm_timing():
    p = Phy(D.SPEC_EXAMPLE["phy"])
    # control response rate from the basic rate set {6, 12, 24}
    assert [p.ctrl_rate(r * 1e6) / 1e6 for r in (6, 9, 12, 18, 24, 36, 48, 54)] == [6, 6, 12, 12, 24, 24, 24, 24]
    # ACK at 24 Mbit/s: 16 + 4 + 4 ceil((22 + 112)/96) + 6 = 34 us; at 6: 16 + 4 + 4*6 + 6 = 50 us
    assert math.isclose(p.t_ack(54e6), 34e-6) and math.isclose(p.t_ack(9e6), 50e-6)
    # 1500-byte packet at 54 Mbit/s: 38 + 1500 bytes -> ceil((22 + 12304)/216) = 58 symbols
    assert math.isclose(p.t_ppdu(38 + 1500, 54e6), 16e-6 + 4e-6 + 58 * 4e-6 + 6e-6)
    # eq. (tsucc): exact <= bound <= exact + T_SYM, AIFS of AC_BE = SIFS + 3 slots
    assert math.isclose(p.aifs(), 37e-6)
    for R in (6e6, 9e6, 12e6, 18e6, 24e6, 36e6, 48e6, 54e6):
        for s in range(0, 2000, 7):
            ex, b = p.t_succ(s, R), p.t_succ_bound(s, R)
            assert ex <= b + 1e-15 and b - ex <= p.t_sym + 1e-12


def test_cost_bounds_frame_by_frame():
    """eq. (cost) >= the exact airtime of every frame of n messages, for random segmentations, rates
    and both QoS levels, with N_seg = min{n + (x + n h_P)/MSS, n ceil((s_l + h_P)/MSS)} (Section V-C);
    the first term alone counts about 2n segments for messages near MSS."""
    d = D.example()
    phy = Phy(d.phy)
    cost = Cost.from_dtp(d)
    rng = random.Random(7)
    rates = [6e6, 9e6, 12e6, 18e6, 24e6, 36e6, 48e6, 54e6]
    worst = 0.0
    for _ in range(3000):
        tp = rng.choice(d.topics)
        n = rng.randint(1, 24)
        contents = [rng.randint(1, tp.s) for _ in range(n)]
        Ri, R0 = rng.choice(rates), rng.choice(rates)
        cuts = [rng.random() < 0.5 for _ in range(n)] if rng.random() < 0.7 else None
        acks = rng.choice([1.0, 0.5])
        ex = exact_airtime(phy, contents, tp.qos, tp.h_P, d.mss, d.h_T, d.h_A, Ri, R0, cuts, acks)
        a1 = cost.a(sum(contents), n, tp.qos, Ri, R0, tp.h_P)
        a = cost.a(sum(contents), n, tp.qos, Ri, R0, tp.h_P, tp.s)
        assert ex <= a + 1e-12 and a <= a1 + 1e-15, (tp.name, n, Ri, R0)
        if cuts is None and acks == 1.0:
            worst = max(worst, a / ex)
    assert worst < 1.55
    print('\nmax a_i / exact (idle connection): %.3f' % worst)


def test_segments():
    assert segments([100, 100], 1448) == [100, 100]
    assert segments([100, 100], 1448, [False, True]) == [200]
    assert segments([1500], 1448) == [1448, 52]
    s = segments([700] * 5, 1448, [False] * 5)
    assert s == [1448, 1448, 604]
    # every segment is full or ends at a write boundary: at most n + total/MSS segments
    rng = random.Random(1)
    for _ in range(500):
        sz = [rng.randint(1, 3000) for _ in range(rng.randint(1, 10))]
        cuts = [rng.random() < 0.5 for _ in sz]
        assert len(segments(sz, 1448, cuts)) <= len(sz) + sum(sz) / 1448


def test_held_cost():
    d = D.example()
    cost = Cost.from_dtp(d)
    R = 24e6
    assert cost.held({"x0": 0, "n0": 0, "x2": 0, "n2": 0}, R, R) == 0
    c0 = cost.held({"x0": 3000, "n0": 2, "x2": 0, "n2": 0}, R, R)
    assert math.isclose(c0, cost.a(3000, 2, 0, R, R, 0))
    c2 = cost.held({"x0": 0, "n0": 0, "x2": 100, "n2": 2}, R, R, n_pubrel=3)
    assert math.isclose(c2, cost.a(100, 2, 2, R, R, 0) + 3 * cost.ctl(R, R))


def test_gsconf_spec5():
    from fleetdt.gsconf import gs_conf, vessel_conf
    c = gs_conf({1: "10.10.0.11"}, data_qos=2)    # the as2 stack keeps the QoS 2 data bridge
    assert "max_inflight_messages 0" in c and "set_tcp_nodelay true" in c
    assert "connection b1-data\naddress 10.10.0.11:1884" in c and "topic boat/b1/tx/# in 2" in c
    c2 = gs_conf({1: "10.10.0.11"}, b2=True, data_qos=2)
    for cls, port in (("be", 1884), ("af41", 1885), ("ef", 1886)):
        assert "connection b1-data-%s\naddress 10.10.0.11:%d" % (cls, port) in c2
        assert "topic # in 2 boat/b1/tx/ boat/b1/tx2/%s/" % cls in c2
    assert "connection b1-data\n" not in c2
    v = vessel_conf(1, "/etc/fleetdt/dtp.json", b2=True)
    assert "listener 1885" in v and "listener 1886" in v
