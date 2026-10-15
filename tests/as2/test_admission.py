"""Algorithm 1, Lemma 1 (brute force), order pi_t, tables, B-bar, slotted variant."""
import itertools
import json
import math
import random

import pytest

from fleetdt.as2 import dtp as D
from fleetdt.as2.admission import (AdditiveBudget, Demand, HeldBytes, SlotBudget, Unit, algorithm1, arrival_units,
                                   build_units, ipact, slots, tables, unit_key)


def small_dtp(M):
    return D.example(vessels=list(range(1, M + 1)), topics=[
        {"name": "state", "rate": 8, "jitter": 0.002, "n": 1, "s": 48, "kind": "stream", "qos": 2},
        {"name": "imu", "rate": 32, "jitter": 0.002, "n": 1, "s": 48, "kind": "stream", "qos": 0},
        {"name": "video", "rate": 15, "jitter": 0.010, "n": 4, "s": 1400, "kind": "output", "qos": 0},
        {"name": "mag", "rate": 16, "jitter": 0.002, "n": 1, "s": 40, "kind": "stream", "qos": 0}],
        order=["state", "video", "imu", "mag"], needed=["state", "imu"])


def random_instance(rng):
    M = rng.choice([2, 3])
    d = small_dtp(M)
    t = rng.randint(1, 50)
    units = []
    for i in d.vessels:
        for l, tp in enumerate(d.topics):
            K = rng.randint(1, 3) if tp.output else 1
            if tp.output and rng.random() < 0.2:
                K = 0
            for k in range(1, K + 1):
                units.append(Unit(i, l, k, 0, 1, tp.qos, c=float(rng.randint(0, 9))))
    needed = {i: set(rng.sample(range(d.L), rng.randint(0, d.L))) | d.R for i in d.vessels}
    sigma = [0] + rng.sample([1, 2, 3], 3)
    rank = [0] * d.L
    for r, l in enumerate(sigma):
        rank[l] = r
    for u in units:
        u.inR = u.l in d.R
        u.key = unit_key(u, t, d, d.R, needed[u.i], rank)
    forced = sorted([u for u in units if u.inR], key=lambda u: u.key)
    cand = sorted([u for u in units if not u.inR], key=lambda u: u.key)
    K = float(rng.randint(0, 40))
    return d, forced, cand, K


def brute(forced, cand, fits):
    """Lexicographically largest set in the order pi_t among the sets that hold the forced units, hold a
    prefix of the outputs of each topic of each vessel, and fit; the forced units alone when none fits."""
    best = None
    ids = [u.uid for u in cand]
    for mask in itertools.product((1, 0), repeat=len(cand)):      # lexicographic order, largest first
        S = [u for u, b in zip(cand, mask) if b]
        sel = set(u.uid for u in S)
        if any(u.k > 1 and (u.i, u.l, u.k - 1) not in sel for u in S):
            continue
        if fits(forced, S):
            best = S
            break
    return forced + (best or [])


def test_lemma1_brute_force_additive():
    rng = random.Random(11)
    n = 0
    while n < 400:
        d, forced, cand, K = random_instance(rng)
        if len(cand) > 13:
            continue
        n += 1
        A = algorithm1(forced, cand, AdditiveBudget(K))
        B = brute(forced, cand, lambda F, S: sum(u.c for u in F) + sum(u.c for u in S) <= K)
        assert sorted(u.uid for u in A) == sorted(u.uid for u in B)
        # prefix rule
        s = set(u.uid for u in A)
        assert all(u.k == 1 or (u.i, u.l, u.k - 1) in s for u in A)
        if sum(u.c for u in forced) > K:
            assert len(A) == len(forced)


def test_lemma1_forced_exceed_budget():
    rng = random.Random(3)
    d, forced, cand, _ = random_instance(rng)
    for u in forced:
        u.c = 10.0
    A = algorithm1(forced, cand, AdditiveBudget(5.0))
    assert [u.uid for u in A] == [u.uid for u in forced]


def test_lemma1_brute_force_slotted():
    """Lemma 1 with the monotone time test of the slotted variant."""
    rng = random.Random(5)
    n = 0
    while n < 200:
        d, forced, cand, _ = random_instance(rng)
        if len(cand) > 12:
            continue
        n += 1
        guard = {i: rng.uniform(0.5, 2.0) for i in d.vessels}
        T_W, Zk, Zk2, cB = rng.uniform(5, 40), rng.uniform(0.3, 1.0), rng.uniform(0.5, 1.2), rng.uniform(0, 3)
        mk = lambda: SlotBudget(T_W, Zk, Zk2, cB, guard)

        def fits(F, S):
            b = mk()
            for u in F:
                b.force(u)
            ci = {}
            for u in S:
                ci[u.i] = ci.get(u.i, 0.0) + u.c
            return b.time(ci) <= T_W
        A = algorithm1(forced, cand, mk())
        B = brute(forced, cand, fits)
        assert sorted(u.uid for u in A) == sorted(u.uid for u in B)


def test_order_interleaves_outputs():
    """Among vessels with the same need, output k of every vessel precedes output k+1 of any vessel, and
    the vessels rotate by one position per step."""
    d = small_dtp(3)
    rank = [0, 2, 1, 3]
    needed = {i: set(d.needed) for i in d.vessels}
    for t in (1, 2, 3, 4):
        us = [Unit(i, 2, k, 0, 1, 0) for i in d.vessels for k in (1, 2, 3)]
        for u in us:
            u.key = unit_key(u, t, d, d.R, needed[u.i], rank)
        us.sort(key=lambda u: u.key)
        assert [u.k for u in us] == [1, 1, 1, 2, 2, 2, 3, 3, 3]
        first = us[0].i
        assert (first - t) % 3 == 0


def demand(d, i, t, held=(1, 0), rep=None, B=None):
    d_ = []
    for l, tp in enumerate(d.topics):
        if tp.output:
            d_.append([[tp.n * tp.s, tp.n]] * (held[0] + tp.outputs_bound(d.T1 + d.T2)))
        else:
            N = tp.outputs_bound(d.T1 + d.T2) + 1
            d_.append([[N * tp.s, N]])
    return {"t": t, "v": i, "d": d_, "held": [0] * d.L, "B": B or {"x0": 0, "n0": 0, "x2": 0, "n2": 0}, "rep": rep}


def test_build_units_tables_absent_vessel():
    d = small_dtp(3)
    rank = [0, 2, 1, 3]
    dem = {i: Demand.parse(demand(d, i, 5), d) for i in (1, 2)}     # vessel 3 outside M(t)
    needed = {1: {0, 2}, 2: {0}}
    cost = lambda u: 1.0
    forced, cand = build_units(d, 5, dem, {1, 2, 3}, needed, rank, cost)
    # forced: state of 1, 2 (demand) and of 3 (arrival bound over Ts: ceil(8 (0.125 + 0.002)) = 2 messages)
    assert sorted((u.i, u.l) for u in forced) == [(1, 0), (2, 0), (3, 0)]
    u3 = [u for u in forced if u.i == 3][0]
    assert (u3.x, u3.n) == (2 * 48, 2)
    assert all(u.i in (1, 2) for u in cand)
    # vessel 1 needs video: its video units precede the units of vessel 2 that it does not need
    first = cand[0]
    assert (first.i, first.l) == (1, 2)
    A = algorithm1(forced, cand, AdditiveBudget(7.0))
    tb = tables(d, 5, A, [1, 2], needed, rank)
    assert set(tb) == {1, 2}
    assert tb[1]["p"][0] == 2 and tb[1]["o"][0] == 0
    assert tb[1]["o"] == [0, 2, 1, 3] and tb[2]["o"] == [0, 2, 1, 3]
    # budget 7 at cost 1 per unit: 3 forced, the 2 video outputs needed by vessel 1, then the 2 video
    # outputs of vessel 2 (rank 1); imu (rank 2) does not fit
    assert tb[1]["k"] == [1, 0, 2, 0] and tb[2]["k"] == [1, 0, 2, 0]
    assert tb[1]["p"] == [2, 0, 2, 0] and tb[2]["p"] == [2, 0, 1, 0]
    json.dumps(tb)
    # a disconnected vessel has no forced unit
    forced2, _ = build_units(d, 5, dem, {1, 2}, needed, rank, cost)
    assert sorted(u.i for u in forced2) == [1, 2]


def test_held_bytes_recursion():
    d = small_dtp(2)
    h = HeldBytes(d)
    dem = {1: Demand.parse(demand(d, 1, 1, B={"x0": 100, "n0": 1, "x2": 0, "n2": 0}), d)}
    b = h.update(dem, {1, 2})
    assert b[1]["x0"] == 100 and b[2] == {"x0": 0, "n0": 0, "x2": 0, "n2": 0}
    h.admitted([Unit(2, 0, 1, 96, 2, 2), Unit(2, 2, 1, 5600, 4, 0)])
    b = h.update({}, {1, 2})
    assert b[2]["x2"] == 96 + 2 * (d.h_P(0) + d.h_A) and b[2]["n2"] == 2
    assert b[2]["x0"] == 5600 + 4 * d.h_P(2) and b[2]["n0"] == 4
    # cap at B_cap
    h.xadm[2] = {"x0": 2 * d.B_cap, "n0": 1000, "x2": 0, "n2": 0}
    b = h.update({}, {1, 2})
    assert b[2]["x0"] + b[2]["x2"] <= d.B_cap + 1e-6
    # disconnected: 0
    b = h.update({}, {1})
    assert b[2] == {"x0": 0, "n0": 0, "x2": 0, "n2": 0}


def test_slots_layout():
    d = small_dtp(3)
    guard = {i: 0.001 for i in d.vessels}
    b = SlotBudget(d.T_W, 0.5, 0.8, 0.002, guard)
    us = [Unit(1, 0, 1, 0, 1, 2, c=0.004), Unit(2, 0, 1, 0, 1, 2, c=0.004)]
    cand = [Unit(2, 1, 1, 0, 1, 0, c=0.008), Unit(1, 1, 1, 0, 1, 0, c=0.008), Unit(3, 1, 1, 0, 1, 0, c=0.008)]
    A = algorithm1(us, cand, b)
    sl = slots(d, 4, A, b, [1, 2, 3])
    w, we = d.window(4)
    shared = (0.008 + 0.002) / 0.5
    starts = sorted(s for s, _ in sl.values())
    assert abs(starts[0] - round((w + shared) * 1e6)) <= 1
    for i, (s, e) in sl.items():
        assert e - s == pytest.approx((0.008 / 0.8 + 0.001) * 1e6, abs=2)
        assert e <= round(we * 1e6)
    # slots do not overlap and follow the rotated order
    seq = sorted(sl.items(), key=lambda kv: kv[1][0])
    assert [i for i, _ in seq] == sorted(sl, key=lambda i: d.rot(i, 4))
    for (a, (s1, e1)), (b_, (s2, e2)) in zip(seq, seq[1:]):
        assert e1 <= s2 + 1


def test_ipact_limited_service():
    """B6: R forced; each vessel at most eta_one (W_max - guard) of cost outside R; prefix rule; windows in
    the rotated order, back to back from the end of the shared slot, each with its grant plus its guard."""
    d = small_dtp(3)
    t = 5
    guard = {i: 0.001 for i in d.vessels}
    eta_all, eta_one = 0.5, 0.8
    forced = [Unit(i, 0, 1, 0, 1, 2, c=0.002, inR=True) for i in d.vessels]
    cB = 0.004
    shared = (3 * 0.002 + cB) / eta_all
    win = (d.T_W - shared) / d.M
    G = eta_one * (win - 0.001)
    # vessel 1 asks for far more than its window; vessel 2 for less; vessel 3 for nothing outside R
    cand = [Unit(1, 2, k, 0, 1, 0, c=G / 3.5) for k in range(1, 7)] + [Unit(2, 1, 1, 0, 1, 0, c=G / 4)]
    for u in cand:
        u.key = (1, 1, u.l, u.k, d.rot(u.i, t))
    cand.sort(key=lambda u: u.key)
    A, sl, sh_end = ipact(d, t, forced, cand, cB, guard, eta_all, eta_one, d.vessels)
    got = {}
    for u in A:
        if not u.inR:
            got.setdefault(u.i, []).append(u.k)
    assert got[1] == [1, 2, 3] and got[2] == [1] and 3 not in got
    assert all(u in A for u in forced)
    w, we = d.window(t)
    assert sh_end == round((w + shared) * 1e6)
    assert set(sl) == {1, 2}
    seq = sorted(sl.items(), key=lambda kv: kv[1][0])
    assert [i for i, _ in seq] == sorted(sl, key=lambda i: d.rot(i, t))
    assert abs(seq[0][1][0] - sh_end) <= 1
    for (a, (s1, e1)), (b, (s2, e2)) in zip(seq, seq[1:]):
        assert abs(e1 - s2) <= 1
    assert sl[1][1] - sl[1][0] == pytest.approx((3 * G / 3.5 / eta_one + 0.001) * 1e6, abs=2)
    assert sl[2][1] - sl[2][0] == pytest.approx((G / 4 / eta_one + 0.001) * 1e6, abs=2)
    assert max(e for _, e in sl.values()) <= round(we * 1e6)
    # the grant of a vessel does not depend on the requests of the other vessels (limited service)
    A2, _, _ = ipact(d, t, forced, [u for u in cand if u.i == 1], cB, guard, eta_all, eta_one, d.vessels)
    assert sorted(u.k for u in A2 if u.i == 1 and not u.inR) == [1, 2, 3]
