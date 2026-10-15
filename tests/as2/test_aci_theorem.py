"""eq. (aci), its projection, Theorem 1 on adversarial and random f sequences, and Theorem 1 driven by
the AS loop (ASCore with a synthetic fleet, random kappa(t) and adversaries)."""
import math
import random

import pytest

from fleetdt.as2 import dtp as D
from fleetdt.as2.aci import ACI, project, theorem_check
from fleetdt.as2.core import ASCore
from fleetdt.as2.failure import Acceptor, indicator
from fleetdt.as2.predictor import Scores

from .fleet import SimFleet

BETA, GAMMA = 0.05, 0.01


def run_seq(f, beta=BETA, gamma=GAMMA, alpha1=0.5):
    a = ACI(beta, gamma, alpha1)
    alphas = []
    for x in f:
        alphas.append(a.alpha)
        a.update(x)
    return a, alphas


def check_all_prefixes(f, alphas, beta, gamma, alpha1):
    for T, (l, r, _) in enumerate(theorem_check(f, alphas, beta, gamma, alpha1), start=1):
        assert l <= r + 1e-12, (T, l, r)


@pytest.mark.parametrize("pattern", ["all1", "all0", "alt", "burst", "random", "floor_attack", "pos_attack"])
def test_theorem1_sequences(pattern):
    rng = random.Random(hash(pattern) % 1000)
    T = 5000
    a = ACI(BETA, GAMMA, 0.5)
    f, alphas = [], []
    for t in range(T):
        if pattern == "all1":
            x = 1
        elif pattern == "all0":
            x = 0
        elif pattern == "alt":
            x = t % 2
        elif pattern == "burst":
            x = 1 if (t // 300) % 2 == 0 else 0
        elif pattern == "random":
            x = 1 if rng.random() < 0.3 else 0
        elif pattern == "floor_attack":     # fail whenever alpha <= 0: maximizes F_R
            x = 1 if a.alpha <= 0 else (1 if rng.random() < 0.1 else 0)
        else:                               # fail whenever alpha > 0
            x = 1 if a.alpha > 0 else 0
        alphas.append(a.alpha)
        f.append(x)
        a.update(x)
        assert -GAMMA * (1 - BETA) - 1e-15 <= a.alpha <= 1
    check_all_prefixes(f, alphas, BETA, GAMMA, 0.5)
    assert sum(f) / T <= a.bound() + 1e-12
    # the identity of the proof: mean f = beta + (alpha_1 - alpha_{T+1} + sum D)/(gamma T)
    assert math.isclose(sum(f) / T, BETA + (0.5 - a.alpha + a.sum_D) / (GAMMA * T), abs_tol=1e-9)
    if pattern == "pos_attack":
        # F_R = 0 (the adversary never fails at alpha <= 0) and f-bar tends to beta
        assert a.FR == 0 and abs(sum(f) / T - BETA) < 0.5 / (GAMMA * T) + 2 / T


def test_projection_recovery():
    """After any run of failures the level sits at -gamma(1-beta) and returns above 0 after at most
    floor((1-beta)/beta) + 1 steps without failure; the level never exceeds 1."""
    for beta, gamma in [(0.05, 0.01), (0.1, 0.05), (0.2, 0.1), (0.01, 0.02)]:
        need = math.floor((1 - beta) / beta + 1e-9) + 1
        for fails in (1, 10, 100, 10000):
            a = ACI(beta, gamma, 1.0)
            for _ in range(fails):
                a.update(1)
            assert a.alpha >= a.floor - 1e-15
            if fails >= 1 / (gamma * (1 - beta)) + 1:
                assert math.isclose(a.alpha, a.floor)
            k = 0
            while a.alpha <= 0:
                a.update(0)
                k += 1
            assert k <= need
        a = ACI(beta, gamma, 0.0)
        for _ in range(100000):
            a.update(0)
        assert a.alpha == 1.0
        # without the projection the level after N failures needs N (1-beta)/beta steps
        assert project(-5.0, gamma, beta) == -gamma * (1 - beta)


def test_scores_Z():
    s = Scores([], empty=1.0)
    assert s.Z(0.5) == 1.0 and s.Z(0.0) == 0.0 and s.Z(-0.1) == 0.0
    for z in [0.9, 0.5, 1.2, 0.7]:
        s.add(z)
    assert [s.Z(a) for a in (0.01, 0.25, 0.26, 0.5, 0.75, 1.0)] == [0.5, 0.5, 0.7, 0.7, 0.9, 1.2]
    prev = 0
    for a in [x / 100 for x in range(-10, 101)]:
        assert s.Z(a) >= prev
        prev = s.Z(a)
    w = Scores([0.1], window=2)
    for z in (0.5, 0.6, 0.7):
        w.add(z)
    assert w.sorted == [0.1, 0.6, 0.7]


def make_core(M=4, **kw):
    d = D.example(vessels=list(range(1, M + 1)))
    return d, ASCore(d, **kw)


@pytest.mark.parametrize("scenario", ["random", "always_fail", "fail_at_floor", "fail_above_zero", "lossy"])
def test_theorem1_driven_by_AS_loop(scenario):
    """The AS loop (demands, reports, accepted messages, monitor batches -> f(t), alpha_t, Z, K, A(t))
    satisfies eq. (thm) at every prefix; f(t) of the AS equals the miss injected by the fleet."""
    d, core = make_core(4)
    rng = random.Random(len(scenario))
    adv = None
    kappa = lambda t: rng.uniform(0.2, 0.9)
    lose_d = lose_t = 0.0
    if scenario == "always_fail":
        adv = lambda t, c: True
    elif scenario == "fail_at_floor":
        adv = lambda t, c: c.prev.alpha <= 0
    elif scenario == "fail_above_zero":
        adv = lambda t, c: c.prev.alpha > 0
    elif scenario == "lossy":
        lose_d, lose_t = 0.03, 0.03
    sim = SimFleet(d, core, seed=3, kappa_fn=kappa, adversary=adv, monitor=True, lose_demand=lose_d,
                   lose_table=lose_t)
    T = 1500
    recs = sim.run(T)
    f, alphas = core.f_hist, core.alpha_hist
    assert len(f) == T - 1 and alphas[0] == d.alpha1
    check_all_prefixes(f, alphas, d.beta, d.gamma, d.alpha1)
    # the records carry f(t-1), alpha_t and F_R
    assert [r["f_prev"] for r in recs[1:]] == f
    assert [r["alpha"] for r in recs] == alphas + [core.aci.alpha]
    if scenario != "lossy":
        assert f == sim.f_true[:-1]
    if scenario == "always_fail":
        assert sum(f) == T - 1 and recs[-1]["Z"] == 0.0
        # at alpha <= 0 the AS admits the units of R alone
        assert all(all(u[1] in d.R for u in r["A"]) for r in recs if r["alpha"] <= 0)
    if scenario == "fail_above_zero":
        assert core.aci.FR == 0
    if scenario == "random":
        assert sum(f) / len(f) < 0.2


def test_failure_indicator_cases():
    d = D.example()
    acc = Acceptor(d)
    t = 9
    from .fleet import tx_payload
    for seq in range(2):
        acc.on_message(d.topic_string(1, 0), tx_payload(t, seq, 48), d.end(t) - 0.01)
    acc.on_message(d.topic_string(1, 0), tx_payload(t, 1, 48), d.end(t) - 0.005)    # repeated
    acc.on_message(d.topic_string(2, 0), tx_payload(t, 0, 48), d.end(t) - 0.01)
    acc.on_message(d.topic_string(2, 0), tx_payload(t, 1, 48), d.end(t) + 0.001)    # late
    a = acc.step(t)
    assert a[1][0].n_in == 2 and a[2][0].n_in == 1
    assert acc.get(1, 0, t).n_dup == 1 and acc.get(2, 0, t).n_late == 1
    rel = [[2]] + [[] for _ in range(d.L - 1)]
    rep = lambda **k: dict({"t": t, "rel": rel, "disc": 0, "tab": True}, **k)
    assert indicator(t, [1], {1: rep()}, a, d.L).f == 0
    assert indicator(t, [1, 2], {1: rep(), 2: rep()}, a, d.L).reasons == [(2, "miss", 0, 2, 1)]
    assert indicator(t, [1], {1: rep(disc=1)}, a, d.L).f == 1
    assert indicator(t, [1], {1: rep(tab=False)}, a, d.L).f == 1
    assert indicator(t, [1, 3], {1: rep()}, a, d.L).reasons == [(3, "noreport")]
    assert indicator(t, [1], {1: rep(t=t - 1)}, a, d.L).reasons == [(1, "noreport")]
    # tab = false of a vessel to which no table was sent (outside M(t))
    r3 = {"t": t, "rel": [[] for _ in range(d.L)], "disc": 0, "tab": False}
    assert indicator(t, [1], {1: rep(), 3: r3}, a, d.L).f == 1
    assert indicator(t, [1], {1: rep(), 3: r3}, a, d.L, count_tab_unsent=False).f == 0
