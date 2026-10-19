"""Model-level check of the guarantee of the backlog-aware grant (eq. 3 with Q(t), Section 2 of SPEC.md).

Pure Python, no broker: M vessels with FIFO send queues (bytes of the channel), a phase-3 service of
K(t) - B_ctl(t) bytes per step drawn at random and shared by the queues in a work-conserving order (a fluid
server: any order that leaves no byte waiting while service remains), demand tables with many small topics,
vessels that miss phase 1 at random (no p(t): they forward nothing new, their queued bytes still drain), and the
AS of fleetdt.as_model: BacklogBook, grant_cap and Algorithm 1 (admission).

Guarantee: in every step with U K_hat(t) <= K(t) - B_ctl(t) and Q(t) <= U K_hat(t), every byte in the queues at the
start of phase 3 leaves within the step. The bytes at the start of phase 3 are the backlog of the vessels of M(t)
(Q_i(t)), the backlog of the vessels outside M(t) (at most Qbar_i(t)) and the admitted bytes (at most
m(t) b(t) = U K_hat(t) - Q(t)), so they sum to at most U K_hat(t). The rule that counts only the vessels of M(t)
misses the backlog of the others and breaks the guarantee in some step of the same seeded run.
"""
import math
import random

from fleetdt.as_model import BacklogBook, admission, grant_cap

M, L, U = 4, 12, 0.95
ORDER = list(range(1, L + 1))
NEEDED = [1, 2, 3]


def simulate(steps, seed, count_outside=True):
    """Returns (condition steps, violations, steps with a vessel outside M(t) and a nonzero queue)."""
    rng = random.Random(seed)
    book = BacklogBook()
    q = {i: 0.0 for i in range(1, M + 1)}       # bytes queued at the end of phase 3 (FIFO per vessel)
    K_prev = 100_000.0
    cond = viol = outside_busy = 0
    for t in range(steps):
        K = max(30_000.0, K_prev * rng.lognormvariate(0.0, 0.2))   # capacity of phase 3, bytes
        K_hat = K_prev                                               # persistence prediction
        B_ctl = rng.uniform(0.0, 3000.0)                             # control bytes outside the data bridges
        S = K - B_ctl                                                # service of the data bridges in phase 3
        K_prev = K
        Mt = sorted(i for i in q if rng.random() >= 0.25)            # the others miss phase 1
        Qi = {i: int(math.ceil(q[i] - 1e-9)) for i in Mt}              # Q_i(t): ceil of the bytes still queued
        D = {i: [rng.randint(400, 6000) for _ in range(L)] for i in Mt}
        Q_M, Q_out, _ = book.split(Mt, Qi)
        Q = Q_M + Q_out if count_outside else Q_M
        a = {}
        if Mt:
            b = grant_cap(K_hat, U, 1.0, 1.0, Q) / len(Mt)            # T3 = Ts: K_hat in bytes of phase 3
            p = admission(D, ORDER, NEEDED, b)
            a = {i: sum(D[i][l - 1] for l in ORDER if p[l - 1] > 0) for i in Mt}
            assert all(a[i] <= b + 1e-9 for i in Mt)
            book.record(t, Mt, Qi, a)
        for i in Mt:
            q[i] += a[i]
        if any(q[i] > 0 for i in q if i not in Mt):
            outside_busy += 1
        total = sum(q.values())                                      # bytes at the start of phase 3
        if U * K_hat <= S and Q <= U * K_hat:
            cond += 1
            if total > S + 1e-6:
                viol += 1
        left = S                                                     # fluid server, random work-conserving order
        order = list(q)
        rng.shuffle(order)
        for i in order:
            x = min(q[i], left)
            q[i] -= x
            left -= x
    return cond, viol, outside_busy


def test_backlog_aware_grant_keeps_the_guarantee():
    for seed in (20261019, 1, 2, 3, 4):
        cond, viol, busy = simulate(4000, seed=seed)
        assert cond > 1000 and busy > 5, seed
        assert viol == 0, seed


def test_rule_without_the_vessels_outside_M_breaks_it():
    cond, viol, busy = simulate(4000, seed=20261019, count_outside=False)
    assert cond > 1000 and busy > 10
    assert viol >= 1
