"""Admission control of phase 2 (Section IV-D): units, order pi_t (Def. grant), Algorithm 1 with the
units of R forced and the prefix rule, tables, held bytes B-bar, and the slotted variant (Def. tdma).

Topic indices l are 0-based (DTP order); output indices k are 1-based.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple


@dataclass
class Unit:
    i: int
    l: int
    k: int
    x: float
    n: float
    q: int
    c: float = 0.0
    inR: bool = False
    key: Tuple = ()

    @property
    def uid(self) -> Tuple[int, int, int]:
        return (self.i, self.l, self.k)


@dataclass
class Demand:
    """Parsed demand of SPEC Section 4.1."""
    t: int
    v: int
    d: List[List[Tuple[int, int]]]
    held: List[int]
    B: Dict[str, int]
    rep: Optional[Dict]
    late_us: Optional[int] = None
    at: Optional[float] = None

    @staticmethod
    def parse(obj: Dict, dtp, at: Optional[float] = None) -> "Demand":
        t, v = int(obj["t"]), int(obj["v"])
        d = obj["d"]
        if not isinstance(d, list) or len(d) != dtp.L:
            raise ValueError("demand of vessel %d: d has %s topics, the DTP %d" % (v, len(d) if isinstance(d, list) else "no", dtp.L))
        dd: List[List[Tuple[int, int]]] = []
        for l, pairs in enumerate(d):
            ps = []
            for p in pairs:
                x, n = int(p[0]), int(p[1])
                if x < 0 or n < 0:
                    raise ValueError("negative demand")
                ps.append((x, n))
            if not dtp.topics[l].output and len(ps) > 1:
                raise ValueError("stream topic %s with %d units" % (dtp.topics[l].name, len(ps)))
            dd.append(ps)
        held = [int(h) for h in obj.get("held", [0] * dtp.L)]
        B = {k: int(obj.get("B", {}).get(k, 0)) for k in ("x0", "n0", "x2", "n2")}
        rep = obj.get("rep")
        if rep is not None:
            rep = {"t": int(rep["t"]), "rel": [[int(c) for c in r] for r in rep.get("rel", [])],
                   "disc": int(rep.get("disc", 0)), "tab": bool(rep.get("tab", False))}
        return Demand(t=t, v=v, d=dd, held=held, B=B, rep=rep, late_us=obj.get("late_us"), at=at)


# ---------------------------------------------------------------------- order pi_t
def topic_key(l: int, R: Set[int], N: Set[int], rank: Sequence[int]) -> Tuple[int, int, int]:
    return (0 if l in R else 1, 0 if l in N else 1, rank[l])


def unit_key(u: Unit, t: int, dtp, R: Set[int], N: Set[int], rank: Sequence[int]) -> Tuple:
    """(1[l not in R], 1[l not in N_i(t)], rank_sigma(l), k, (i - t) mod M)."""
    return topic_key(u.l, R, N, rank) + (u.k, dtp.rot(u.i, t))


def arrival_units(dtp, i: int, topics: Iterable[int], duration: float) -> List[Unit]:
    """Units of the given topics with the arrival-curve bound over `duration` as the demand."""
    out = []
    for l in topics:
        tp = dtp.topics[l]
        N = tp.outputs_bound(duration)
        if tp.output:
            out += [Unit(i, l, k, tp.n * tp.s, tp.n, tp.qos) for k in range(1, N + 1)]
        else:
            out.append(Unit(i, l, 1, N * tp.n * tp.s, N * tp.n, tp.qos))
    return out


def demand_units(dtp, dem: Demand) -> List[Unit]:
    out = []
    for l, pairs in enumerate(dem.d):
        q = dtp.topics[l].qos
        for k, (x, n) in enumerate(pairs, start=1):
            out.append(Unit(dem.v, l, k, x, n, q))
    return out


def build_units(dtp, t: int, demands: Dict[int, Demand], connected: Set[int], needed: Dict[int, Set[int]],
                rank: Sequence[int], cost: Callable[[Unit], float]):
    """Returns (forced, candidates in the order pi_t). forced: the units of the topics of R of the
    vessels with a connected data bridge, with the arrival-curve bound over Ts for the vessels outside
    M(t). candidates: the other units of the vessels of M(t)."""
    R = dtp.R
    forced: List[Unit] = []
    cand: List[Unit] = []
    for i in dtp.vessels:
        N = needed.get(i, set(dtp.needed)) | R
        if i in demands:
            us = demand_units(dtp, demands[i])
        elif i in connected:
            us = arrival_units(dtp, i, sorted(R), dtp.Ts)
        else:
            continue
        for u in us:
            u.inR = u.l in R
            u.c = cost(u)
            u.key = unit_key(u, t, dtp, R, N, rank)
            if u.inR and i in connected:
                forced.append(u)
            elif i in demands:
                cand.append(u)
    cand.sort(key=lambda u: u.key)
    forced.sort(key=lambda u: u.key)
    return forced, cand


# ---------------------------------------------------------------------- budgets (monotone)
class AdditiveBudget:
    """Test of line (alg:test): c_A + c_u <= K."""

    def __init__(self, K: float):
        self.K, self.cA = K, 0.0

    def force(self, u: Unit) -> None:
        self.cA += u.c

    def can_add(self, u: Unit) -> bool:
        return self.cA + u.c <= self.K

    def add(self, u: Unit) -> None:
        self.cA += u.c


class SlotBudget:
    """Slotted variant (Def. tdma): the shared slot lasts (cost of R + held bytes)/(Z kappa_hat); the
    slot of vessel i lasts (cost of its admitted units outside R)/(Z kappa_hat_2) + 2 eps + T_big_i.
    A unit fits when the shared slot and the slots of the vessels fit in T_W. The time is monotone in
    the set of units, so Lemma 1 holds with this test in place of eq. (feas)."""

    def __init__(self, T_W: float, Zk: float, Zk2: float, cB: float, guard: Dict[int, float]):
        self.T_W, self.Zk, self.Zk2, self.cB, self.guard = T_W, Zk, Zk2, cB, guard
        self.cR = 0.0
        self.cA = 0.0
        self.ci: Dict[int, float] = {}

    def force(self, u: Unit) -> None:
        self.cR += u.c
        self.cA += u.c

    def shared(self) -> float:
        if self.Zk <= 0:
            return math.inf
        return (self.cR + self.cB) / self.Zk

    def time(self, ci: Dict[int, float]) -> float:
        if self.Zk2 <= 0 and ci:
            return math.inf
        return self.shared() + sum(c / self.Zk2 + self.guard[i] for i, c in ci.items())

    def can_add(self, u: Unit) -> bool:
        ci = dict(self.ci)
        ci[u.i] = ci.get(u.i, 0.0) + u.c
        return self.time(ci) <= self.T_W

    def add(self, u: Unit) -> None:
        self.ci[u.i] = self.ci.get(u.i, 0.0) + u.c
        self.cA += u.c


def algorithm1(forced: List[Unit], cand: List[Unit], budget) -> List[Unit]:
    """Algorithm 1: A <- forced; for each candidate in the order pi_t, admit it when it fits and it is
    the first output of its topic or the previous output of its topic and vessel is in A."""
    A = list(forced)
    adm = set(u.uid for u in forced)
    for u in forced:
        budget.force(u)
    for u in cand:
        if u.uid in adm:
            continue
        if (u.k == 1 or (u.i, u.l, u.k - 1) in adm) and budget.can_add(u):
            A.append(u)
            adm.add(u.uid)
            budget.add(u)
    return A


def tables(dtp, t: int, A: Iterable[Unit], vessels: Iterable[int], needed: Dict[int, Set[int]],
           rank: Sequence[int]) -> Dict[int, Dict]:
    """Table {"t","p","k","o"} of each vessel of M(t)."""
    R = dtp.R
    adm: Dict[int, Dict[int, int]] = {}
    for u in A:
        d = adm.setdefault(u.i, {})
        d[u.l] = d.get(u.l, 0) + 1
    out = {}
    for i in vessels:
        N = needed.get(i, set(dtp.needed)) | R
        k = [adm.get(i, {}).get(l, 0) for l in range(dtp.L)]
        p = [(2 if l in N else 1) if k[l] > 0 else 0 for l in range(dtp.L)]
        o = sorted(range(dtp.L), key=lambda l: topic_key(l, R, N, rank))
        out[i] = {"t": t, "p": p, "k": k, "o": o}
    return out


# ---------------------------------------------------------------------- held bytes B-bar
ZERO_B = {"x0": 0, "n0": 0, "x2": 0, "n2": 0}


def cap_B(B: Dict, cap: int) -> Dict:
    tot = B["x0"] + B["x2"]
    if tot <= cap:
        return dict(B)
    f = cap / tot
    return {"x0": B["x0"] * f, "n0": math.ceil(B["n0"] * f), "x2": B["x2"] * f, "n2": math.ceil(B["n2"] * f)}


class HeldBytes:
    """B-bar_i(t) = B_i(t) for i in M(t); min{B-bar_i(t-1) + x^adm_i(t-1), B_cap} for i outside M(t);
    0 while the data bridge of i is disconnected. x^adm: content and PUBLISH headers of the admitted
    units, plus the PUBREL bytes of the QoS 2 messages, split by QoS."""

    def __init__(self, dtp):
        self.dtp = dtp
        self.bbar: Dict[int, Dict] = {i: dict(ZERO_B) for i in dtp.vessels}
        self.xadm: Dict[int, Dict] = {i: dict(ZERO_B) for i in dtp.vessels}

    def update(self, demands: Dict[int, Demand], connected: Set[int]) -> Dict[int, Dict]:
        for i in self.dtp.vessels:
            if i not in connected:
                self.bbar[i] = dict(ZERO_B)
            elif i in demands:
                self.bbar[i] = dict(demands[i].B)
            else:
                b, a = self.bbar[i], self.xadm[i]
                self.bbar[i] = cap_B({k: b[k] + a[k] for k in ZERO_B}, self.dtp.B_cap)
        return self.bbar

    def admitted(self, A: Iterable[Unit]) -> None:
        x = {i: dict(ZERO_B) for i in self.dtp.vessels}
        for u in A:
            hP = self.dtp.h_P(u.l)
            if u.q == 2:
                x[u.i]["x2"] += u.x + u.n * (hP + self.dtp.h_A)
                x[u.i]["n2"] += u.n
            else:
                x[u.i]["x0"] += u.x + u.n * hP
                x[u.i]["n0"] += u.n
        self.xadm = x


# ---------------------------------------------------------------------- slots (Def. tdma)
def slots(dtp, t: int, A: Iterable[Unit], budget: SlotBudget, Mt: Iterable[int]) -> Dict[int, Tuple[int, int]]:
    """Slots in microseconds of the GS clock: the shared slot from w(t), then one slot per vessel of M(t)
    with admitted units outside R, in the rotated order (i - t) mod M."""
    w, wend = dtp.window(t)
    t0 = w + (budget.shared() if math.isfinite(budget.shared()) else dtp.T_W)
    out = {}
    cur = t0
    for i in sorted(Mt, key=lambda i: dtp.rot(i, t)):
        c = budget.ci.get(i)
        if c is None:
            continue
        d = c / budget.Zk2 + budget.guard[i]
        out[i] = (int(round(cur * 1e6)), int(round(min(cur + d, wend) * 1e6)))
        cur += d
    return out


def tdma_slots(dtp, t: int) -> Dict[int, Tuple[int, int]]:
    """Overlay TDMA: W(t) split into M equal slots in the DTP order of the vessels."""
    w, wend = dtp.window(t)
    d = (wend - w) / dtp.M
    return {i: (int(round((w + k * d) * 1e6)), int(round((w + (k + 1) * d) * 1e6)))
            for k, i in enumerate(dtp.vessels)}
