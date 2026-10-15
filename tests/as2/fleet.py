"""Synthetic fleet for the tests of the AS v2: demands (SPEC 4.1), reports, accepted messages and monitor
batches, driven step by step around ASCore.phase2."""
from __future__ import annotations

import json
import random
import struct
from typing import Callable, Dict, List, Optional, Tuple

from fleetdt.as2.core import ASCore
from fleetdt.as2.dtp import DTP


def demand_obj(dtp: DTP, i: int, t: int, rng: random.Random, held_out: int = 2, B=None, rep=None, scale: float = 1.0):
    """Demand of vessel i for step t: streams at the arrival bound over Ts, outputs with `held_out` held
    outputs plus the arrival bound over T1 + T2."""
    d, held = [], []
    for tp in dtp.topics:
        if tp.output:
            h = held_out
            na = tp.outputs_bound(dtp.T1 + dtp.T2)
            units = []
            for k in range(h + na):
                n = tp.n if k >= h else rng.randint(max(1, tp.n // 2), tp.n)
                x = n * tp.s if k >= h else int(n * tp.s * rng.uniform(0.6, 1.0) * scale)
                units.append([x, n])
            d.append(units)
            held.append(h)
        else:
            N = tp.outputs_bound(dtp.Ts)
            d.append([[int(N * tp.n * tp.s * scale), N * tp.n]])
            held.append(0)
    return {"t": t, "v": i, "d": d, "held": held, "B": B or {"x0": 0, "n0": 0, "x2": 0, "n2": 0},
            "rep": rep, "late_us": 0}


def tx_payload(t: int, seq: int, size: int) -> bytes:
    return struct.pack("!IH", t, seq) + bytes(size)


class SimFleet:
    """Runs ASCore over T steps. kappa(t) comes from `kappa_fn(t)`. A vessel delivers every released
    message within step t when the admitted cost plus the held cost fits kappa(t) T_W (Lemma 2);
    otherwise the GS misses the last messages. `adversary(t, core)` may force a miss in step t."""

    def __init__(self, dtp: DTP, core: ASCore, seed: int = 1, kappa_fn: Optional[Callable[[int], float]] = None,
                 adversary: Optional[Callable[[int, ASCore], bool]] = None, lose_demand: float = 0.0,
                 lose_table: float = 0.0, monitor: bool = False, b_fn: Optional[Callable[[int], float]] = None):
        self.dtp, self.core = dtp, core
        self.rng = random.Random(seed)
        self.kappa_fn = kappa_fn or (lambda t: 0.6)
        self.adversary = adversary
        self.lose_demand, self.lose_table = lose_demand, lose_table
        self.rep: Dict[int, Dict] = {}
        self.B: Dict[int, Dict] = {}
        self.records: List[Dict] = []
        self.f_true: List[int] = []
        self.monitor = monitor
        self.b_fn = b_fn or (lambda t: 0.1)
        from fleetdt.as2.airtime import Phy
        self.phy = Phy(dtp.phy)

    def batch(self, t: int, kappa: float) -> Dict:
        """Monitor batch over step t: busy fraction b_fn(t) in Q(t+1) (outside the fleet), and in W(t)
        uplink data-bridge frames of 1528 bytes at 24 Mbit/s whose airtime is kappa of W(t), plus one
        frame after the end of W(t) (the data bridge holds bytes, so e(t) is the end of W(t))."""
        dtp = self.dtp
        us = lambda x: int(round(x * 1e6))
        w, wend = dtp.window(t)
        ts = self.phy.t_succ(1528, 24e6)
        dur = ts - self.phy.aifs()
        fr, busy = [], []
        if kappa > 0:
            gap = ts / kappa
            x = w
            while x + dur < wend:
                fr.append([us(x), us(x + dur), 1, 0, 1528, 24000, 1, 0])
                busy.append([us(x), us(x + dur)])
                x += gap
        fr.append([us(wend) + 50, us(wend) + 50 + us(dur), 1, 0, 1528, 24000, 1, 0])
        q0, q1 = dtp.quiet(t + 1)
        b = self.b_fn(t + 1)
        if b > 0:
            busy.append([us(q0), us(q0 + b * (q1 - q0))])
        return {"from_us": us(dtp.start(t)), "to_us": us(dtp.phase2(t + 1)), "busy": busy, "fr": fr}

    def run(self, T: int) -> List[Dict]:
        dtp, core, rng = self.dtp, self.core, self.rng
        for t in range(1, T + 1):
            for i in dtp.vessels:
                obj = demand_obj(dtp, i, t, rng, held_out=rng.randint(0, 2), B=self.B.get(i), rep=self.rep.get(i))
                if rng.random() >= self.lose_demand:
                    core.on_demand(i, json.loads(json.dumps(obj)), dtp.start(t) + 0.001)
            tabs, rec = core.phase2(t)
            self.records.append(rec)
            kappa = self.kappa_fn(t)
            if self.monitor:
                core.monitor.add(self.batch(t, kappa))
            work = rec["cost_A"] + sum(rec["cB"])
            ok = work <= kappa * dtp.T_W
            if self.adversary is not None and self.adversary(t, core):
                ok = False
            # released messages: table (or the default table: R at the arrival bound over Ts)
            per_vessel: Dict[int, Dict[int, List[int]]] = {}
            for (i, l, k) in rec["A"]:
                per_vessel.setdefault(i, {}).setdefault(l, []).append(k)
            self.rep = {}
            self.B = {}
            victim = rng.choice(dtp.vessels) if not ok else None
            self.f_true.append(0 if ok else 1)
            for i in dtp.vessels:
                got_table = i in tabs and rng.random() >= self.lose_table
                rel = [[] for _ in range(dtp.L)]
                units = per_vessel.get(i, {}) if got_table else {l: [1] for l in dtp.required}
                for l, ks in units.items():
                    tp = dtp.topics[l]
                    for k in sorted(ks):
                        n = tp.n if tp.output else tp.outputs_bound(dtp.Ts) * tp.n
                        rel[l].append(n)
                for l in range(dtp.L):
                    n_rel = sum(rel[l])
                    n_acc = n_rel
                    if i == victim and n_rel > 0:
                        n_acc = n_rel - 1
                    for seq in range(n_acc):
                        core.acceptor.on_message(dtp.topic_string(i, l), tx_payload(t, seq, 8), dtp.end(t) - 0.001)
                self.rep[i] = {"t": t, "rel": rel, "disc": 0, "tab": got_table}
                if i == victim:
                    self.B[i] = {"x0": 3000, "n0": 2, "x2": 0, "n2": 0}
        return self.records
