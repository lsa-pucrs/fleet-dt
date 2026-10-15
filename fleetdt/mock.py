"""Synthetic fleet for tests and demonstrations of the VE (no vessel needed).

The simulator follows Sections IV and V of the paper: a saturation throughput C(t) with bursts of
interference, the measurement y(t), the AS (eq. 2, eq. 3, Algorithm 1), and phase 3 with strict
priority inside each vessel and a share of the channel per vessel proportional to its admitted bytes.
"""
from __future__ import annotations

import json
import os
import math
import random
import time
from dataclasses import dataclass
from typing import Dict, Iterator, List, Optional, Tuple

from .as_model import DTP, admission, grant_limit, smooth
from .model import Graph, encode_demand, encode_levels, encode_step_header, tcp_nodelay

# topic: (messages per step, bytes per message); the video size varies with the scene
DEFAULT_TOPICS: List[Tuple[str, int, int]] = [
    ("state", 1, 64), ("imu", 4, 48), ("gps", 1, 80), ("baro", 1, 32),
    ("power", 1, 40), ("mag", 2, 40), ("video", 2, 14_000),
]


@dataclass
class StepPlan:
    graph: Graph
    msgs: Dict[Tuple[int, int], List[int]]   # (i, l) -> sizes of the messages of step t
    delivered: Dict[Tuple[int, int], int]    # (i, l) -> number of messages delivered in step t


class FleetSim:
    def __init__(self, M: int = 3, dtp: Optional[DTP] = None, seed: int = 1,
                 C_base: float = 320_000.0, burst_every: int = 240, burst_len: int = 60,
                 mode_every: int = 80, miss_prob: float = 0.01):
        self.rng = random.Random(seed)
        self.M = M
        self.dtp = dtp or DTP(topics=[t[0] for t in DEFAULT_TOPICS], c_b=1.06, c_m=180.0)
        self.spec = {k + 1: (n, s) for k, (_, n, s) in enumerate(DEFAULT_TOPICS)}
        self.C_base, self.burst_every, self.burst_len = C_base, burst_every, burst_len
        self.mode_every, self.miss_prob = mode_every, miss_prob
        self.C_hat = self.dtp.C0
        self.y_prev: Optional[float] = None
        self.walk = 0.0

    def capacity(self, t: int) -> float:
        self.walk = max(-0.25, min(0.25, self.walk + self.rng.gauss(0, 0.01)))
        burst = 0.45 if (t % self.burst_every) >= self.burst_every - self.burst_len else 0.0
        return self.C_base * (1 + self.walk) * (1 - burst)

    def order(self, t: int) -> Tuple[List[int], List[int]]:
        L = self.dtp.L
        survey = (t // self.mode_every) % 2 == 0
        needed = [1, 2, 3, L] if survey else [1, 2, 3]
        rest = [l for l in range(1, L + 1) if l not in needed]
        return needed + rest, needed

    def step(self, t: int) -> StepPlan:
        dtp = self.dtp
        g = Graph(t=t, L=dtp.L, topics=list(dtp.topics), U=dtp.U, alpha=dtp.alpha)
        g.order, g.needed = self.order(t)
        # phase 1: demand tables
        msgs: Dict[Tuple[int, int], List[int]] = {}
        D: Dict[int, List[int]] = {}
        for i in range(1, self.M + 1):
            row = []
            for l, (n, s) in self.spec.items():
                if dtp.topics[l - 1] == "video":
                    sizes = [int(s * self.rng.uniform(0.6, 1.4)) for _ in range(n)]
                else:
                    sizes = [s] * n
                msgs[(i, l)] = sizes
                row.append(int(math.ceil(dtp.channel_bytes(sum(sizes), len(sizes)))))
            if self.rng.random() >= self.miss_prob:
                D[i] = row
                g.vessels.append(i)
            for l, dl in enumerate(row, start=1):
                g.edge(i, l).d = dl
        # phase 2: AS
        self.C_hat = smooth(self.C_hat, self.y_prev, dtp.alpha) if t > 0 else dtp.C0
        g.C_hat = self.C_hat
        g.K_hat = dtp.U * self.C_hat * dtp.T3 / dtp.Ts
        g.b = grant_limit(self.C_hat, dtp.U, dtp.T3, dtp.Ts, g.m)
        p = admission(D, g.order, g.needed, g.b) if g.m else [0] * dtp.L
        for e in g.edges:
            e.p = p[e.l - 1] if e.i in D else None
        # phase 3: channel
        C = self.capacity(t)
        self.C_last = C
        cap = C * dtp.T3 / dtp.Ts
        a = {i: sum(D[i][l - 1] for l in range(1, dtp.L + 1) if p[l - 1] > 0) for i in D}
        X = sum(a.values())
        delivered: Dict[Tuple[int, int], int] = {}
        for i in D:
            budget = a[i] if X <= cap else cap * a[i] / X
            prio = [l for l in g.order if p[l - 1] == 2] + [l for l in g.order if p[l - 1] == 1]
            for l in prio:
                k = 0
                for size in msgs[(i, l)]:
                    cost = dtp.channel_bytes(size, 1)
                    if cost > budget:
                        budget = 0
                        break
                    budget -= cost
                    k += 1
                delivered[(i, l)] = k
                e = g.edge(i, l)
                sizes = msgs[(i, l)][:k]
                e.n, e.x = k, sum(sizes)
                e.r = int(round(dtp.channel_bytes(e.x, e.n)))
        y = C * (1 + self.rng.gauss(0, 0.04))
        g.y = y
        self.y_prev = y
        g.edges.sort(key=lambda e: (e.i, e.l))
        return StepPlan(graph=g, msgs=msgs, delivered=delivered)

    def graphs(self, steps: int) -> Iterator[Graph]:
        for t in range(steps):
            yield self.step(t).graph


def publish(host: str = "localhost", port: int = 1883, M: int = 3, steps: int = 400, seed: int = 1,
            epoch: Optional[float] = None, publish_as: bool = False) -> None:
    """Publishes the topics of the GS broker in real time, in the formats of Section V-C."""
    import paho.mqtt.client as mqtt
    sim = FleetSim(M=M, seed=seed)
    dtp = sim.dtp
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fleetdt-mock-%d" % os.getpid())
    tcp_nodelay(c)
    c.connect(host, port)
    c.loop_start()
    epoch = dtp.t0 if epoch is None else epoch  # start of step 0 of the operation
    t0 = math.floor((time.time() - epoch) / dtp.Ts) + 1

    def wait_until(at: float) -> None:
        dt = at - time.time()
        if dt > 0:
            time.sleep(dt)

    for k in range(steps):
        t = t0 + k
        plan = sim.step(k)
        g = plan.graph
        start = epoch + t * dtp.Ts
        wait_until(start + 0.002)
        c.publish("fleet/order", json.dumps({"t": t, "order": [dtp.topics[l - 1] for l in g.order],
                                             "needed": [dtp.topics[l - 1] for l in g.needed]}))
        for i in g.vessels:
            d = [g.edge(i, l).d for l in range(1, dtp.L + 1)]
            c.publish("boat/b%d/demand" % i, encode_demand(t, d), qos=1)
        wait_until(start + dtp.T1 + 0.002)
        p = [g.edge(g.vessels[0], l).p or 0 for l in range(1, dtp.L + 1)] if g.vessels else None
        if p is not None:
            for i in g.vessels:
                c.publish("boat/b%d/levels" % i, encode_levels(t, p), qos=1)
        if publish_as:
            c.publish("fleet/as", json.dumps({"t": t, "C_hat": g.C_hat, "K_hat": g.K_hat, "b": g.b,
                                              "U": g.U, "alpha": g.alpha}))
        wait_until(start + dtp.T1 + dtp.T2 + 0.002)
        for (i, l), k_del in plan.delivered.items():
            for s, size in enumerate(plan.msgs[(i, l)][:k_del]):
                payload = encode_step_header(t, s) + bytes(size)
                c.publish("boat/b%d/tx/%s" % (i, dtp.topics[l - 1]), payload, qos=0)
        wait_until(start + dtp.Ts + 0.003)
        c.publish("fleet/throughput", json.dumps({"t": t, "y": g.y}))
    c.loop_stop()
    c.disconnect()
