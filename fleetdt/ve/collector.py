"""Assembles G(t) from the topics of the GS MQTT broker and emits G(t) at the end of each step.

Topics (prefixes configurable):
  boat/b<i>/demand      demand table D_i(t) with the backlog Q_i(t) (binary, Section V-C); vertex i of G(t)
                        keeps Q_i(t) and its membership in M(t)
  boat/b<i>/levels      priority table p(t) (binary)
  boat/b<i>/tx/<topic>  forwarded uplink messages, 6-byte step header
  fleet/throughput      y(t): JSON {"t": t, "y": bytes} or binary !Id
  fleet/order           JSON {"t": t, "order": [...], "needed": [...]} with topic names or 1-based indices
  fleet/as (optional)   JSON {"t": t, "C_hat":..., "K_hat":..., "b":..., "U":..., "alpha":...}
  boat/b<i>/regulator   JSON step log of the vessel regulator (forwarded and withheld messages per topic)

r_il(t) charges each received message the channel bytes that the demand d_il(t) and the policing of the
vessel regulator charge: c_b (x + 6) + c_m,l for x content bytes (DTP.message_bytes), so a topic whose
declared messages all arrive gives r_il = d_il when the messages have the size of the bound.
The priority table of boat/b<i>/levels sets the levels of the edges of vessel i (baseline B7 sends one
table per vessel); the first table of a step also stands for the vessels without a table of their own.
"""
from __future__ import annotations

import json
import os
import math
import re
import struct
import threading
import time
from typing import Callable, Dict, List, Optional, Set, Tuple

from ..as_model import DTP, grant_limit, smooth
from ..model import Graph, decode_demand, decode_levels, decode_step_header, tcp_nodelay

VESSEL_RE = re.compile(r"^boat/b(\d+)/(demand|levels|regulator|tx/(.+))$")


class StepClock:
    """Step index on the GS clock: t = floor((now - epoch) / Ts), with the epoch at the start of step 0.

    With epoch None, the Assembler sets the epoch from the first demand table, which the vessel
    publishes at the start of phase 1 of its step."""

    def __init__(self, Ts: float, epoch: Optional[float] = None, now: Callable[[], float] = time.time):
        self.Ts, self.epoch, self.now = Ts, epoch, now
        self.auto = epoch is None

    def step(self, at: Optional[float] = None) -> int:
        at = self.now() if at is None else at
        return math.floor((at - self.epoch) / self.Ts)

    def start(self, t: int) -> float:
        return self.epoch + t * self.Ts


class Assembler:
    """Holds the partial graphs of the open steps; thread-safe."""

    def __init__(self, dtp: DTP, clock: StepClock, phase1_tolerance: float = 0.0):
        self.dtp, self.clock = dtp, clock
        self.tol = phase1_tolerance
        self.lock = threading.Lock()
        self.open: Dict[int, Graph] = {}
        self.seen: Dict[int, Set[Tuple[int, int, int]]] = {}
        self.as_values: Dict[int, Dict] = {}
        self.y: Dict[int, float] = {}
        self.order: Optional[Tuple[List[int], List[int]]] = None
        self.last_C_hat: Optional[float] = None
        self.last_t: Optional[int] = None
        self._y_prev: Optional[float] = None
        self.index = {name: k + 1 for k, name in enumerate(dtp.topics)}

    # ------------------------------------------------------------------ helpers
    def _graph(self, t: int) -> Graph:
        g = self.open.get(t)
        if g is None:
            g = Graph(t=t, L=self.dtp.L, topics=list(self.dtp.topics))
            self.open[t] = g
            self.seen[t] = set()
        return g

    def _topic_index(self, v) -> Optional[int]:
        if isinstance(v, int):
            return v if 1 <= v <= self.dtp.L else None
        return self.index.get(str(v))

    # ------------------------------------------------------------------ input
    def on_message(self, topic: str, payload: bytes, at: Optional[float] = None) -> None:
        at = self.clock.now() if at is None else at
        with self.lock:
            m = VESSEL_RE.match(topic)
            if self.clock.epoch is None and not (m and m.group(2) == "demand"):
                return  # the step clock starts with the first demand table
            if m:
                i = int(m.group(1))
                kind = m.group(2)
                if kind == "demand":
                    t, Q, d = decode_demand(payload)
                    if self.clock.auto:
                        # a demand table can only arrive after the start of its step, so the smallest
                        # estimate of the epoch over the received tables converges to the start of step 0
                        cand = at - t * self.clock.Ts - 0.001
                        if self.clock.epoch is None or cand < self.clock.epoch:
                            self.clock.epoch = cand
                    if len(d) != self.dtp.L:
                        return
                    in_phase1 = at <= self.clock.start(t) + self.dtp.T1 + self.tol
                    g = self._graph(t)
                    if in_phase1 and i not in g.vessels:
                        g.vessels.append(i)
                        g.vessels.sort()
                    v = g.vertex(i)
                    v.Q = int(Q)
                    v.member = i in g.vessels
                    for l, dl in enumerate(d, start=1):
                        g.edge(i, l).d = dl
                elif kind == "levels":
                    t, p = decode_levels(payload)
                    if len(p) != self.dtp.L:
                        return
                    g = self._graph(t)
                    if not hasattr(g, "_levels"):
                        g._levels = p      # the table of the vessels without a table of their own
                        g._levels_by = {}
                    g._levels_by[i] = p
                    for e in g.edges:      # applied again at the end of the step to the edges created later
                        e.p = g._levels_by.get(e.i, g._levels)[e.l - 1]
                elif kind == "regulator":
                    o = json.loads(payload)
                    t = int(o["t"])
                    if self.clock.step(at) > t + 1 and t not in self.open:
                        return  # G(t) already closed
                    g = self._graph(t)
                    for l, (f, h) in enumerate(zip(o.get("fwd", []), o.get("held", [])), start=1):
                        if l <= self.dtp.L:
                            e = g.edge(i, l)
                            e.fwd, e.held = int(f), int(h)
                else:
                    l = self.index.get(m.group(3))
                    if l is None:
                        return
                    t, s = decode_step_header(payload)
                    g = self._graph(t)
                    e = g.edge(i, l)
                    if self.clock.step(at) != t:
                        e.late += 1
                        return
                    key = (i, l, s)
                    if key in self.seen[t]:
                        e.dup += 1
                        return
                    self.seen[t].add(key)
                    e.n += 1
                    e.x += len(payload) - 6
                    e.r = int(round(self.dtp.message_bytes(l, e.x, e.n)))
                return
            if topic == "fleet/throughput":
                t, y = _decode_throughput(payload)
                self.y[t] = y
            elif topic == "fleet/order":
                o = json.loads(payload)
                order = [self._topic_index(v) for v in o.get("order", [])]
                needed = [self._topic_index(v) for v in o.get("needed", [])]
                self.order = ([v for v in order if v], [v for v in needed if v])
            elif topic == "fleet/as":
                o = json.loads(payload)
                self.as_values[int(o["t"])] = o

    # ------------------------------------------------------------------ output
    def close_ready(self, grace: float) -> List[Graph]:
        """Closes every open step whose end lies at least `grace` seconds in the past."""
        now = self.clock.now()
        out = []
        if self.clock.epoch is None:
            return out
        with self.lock:
            for t in sorted(self.open):
                if now < self.clock.start(t + 1) + grace:
                    break
                out.append(self._finish(t))
        return out

    def _finish(self, t: int) -> Graph:
        g = self.open.pop(t)
        self.seen.pop(t, None)
        p = getattr(g, "_levels", None)
        if p is not None:
            by = g._levels_by
            for e in g.edges:
                e.p = by.get(e.i, p)[e.l - 1]
            del g._levels, g._levels_by
        g.edges.sort(key=lambda e: (e.i, e.l))
        for v in g.vertices:
            v.member = v.i in g.vessels
        g.y = self.y.pop(t, None)
        a = self.as_values.pop(t, None)
        g.Q = float(a["Q"]) if a and a.get("Q") is not None else float(g.backlog())
        if a and a.get("order"):          # sigma_t and N(t) that the AS used
            g.order, g.needed = [int(v) for v in a["order"]], [int(v) for v in a.get("needed", [])]
        elif self.order:                  # last fleet/order
            g.order, g.needed = list(self.order[0]), list(self.order[1])
        else:                             # initial sigma and N of the DTP
            g.order = [self.index[n] for n in self.dtp.order if n in self.index] or list(range(1, self.dtp.L + 1))
            g.needed = [self.index[n] for n in self.dtp.needed if n in self.index]
        if a:
            for k in ("C_hat", "K_hat", "b", "U", "alpha", "A"):
                if k in a:
                    setattr(g, k, a[k])
        else:  # the VE reproduces eq. (2) and eq. (3)
            g.U, g.alpha = self.dtp.U, self.dtp.alpha
            if self.last_C_hat is None:
                g.C_hat = self.dtp.C0
            else:
                late_y = self.y.pop(t - 1, None)  # y(t-1) that arrived after G(t-1) closed
                y_prev = late_y if late_y is not None else (self._y_prev if self.last_t == t - 1 else None)
                g.C_hat = smooth(self.last_C_hat, y_prev, self.dtp.alpha)
            g.K_hat = self.dtp.U * g.C_hat * self.dtp.T3 / self.dtp.Ts
            g.b = grant_limit(g.C_hat, self.dtp.U, self.dtp.T3, self.dtp.Ts, g.m)
        self.last_C_hat, self.last_t, self._y_prev = g.C_hat, t, g.y
        for k in [k for k in self.y if k < t - 1]:
            del self.y[k]
        return g


def _decode_throughput(payload: bytes) -> Tuple[int, float]:
    if payload[:1] == b"{":
        o = json.loads(payload)
        return int(o["t"]), float(o["y"])
    t, y = struct.unpack("!Id", payload)
    return t, y


class MqttCollector:
    """Connects the Assembler to a broker with paho-mqtt and calls `emit(G)` at the end of each step."""

    TOPICS = ["boat/+/demand", "boat/+/levels", "boat/+/regulator", "boat/+/tx/#", "fleet/throughput", "fleet/order",
              "fleet/as"]

    def __init__(self, assembler: Assembler, emit: Callable[[Graph], None], host: str = "localhost",
                 port: int = 1883, grace: float = 0.0625):
        import paho.mqtt.client as mqtt
        self.asm, self.emit, self.grace = assembler, emit, grace
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fleetdt-ve-%d" % os.getpid(), clean_session=True)
        tcp_nodelay(self.client)
        self.client.on_connect = lambda c, u, f, rc, props=None: [c.subscribe(tp, qos=0) for tp in self.TOPICS]
        self.client.on_message = lambda c, u, msg: self._safe(msg.topic, msg.payload)
        self.host, self.port = host, port
        self._stop = threading.Event()

    def _safe(self, topic: str, payload: bytes) -> None:
        try:
            self.asm.on_message(topic, payload)
        except (ValueError, KeyError, struct.error, json.JSONDecodeError):
            pass  # malformed datum: the step keeps its other values

    def start(self) -> None:
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()
        threading.Thread(target=self._ticker, daemon=True).start()

    def _ticker(self) -> None:
        Ts = self.asm.dtp.Ts
        while not self._stop.is_set():
            for g in self.asm.close_ready(self.grace):
                self.emit(g)
            time.sleep(Ts / 8)

    def stop(self) -> None:
        self._stop.set()
        self.client.loop_stop()
        self.client.disconnect()
