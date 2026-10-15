"""Shared helpers of the vessel publishers: DTP file, step clock, per-topic rate limiter, MQTT client.

The publishers run on the vessel RP4 without the fleetdt package, so this module only depends on
the standard library and paho-mqtt 2.x.
"""
from __future__ import annotations

import json
import math
import os
import random
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import paho.mqtt.client as mqtt

# Bytes of the step header (t:u32, s:u16) that the vessel regulator writes in front of each forwarded message.
STEP_HEADER_BYTES = 6
# epoch of a DTP file with "epoch": null (2026-01-01T00:00:00Z), the same in every program
DEFAULT_EPOCH = 1767225600.0


@dataclass
class Topic:
    name: str
    rate: float   # messages per second
    bound: int    # largest content of one message, bytes (before the step header)


@dataclass
class DTPFile:
    """The fields of dtp.json (SPEC.md, Section 2) that the publishers use."""
    Ts: float
    T1: float
    T2: float
    epoch: Optional[float]
    topics: Dict[str, Topic] = field(default_factory=dict)

    @staticmethod
    def load(path: str) -> "DTPFile":
        with open(path) as f:
            d = json.load(f)
        topics = {}
        for t in d["topics"]:
            topics[t["name"]] = Topic(t["name"], float(t["rate"]), int(t["bound"]))
        return DTPFile(float(d["Ts"]), float(d["T1"]), float(d["T2"]), d.get("epoch"), topics)

    def per_step(self, name: str) -> int:
        """n_l = ceil(r_l Ts): number of messages of topic l that the demand of one step covers (SPEC.md, Section 5)."""
        return int(math.ceil(self.topics[name].rate * self.Ts - 1e-9))


class StepClock:
    """Step clock of the DTP: step t starts at epoch + t*Ts on the host clock (synchronized by chrony).

    With epoch None (dtp.json of a bench without a fixed operation), the clock uses DEFAULT_EPOCH, the
    value that the DTI program, the vessel regulator, the channel monitor, the AS and the VE use, so the
    step index t agrees on every host.
    """

    def __init__(self, Ts: float, T1: float, T2: float, epoch: Optional[float]):
        self.Ts, self.T1, self.T2 = Ts, T1, T2
        self.epoch = DEFAULT_EPOCH if epoch is None else float(epoch)

    def step(self, now: Optional[float] = None) -> int:
        now = time.time() if now is None else now
        return math.floor((now - self.epoch) / self.Ts)

    def start(self, t: int) -> float:
        return self.epoch + t * self.Ts

    def regulator_step(self, now: Optional[float] = None) -> int:
        """Step of a raw message that reaches the vessel regulator at `now`:
        the interval [start(t-1) + T1 + T2, start(t) + T1 + T2) belongs to step t (SPEC.md, Section 2)."""
        now = time.time() if now is None else now
        return math.floor((now - self.epoch - self.T1 - self.T2) / self.Ts) + 1


class RateLimiter:
    """Per-topic limiter: a token bucket with the DTP rate and a depth of `depth` messages, plus a cap of
    n_l = ceil(r Ts) messages per regulator step window.

    The bucket bounds the long-run rate by the DTP rate; the depth of 2 absorbs the jitter of a source
    that runs at the DTP rate, so that a message that arrives a little early is not dropped. The window
    cap keeps the messages of one step within the n_l messages that the demand of the step covers, so
    the publisher drops a message that the regulator would withhold anyway.
    """

    def __init__(self, rate: float, per_step: int, clock: StepClock, depth: float = 2.0):
        self.rate, self.per_step, self.clock, self.depth = rate, per_step, clock, depth
        self.tokens = 1.0
        self.last: Optional[float] = None
        self.window = None
        self.count = 0
        self.lock = threading.Lock()

    def allow(self, now: Optional[float] = None) -> bool:
        now = time.time() if now is None else now
        with self.lock:
            if self.last is not None:
                self.tokens = min(self.depth, self.tokens + (now - self.last) * self.rate)
            self.last = now
            w = self.clock.regulator_step(now)
            if w != self.window:
                self.window, self.count = w, 0
            if self.tokens < 1.0 or self.count >= self.per_step:
                return False
            self.tokens -= 1.0
            self.count += 1
            return True


def sleep_until(when: float, stop: Optional[threading.Event] = None) -> bool:
    """Sleep until the wall-clock instant `when`; returns False when `stop` is set first."""
    while True:
        dt = when - time.time()
        if dt <= 0:
            return True
        if stop is not None:
            if stop.wait(min(dt, 0.5)):
                return False
        else:
            time.sleep(min(dt, 0.5))


def raw_topic(vessel: int, name: str) -> str:
    return "boat/b%d/raw/%s" % (vessel, name)


def make_client(role: str, vessel: int, host: str, port: int,
                on_connect=None, on_message=None) -> mqtt.Client:
    """paho-mqtt 2.x client with a unique client id; connects asynchronously and reconnects by itself."""
    cid = "fleetdt-%s-b%d-%d-%06x" % (role, vessel, os.getpid(), random.getrandbits(24))
    c = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=cid,
                    clean_session=True)
    c.max_queued_messages_set(0)

    def on_socket_open(client, userdata, sock):   # paho leaves Nagle's algorithm on (R8)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except (OSError, AttributeError):
            pass

    c.on_socket_open = on_socket_open
    c.reconnect_delay_set(min_delay=1, max_delay=4)
    if on_connect is not None:
        c.on_connect = on_connect
    if on_message is not None:
        c.on_message = on_message
    c.connect_async(host, port, keepalive=10)
    c.loop_start()
    return c
