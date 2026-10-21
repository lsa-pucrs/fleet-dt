"""Fleet feedback of the GS for the baselines B3, B7 and B8 of the paper (Table S3; vessel regulator in mode
paper_b3, b8 or paper_tdma, vessel/regulator/fdt_reg_paper.h), in place of the priority table.

Once per step t, at start(t) + T1 (the end of phase 1, the instant at which the AS closes M(t)), the GS
publishes on fleet/feedback (QoS 0; the control bridge of each vessel carries it, fleetdt/gsconf.py gs
--feedback) the binary payload

    t:u32, y:f64, U:f32, m:u8, L:u8, sigma_1 .. sigma_L:u8      (network byte order)

  t      the step that the feedback serves
  y      y(t - 1) of the channel monitor (fleet/throughput), bytes per step, when it reached the GS before
         start(t) + T1 (the rule of eq. (2) in the AS); NaN otherwise (the vessels then keep their last y)
  U      target utilization of the budget of B7, B(t) = U y(t - 1) T3 / (Ts m(t)) (--U, default "U" of the DTP)
  m      m(t) = |M(t)|: the vessels whose demand table D_i(t) (boat/b<i>/demand, published by the DTI program
         of every method) reached the GS before start(t) + T1, with the acceptance rule of the AS
         (fleetdt.as_service.AS.on_demand); --m fixes m instead. 0 (no table) makes the vessels keep their m
  sigma  sigma_t: the order of the last fleet/order received before start(t) + T1 (1-based topics; the
         "order" of the DTP before the first)

The vessels receive the feedback of step t before v(t) = start(t) + T1 + T2, as the priority table of the AS.
B3 takes sigma_t from it, B7 sigma_t and its budget, B8 sigma_t. A step is published once.

  python -m fleetdt.feedback --dtp dtp.json --broker host:port [--epoch T0] [--U 0.8] [--m 4]
                             [--log feedback.jsonl] [-v]
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import re
import struct
import sys
import threading
import time
import uuid
from typing import Callable, Dict, List, Optional, Sequence, Set, Tuple

from .as_model import DTP
from .as_service import decode_throughput
from .model import decode_demand, tcp_nodelay

log = logging.getLogger("fleetdt.feedback")
TOPIC = "fleet/feedback"
HEAD = struct.Struct("!IdfBB")
DEMAND_RE = re.compile(r"^boat/b(\d+)/demand$")


def encode_feedback(t: int, y: Optional[float], U: float, m: int, sigma: Sequence[int]) -> bytes:
    """sigma: 1-based topic indices."""
    return HEAD.pack(int(t) & 0xFFFFFFFF, float("nan") if y is None else float(y), float(U), int(m) & 0xFF,
                     len(sigma)) + bytes(int(v) for v in sigma)


def decode_feedback(buf: bytes) -> Tuple[int, Optional[float], float, int, List[int]]:
    t, y, U, m, L = HEAD.unpack_from(buf, 0)
    if len(buf) != HEAD.size + L:
        raise ValueError("feedback of %d bytes for L=%d" % (len(buf), L))
    return t, (None if math.isnan(y) else y), U, m, list(buf[HEAD.size:])


class FeedbackCore:
    """Pure logic (no broker). Times are Unix times on the GS clock; close(t) at start(t) + T1 gives the
    payload of step t, once."""

    def __init__(self, dtp: DTP, U: Optional[float] = None, m: Optional[int] = None, epoch: float = 0.0):
        self.dtp = dtp
        self.epoch = float(epoch)
        self.U = dtp.U if U is None else float(U)
        self.m_fixed = None if m is None else int(m)
        self.index = {n: k + 1 for k, n in enumerate(dtp.topics)}
        self.sigma = [self.index[n] for n in dtp.order if n in self.index] if dtp.order else []
        self.sigma += [l for l in range(1, dtp.L + 1) if l not in self.sigma]
        self.sent: Dict[int, Dict] = {}         # t -> record of the feedback of step t
        self._y: Dict[int, float] = {}          # k -> y(k) that arrived before start(k + 1) + T1
        self._M: Dict[int, Set[int]] = {}       # t -> vessels of M(t)
        self.lock = threading.Lock()

    def deadline(self, t: int) -> float:
        """start(t) + T1: the end of phase 1 of step t, the instant of the feedback of step t."""
        return self.epoch + t * self.dtp.Ts + self.dtp.T1

    def step_of(self, at: float) -> int:
        return math.floor((at - self.epoch) / self.dtp.Ts)

    def on_order(self, order: Sequence) -> bool:
        idx = []
        for v in order:
            l = v if isinstance(v, int) and not isinstance(v, bool) else self.index.get(str(v), -1)
            idx.append(l)
        if sorted(idx) != list(range(1, self.dtp.L + 1)):
            return False
        with self.lock:
            self.sigma = idx
        return True

    def on_throughput(self, t: int, y: float, at: float) -> bool:
        """y(t) of the channel monitor; the feedback of step t + 1 carries it when it arrives before
        start(t + 1) + T1. Returns False for a late y(t)."""
        with self.lock:
            t = int(t)
            if at >= self.deadline(t + 1) or (t + 1) in self.sent:
                return False
            self._y[t] = float(y)
            return True

    def on_demand(self, i: int, t: int, L: int, at: float) -> bool:
        """Demand table D_i(t) of vessel i seen at the GS: vessel i enters M(t) when the table has the L of
        the DTP and arrives before start(t) + T1, at most one step ahead (fleetdt.as_service.AS.on_demand)."""
        with self.lock:
            t = int(t)
            if L != self.dtp.L or t in self.sent or at >= self.deadline(t) or t > self.step_of(at) + 1:
                return False
            self._M.setdefault(t, set()).add(int(i))
            return True

    def close(self, t: int) -> Optional[Tuple[bytes, Dict]]:
        """(payload, record) of the feedback of step t: y(t - 1), U, m(t), sigma_t; None when step t was
        already published."""
        with self.lock:
            t = int(t)
            if t in self.sent:
                return None
            y = self._y.get(t - 1)
            M = sorted(self._M.get(t, ()))
            m = self.m_fixed if self.m_fixed is not None else len(M)
            rec = {"t": t, "y_t": t - 1, "y": y, "U": self.U, "m": m, "M": M, "sigma": list(self.sigma)}
            self.sent[t] = rec
            for k in [k for k in self.sent if k < t - 64]:
                del self.sent[k]
            for k in [k for k in self._y if k < t]:
                del self._y[k]
            for k in [k for k in self._M if k <= t]:
                del self._M[k]
            return encode_feedback(t, y, self.U, m, self.sigma), rec


class Service:
    def __init__(self, dtp: DTP, host: str, port: int, epoch: float, U: Optional[float] = None,
                 m: Optional[int] = None, log_path: Optional[str] = None, now: Callable[[], float] = time.time):
        import paho.mqtt.client as mqtt
        self.dtp, self.epoch, self.now = dtp, float(epoch), now
        self.core = FeedbackCore(dtp, U=U, m=m, epoch=epoch)
        self.host, self.port = host, port
        self.log_file = open(log_path, "a", encoding="utf-8") if log_path else None
        self.subscribed = threading.Event()
        self._stop = threading.Event()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="fleetdt-feedback-%d-%s" % (os.getpid(), uuid.uuid4().hex[:8]),
                                  clean_session=True)
        tcp_nodelay(self.client)
        self.client.on_connect = lambda c, u, f, rc, props=None: c.subscribe([("fleet/throughput", 0),
                                                                               ("fleet/order", 1),
                                                                               ("boat/+/demand", 0)])
        self.client.on_subscribe = lambda c, u, mid, rcs, props=None: self.subscribed.set()
        self.client.on_message = self._on_message

    def _send(self, t: int) -> None:
        out = self.core.close(t)
        if out is None:
            return
        p, rec = out
        self.client.publish(TOPIC, p, qos=0)
        if self.log_file:
            self.log_file.write(json.dumps(dict(rec, at=self.now()), separators=(",", ":")) + "\n")
            self.log_file.flush()

    def _on_message(self, client, userdata, msg) -> None:
        try:
            at = self.now()
            m = DEMAND_RE.match(msg.topic)
            if m:
                t, _, d = decode_demand(msg.payload)
                self.core.on_demand(int(m.group(1)), t, len(d), at)
            elif msg.topic == "fleet/throughput":
                t, y = decode_throughput(msg.payload)
                self.core.on_throughput(t, y, at)
            elif msg.topic == "fleet/order":
                if not self.core.on_order(json.loads(msg.payload).get("order", [])):
                    log.warning("fleet/order rejected: %s", msg.payload[:200])
        except (ValueError, KeyError, TypeError, struct.error) as e:
            log.warning("malformed message on %s: %s", msg.topic, e)

    def connect(self, timeout: float = 5.0) -> None:
        self.client.connect(self.host, self.port, keepalive=30)
        self.client.loop_start()
        if not self.subscribed.wait(timeout):
            raise TimeoutError("no SUBACK from %s:%d" % (self.host, self.port))

    def run(self) -> None:
        """Publishes the feedback of each step t at start(t) + T1."""
        t = math.floor((self.now() - self.epoch - self.dtp.T1) / self.dtp.Ts) + 1
        while not self._stop.is_set():
            if self._stop.wait(max(0.0, self.core.deadline(t) - self.now())):
                break
            self._send(t)
            t += 1

    def stop(self) -> None:
        self._stop.set()
        self.client.loop_stop()
        self.client.disconnect()
        if self.log_file:
            self.log_file.close()
            self.log_file = None


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.feedback", description=__doc__.split("\n")[0])
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--broker", default="localhost:1883")
    ap.add_argument("--epoch", type=float, default=None)
    ap.add_argument("--U", type=float, default=None, help="U of the budget of B7 (default: the DTP)")
    ap.add_argument("--m", type=int, default=None,
                    help="fixed m in place of m(t) = |M(t)| of the demand tables (default: m(t))")
    ap.add_argument("--log", default=None)
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO if a.verbose else logging.WARNING,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    dtp = DTP.load(a.dtp)
    host, _, port = a.broker.partition(":")
    svc = Service(dtp, host or "localhost", int(port or 1883), a.epoch if a.epoch is not None else dtp.t0,
                  U=a.U, m=a.m, log_path=a.log)
    svc.connect()
    try:
        svc.run()
    except KeyboardInterrupt:
        pass
    finally:
        svc.stop()
    print("fleetdt.feedback: %d steps" % len(svc.core.sent), file=sys.stderr)


if __name__ == "__main__":
    main()
