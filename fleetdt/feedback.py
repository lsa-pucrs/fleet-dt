"""Fleet feedback of the GS for the baselines B3, B8 and B9 of the paper stack (vessel regulator in mode
paper_b3, b8 or paper_tdma, vessel/regulator/fdt_reg_paper.h), in place of the priority table.

Once per step t the GS publishes on fleet/feedback (QoS 1; the control bridge of each vessel carries it,
fleetdt/gsconf.py gs --feedback) the binary payload

    t:u32, y:f64, U:f32, m:u8, L:u8, sigma_1 .. sigma_L:u8      (network byte order)

  y      y(t) of the channel monitor (fleet/throughput), bytes per step; NaN when y(t) did not arrive by
         start(t + 1) + T1 (the vessels then keep their last y)
  U      target utilization of the budget of b8, B(t+1) = U y(t) T3 / (Ts m) (--U, default "U" of the DTP)
  m      vessels that share the channel (--m, default the vessels of the DTP)
  sigma  order sigma of the last fleet/order (1-based topics; the "order" of the DTP before the first)

The feedback of step t leaves when y(t) arrives (LIMERIC/DCC-like: the channel load measured in the
last interval), else at start(t + 1) + T1, in both cases before v(t + 1) = start(t + 1) + T1 + T2, the
release instant of step t + 1 at the vessels. A step is published once.

  python -m fleetdt.feedback --dtp dtp.json --broker host:port [--epoch T0] [--U 0.8] [--m 4]
                             [--log feedback.jsonl] [-v]
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import struct
import sys
import threading
import time
import uuid
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .as_model import DTP
from .as_service import decode_throughput
from .model import tcp_nodelay

log = logging.getLogger("fleetdt.feedback")
TOPIC = "fleet/feedback"
HEAD = struct.Struct("!IdfBB")


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
    """Pure logic (no broker): the payload of each step, once."""

    def __init__(self, dtp: DTP, U: Optional[float] = None, m: Optional[int] = None):
        self.dtp = dtp
        self.U = dtp.U if U is None else float(U)
        self.m = int(m) if m is not None else max(1, len(dtp.vessels or []))
        self.index = {n: k + 1 for k, n in enumerate(dtp.topics)}
        self.sigma = [self.index[n] for n in dtp.order if n in self.index] if dtp.order else []
        self.sigma += [l for l in range(1, dtp.L + 1) if l not in self.sigma]
        self.sent: Dict[int, Optional[float]] = {}
        self.lock = threading.Lock()

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

    def payload(self, t: int, y: Optional[float]) -> Optional[bytes]:
        """The feedback of step t, or None when step t was already published."""
        with self.lock:
            if t in self.sent:
                return None
            self.sent[t] = y
            for k in [k for k in self.sent if k < t - 64]:
                del self.sent[k]
            return encode_feedback(t, y, self.U, self.m, self.sigma)


class Service:
    def __init__(self, dtp: DTP, host: str, port: int, epoch: float, U: Optional[float] = None,
                 m: Optional[int] = None, log_path: Optional[str] = None, now: Callable[[], float] = time.time):
        import paho.mqtt.client as mqtt
        self.dtp, self.epoch, self.now = dtp, float(epoch), now
        self.core = FeedbackCore(dtp, U=U, m=m)
        self.host, self.port = host, port
        self.log_file = open(log_path, "a", encoding="utf-8") if log_path else None
        self.subscribed = threading.Event()
        self._stop = threading.Event()
        self._wlock = threading.Lock()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="fleetdt-feedback-%d-%s" % (os.getpid(), uuid.uuid4().hex[:8]),
                                  clean_session=True)
        tcp_nodelay(self.client)
        self.client.on_connect = lambda c, u, f, rc, props=None: c.subscribe([("fleet/throughput", 0),
                                                                               ("fleet/order", 1)])
        self.client.on_subscribe = lambda c, u, mid, rcs, props=None: self.subscribed.set()
        self.client.on_message = self._on_message

    def _send(self, t: int, y: Optional[float], why: str) -> None:
        p = self.core.payload(t, y)
        if p is None:
            return
        self.client.publish(TOPIC, p, qos=0)
        if self.log_file:
            with self._wlock:
                self.log_file.write(json.dumps({"t": t, "y": y, "U": self.core.U, "m": self.core.m,
                                                "sigma": list(self.core.sigma), "why": why,
                                                "at": self.now()}, separators=(",", ":")) + "\n")
                self.log_file.flush()

    def _on_message(self, client, userdata, msg) -> None:
        try:
            if msg.topic == "fleet/throughput":
                t, y = decode_throughput(msg.payload)
                self._send(t, y, "y")
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
        """Publishes, at start(t + 1) + T1, the feedback of a step t whose y(t) did not arrive."""
        Ts, T1 = self.dtp.Ts, self.dtp.T1
        t = math.floor((self.now() - self.epoch) / Ts)
        while not self._stop.is_set():
            due = self.epoch + (t + 1) * Ts + T1
            if self._stop.wait(max(0.0, due - self.now())):
                break
            self._send(t, None, "deadline")
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
    ap.add_argument("--U", type=float, default=None, help="U of the budget of b8 (default: the DTP)")
    ap.add_argument("--m", type=int, default=None, help="vessels that share the channel (default: the DTP)")
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
