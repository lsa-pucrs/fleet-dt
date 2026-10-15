#!/usr/bin/env python3
"""Synthetic uplink load of a Fleet-DT vessel: publishes on boat/b<i>/raw/<topic> (QoS 0) of the vessel
broker the topics of the DTP at their rates, for bench runs, hardware-in-the-loop runs and tests
without the autopilot and the camera.

Each message of topic l carries a 12-byte content header (topic index:u8, pad:u8, k:u32 message
counter of the topic, sent:u48 microseconds of the Unix time) and filler up to its size. The size is
the DTP bound s_l (--size bound), uniform in [--min-frac s_l, s_l] (--size random), or a fixed
fraction of s_l (--size 0.5). --scale multiplies every rate (scale > 1 offers more messages than
the demand covers, so the vessel regulator withholds the excess).

  raw_load.py --dtp dtp.json --vessel 1 [--host 127.0.0.1 --port 1883] [--topics state,imu,video]
              [--scale 1.0] [--size bound|random|F] [--min-frac 0.5] [--duration S] [--seed 1]
"""
from __future__ import annotations

import argparse
import heapq
import os
import random
import signal
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import DTPFile, make_client, raw_topic  # noqa: E402

HEADER = struct.Struct("!BBIHI")   # l, pad, k, sent (48-bit as u16 high + u32 low)


def content(l: int, k: int, size: int) -> bytes:
    us = int(time.time() * 1e6)
    h = HEADER.pack(l & 0xFF, 0, k & 0xFFFFFFFF, (us >> 32) & 0xFFFF, us & 0xFFFFFFFF)
    if size <= len(h):
        return h[:size]
    return h + bytes(size - len(h))


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--vessel", type=int, required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--topics", default=None, help="comma-separated subset of the DTP topics (default: all)")
    ap.add_argument("--scale", type=float, default=1.0, help="multiplier of every rate")
    ap.add_argument("--size", default="random", help="bound, random, or a fraction of the bound")
    ap.add_argument("--min-frac", type=float, default=0.5, help="smallest size of --size random, fraction of the bound")
    ap.add_argument("--duration", type=float, default=None, help="seconds (default: until SIGTERM)")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(argv)

    dtp = DTPFile.load(a.dtp)
    names = list(dtp.topics) if not a.topics else [n for n in a.topics.split(",") if n]
    for n in names:
        if n not in dtp.topics:
            ap.error("topic %s is not in the DTP" % n)
    rng = random.Random(a.seed + a.vessel)
    index = {n: k + 1 for k, n in enumerate(dtp.topics)}
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    client = make_client("load", a.vessel, a.host, a.port)
    t0 = time.time()
    end = t0 + a.duration if a.duration else None
    heap = []
    for n in names:
        r = dtp.topics[n].rate * a.scale
        if r > 0:
            heapq.heappush(heap, (t0 + rng.uniform(0, 1.0 / r), n, 0))
    sent = {n: 0 for n in names}
    while heap and not stop.is_set():
        at, n, k = heapq.heappop(heap)
        if end is not None and at >= end:
            break
        dt = at - time.time()
        if dt > 0 and stop.wait(dt):
            break
        bound = dtp.topics[n].bound
        if a.size == "bound":
            size = bound
        elif a.size == "random":
            size = rng.randint(max(1, int(a.min_frac * bound)), bound)
        else:
            size = max(1, min(bound, int(float(a.size) * bound)))
        client.publish(raw_topic(a.vessel, n), content(index[n], k, size), qos=0)
        sent[n] += 1
        nxt = at + 1.0 / (dtp.topics[n].rate * a.scale)
        if time.time() - nxt > 1.0:   # far behind (suspended process): resynchronize
            nxt = time.time()
        heapq.heappush(heap, (nxt, n, k + 1))
    time.sleep(0.2)
    client.loop_stop()
    client.disconnect()
    print("raw_load: sent " + ", ".join("%s=%d" % kv for kv in sent.items()), file=sys.stderr)


if __name__ == "__main__":
    main()
