#!/usr/bin/env python3
"""Synthetic sources of a Fleet-DT v2 vessel (SPEC_v2.md Sections 1, 2 and 4; paper Def. topic).

One process per vessel publishes every topic of the DTP v2 on boat/b<i>/raw/<topic> (QoS 0) of the
vessel broker:

* release instants: output j of topic l is released at phase_l + j / r_l + U(0, J_l) on the host clock;
  phase_l is 0 (aligned with the step clock, --phase zero) or uniform in [0, 1 / r_l) (--phase random);
  with --t0 (experiments), j counts from the instant t0 (the start of the first step of the run), every
  random draw (phase, jitter, size) is a function of (--seed, vessel, topic, j) only, so two runs with the
  same seed release the same outputs at the same instants relative to t0, and the phase of each topic of R
  stays more than J_l + 2 ms away from v_i(t) = start(t) + T1 + T2 (Corollary state);
* stream topics: each output is n_l messages of --stream-size bytes (max: s_l; uniform: U[s_l / 2, s_l]);
  the content starts with seq:u32 and the release time in microseconds (u64) when room allows;
* output topics: the size of each output comes from a lognormal model (--median, --sigma) or from a
  size trace (--size-trace, one size in bytes per line, replayed cyclically); the source splits the
  output into at most n_l messages of at most s_l bytes, each starting with the 8-byte header
  out:u32, k:u16, n:u16 (network order); an output larger than n_l (s_l - 8) bytes is truncated (logged);
* --load rho scales the offered load of the output topics: --load-mode size multiplies each output size
  (capped at n_l (s_l - 8)), --load-mode rate multiplies the output rate (rho > 1 exceeds the DTP);
* the socket of the MQTT client runs without the Nagle algorithm (TCP_NODELAY);
* every published output is logged as one JSON line {"topic", "out", "n", "bytes", "t_us", "trunc"}
  (t_us: release time, microseconds of the Unix time) to --log (default: none).

Python >= 3.7, paho-mqtt 1.x or 2.x.

  fdt_source.py --dtp dtp_v2.json --vessel 1 [--host 127.0.0.1 --port 1883] [--topics state,video]
                [--stream-size max|uniform] [--median 11000 --sigma 0.5 | --size-trace sizes.txt]
                [--load 1.0 --load-mode size|rate] [--phase random|zero] [--duration S] [--seed 1]
                [--log out.jsonl]
"""
import argparse
import heapq
import json
import math
import os
import random
import signal
import socket
import struct
import sys
import threading
import time

import paho.mqtt.client as mqtt

OUT_HEADER = struct.Struct("!IHH")   # out:u32, k:u16, n:u16
DEFAULT_EPOCH = 1767225600.0


class Topic(object):
    def __init__(self, d):
        self.name = str(d["name"])
        self.rate = float(d["rate"])
        self.jitter = float(d.get("jitter", 0.0))
        self.n = int(d.get("n", 1))
        self.s = int(d["s"])
        self.kind = str(d["kind"])
        self.qos = int(d.get("qos", 0))
        if self.kind not in ("stream", "output"):
            raise ValueError("topic %s: kind must be stream or output" % self.name)
        if self.kind == "output" and self.s <= OUT_HEADER.size:
            raise ValueError("topic %s: s must exceed the 8-byte output header" % self.name)


def load_dtp(path):
    with open(path) as f:
        d = json.load(f)
    epoch = d.get("epoch")
    return {"epoch": DEFAULT_EPOCH if epoch is None else float(epoch), "Ts": float(d["Ts"]),
            "T1": float(d.get("T1", 0.0)), "T2": float(d.get("T2", 0.0)),
            "required": set(d.get("required", [])), "topics": [Topic(t) for t in d["topics"]]}


def load_trace(path):
    sizes = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                sizes.append(max(1, int(float(line.split()[0]))))
    if not sizes:
        raise ValueError("%s: no sizes" % path)
    return sizes


def make_client(vessel, host, port):
    cid = "fdt-source-b%d-%d-%06x" % (vessel, os.getpid(), random.getrandbits(24))
    try:   # paho-mqtt 2.x
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid, clean_session=True)
    except AttributeError:   # paho-mqtt 1.x
        c = mqtt.Client(client_id=cid, clean_session=True)
    connected = threading.Event()

    def on_connect(client, userdata, flags, rc, *rest):
        if (rc if isinstance(rc, int) else getattr(rc, "value", 1)) == 0:
            connected.set()

    def on_socket_open(client, userdata, sock):
        set_nodelay(sock)

    c.on_connect = on_connect
    try:
        c.on_socket_open = on_socket_open
    except AttributeError:
        pass
    c.max_queued_messages_set(0)
    c.connect(host, port, keepalive=10)
    set_nodelay(c.socket())
    c.loop_start()
    return c, connected


def set_nodelay(sock):
    try:
        if sock is not None:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except (OSError, AttributeError):
        pass


class Source(object):
    def __init__(self, a, dtp, rng):
        self.a, self.dtp, self.rng = a, dtp, rng
        self.trace = load_trace(a.size_trace) if a.size_trace else None
        self.trace_pos = {}
        self.out_id = {}
        self.log = open(a.log, "a") if a.log else None
        self.count = {}

    def rate(self, tp):
        if tp.kind == "output" and self.a.load_mode == "rate":
            return tp.rate * self.a.load
        return tp.rate

    def draw(self, tp, j, what):
        """random.Random of (seed, vessel, topic, j, what): the draws of output j do not depend on the
        start time of the process (mode --t0)"""
        return random.Random("%d:%d:%s:%d:%s" % (self.a.seed, self.a.vessel, tp.name, j, what))

    def output_size(self, tp, j=None):
        cap = tp.n * (tp.s - OUT_HEADER.size)
        if self.trace is not None:
            if j is not None:     # mode --t0: a seeded start position in the trace, then output j
                off = random.Random("%d:%d:%s:trace" % (self.a.seed, self.a.vessel, tp.name)).randrange(len(self.trace))
                size = self.trace[(off + j) % len(self.trace)]
            else:
                k = self.trace_pos.get(tp.name, 0)
                size = self.trace[k % len(self.trace)]
                self.trace_pos[tp.name] = k + 1
        else:
            median = self.a.median if self.a.median else 0.5 * cap
            rng = self.draw(tp, j, "size") if j is not None else self.rng
            size = int(round(rng.lognormvariate(math.log(median), self.a.sigma)))
        if self.a.load_mode == "size":
            size = int(round(size * self.a.load))
        size = max(1, size)
        return min(size, cap), size > cap

    def stream_size(self, tp):
        if self.a.stream_size == "uniform":
            return self.rng.randint(max(1, tp.s // 2), tp.s)
        return tp.s

    def messages(self, tp, j, now_us):
        """the messages of output j of topic tp and the logged size"""
        seeded = self.a.t0 is not None
        if tp.kind == "stream":
            msgs, total = [], 0
            rng = self.draw(tp, j, "stream") if seeded else self.rng
            for _ in range(tp.n):
                size = rng.randint(max(1, tp.s // 2), tp.s) if self.a.stream_size == "uniform" else tp.s
                head = struct.pack("!IQ", j & 0xFFFFFFFF, now_us)
                body = (head + bytes(max(0, size - len(head))))[:size]
                msgs.append(body)
                total += size
            return msgs, total, False, j
        size, trunc = self.output_size(tp, j if seeded else None)
        chunk = tp.s - OUT_HEADER.size
        m = max(1, int(math.ceil(size / float(chunk))))
        if seeded:
            oid = j & 0xFFFFFFFF
        else:
            oid = self.out_id.get(tp.name, 0)
            self.out_id[tp.name] = (oid + 1) & 0xFFFFFFFF
        msgs, left = [], size
        for k in range(m):
            part = min(chunk, left)
            left -= part
            body = struct.pack("!Q", now_us) if part >= 8 else b""
            body = (body + bytes(max(0, part - len(body))))[:part]
            msgs.append(OUT_HEADER.pack(oid, k, m) + body)
        return msgs, size, trunc, oid

    def phase(self, tp, period):
        """seeded phase of topic tp relative to t0 (mode --t0)"""
        rng = self.draw(tp, 0, "phase")
        if self.a.phase == "zero":
            return 0.0
        d = self.dtp
        if tp.name in d["required"] and abs(period - d["Ts"]) < 1e-9:
            # keep the release (with its jitter) more than J + 2 ms away from v(t) = start + T1 + T2
            lo = d["T1"] + d["T2"] + tp.jitter + 0.002
            hi = d["T1"] + d["T2"] + d["Ts"] - 2 * tp.jitter - 0.002
            if hi > lo:
                return rng.uniform(lo, hi) % period
        return rng.uniform(0, period)

    def run(self, client, stop, end):
        heap = []
        t0 = time.time()
        for tp in self.dtp["topics"]:
            r = self.rate(tp)
            if r <= 0 or (self.a.topics and tp.name not in self.a.topics):
                continue
            period = 1.0 / r
            if self.a.t0 is not None:
                ph = self.a.t0 + self.phase(tp, period)
                j0 = int(math.ceil((t0 + 0.05 - ph) / period))     # first output at least 50 ms ahead
                base = ph + j0 * period
                heapq.heappush(heap, (base + self.draw(tp, j0, "jit").uniform(0, tp.jitter), base, tp.name, tp, j0))
                continue
            if self.a.phase == "zero":
                base = self.dtp["epoch"] + math.ceil((t0 - self.dtp["epoch"]) / period) * period
            else:
                base = t0 + self.rng.uniform(0, period)
            heapq.heappush(heap, (base + self.rng.uniform(0, tp.jitter), base, tp.name, tp, 0))
        while heap and not stop.is_set():
            at, nominal, name, tp, j = heapq.heappop(heap)
            if end is not None and at >= end:
                break
            dt = at - time.time()
            if dt > 0 and stop.wait(dt):
                break
            now_us = int(time.time() * 1e6)
            res = self.messages(tp, j, now_us)
            topic = "boat/b%d/raw/%s" % (self.a.vessel, name)
            for body in res[0]:
                client.publish(topic, body, qos=0)
            self.count[name] = self.count.get(name, 0) + 1
            if self.log is not None:
                rec = {"topic": name, "out": res[3], "j": j, "n": len(res[0]), "bytes": res[1],
                       "t_us": now_us, "trunc": bool(res[2])}
                self.log.write(json.dumps(rec, separators=(",", ":")) + "\n")
                self.log.flush()
            nxt = nominal + 1.0 / self.rate(tp)
            if self.a.t0 is not None:
                heapq.heappush(heap, (nxt + self.draw(tp, j + 1, "jit").uniform(0, tp.jitter), nxt, name, tp, j + 1))
                continue
            if time.time() - nxt > 1.0:   # far behind (suspended process): resynchronize
                nxt = time.time()
            heapq.heappush(heap, (nxt + self.rng.uniform(0, tp.jitter), nxt, name, tp, j + 1))
        if self.log is not None:
            self.log.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--vessel", type=int, required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--topics", default=None, help="comma-separated subset of the DTP topics (default: all)")
    ap.add_argument("--stream-size", choices=("max", "uniform"), default="max")
    ap.add_argument("--median", type=float, default=None,
                    help="median output size of the lognormal model, bytes (default: n (s - 8) / 2)")
    ap.add_argument("--sigma", type=float, default=0.5, help="sigma of the lognormal model")
    ap.add_argument("--size-trace", default=None, help="file with one output size (bytes) per line")
    ap.add_argument("--load", type=float, default=1.0, help="offered-load ratio rho of the output topics")
    ap.add_argument("--load-mode", choices=("size", "rate"), default="size")
    ap.add_argument("--phase", choices=("random", "zero"), default="random")
    ap.add_argument("--duration", type=float, default=None, help="seconds (default: until SIGTERM)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--t0", type=float, default=None,
                    help="start of the first step of the run (Unix seconds): seeded draws per output (experiments)")
    ap.add_argument("--log", default=None, help="JSON lines of the published outputs")
    a = ap.parse_args(argv)
    a.topics = set(n for n in a.topics.split(",") if n) if a.topics else None
    dtp = load_dtp(a.dtp)
    names = set(tp.name for tp in dtp["topics"])
    if a.topics and not a.topics <= names:
        ap.error("unknown topics: %s" % ",".join(sorted(a.topics - names)))
    if a.load <= 0:
        ap.error("--load must be positive")
    rng = random.Random(a.seed * 1000 + a.vessel)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    client, connected = make_client(a.vessel, a.host, a.port)
    if not connected.wait(10):
        print("fdt_source: no connection to %s:%d" % (a.host, a.port), file=sys.stderr)
        sys.exit(1)
    src = Source(a, dtp, rng)
    end = time.time() + a.duration if a.duration else None
    src.run(client, stop, end)
    time.sleep(0.2)
    client.loop_stop()
    client.disconnect()
    print("fdt_source b%d: outputs %s" % (a.vessel, ", ".join("%s=%d" % kv for kv in sorted(src.count.items()))),
          file=sys.stderr)


if __name__ == "__main__":
    main()
