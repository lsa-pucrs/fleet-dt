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
  (t_us: release time, microseconds of the Unix time) to --log (default: none);
* temporal video layers (topics with "group", "layer" and "layers", e.g. video_1 .. video_16 of
  dtp_v2.json, see video_layers.py): frame k of the group (frames at k / f, f = layers * rate of a layer,
  times --fps-scale) is output j = (k - rev(layer - 1)) / layers of topic <group>_<rev(k mod layers) + 1>,
  so the layer topics interleave and the first j layers carry j / layers of the frame rate spread
  uniformly; the phase of the group is common to its layers, the size of frame k comes from the trace
  (one position per frame of the group, a seeded offset per group) or from the lognormal model seeded by
  (seed, vessel, group, k), times --video-scale; "out" is the frame index k. The defaults of
  --video-scale and --fps-scale come from "video_layers"."vessels".<vessel> of the DTP (size_scale and
  fps_scale), so the vessels offer different video loads;
* the DTP of the paper stack (dtp.json, SPEC.md format: "rate" in messages per second and "bound")
  works as well: its layer topics become the output topics of the layers (frame rate fps / layers,
  n = ceil(rate Ts) messages of at most "bound" bytes per frame) and the other topics streams of one
  message of "bound" bytes; J_l is the "jitter" of the topic, else --jitter-ms (default 0). The
  per-vessel DTP of `video_layers.py resolve` gives the bounds of the vessel.

Python >= 3.7, paho-mqtt 1.x or 2.x.

  fdt_source.py --dtp dtp_v2.json --vessel 1 [--host 127.0.0.1 --port 1883] [--topics state,video]
                [--stream-size max|uniform] [--median 11000 --sigma 0.5 | --size-trace sizes.txt]
                [--load 1.0 --load-mode size|rate] [--phase random|zero] [--duration S] [--seed 1]
                [--video-scale S --fps-scale F] [--jitter-ms J] [--log out.jsonl]
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
        self.group = d.get("group")
        self.layer = int(d["layer"]) if d.get("layer") is not None else None
        self.layers = int(d.get("layers", 0)) or None
        if self.layer is not None:
            if self.group is None or not self.layers or not 1 <= self.layer <= self.layers:
                raise ValueError("topic %s: a layer topic needs group, layers and 1 <= layer <= layers" % self.name)
            self.residue = bitrev(self.layer - 1, layer_bits(self.layers))
        else:
            self.residue = None
        if self.kind not in ("stream", "output"):
            raise ValueError("topic %s: kind must be stream or output" % self.name)
        if self.kind == "output" and self.s <= OUT_HEADER.size:
            raise ValueError("topic %s: s must exceed the 8-byte output header" % self.name)


def layer_bits(layers):
    b = int(round(math.log(layers, 2)))
    if (1 << b) != layers:
        raise ValueError("the number of layers must be a power of two (got %d)" % layers)
    return b


def bitrev(k, bits):
    """Reverses the `bits` low bits of k (video_layers.bitrev)."""
    r = 0
    for _ in range(bits):
        r = (r << 1) | (k & 1)
        k >>= 1
    return r


LAYER_DEFAULTS = {"base": "video", "layers": 16, "fps": 15.0}


def v1_topic(t, d, jitter_s):
    """Topic of a DTP in the format of SPEC.md (dtp.json of the paper stack: "name", "rate" in messages per
    second, "bound" in bytes, optional "jitter") in the terms of this source.

    A layer topic <base>_<j> of "video_layers" (video_layers.py) becomes the output topic of layer j:
    one frame every layers / fps seconds, split into at most n = ceil(rate Ts) messages of at most
    "bound" bytes (the msgs_per_frame of the DTP). Every other topic is a stream of one message of
    "bound" bytes per output at "rate". J_l is the "jitter" of the topic, else jitter_s."""
    J = float(t["jitter"]) if t.get("jitter") is not None else float(jitter_s)
    vl = dict(LAYER_DEFAULTS)
    vl.update({k: v for k, v in (d.get("video_layers") or {}).items() if k in LAYER_DEFAULTS})
    L = int(vl["layers"])
    base = str(vl["base"])
    name = str(t["name"])
    j = None
    if d.get("video_layers") is not None or name.startswith(base + "_"):
        suffix = name[len(base) + 1:] if name.startswith(base + "_") else ""
        if suffix.isdigit() and 1 <= int(suffix) <= L:
            j = int(suffix)
    if j is not None:
        n = max(1, int(math.ceil(float(t["rate"]) * float(d["Ts"]) - 1e-9)))
        return Topic({"name": name, "rate": float(vl["fps"]) / L, "jitter": J, "n": n, "s": int(t["bound"]),
                      "kind": "output", "group": base, "layer": j, "layers": L})
    return Topic({"name": name, "rate": float(t["rate"]), "jitter": J, "n": 1, "s": int(t["bound"]), "kind": "stream"})


def load_dtp(path, jitter_s=0.0):
    """DTP v2 (SPEC_v2.md) or the DTP of the paper stack (SPEC.md format, see v1_topic)."""
    with open(path) as f:
        d = json.load(f)
    epoch = d.get("epoch")
    topics = [Topic(t) if "s" in t else v1_topic(t, d, jitter_s) for t in d["topics"]]
    return {"epoch": DEFAULT_EPOCH if epoch is None else float(epoch), "Ts": float(d["Ts"]),
            "T1": float(d.get("T1", 0.0)), "T2": float(d.get("T2", 0.0)),
            "required": set(d.get("required", [])), "topics": topics,
            "video_layers": d.get("video_layers") or {}}


def vessel_video(dtp, vessel):
    """size_scale and fps_scale of a vessel from "video_layers"."vessels" (1.0 when absent)."""
    v = (dtp.get("video_layers") or {}).get("vessels", {}).get(str(vessel), {})
    return float(v.get("size_scale", 1.0)), float(v.get("fps_scale", 1.0))


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
        r = tp.rate * (self.a.fps_scale if tp.layer is not None else 1.0)
        if tp.kind == "output" and self.a.load_mode == "rate":
            return r * self.a.load
        return r

    def frame(self, tp, j):
        """Frame index k of the group of a layer topic for its output j: k = rev(layer - 1) + layers j."""
        return tp.residue + tp.layers * j

    def draw(self, tp, j, what):
        """random.Random of (seed, vessel, topic, j, what): the draws of output j do not depend on the
        start time of the process (mode --t0)"""
        return random.Random("%d:%d:%s:%d:%s" % (self.a.seed, self.a.vessel, tp.name, j, what))

    def output_size(self, tp, j=None):
        cap = tp.n * (tp.s - OUT_HEADER.size)
        if tp.layer is not None:   # frame k of the group: one trace position or one seeded draw per frame
            k = self.frame(tp, j)
            if self.trace is not None:
                off = random.Random("%d:%d:%s:trace" % (self.a.seed, self.a.vessel, tp.group)).randrange(len(self.trace))
                size = self.trace[(off + k) % len(self.trace)]
            else:
                median = self.a.median if self.a.median else 0.5 * cap
                rng = random.Random("%d:%d:%s:%d:size" % (self.a.seed, self.a.vessel, tp.group, k))
                size = int(round(rng.lognormvariate(math.log(median), self.a.sigma)))
            size = int(round(size * self.a.video_scale))
            if self.a.load_mode == "size":
                size = int(round(size * self.a.load))
            size = max(1, size)
            return min(size, cap), size > cap
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
        size, trunc = self.output_size(tp, j if (seeded or tp.layer is not None) else None)
        chunk = tp.s - OUT_HEADER.size
        m = max(1, int(math.ceil(size / float(chunk))))
        if tp.layer is not None:
            oid = self.frame(tp, j) & 0xFFFFFFFF
        elif seeded:
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

    def group_phase(self, tp, period):
        """Phase of layer topic tp: a phase common to the group (zero, or seeded in one frame period)
        plus rev(layer - 1) frame periods, so frame k of the group is released at phase + k / f."""
        fp = period / tp.layers
        if self.a.phase == "zero":
            g = 0.0
        else:
            g = random.Random("%d:%d:%s:phase" % (self.a.seed, self.a.vessel, tp.group)).uniform(0, fp)
        return g + tp.residue * fp

    def phase(self, tp, period):
        """seeded phase of topic tp relative to t0 (mode --t0)"""
        if tp.layer is not None:
            return self.group_phase(tp, period)
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
            if tp.layer is not None:    # layers keep their interleaving relative to the epoch
                ph = self.dtp["epoch"] + self.group_phase(tp, period)
                j0 = int(math.ceil((t0 - ph) / period))
                base = ph + j0 * period
                heapq.heappush(heap, (base + self.rng.uniform(0, tp.jitter), base, tp.name, tp, j0))
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
            if time.time() - nxt > 1.0 and tp.layer is None:   # far behind (suspended process): resynchronize
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
    ap.add_argument("--video-scale", type=float, default=None,
                    help="size scale of the frames of the layer topics (default: size_scale of the vessel in the DTP)")
    ap.add_argument("--fps-scale", type=float, default=None,
                    help="frame-rate scale of the layer topics (default: fps_scale of the vessel in the DTP)")
    ap.add_argument("--jitter-ms", type=float, default=0.0,
                    help="release jitter J of the topics without \"jitter\" in a DTP of the SPEC.md format")
    a = ap.parse_args(argv)
    a.topics = set(n for n in a.topics.split(",") if n) if a.topics else None
    if a.jitter_ms < 0:
        ap.error("--jitter-ms must be >= 0")
    dtp = load_dtp(a.dtp, a.jitter_ms * 1e-3)
    vs, vf = vessel_video(dtp, a.vessel)
    a.video_scale = vs if a.video_scale is None else a.video_scale
    a.fps_scale = vf if a.fps_scale is None else a.fps_scale
    if not a.video_scale > 0 or not a.fps_scale > 0:
        ap.error("--video-scale and --fps-scale must be positive")
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
