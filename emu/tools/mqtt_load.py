#!/usr/bin/env python3
"""MQTT load and measurement clients for the emulator smoke tests (run inside the VMs).

  mqtt_load.py pub --topic boat/b1/tx/load --rate-bps 2e6 --size 1000 --duration 20 [--qos 0]
  mqtt_load.py sub --topic 'boat/+/tx/#' --duration 25 --out sub.json
  mqtt_load.py monsub --duration 20 --out mon.json        (fleet/monitor batches: age, continuity)

The publisher writes, in the first 12 bytes of each payload, the CLOCK_REALTIME of the publish (u64 ns)
and a sequence number (u32); the subscriber reports per topic the messages, the bytes, the lost
sequence numbers and the one-way delay (subscriber clock minus publisher clock; the clocks follow the
GS through chrony).
"""
import argparse
import json
import struct
import threading
import time

import paho.mqtt.client as mqtt


def client(host, port, cid):
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)
    c.connect(host, port, keepalive=30)
    c.loop_start()
    return c


def pct(v, q):
    if not v:
        return None
    v = sorted(v)
    return round(v[min(len(v) - 1, int(q * len(v)))], 1)


def pub(a):
    c = client(a.host, a.port, "load-pub-%s" % a.topic)
    time.sleep(0.5)
    interval = a.size * 8 / a.rate_bps
    pad = b"\0" * (a.size - 12)
    t_end = time.monotonic() + a.duration
    nxt = time.monotonic()
    seq = 0
    while time.monotonic() < t_end:
        c.publish(a.topic, struct.pack("<QI", time.clock_gettime_ns(time.CLOCK_REALTIME), seq) + pad, qos=a.qos)
        seq += 1
        nxt += interval
        dt = nxt - time.monotonic()
        if dt > 0:
            time.sleep(dt)
    time.sleep(1)
    c.loop_stop()
    print(json.dumps({"topic": a.topic, "sent": seq, "rate_bps": a.rate_bps, "size": a.size}))


def sub(a):
    st = {}
    lock = threading.Lock()

    def on_msg(c, u, m):
        t = time.clock_gettime_ns(time.CLOCK_REALTIME)
        if len(m.payload) < 12:
            return
        t0, seq = struct.unpack("<QI", m.payload[:12])
        with lock:
            s = st.setdefault(m.topic, {"n": 0, "bytes": 0, "seqs": set(), "d": [], "t_first": t, "t_last": t})
            s["n"] += 1
            s["bytes"] += len(m.payload)
            s["seqs"].add(seq)
            s["d"].append((t - t0) / 1e3)
            s["t_last"] = t

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="load-sub")
    c.on_message = on_msg
    c.on_connect = lambda c, u, f, rc, p=None: c.subscribe(a.topic, a.qos)
    c.connect(a.host, a.port, keepalive=30)
    c.loop_start()
    time.sleep(a.duration)
    c.loop_stop()
    out = {}
    for k, s in st.items():
        span = max(1e-9, (s["t_last"] - s["t_first"]) / 1e9)
        lost = (max(s["seqs"]) + 1 - len(s["seqs"])) if s["seqs"] else 0
        out[k] = {"msgs": s["n"], "bytes": s["bytes"], "goodput_bps": round(8 * s["bytes"] / span), "lost": lost,
                  "delay_us": {"p50": pct(s["d"], .5), "p99": pct(s["d"], .99), "max": pct(s["d"], 1)}}
    txt = json.dumps(out, indent=1)
    print(txt)
    if a.out:
        open(a.out, "w").write(txt)


def monsub(a):
    st = {"n": 0, "gap": 0, "fr": 0, "busy_us": 0, "age": [], "last_to": None, "span_us": 0}

    def on_msg(c, u, m):
        t = time.clock_gettime_ns(time.CLOCK_REALTIME) / 1e3
        j = json.loads(m.payload)
        st["n"] += 1
        st["fr"] += len(j["fr"])
        st["busy_us"] += sum(e - s for s, e in j["busy"])
        st["span_us"] += j["to_us"] - j["from_us"]
        if st["last_to"] is not None and j["from_us"] != st["last_to"]:
            st["gap"] += 1
        st["last_to"] = j["to_us"]
        st["age"].append(t - j["to_us"])

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="mon-sub")
    c.on_message = on_msg
    c.on_connect = lambda c, u, f, rc, p=None: c.subscribe("fleet/monitor", 0)
    c.connect(a.host, a.port, keepalive=30)
    c.loop_start()
    time.sleep(a.duration)
    c.loop_stop()
    out = {"batches": st["n"], "gaps": st["gap"], "fr_records": st["fr"],
           "busy_fraction": round(st["busy_us"] / max(1, st["span_us"]), 4),
           "age_us": {"p50": pct(st["age"], .5), "p99": pct(st["age"], .99), "max": pct(st["age"], 1)}}
    txt = json.dumps(out, indent=1)
    print(txt)
    if a.out:
        open(a.out, "w").write(txt)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("mode", choices=["pub", "sub", "monsub"])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--topic", default="boat/b1/tx/load")
    ap.add_argument("--rate-bps", type=float, default=2e6)
    ap.add_argument("--size", type=int, default=1000)
    ap.add_argument("--qos", type=int, default=0)
    ap.add_argument("--duration", type=float, default=20)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    {"pub": pub, "sub": sub, "monsub": monsub}[a.mode](a)


if __name__ == "__main__":
    main()
