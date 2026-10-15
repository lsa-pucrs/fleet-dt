#!/usr/bin/env python3
"""Monitor bridge of the emulator (SPEC_v2.md Sections 1 and 4.3, PR6).

Receives the JSON batches of the AP monitor that fdt-emu sends over UDP every 5 ms and publishes each
one unchanged, QoS 0, on `fleet/monitor` of the GS broker, which the host reaches through the QEMU
user-net forward of the GS VM (127.0.0.1:2883 -> GS:1883).

Latency accounting (one JSON line per --stats-every seconds in --stats):
  age_rx    host time at the UDP reception minus the batch end to_us (emulator -> bridge)
  pub       duration of the publish call
  loop      host time at which the bridge's own subscription to fleet/monitor delivers the batch,
            minus the UDP reception (bridge -> GS broker -> back to the host: twice the one-way path)
  gap       batches whose from_us differs from the to_us of the previous batch
"""
import argparse
import json
import socket
import threading
import time

import paho.mqtt.client as mqtt


def pct(v, q):
    if not v:
        return None
    v = sorted(v)
    return round(v[min(len(v) - 1, int(q * len(v)))], 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--listen", default="127.0.0.1:9550")
    ap.add_argument("--broker", default="127.0.0.1:2883")
    ap.add_argument("--topic", default="fleet/monitor")
    ap.add_argument("--stats", default="")
    ap.add_argument("--stats-every", type=float, default=10)
    a = ap.parse_args()
    lh, lp = a.listen.rsplit(":", 1)
    bh, bp = a.broker.rsplit(":", 1)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    s.bind((lh, int(lp)))

    lock = threading.Lock()
    st = {"n": 0, "pub_fail": 0, "gap": 0, "age_rx": [], "pub": [], "loop": []}
    sent = {}   # to_us -> host time of reception (for the loop measurement)

    def on_message(c, u, m):
        t = time.time_ns() / 1000
        try:
            to = json.loads(m.payload)["to_us"]
        except (ValueError, KeyError):
            return
        with lock:
            t0 = sent.pop(to, None)
            if t0 is not None:
                st["loop"].append(t - t0)

    def on_connect(c, u, flags, rc, props=None):
        print("connected to %s: %s" % (a.broker, rc), flush=True)
        c.subscribe(a.topic, 0)

    pub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fdt-monitor-bridge")
    pub.on_connect = on_connect
    pub.on_message = on_message
    pub.reconnect_delay_set(0.2, 2)
    pub.connect_async(bh, int(bp), keepalive=10)
    pub.loop_start()
    sock = pub.socket()
    last_to, t_stats = None, time.time()
    out = open(a.stats, "a") if a.stats else None
    while True:
        data, _ = s.recvfrom(1 << 20)
        trx = time.time_ns() / 1000
        try:
            j = json.loads(data)
            frm, to = j["from_us"], j["to_us"]
        except (ValueError, KeyError):
            continue
        t1 = time.perf_counter()
        info = pub.publish(a.topic, data, qos=0)
        dpub = (time.perf_counter() - t1) * 1e6
        with lock:
            st["n"] += 1
            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                st["pub_fail"] += 1
            else:
                sent[to] = trx
            if last_to is not None and frm != last_to:
                st["gap"] += 1
            st["age_rx"].append(trx - to)
            st["pub"].append(dpub)
            if len(sent) > 10000:
                sent.clear()
        last_to = to
        if time.time() - t_stats >= a.stats_every:
            with lock:
                rec = {"t": time.time(), "batches": st["n"], "pub_fail": st["pub_fail"], "gap": st["gap"],
                       "age_rx_us": {"p50": pct(st["age_rx"], .5), "p99": pct(st["age_rx"], .99),
                                     "max": pct(st["age_rx"], 1)},
                       "pub_us": {"p50": pct(st["pub"], .5), "p99": pct(st["pub"], .99)},
                       "loop_us": {"n": len(st["loop"]), "p50": pct(st["loop"], .5), "p99": pct(st["loop"], .99),
                                   "max": pct(st["loop"], 1)}}
                for k in ("n", "pub_fail", "gap"):
                    st[k] = 0
                for k in ("age_rx", "pub", "loop"):
                    st[k] = []
            line = json.dumps(rec)
            print(line, flush=True)
            if out:
                out.write(line + "\n")
                out.flush()
            t_stats = time.time()


if __name__ == "__main__":
    main()
