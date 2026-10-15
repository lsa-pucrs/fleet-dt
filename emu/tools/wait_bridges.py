#!/usr/bin/env python3
"""Waits (in the GS VM) until N bridges of the GS broker report connected on fleet/bridge/<name>/state
(Mosquitto notifications, retained, payload "1"). Exit status 0 when all are up within --timeout.

  wait_bridges.py --n 8 [--broker 127.0.0.1:1883] [--timeout 30]
"""
import argparse
import sys
import threading
import time

import paho.mqtt.client as mqtt


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--broker", default="127.0.0.1:1883")
    ap.add_argument("--timeout", type=float, default=30)
    a = ap.parse_args()
    state = {}
    done = threading.Event()

    def on_message(c, u, m):
        state[m.topic] = m.payload.strip() == b"1"
        if sum(state.values()) >= a.n:
            done.set()

    h, p = a.broker.rsplit(":", 1)
    t0 = time.time()
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fdt-wait-bridges")
    c.on_message = on_message
    c.on_connect = lambda c, u, f, rc, pr=None: c.subscribe("fleet/bridge/+/state", 0)
    while True:
        try:
            c.connect(h, int(p), keepalive=10)
            break
        except OSError:
            if time.time() - t0 > a.timeout:
                print("no broker at %s" % a.broker)
                sys.exit(1)
            time.sleep(0.2)
    c.loop_start()
    ok = done.wait(max(0.0, a.timeout - (time.time() - t0)))
    c.loop_stop()
    up = sorted(k.split("/")[2] for k, v in state.items() if v)
    print("%s: %d/%d bridges up after %.1f s: %s" % ("ok" if ok else "timeout", len(up), a.n, time.time() - t0,
                                                      " ".join(up)))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
