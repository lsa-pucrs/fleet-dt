#!/usr/bin/env python3
"""Host-side capture of fleet/monitor from the GS broker (through the QEMU forward 127.0.0.1:2883):
one JSON batch per line, prefixed with the host reception time in us.

  mon_capture.py --broker 127.0.0.1:2883 --out RUN/monitor.jsonl
"""
import argparse
import signal
import sys
import time

import paho.mqtt.client as mqtt


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--broker", default="127.0.0.1:2883")
    ap.add_argument("--topic", default="fleet/monitor")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = open(a.out, "w", buffering=1 << 16)
    h, p = a.broker.rsplit(":", 1)

    def on_msg(c, u, m):
        out.write('{"rx_us":%d,"b":%s}\n' % (time.time_ns() // 1000, m.payload.decode()))

    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fdt-mon-capture")
    c.on_message = on_msg
    c.on_connect = lambda c, u, f, rc, pr=None: c.subscribe(a.topic, 0)
    c.reconnect_delay_set(0.2, 2)
    c.connect_async(h, int(p), keepalive=10)
    signal.signal(signal.SIGTERM, lambda *x: (c.disconnect(), out.flush(), sys.exit(0)))
    c.loop_forever(retry_first_connection=True)


if __name__ == "__main__":
    main()
