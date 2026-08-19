"""Station-side RTT probe: ping through the broker, the Pi echoes, measure.

Publishes opaque payloads on ``boat/<id>/ping``; the Pi publisher echoes them
on ``boat/<id>/pong``; the round trip crosses the link twice. Every ping
carries a sequence number, so a late echo is counted late and never
miscounted as the echo of a different ping.

    python3 mqtt/jmcs/probe.py --broker <pi-address> --boat-id b1 --count 200 --csv rtt.csv
"""

import argparse
import csv
import statistics
import sys
import time
from pathlib import Path

import paho.mqtt.client as mqtt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from topics import ping, pong  # noqa: E402


def main() -> None:
    """Send the pings, wait for stragglers, print stats, write the CSV."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", required=True, help="the BOAT's broker, across the link")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--boat-id", default="b1")
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--size-bytes", type=int, default=256)
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument("--csv", default="")
    args = parser.parse_args()

    sent_at_ns: dict[int, int] = {}
    rtts_ms: dict[int, float] = {}

    def on_pong(_client, _userdata, wire) -> None:
        """Match one echo by sequence number and record its RTT."""
        now = time.monotonic_ns()
        sequence = int(wire.payload.split(b":", 1)[0])
        sent = sent_at_ns.pop(sequence, None)
        if sent is not None:
            rtts_ms[sequence] = (now - sent) / 1e6

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="jmcs-probe")
    client.on_message = on_pong
    client.connect(args.broker, args.port)
    client.subscribe(pong(args.boat_id))
    client.loop_start()

    for sequence in range(args.count):
        header = f"{sequence}:".encode()
        payload = header + b"x" * max(0, args.size_bytes - len(header))
        sent_at_ns[sequence] = time.monotonic_ns()
        client.publish(ping(args.boat_id), payload)
        time.sleep(1.0 / args.rate_hz)

    deadline = time.monotonic() + args.timeout_s
    while sent_at_ns and time.monotonic() < deadline:
        time.sleep(0.1)
    client.loop_stop()
    client.disconnect()

    lost = len(sent_at_ns)
    rtts = sorted(rtts_ms.values())
    print(f"pings {args.count}  echoed {len(rtts)}  lost {lost} "
          f"({100 * lost / args.count:.1f}%)  size {args.size_bytes} B")
    if rtts:
        q = statistics.quantiles(rtts, n=100) if len(rtts) >= 2 else [rtts[0]] * 99
        print(f"rtt ms  min {rtts[0]:.2f}  p50 {q[49]:.2f}  p90 {q[89]:.2f}  "
              f"p99 {q[98]:.2f}  max {rtts[-1]:.2f}")

    if args.csv:
        with open(args.csv, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["sequence", "rtt_ms", "size_bytes"])
            for sequence, rtt_ms in sorted(rtts_ms.items()):
                writer.writerow([sequence, f"{rtt_ms:.3f}", args.size_bytes])
        print(f"wrote {args.csv}")


if __name__ == "__main__":
    main()
