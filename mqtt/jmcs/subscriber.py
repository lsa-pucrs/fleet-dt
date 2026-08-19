"""Station-side consumer: everything the boat publishes, counted per second.

Subscribes ``boat/<id>/#`` on the boat's broker across the link and prints a
live table -- messages/s and kbps per topic -- once per second; optionally
writes one CSV row per (second, topic) for later analysis. This is the
measured side of "Mbps? which packets?": run it while the publisher (and the
ZED publisher) run on the Pi.

    python3 mqtt/jmcs/subscriber.py --broker <pi-address> --boat-id b1 --csv load.csv
"""

import argparse
import csv
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

import paho.mqtt.client as mqtt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from topics import boat_subscription  # noqa: E402


def main() -> None:
    """Subscribe, print the per-topic table every second, write the CSV."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", required=True, help="the BOAT's broker, across the link")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--boat-id", default="b1")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--csv", default="", help="write (second, topic, msgs, bytes) rows")
    args = parser.parse_args()

    lock = threading.Lock()
    window: dict[str, list[int]] = defaultdict(lambda: [0, 0])
    history: list[tuple[int, str, int, int]] = []

    def on_message(_client, _userdata, wire) -> None:
        """Account one message to the current window."""
        with lock:
            bucket = window[wire.topic]
            bucket[0] += 1
            bucket[1] += len(wire.payload)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="jmcs-subscriber")
    client.on_message = on_message
    client.connect(args.broker, args.port)
    client.subscribe(boat_subscription(args.boat_id))
    client.loop_start()
    print(f"subscribed {boat_subscription(args.boat_id)} for {args.duration_s:.0f} s")

    try:
        for second in range(int(args.duration_s)):
            time.sleep(1.0)
            with lock:
                snapshot = {topic: counts[:] for topic, counts in window.items()}
                window.clear()
            total_kbps = sum(size for _, size in snapshot.values()) * 8 / 1000
            print(f"\n[{second + 1:4d}s] total {total_kbps:8.1f} kbps")
            for topic, (messages, size) in sorted(snapshot.items(), key=lambda e: -e[1][1]):
                print(f"  {topic:30} {messages:5d} msg/s {size * 8 / 1000:8.1f} kbps")
                history.append((second, topic, messages, size))
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()

    if args.csv:
        with open(args.csv, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["second", "topic", "messages", "bytes"])
            writer.writerows(history)
        print(f"wrote {args.csv}")


if __name__ == "__main__":
    main()
