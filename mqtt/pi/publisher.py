"""Boat-side publisher: the Navio2 sensor groups over MQTT, plus echo and cmd.

Publishes every sensor group of ``topics.py`` at its own rate (synthetic
values shaped like the real Navio2 channels -- swap in MAVLink readings when
the test moves onto the boat), echoes ``ping`` payloads byte-for-byte on
``pong`` for the station's RTT probe, and logs received ``cmd`` messages.

Run on the Pi (or any host on the boat side of the link):

    python3 mqtt/pi/publisher.py --broker localhost --boat-id b1
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import paho.mqtt.client as mqtt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from topics import SENSOR_FIELDS, SENSOR_RATES_HZ, cmd, ping, pong, sensor  # noqa: E402


def synthetic_payload(group: str, step: int) -> bytes:
    """One JSON payload for a sensor group, values varying with the step."""
    wave = math.sin(step / 8.0)
    values = {
        "imu": (0.1 * wave, 0.05 * wave, 9.81, 0.01 * wave, 0.02 * wave, 0.1 * wave),
        "mag": (22.0 + wave, -5.0, 40.0),
        "gps": (-30.05 + step * 1e-6, -51.20 + step * 1e-6, 2.0, 1.2, 0.4, 0.0),
        "baro": (101_325.0 + 10 * wave, 24.0),
        "power": (12.4 - step * 1e-4, 3.1 + 0.2 * wave),
        "state": (-30.05, -51.20, 2.0, 2.0 * wave, 1.0 * wave, 90.0 + 5 * wave,
                  1.2, 0.1, 0.0, 0.01, 0.02, 0.05),
    }[group]
    payload = dict(zip(SENSOR_FIELDS[group], values))
    payload["seq"] = step
    payload["t"] = time.time()
    return json.dumps(payload).encode()


def main() -> None:
    """Connect, wire echo and cmd, publish every group at its rate."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--boat-id", default="b1")
    parser.add_argument("--duration-s", type=float, default=0.0, help="0 runs until Ctrl-C")
    args = parser.parse_args()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"pi-{args.boat_id}")

    def on_message(_client, _userdata, wire) -> None:
        """Echo pings untouched; log commands."""
        if wire.topic == ping(args.boat_id):
            client.publish(pong(args.boat_id), wire.payload)
        else:
            print(f"cmd: {wire.payload.decode(errors='replace')}")

    client.on_message = on_message
    client.connect(args.broker, args.port)
    client.subscribe(ping(args.boat_id))
    client.subscribe(cmd(args.boat_id))
    client.loop_start()

    next_due = {group: 0.0 for group in SENSOR_RATES_HZ}
    steps = dict.fromkeys(SENSOR_RATES_HZ, 0)
    started = time.monotonic()
    print(f"publishing {sorted(SENSOR_RATES_HZ)} for boat {args.boat_id}; Ctrl-C stops")
    try:
        while args.duration_s <= 0 or time.monotonic() - started < args.duration_s:
            now = time.monotonic() - started
            for group, rate_hz in SENSOR_RATES_HZ.items():
                if now >= next_due[group]:
                    client.publish(sensor(args.boat_id, group),
                                   synthetic_payload(group, steps[group]))
                    steps[group] += 1
                    next_due[group] += 1.0 / rate_hz
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()
        print(f"done: {dict(steps)}")


if __name__ == "__main__":
    main()
