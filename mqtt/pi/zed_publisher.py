"""Boat-side video: the ZED through the broker as MJPEG, one frame per message.

Grabs frames (the ZED enumerates as a UVC stereo camera; any V4L2 camera
works for the software test), JPEG-encodes each one and publishes it on
``zed/frame``; ``zed/info`` is published retained so a late subscriber learns
the stream's shape first. Whole frames, independently decodable: a lost
message costs exactly one frame. Video through the broker is the design under
test: it measures what the broker adds to a video path.

Requires opencv-python. Run on the Pi:

    python3 mqtt/pi/zed_publisher.py --broker localhost --boat-id b1 --fps 15
"""

import argparse
import json
import sys
import time
from pathlib import Path

import paho.mqtt.client as mqtt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from topics import zed_frame, zed_info  # noqa: E402

try:
    import cv2
except ImportError:  # pragma: no cover
    sys.exit("zed_publisher needs opencv-python: pip install opencv-python")


def main() -> None:
    """Open the camera, announce the stream, publish frames at the rate."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--boat-id", default="b1")
    parser.add_argument("--camera", type=int, default=0, help="V4L2 index")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--quality", type=int, default=70, help="JPEG quality 1-100")
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.camera)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not capture.isOpened():
        sys.exit(f"camera {args.camera} did not open")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"zed-{args.boat_id}")
    client.connect(args.broker, args.port)
    client.loop_start()
    info = {"resolution": f"{args.width}x{args.height}", "fps": args.fps,
            "quality": args.quality}
    client.publish(zed_info(args.boat_id), json.dumps(info).encode(), retain=True)

    period_s = 1.0 / args.fps
    frames = 0
    total_bytes = 0
    started = time.monotonic()
    print(f"streaming {info} on {zed_frame(args.boat_id)}; Ctrl-C stops")
    try:
        while True:
            grabbed, frame = capture.read()
            if not grabbed:
                print("frame grab failed; stopping")
                break
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality]
            )
            if ok:
                client.publish(zed_frame(args.boat_id), encoded.tobytes())
                frames += 1
                total_bytes += encoded.size
            time.sleep(period_s)
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = max(1e-6, time.monotonic() - started)
        capture.release()
        client.loop_stop()
        client.disconnect()
        print(f"{frames} frames, {total_bytes} B, "
              f"{total_bytes * 8 / elapsed / 1000:.0f} kbps average")


if __name__ == "__main__":
    main()
