#!/usr/bin/env python3
"""Video publisher of a Fleet-DT vessel (R2).

Source: ZED stereo camera on USB 3.0 through V4L2 (side-by-side frames of 1344x376, YUV 4:2:2, 15 fps),
read with OpenCV VideoCapture (CAP_V4L2), or a synthetic moving pattern (--synthetic) for tests and
bench runs without a camera.

Each published frame is one JPEG (quality --quality). The program publishes one frame out of every
--divider captured frames; the experiments set the offered load through the quality and the divider.
Each JPEG is split into messages of at most `bound - 8` bytes of JPEG data, where `bound` is the DTP
bound of the topic video; the content of each message is

    frame:u32  k:u16  n:u16  JPEG bytes [k*(bound-8), (k+1)*(bound-8))      (network byte order)

with frame = index of the published frame (mod 2^32), k = index of the message in the frame and
n = number of messages of the frame, so every content holds at most `bound` bytes. The messages go
to boat/b<i>/raw/video with QoS 0.

Usage:
  zed_publisher.py --dtp /etc/fleetdt/dtp.json --vessel 1 [--device /dev/video0] [--quality 50]
                   [--divider 1] [--host 127.0.0.1] [--port 1883]
  zed_publisher.py --dtp dtp.json --vessel 1 --synthetic [--frames 100] [--save-dir out/]
"""
from __future__ import annotations

import argparse
import logging
import math
import os
import signal
import struct
import sys
import threading
import time
from typing import Iterator, List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from common import DTPFile, make_client, raw_topic, sleep_until  # noqa: E402

log = logging.getLogger("zed_publisher")

FRAME_HEADER = struct.Struct("!IHH")   # frame, k, n


def split_frame(jpeg: bytes, frame: int, bound: int) -> List[bytes]:
    """Contents of the messages of one frame: 8-byte header + at most bound - 8 bytes of JPEG data."""
    chunk = bound - FRAME_HEADER.size
    if chunk <= 0:
        raise ValueError("bound %d leaves no room for JPEG data" % bound)
    n = max(1, math.ceil(len(jpeg) / chunk))
    if n > 0xFFFF:
        raise ValueError("frame of %d bytes needs %d messages (more than 65535)" % (len(jpeg), n))
    f = frame & 0xFFFFFFFF
    return [FRAME_HEADER.pack(f, k, n) + jpeg[k * chunk:(k + 1) * chunk] for k in range(n)]


def synthetic_frames(width: int, height: int, fps: float, stop: threading.Event) -> Iterator[np.ndarray]:
    """Side-by-side test pattern at `fps`: moving gradient, moving blocks and the frame index; the right
    half repeats the left half with a horizontal disparity, as a stereo pair."""
    half = width // 2
    yy, xx = np.mgrid[0:height, 0:half].astype(np.float32)
    rng = np.random.default_rng(1)
    noise = rng.integers(0, 24, size=(height, half, 3), dtype=np.uint8)
    period = 1.0 / fps
    t_next = time.time()
    i = 0
    while not stop.is_set():
        if not sleep_until(t_next, stop):
            return
        t_next += period
        if time.time() - t_next > 1.0:          # far behind (suspended process): resynchronize
            t_next = time.time() + period
        ph = i * 0.15
        left = np.empty((height, half, 3), np.uint8)
        left[..., 0] = (127 + 120 * np.sin(xx / 37.0 + ph)).astype(np.uint8)
        left[..., 1] = (127 + 120 * np.sin(yy / 23.0 - ph * 0.7)).astype(np.uint8)
        left[..., 2] = (127 + 120 * np.sin((xx + yy) / 51.0 + ph * 1.3)).astype(np.uint8)
        left = cv2.add(left, noise)
        for b in range(4):
            cx = int((i * (3 + b) + b * 150) % half)
            cy = int(height / 2 + (height / 3) * math.sin(i * 0.05 + b))
            cv2.rectangle(left, (cx - 20, cy - 20), (cx + 20, cy + 20), (255 - 60 * b, 40 * b, 60 + 40 * b), -1)
        cv2.putText(left, "frame %d" % i, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        right = np.roll(left, -12, axis=1)
        yield np.hstack([left, right]) if width == 2 * half else cv2.resize(np.hstack([left, right]), (width, height))
        i += 1


def camera_frames(device: str, width: int, height: int, fps: float, stop: threading.Event) -> Iterator[np.ndarray]:
    dev = int(device) if device.isdigit() else device
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit("cannot open the camera %s with V4L2" % device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    log.info("camera %s: %dx%d at %.1f fps", device, w, h, cap.get(cv2.CAP_PROP_FPS))
    if (w, h) != (width, height):
        log.warning("camera gives %dx%d instead of %dx%d", w, h, width, height)
    failures = 0
    try:
        while not stop.is_set():
            ok, img = cap.read()
            if not ok:
                failures += 1
                if failures > 50:
                    raise SystemExit("camera %s stopped delivering frames" % device)
                time.sleep(0.02)
                continue
            failures = 0
            yield img
    finally:
        cap.release()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Fleet-DT video publisher (R2)")
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--vessel", type=int, required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--device", default="/dev/video0", help="V4L2 device (path or index)")
    ap.add_argument("--width", type=int, default=1344)
    ap.add_argument("--height", type=int, default=376)
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--quality", type=int, default=50, help="JPEG quality 1..100")
    ap.add_argument("--divider", type=int, default=1, help="publish one frame out of every DIVIDER frames")
    ap.add_argument("--synthetic", action="store_true", help="moving test pattern instead of the camera")
    ap.add_argument("--frames", type=int, default=0, help="stop after this number of published frames (0: run)")
    ap.add_argument("--save-dir", default=None, help="also write each published JPEG as <frame>.jpg")
    ap.add_argument("--stats-period", type=float, default=10.0)
    ap.add_argument("--log-level", default="INFO")
    a = ap.parse_args(argv)
    logging.basicConfig(level=a.log_level.upper(), format="%(asctime)s %(name)s %(levelname)s %(message)s")
    if not 1 <= a.quality <= 100 or a.divider < 1:
        ap.error("quality must be in 1..100 and divider >= 1")

    dtp = DTPFile.load(a.dtp)
    bound = dtp.topics["video"].bound
    topic = raw_topic(a.vessel, "video")
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    connected = threading.Event()

    def on_connect(client, userdata, flags, reason_code, properties):
        if not reason_code.is_failure:
            connected.set()

    def on_disconnect(client, userdata, flags, reason_code, properties):
        connected.clear()

    client = make_client("zed", a.vessel, a.host, a.port, on_connect=on_connect)
    client.on_disconnect = on_disconnect
    if not connected.wait(10.0):
        log.warning("no connection to %s:%d yet; frames before the connection are dropped", a.host, a.port)
    if a.save_dir:
        os.makedirs(a.save_dir, exist_ok=True)

    src = (synthetic_frames(a.width, a.height, a.fps, stop) if a.synthetic
           else camera_frames(a.device, a.width, a.height, a.fps, stop))
    params = [int(cv2.IMWRITE_JPEG_QUALITY), a.quality]
    frame = captured = 0
    info = None
    s_frames = s_msgs = s_bytes = 0
    s_t = time.time()
    for img in src:
        captured += 1
        if (captured - 1) % a.divider:
            continue
        ok, enc = cv2.imencode(".jpg", img, params)
        if not ok:
            log.warning("JPEG encoding failed")
            continue
        jpeg = enc.tobytes()
        if connected.is_set():
            msgs = split_frame(jpeg, frame, bound)
            for m in msgs:
                info = client.publish(topic, m, qos=0)
            if a.save_dir:
                with open(os.path.join(a.save_dir, "%d.jpg" % (frame & 0xFFFFFFFF)), "wb") as f:
                    f.write(jpeg)
            s_frames += 1
            s_msgs += len(msgs)
            s_bytes += sum(len(m) for m in msgs)
            frame += 1
        if a.frames and frame >= a.frames:
            break
        if a.stats_period > 0 and time.time() - s_t >= a.stats_period:
            dt = time.time() - s_t
            log.info("%.2f frames/s, %.2f messages/s, %.0f content bytes/s, %.0f bytes/frame",
                     s_frames / dt, s_msgs / dt, s_bytes / dt, s_bytes / max(s_frames, 1))
            s_frames = s_msgs = s_bytes = 0
            s_t = time.time()
    stop.set()
    if info is not None:
        try:
            info.wait_for_publish(2.0)   # the network thread writes the last QoS 0 messages
        except (RuntimeError, ValueError):
            pass
    client.disconnect()
    client.loop_stop()


if __name__ == "__main__":
    main()
