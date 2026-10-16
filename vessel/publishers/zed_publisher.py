#!/usr/bin/env python3
"""Video publisher of a Fleet-DT vessel (R2).

Source: ZED stereo camera on USB 3.0 through V4L2 (side-by-side frames of 1344x376, YUV 4:2:2, 15 fps),
read with OpenCV VideoCapture (CAP_V4L2), or a synthetic moving pattern (--synthetic) for tests and
bench runs without a camera.

Each published frame is one JPEG (quality --quality). The program publishes one frame out of every
--divider captured frames, and of those a fraction --fps-scale (frame-rate scale of the vessel); the
experiments set the offered load through the quality, the divider, the frame-rate scale and the
per-vessel bound of the layer topics.

Temporal layers (DTP with the topics video_1 .. video_Lv, see video_layers.py): published frame k goes
to layer topic video_j with j = rev(k mod Lv) + 1 (bit reversal), so the first j layers carry j / Lv of
the frame rate spread uniformly in time. A frame of layer j fits in ceil(r_j Ts) messages of at most
bound_j bytes (the arrival curve of the topic in the DTP): when the JPEG exceeds that capacity, the
publisher encodes it again with a lower quality (steps of --quality-step, down to --min-quality) and
then at half the resolution, so every frame stays within the demand of its topic. The defaults of
--quality and --fps-scale come from the entry of the vessel in "video_layers"."vessels" of the DTP.
Without layer topics (DTP with one topic video), every frame goes to video as before.

Each JPEG is split into messages of at most `bound - 8` bytes of JPEG data, where `bound` is the DTP
bound of the topic; the content of each message is

    frame:u32  k:u16  n:u16  JPEG bytes [k*(bound-8), (k+1)*(bound-8))      (network byte order)

with frame = index of the published frame (mod 2^32), k = index of the message in the frame and
n = number of messages of the frame, so every content holds at most `bound` bytes. The messages go
to boat/b<i>/raw/<topic> with QoS 0.

Usage:
  zed_publisher.py --dtp /etc/fleetdt/dtp.json --vessel 1 [--device /dev/video0] [--quality 50]
                   [--divider 1] [--fps-scale 1.0] [--host 127.0.0.1] [--port 1883]
  zed_publisher.py --dtp dtp.json --vessel 1 --synthetic [--frames 100] [--save-dir out/]
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import signal
import struct
import sys
import threading
import time
from typing import Iterator, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from common import DTPFile, make_client, raw_topic, sleep_until  # noqa: E402
import video_layers  # noqa: E402

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


def frame_topic(k: int, layers: List[str]) -> str:
    """Layer topic of published frame k: layers[rev(k mod Lv)] (layers = [video_1, ..., video_Lv])."""
    return layers[video_layers.layer_of_frame(k, len(layers)) - 1]


class FrameRateScale:
    """Keeps a fraction `scale` of the frames, spread uniformly: frame c is kept when
    floor((c + 1) scale) > floor(c scale)."""

    def __init__(self, scale: float):
        if not 0.0 < scale <= 1.0:
            raise ValueError("frame-rate scale must be in (0, 1]")
        self.scale, self.c = scale, 0

    def keep(self) -> bool:
        c, self.c = self.c, self.c + 1
        return math.floor((c + 1) * self.scale + 1e-9) > math.floor(c * self.scale + 1e-9)


def encode_within(img: np.ndarray, quality: int, cap: Optional[int], step: int = 10, min_quality: int = 5,
                  max_halvings: int = 3) -> Optional[Tuple[bytes, int, int]]:
    """JPEG of img with at most `cap` bytes: quality `quality`, then lower by `step` down to
    `min_quality`, then the same at half the resolution (up to `max_halvings` times).
    Returns (jpeg, quality used, halvings) or None when no encoding fits."""
    im = img
    for h in range(max_halvings + 1):
        q = quality
        while True:
            ok, enc = cv2.imencode(".jpg", im, [int(cv2.IMWRITE_JPEG_QUALITY), int(q)])
            if ok and (cap is None or len(enc) <= cap):
                return enc.tobytes(), int(q), h
            if q <= min_quality:
                break
            q = max(min_quality, q - step)
        if im.shape[0] < 16 or im.shape[1] < 16:
            break
        im = cv2.resize(im, (im.shape[1] // 2, im.shape[0] // 2), interpolation=cv2.INTER_AREA)
    return None


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
    ap.add_argument("--quality", type=int, default=None,
                    help="JPEG quality 1..100 (default: the vessel entry of video_layers in the DTP, else 50)")
    ap.add_argument("--quality-step", type=int, default=10, help="quality decrement when a frame exceeds its layer")
    ap.add_argument("--min-quality", type=int, default=5)
    ap.add_argument("--divider", type=int, default=1, help="publish one frame out of every DIVIDER frames")
    ap.add_argument("--fps-scale", type=float, default=None,
                    help="fraction of the frames kept after the divider (default: the vessel entry of video_layers, else 1)")
    ap.add_argument("--synthetic", action="store_true", help="moving test pattern instead of the camera")
    ap.add_argument("--frames", type=int, default=0, help="stop after this number of published frames (0: run)")
    ap.add_argument("--save-dir", default=None, help="also write each published JPEG as <frame>.jpg")
    ap.add_argument("--stats-period", type=float, default=10.0)
    ap.add_argument("--log-level", default="INFO")
    a = ap.parse_args(argv)
    logging.basicConfig(level=a.log_level.upper(), format="%(asctime)s %(name)s %(levelname)s %(message)s")
    with open(a.dtp) as f:
        raw = json.load(f)
    vc = video_layers.vessel_config(raw, a.vessel)
    if a.quality is None:
        a.quality = int(vc["quality"])
    if a.fps_scale is None:
        a.fps_scale = float(vc["fps_scale"])
    if not 1 <= a.quality <= 100 or a.divider < 1 or not 0.0 < a.fps_scale <= 1.0:
        ap.error("quality must be in 1..100, divider >= 1 and fps-scale in (0, 1]")

    dtp = DTPFile.load(a.dtp)
    layers = video_layers.layer_topics(raw)
    if layers:
        names = layers
        log.info("video in %d layer topics %s .. %s (frame k -> layer rev(k mod %d) + 1)",
                 len(layers), layers[0], layers[-1], len(layers))
    elif "video" in dtp.topics:
        names = ["video"]
    else:
        raise SystemExit("the DTP has neither the topic video nor the layer topics video_1 ..")
    bound = {n: dtp.topics[n].bound for n in names}
    cap = {n: dtp.per_step(n) * (bound[n] - FRAME_HEADER.size) for n in names} if layers else {n: None for n in names}
    scaler = FrameRateScale(a.fps_scale)
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
    frame = captured = 0
    info = None
    s_frames = s_msgs = s_bytes = s_reduced = 0
    s_t = time.time()
    for img in src:
        captured += 1
        if (captured - 1) % a.divider or not scaler.keep():
            continue
        name = frame_topic(frame, names) if layers else names[0]
        res = encode_within(img, a.quality, cap[name], a.quality_step, a.min_quality)
        if res is None:
            log.warning("frame %d: no JPEG fits the %s bytes of %s", frame, cap[name], name)
            continue
        jpeg, q_used, halvings = res
        s_reduced += int(q_used != a.quality or halvings > 0)
        if connected.is_set():
            msgs = split_frame(jpeg, frame, bound[name])
            topic = raw_topic(a.vessel, name)
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
            log.info("%.2f frames/s, %.2f messages/s, %.0f content bytes/s, %.0f bytes/frame, %d frames re-encoded",
                     s_frames / dt, s_msgs / dt, s_bytes / dt, s_bytes / max(s_frames, 1), s_reduced)
            if not layers and s_msgs / dt > dtp.topics["video"].rate:
                log.warning("%.1f messages/s exceed the DTP rate of video (%.1f/s): the vessel regulator "
                            "withholds the excess, so frames arrive incomplete; raise --divider or lower "
                            "--quality", s_msgs / dt, dtp.topics["video"].rate)
            s_frames = s_msgs = s_bytes = s_reduced = 0
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
