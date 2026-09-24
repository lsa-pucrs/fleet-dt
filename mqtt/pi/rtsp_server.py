"""Boat-side video: the ZED over RTSP, H.264 from the Pi's hardware encoder.

The ZED enumerates as a UVC camera that only speaks YUYV. v4l2h264enc (the
bcm2835 codec, /dev/video11) takes YUYV directly, so no pixel ever touches the
600 MHz CPU. One shared pipeline serves every client on
``rtsp://<bind>:8554/zed``.

Timestamps: RTP carries a 90 kHz clock that starts at a random offset; the
RTCP sender reports map it to NTP wall time taken from the Pi's system clock,
which chrony disciplines against the ground station. That mapping is what a
receiver uses to put an absolute time on each frame.

Run on the Pi (mission address only, never wlan0):

    python3 mqtt/pi/rtsp_server.py --bind 192.168.1.99
"""

import argparse
import signal
import sys

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import GLib, Gst, GstRtspServer  # noqa: E402

# ZED 1 UVC modes, all YUYV: 2560x720 @60/30/15, 1344x376 @100/60/30/15,
# 3840x1080 @30/15, 4416x1242 @15 (v4l2-ctl --list-formats-ext).


def launch_line(args: argparse.Namespace) -> str:
    """The media pipeline. capssetter relabels the colorimetry: GStreamer 1.14
    reads 2:4:5:1 from the ZED and v4l2h264enc refuses it, although the pixels
    are plain BT.601 limited range."""
    return (
        f"( v4l2src device={args.device} do-timestamp=true "
        f"! video/x-raw,format=YUY2,width={args.width},height={args.height},"
        f"framerate={args.fps}/1 "
        "! capssetter caps=video/x-raw,colorimetry=bt601 "
        f"! v4l2h264enc extra-controls=\"controls,video_bitrate={args.bitrate},"
        f"h264_i_frame_period={args.gop}\" "
        "! video/x-h264,level=(string)4 ! h264parse "
        "! rtph264pay name=pay0 pt=96 config-interval=1 )"
    )


def main() -> None:
    """Serve the ZED until SIGTERM."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bind", default="192.168.1.99")
    parser.add_argument("--port", default="8554")
    parser.add_argument("--mount", default="/zed")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=1344)
    parser.add_argument("--height", type=int, default=376)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--bitrate", type=int, default=2_000_000, help="bit/s")
    parser.add_argument("--gop", type=int, default=15, help="frames between IDR")
    args = parser.parse_args()

    Gst.init(None)
    server = GstRtspServer.RTSPServer()
    server.set_address(args.bind)
    server.set_service(args.port)
    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_launch(launch_line(args))
    factory.set_shared(True)
    server.get_mount_points().add_factory(args.mount, factory)
    if server.attach(None) == 0:
        sys.exit(f"cannot bind rtsp://{args.bind}:{args.port}")

    loop = GLib.MainLoop()
    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGTERM, loop.quit)
    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGINT, loop.quit)
    print(f"serving rtsp://{args.bind}:{args.port}{args.mount} "
          f"H.264 {args.width}x{args.height}@{args.fps} {args.bitrate} bit/s", flush=True)
    loop.run()


if __name__ == "__main__":
    main()
