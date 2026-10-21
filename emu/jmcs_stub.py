#!/usr/bin/env python3
"""Operation control stub (JMCS) of the GS for the paper stack of the emulator (emu/fleet.py --stack paper).

Publishes the order sigma_t and the needed topics N(t) of the fleet (paper Section III) on fleet/order of
the GS broker, once per step, and switches between two modes every --period-s seconds (software-only
evaluation design v2):

  survey   the video topics are needed: N = the needed topics of the DTP and every video topic, in the
           order of the DTP; sigma_t = N, then the other topics in the order of the DTP
  transit  the video topics are not needed: N = the needed topics of the DTP without the video topics;
           sigma_t = N, then the other non-video topics, then the video topics

A video topic is a topic whose name starts with --video-prefix (default "video", which also matches
layered video topics such as video0 .. video15). --modes FILE replaces both modes by
{"survey": {"order": [...], "needed": [...]}, "transit": {...}}.

Mode of step t: k = floor((t - S0) / P) with P = round(period_s / Ts); the first mode (--first) for
even k and for t < S0, the other one for odd k. The order of step t leaves at start(t) + --offset-ms
(default 2 ms, before the end of phase 1 at start(t) + T1), so the AS, which takes the last fleet/order
received before start(t) + T1, uses the mode of step t. Steps are 0-based: start(t) = epoch + t Ts.

Payload (JSON, QoS 0): {"t": t, "mode": "survey", "order": [names], "needed": [names]}.
With --log, one JSON line per publication: {"t", "mode", "pub"} (pub: Unix time of the publication).

  jmcs_stub.py --dtp dtp.json --broker 127.0.0.1:1883 --s0 S0 [--epoch E] [--period-s 10]
               [--first survey|transit] [--offset-ms 2] [--steps N] [--log order.jsonl]
"""
import argparse
import json
import math
import os
import signal
import sys
import threading
import time

DEFAULT_EPOCH = 1767225600.0
MODES = ("survey", "transit")


def topic_names(dtp):
    return [t["name"] if isinstance(t, dict) else str(t) for t in dtp["topics"]]


def build_modes(dtp, video_prefix="video"):
    """The orders and needed topics of the two modes from the DTP."""
    names = topic_names(dtp)
    sigma = list(dtp.get("order") or names)
    sigma += [n for n in names if n not in sigma]
    base = set(dtp.get("needed") or [])
    video = lambda n: n.startswith(video_prefix)
    n_s = [n for n in sigma if n in base or video(n)]
    survey = {"order": n_s + [n for n in sigma if n not in n_s], "needed": n_s}
    n_t = [n for n in sigma if n in base and not video(n)]
    transit = {"order": n_t + [n for n in sigma if n not in n_t and not video(n)] + [n for n in sigma if video(n)],
               "needed": n_t}
    return {"survey": survey, "transit": transit}


def mode_of(t, s0, period_steps, first="survey"):
    other = MODES[1] if first == MODES[0] else MODES[0]
    if t < s0:
        return first
    return first if ((t - s0) // period_steps) % 2 == 0 else other


def payload(t, mode, modes):
    m = modes[mode]
    return json.dumps({"t": int(t), "mode": mode, "order": m["order"], "needed": m["needed"]},
                      separators=(",", ":"))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--broker", default="127.0.0.1:1883")
    ap.add_argument("--epoch", type=float, default=None, help="start of step 0 (default: the DTP epoch)")
    ap.add_argument("--s0", type=int, required=True, help="first step of the run (mode switches count from it)")
    ap.add_argument("--period-s", type=float, default=10.0)
    ap.add_argument("--first", choices=MODES, default="survey")
    ap.add_argument("--offset-ms", type=float, default=2.0)
    ap.add_argument("--video-prefix", default="video")
    ap.add_argument("--modes", default=None, help="JSON file with the two modes (replaces the DTP rule)")
    ap.add_argument("--steps", type=int, default=None)
    ap.add_argument("--log", default=None)
    a = ap.parse_args(argv)
    with open(a.dtp) as f:
        dtp = json.load(f)
    Ts = float(dtp["Ts"])
    epoch = a.epoch if a.epoch is not None else (DEFAULT_EPOCH if dtp.get("epoch") is None else float(dtp["epoch"]))
    if not 0 <= a.offset_ms / 1e3 < float(dtp["T1"]):
        ap.error("--offset-ms must lie in [0, T1)")
    if a.modes:
        with open(a.modes) as f:
            modes = json.load(f)
    else:
        modes = build_modes(dtp, a.video_prefix)
    P = max(1, int(round(a.period_s / Ts)))

    import paho.mqtt.client as mqtt
    host, _, port = a.broker.rpartition(":")
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fdt-jmcs-%d" % os.getpid(), clean_session=True)
    c.max_inflight_messages_set(0)
    c.reconnect_delay_set(0.2, 2)
    c.connect_async(host or "127.0.0.1", int(port or 1883), keepalive=10)
    c.loop_start()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    out = open(a.log, "a") if a.log else None
    mono0, wall0 = time.monotonic(), time.time()
    now = lambda: wall0 + (time.monotonic() - mono0)
    off = a.offset_ms / 1e3
    t = math.floor((now() - epoch - off) / Ts) + 1
    done = 0
    print("jmcs_stub: epoch %.3f, S0 %d, period %d steps, first %s, modes %s" % (
        epoch, a.s0, P, a.first, json.dumps(modes)), file=sys.stderr, flush=True)
    while not stop.is_set() and (a.steps is None or done < a.steps):
        at = epoch + t * Ts + off
        rest = at - now()
        if rest > 0 and stop.wait(rest):
            break
        if now() - at >= Ts:   # overrun: skip to the next step
            t = math.floor((now() - epoch - off) / Ts) + 1
            continue
        mode = mode_of(t, a.s0, P, a.first)
        c.publish("fleet/order", payload(t, mode, modes), qos=0)
        if out:
            out.write(json.dumps({"t": t, "mode": mode, "pub": round(now(), 6)}) + "\n")
            out.flush()
        t += 1
        done += 1
    time.sleep(0.2)
    c.loop_stop()
    c.disconnect()
    if out:
        out.close()


if __name__ == "__main__":
    main()
