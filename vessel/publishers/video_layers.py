#!/usr/bin/env python3
"""Temporal layering of the video of a Fleet-DT vessel into layer topics video_1 .. video_Lv.

Frame k of the video source goes to layer topic video_j with j = rev(k mod Lv) + 1, where rev reverses
the log2(Lv) bits of its argument (Lv = 16: frames 0, 1, 2, 3, ... go to layers 1, 9, 5, 13, ...). The
first j layers then hold the frames k with rev(k mod Lv) < j, which are the multiples of Lv / j' (j' the
largest power of two <= j) plus the frames of the remaining layers in the gaps, so admitting layers
1..j delivers j / Lv of the frame rate, spread uniformly in time (for j a power of two, exactly one
frame out of every Lv / j). The priority order sigma lists video_1 .. video_Lv after the telemetry,
so Algorithm 1 admits a prefix of layers: each admitted layer adds 1 / Lv of the frame rate.

Arrival curve of a layer topic at duration Ts (dtp.json, DTP format of SPEC.md): one layer receives
one frame every Lv / fps seconds (16 / 15 s for the ZED at 15 fps), which is longer than Ts, so a step
holds at most one frame of each layer. A frame is at most `msgs_per_frame` messages of at most `bound`
bytes, so the DTP declares rate = msgs_per_frame / Ts for each layer topic (ceil(rate Ts) =
msgs_per_frame messages per step, the demand of the DTI program) and "jitter": 0: the one-frame bound
holds for any release jitter J < layers / fps - Ts, so the jitter allowance n = ceil(rate (Ts + J)) of
the DTI program and of the vessel regulator (fdt_paper.c) must not add a message to a layer topic.

Per-vessel settings (dtp.json, object "video_layers"):

    "video_layers": {"base": "video", "layers": 16, "fps": 15, "bound": 1400, "msgs_per_frame": 8,
                     "vessels": {"1": {"quality": 50, "fps_scale": 1.0, "size_scale": 1.0}, ...}}

  quality     initial JPEG quality of the video publisher of the vessel (zed_publisher.py)
  fps_scale   frame-rate scale of the vessel (published frames per captured frame)
  size_scale  scale of the bound of the layer topics of the vessel: the per-vessel DTP written by
              `video_layers.py resolve` sets bound_i = round(size_scale * bound), so the demand tables
              of the vessels differ (the DTI program computes d_il from the DTP file it reads)

The AS, the VE and the GS read the common dtp.json: the list of topics is the same for every vessel
(homogeneous fleet), and only the bounds of the layer topics differ in the per-vessel files.

  video_layers.py resolve --dtp dtp.json --vessel 2 [--out dtp_b2.json]
  video_layers.py show    --dtp dtp.json [--vessel 2]
  video_layers.py make    --dtp dtp_7topics.json [--layers 16 --fps 15 --bound 1400 --msgs-per-frame 8]

Python >= 3.7, standard library only.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import sys

DEFAULTS = {"base": "video", "layers": 16, "fps": 15.0, "bound": 1400, "msgs_per_frame": 8}
VESSEL_DEFAULTS = {"quality": 50, "fps_scale": 1.0, "size_scale": 1.0}
CONTENT_HEADER = 8   # frame:u32, k:u16, n:u16 in front of the JPEG bytes of each message


def _bits(layers):
    b = int(round(math.log(layers, 2)))
    if layers < 1 or (1 << b) != layers:
        raise ValueError("the number of layers must be a power of two (got %d)" % layers)
    return b


def bitrev(k, bits):
    """Reverses the `bits` low bits of k."""
    r = 0
    for _ in range(bits):
        r = (r << 1) | (k & 1)
        k >>= 1
    return r


def layer_of_frame(k, layers=16):
    """1-based layer of frame k: rev(k mod layers) + 1."""
    return bitrev(k % layers, _bits(layers)) + 1


def residue_of_layer(j, layers=16):
    """Frames k of layer j (1-based) are the k with k mod layers = rev(j - 1) (rev is an involution)."""
    return bitrev(j - 1, _bits(layers))


def layer_name(base, j):
    return "%s_%d" % (base, j)


def layer_names(base="video", layers=16):
    return [layer_name(base, j) for j in range(1, layers + 1)]


def admitted_frames(j, layers=16):
    """Sorted residues k mod layers of the frames that layers 1..j carry."""
    return sorted(residue_of_layer(l, layers) for l in range(1, j + 1))


def config(dtp):
    """The "video_layers" object of a DTP dict with its defaults, or None for a DTP without layers."""
    v = dtp.get("video_layers")
    if v is None:
        names = [t["name"] for t in dtp.get("topics", [])]
        if layer_name(DEFAULTS["base"], 1) not in names:
            return None
        v = {}
    out = dict(DEFAULTS)
    out.update({k: v[k] for k in v if k != "vessels"})
    out["vessels"] = dict(v.get("vessels", {}))
    _bits(int(out["layers"]))
    return out


def vessel_config(dtp, vessel):
    """quality, fps_scale and size_scale of a vessel (defaults for a vessel without an entry)."""
    c = config(dtp) or {"vessels": {}}
    out = dict(VESSEL_DEFAULTS)
    out.update(c["vessels"].get(str(vessel), {}))
    return out


def layer_topics(dtp):
    """Names of the layer topics present in the DTP topic list, in layer order."""
    c = config(dtp)
    if c is None:
        return []
    names = set(t["name"] for t in dtp.get("topics", []))
    return [n for n in layer_names(c["base"], int(c["layers"])) if n in names]


def topic_spec(bound, msgs_per_frame, Ts, name):
    """DTP entry (SPEC.md format) of one layer topic: at most one frame of msgs_per_frame messages per step."""
    return {"name": name, "rate": msgs_per_frame / float(Ts), "bound": int(bound), "jitter": 0}


def make_layered(dtp, layers=16, fps=15.0, bound=1400, msgs_per_frame=8, base="video", vessels=None):
    """Replaces topic `base` of a DTP dict (SPEC.md format) by `layers` layer topics placed after every
    other topic in "order" (state first, then the telemetry, then base_1 .. base_layers) and adds the
    "video_layers" object. A needed `base` makes every layer needed."""
    _bits(layers)
    d = copy.deepcopy(dtp)
    Ts = float(d["Ts"])
    if fps / layers > 1.0 / Ts:
        raise ValueError("more than one frame of a layer per step: fps / layers > 1 / Ts")
    names = layer_names(base, layers)
    d["topics"] = [t for t in d["topics"] if t["name"] != base] + [topic_spec(bound, msgs_per_frame, Ts, n) for n in names]
    order = [n for n in d.get("order", [t["name"] for t in d["topics"]]) if n != base and n not in names]
    d["order"] = order + names
    if base in d.get("needed", []):
        d["needed"] = [n for n in d["needed"] if n != base] + names
    d["video_layers"] = {"base": base, "layers": layers, "fps": fps, "bound": bound, "msgs_per_frame": msgs_per_frame,
                         "vessels": dict(vessels or {})}
    return d


def resolve_vessel(dtp, vessel):
    """Per-vessel DTP: the bound of each layer topic becomes round(size_scale * bound) (at least
    CONTENT_HEADER + 1); everything else, the topic list and the order included, stays the same.
    The result records the vessel in video_layers.resolved_for."""
    c = config(dtp)
    d = copy.deepcopy(dtp)
    if c is None:
        return d
    s = float(vessel_config(dtp, vessel)["size_scale"])
    if not s > 0:
        raise ValueError("size_scale of vessel %s must be positive" % vessel)
    lt = set(layer_topics(dtp))
    for t in d["topics"]:
        if t["name"] in lt:
            t["bound"] = max(CONTENT_HEADER + 1, int(round(s * float(t["bound"]))))
    d["video_layers"]["resolved_for"] = int(vessel)
    return d


def msgs_per_step(rate, Ts):
    return int(math.ceil(rate * Ts - 1e-9))


def frame_cap(dtp, name):
    """Largest JPEG of one frame on layer topic `name`: messages per step times (bound - 8) bytes."""
    t = [x for x in dtp["topics"] if x["name"] == name][0]
    return msgs_per_step(float(t["rate"]), float(dtp["Ts"])) * (int(t["bound"]) - CONTENT_HEADER)


def channel_demand(dtp, name, vessel_bound=None, jitter_s=0.0):
    """Demand of one step of a topic as the DTI program computes it (SPEC.md Section 5):
    ceil(c_b (x + 6 n) + c_m n), n = ceil(r (Ts + J)), x = n s, with J the "jitter" of the topic, else
    jitter_s (the --jitter-ms of fdt_dti; 0 gives n = ceil(r Ts))."""
    t = [x for x in dtp["topics"] if x["name"] == name][0]
    J = float(t["jitter"]) if t.get("jitter") is not None else float(jitter_s)
    n = msgs_per_step(float(t["rate"]), float(dtp["Ts"]) + J)
    s = int(vessel_bound if vessel_bound is not None else t["bound"])
    return int(math.ceil(float(dtp.get("c_b", 1.0)) * (n * s + 6 * n) + float(dtp.get("c_m", 0.0)) * n - 1e-9))


def _load(path):
    with open(path) as f:
        return json.load(f)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd")
    r = sub.add_parser("resolve", help="write the per-vessel DTP (layer bounds scaled by size_scale)")
    r.add_argument("--dtp", required=True)
    r.add_argument("--vessel", type=int, required=True)
    r.add_argument("--out", default=None)
    s = sub.add_parser("show", help="print the layer schedule and the per-step demand of each layer")
    s.add_argument("--dtp", required=True)
    s.add_argument("--vessel", type=int, default=None)
    m = sub.add_parser("make", help="split topic video of a DTP into layer topics")
    m.add_argument("--dtp", required=True)
    m.add_argument("--layers", type=int, default=16)
    m.add_argument("--fps", type=float, default=15.0)
    m.add_argument("--bound", type=int, default=1400)
    m.add_argument("--msgs-per-frame", type=int, default=8)
    m.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    if a.cmd == "resolve":
        d = resolve_vessel(_load(a.dtp), a.vessel)
        txt = json.dumps(d, indent=1)
        if a.out:
            with open(a.out, "w") as f:
                f.write(txt + "\n")
        else:
            print(txt)
    elif a.cmd == "show":
        d = _load(a.dtp)
        if a.vessel is not None:
            d = resolve_vessel(d, a.vessel)
        c = config(d)
        if c is None:
            print("no video layers in %s" % a.dtp)
            return
        L = int(c["layers"])
        for j, name in enumerate(layer_topics(d), start=1):
            print("%-9s frames k = %2d mod %d  cap %6d B/frame  demand %6d B/step" % (
                name, residue_of_layer(j, L), L, frame_cap(d, name), channel_demand(d, name)))
    elif a.cmd == "make":
        d = make_layered(_load(a.dtp), a.layers, a.fps, a.bound, a.msgs_per_frame)
        txt = json.dumps(d, indent=1)
        if a.out:
            with open(a.out, "w") as f:
                f.write(txt + "\n")
        else:
            print(txt)
    else:
        ap.print_help()
        sys.exit(2)


if __name__ == "__main__":
    main()
