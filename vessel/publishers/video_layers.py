#!/usr/bin/env python3
"""Spatial stripe layering of the video of a Fleet-DT vessel into layer topics video_1 .. video_Lv.

Every frame splits into Lv horizontal stripes (Lv = 16; stripe_rows: stripe s holds the rows
[r_s, r_{s+1}) of the frame, s = 0 at the top), and the publisher encodes each stripe as its own JPEG.
Layer topic video_j carries stripe rev(j - 1) of every frame, where rev reverses the log2(Lv) bits of its
argument (Lv = 16: layers 1, 2, 3, 4, 5, ... carry the stripes 0, 8, 4, 12, 2, ...). Admitting layers
1..j delivers j of the Lv stripes of every frame, spread over the height of the image (for j a power of
two, one stripe out of every Lv / j). The priority order sigma lists video_1 .. video_Lv after the
telemetry, so Algorithm 1 admits a prefix of layers (or a prefix with later, smaller layers that still
fit).

Arrival curve of a layer topic at duration Ts (dtp.json, DTP format of SPEC.md): one stripe per frame at
the frame rate f of the vessel, so a step holds at most F = ceil(f Ts) stripes of the topic (jitter 0;
F = f Ts when f Ts is an integer, which the experiments use: 8, 16, 24 or 32 frames per second at
Ts = 125 ms). A stripe of the topic is at most cap_j bytes of JPEG and goes out as m_j messages of at
most `bound` bytes (8-byte header + JPEG data), with

    cap_j = ceil(size_scale * frame_cap * w_{rev(j - 1)})       (bytes of JPEG of one stripe)
    m_j   = ceil(cap_j / (max_msg - 8)),  bound_j = ceil(cap_j / m_j) + 8   (<= max_msg)
    rate_j = m_j F / Ts                                           (messages per second)

so the DTI program declares ceil(rate_j Ts) = m_j F messages of bound_j bytes per step (d_il of
SPEC.md Section 5). frame_cap bounds the trace size of one frame (whole-frame JPEG bytes, the unit of the
traces of emu/traces) and w_s is the share profile of the stripes: w_s = mean (JPEG bytes of stripe s) /
(JPEG bytes of the whole frame) over representative frames (emu/traces/make_stripe_profile.py,
emu/traces/stripe-shares.json); sum_s w_s > 1 because every stripe carries its own JPEG headers. The
source replay gives stripe s of a frame of trace size F_k the size round(F_k w_s) (fdt_source.py); the
camera publisher encodes each stripe within cap_j (zed_publisher.py). Splitting the cap into m_j equal
messages keeps the declared bound close to the sent messages, so the forwarded bytes of a layer stay
close to its demand (tests/test_video_layers.py: mean forwarded / declared >= 0.8 per layer).

Per-vessel settings (dtp.json, object "video_layers"):

    "video_layers": {"base": "video", "layers": 16, "fps": 16, "max_msg": 1400, "frame_cap": 98774,
                     "profile": "q90-s1", "shares": [w_0, ..., w_15],
                     "vessels": {"1": {"quality": 50, "fps_scale": 1.0, "size_scale": 1.0}, ...}}

  quality     initial JPEG quality of the camera publisher of the vessel (zed_publisher.py)
  fps_scale   frame-rate scale of the vessel (published frames per captured frame)
  size_scale  size factor s_i of the vessel: its stripes, and the caps of its layer topics, scale by s_i;
              the per-vessel DTP of `video_layers.py resolve` holds the rate and bound of its layer
              topics, so the demand tables of the vessels differ (the DTI program computes d_il from
              the DTP file it reads)
  video       false: the vessel has no video (E4 fairness cells, vessel 4): its layer topics get rate 0
              in the per-vessel DTP, so the DTI program and the regulator declare d_il = 0 and the
              source publishes none of them (default true)

The AS, the VE and the GS read the common dtp.json (s_i = 1, fps_scale = 1): the list of topics is the
same for every vessel (homogeneous fleet), and only the rates and bounds of the layer topics differ in
the per-vessel files.

  video_layers.py resolve --dtp dtp.json --vessel 2 [--out dtp_b2.json]
  video_layers.py show    --dtp dtp.json [--vessel 2]
  video_layers.py make    --dtp dtp_7topics.json [--layers 16 --fps 16 --frame-cap 98774 --profile q90-s1]

Python >= 3.7, standard library only.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import random
import sys

CONTENT_HEADER = 8   # frame:u32, k:u16, n:u16 in front of the JPEG bytes of each message
PROFILE_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                            "emu", "traces", "stripe-shares.json")
# share profile q90-s1 of emu/traces/stripe-shares.json (24 pairs of the water sequence, OpenCV 4.13.0):
# JPEG bytes of stripe s (top to bottom) over the JPEG bytes of the whole frame
DEFAULT_PROFILE = "q90-s1"
DEFAULT_SHARES = [0.038647, 0.042333, 0.04599, 0.049985, 0.05201, 0.050403, 0.044597, 0.053222,
                  0.109677, 0.112202, 0.10692, 0.100237, 0.099602, 0.095414, 0.09074, 0.085778]
DEFAULTS = {"base": "video", "layers": 16, "fps": 16.0, "max_msg": 1400, "frame_cap": 98774,
            "profile": DEFAULT_PROFILE}
VESSEL_DEFAULTS = {"quality": 50, "fps_scale": 1.0, "size_scale": 1.0, "video": True}


def _bits(layers):
    b = int(round(math.log(layers, 2))) if layers >= 1 else -1
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


def stripe_of_layer(j, layers=16):
    """0-based stripe (0 = top) of the 1-based layer j: rev(j - 1)."""
    if not 1 <= j <= layers:
        raise ValueError("layer %d outside 1..%d" % (j, layers))
    return bitrev(j - 1, _bits(layers))


def layer_of_stripe(s, layers=16):
    """1-based layer of the 0-based stripe s (rev is an involution)."""
    if not 0 <= s < layers:
        raise ValueError("stripe %d outside 0..%d" % (s, layers - 1))
    return bitrev(s, _bits(layers)) + 1


def admitted_stripes(j, layers=16):
    """Sorted stripes that layers 1..j carry."""
    return sorted(stripe_of_layer(l, layers) for l in range(1, j + 1))


def stripe_rows(height, layers=16):
    """[(r0, r1)] of the stripes s = 0 .. layers - 1 of a frame of `height` rows: the first
    height mod layers stripes hold one row more (numpy.array_split)."""
    if height < layers:
        raise ValueError("a frame of %d rows has fewer rows than %d stripes" % (height, layers))
    q, r = divmod(int(height), int(layers))
    out, r0 = [], 0
    for s in range(layers):
        h = q + (1 if s < r else 0)
        out.append((r0, r0 + h))
        r0 += h
    return out


def layer_name(base, j):
    return "%s_%d" % (base, j)


def layer_names(base="video", layers=16):
    return [layer_name(base, j) for j in range(1, layers + 1)]


def layer_index(name, base="video", layers=16):
    """1-based layer of a topic name <base>_<j>, or None."""
    if not name.startswith(base + "_"):
        return None
    suf = name[len(base) + 1:]
    if not suf.isdigit() or not 1 <= int(suf) <= layers:
        return None
    return int(suf)


def load_profile(key=DEFAULT_PROFILE, path=PROFILE_FILE):
    """Shares of profile `key` of emu/traces/stripe-shares.json (DEFAULT_SHARES when the file is absent
    and key is the default profile)."""
    try:
        with open(path) as f:
            return list(json.load(f)["profiles"][key]["shares"])
    except (OSError, KeyError, ValueError):
        if key == DEFAULT_PROFILE:
            return list(DEFAULT_SHARES)
        raise


def frames_per_step(f, Ts):
    """Largest number of frames (stripes of one layer) that a window of Ts holds: ceil(f Ts)."""
    return max(1, int(math.ceil(float(f) * float(Ts) - 1e-9)))


def msg_split(cap, max_msg=1400):
    """(m, bound): cap bytes of JPEG in m messages of at most max_msg bytes, each message 8 header bytes
    plus at most bound - 8 JPEG bytes, bound = ceil(cap / m) + 8."""
    room = int(max_msg) - CONTENT_HEADER
    if room <= 0:
        raise ValueError("max_msg %d leaves no room for JPEG data" % max_msg)
    cap = max(1, int(cap))
    m = max(1, int(math.ceil(cap / float(room))))
    return m, int(math.ceil(cap / float(m))) + CONTENT_HEADER


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
    L = int(out["layers"])
    _bits(L)
    if out.get("shares") is None:
        out["shares"] = load_profile(out["profile"]) if L == 16 else [1.0 / L] * L
    if len(out["shares"]) != L or not all(float(w) > 0 for w in out["shares"]):
        raise ValueError("video_layers.shares must hold %d positive shares" % L)
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


def layer_cap(c, j, size_scale=1.0):
    """JPEG bytes of one stripe of layer j: ceil(size_scale frame_cap w_{rev(j-1)})."""
    w = float(c["shares"][stripe_of_layer(j, int(c["layers"]))])
    return max(1, int(math.ceil(float(size_scale) * float(c["frame_cap"]) * w - 1e-9)))


def layer_topic(c, j, Ts, size_scale=1.0, fps_scale=1.0):
    """DTP entry (SPEC.md format) of layer topic j: rate m_j F / Ts, bound_j, jitter 0."""
    m, bound = msg_split(layer_cap(c, j, size_scale), int(c["max_msg"]))
    F = frames_per_step(float(c["fps"]) * float(fps_scale), Ts)
    return {"name": layer_name(c["base"], j), "rate": round(m * F / float(Ts), 9), "bound": int(bound), "jitter": 0}


def layer_topic_v2(c, j, size_scale=1.0):
    """Fields of layer topic j in a DTP v2 (SPEC_v2.md, as2 stack): "rate" = the frame rate (outputs per
    second, one stripe per output), "n" = m_j messages of at most "s" = bound_j bytes per stripe."""
    m, bound = msg_split(layer_cap(c, j, size_scale), int(c["max_msg"]))
    return {"rate": float(c["fps"]), "n": int(m), "s": int(bound)}


def msgs_per_stripe(dtp, name):
    """m_j of a layer topic of a (resolved) DTP: ceil(rate Ts) / F."""
    c = config(dtp)
    t = [x for x in dtp["topics"] if x["name"] == name][0]
    vc = vessel_config(dtp, (c.get("resolved_for") if c else None) or 0)
    F = frames_per_step(float(c["fps"]) * float(vc["fps_scale"] if c.get("resolved_for") else 1.0), dtp["Ts"])
    n = msgs_per_step(float(t["rate"]), float(dtp["Ts"]))
    return max(1, n // F)


def stripe_cap(dtp, name):
    """Largest JPEG of one stripe on layer topic `name`: m_j (bound_j - 8) bytes."""
    t = [x for x in dtp["topics"] if x["name"] == name][0]
    return msgs_per_stripe(dtp, name) * (int(t["bound"]) - CONTENT_HEADER)


def set_layers(d, c, vessel=None):
    """Writes the layer topics of DTP dict d from the configuration c (vessel None: the common DTP with
    size_scale = fps_scale = 1)."""
    vc = dict(VESSEL_DEFAULTS)
    if vessel is not None:
        vc.update(c["vessels"].get(str(vessel), {}))
    s, fs = float(vc["size_scale"]), float(vc["fps_scale"])
    if not s > 0 or not fs > 0:
        raise ValueError("size_scale and fps_scale of vessel %s must be positive" % vessel)
    L = int(c["layers"])
    for t in d["topics"]:
        j = layer_index(t["name"], c["base"], L)
        if j is not None:
            t.update(layer_topic(c, j, float(d["Ts"]), s, fs))
            if not vc.get("video", True):
                t["rate"] = 0.0
    return d


def make_layered(dtp, layers=16, fps=16.0, frame_cap=DEFAULTS["frame_cap"], shares=None, max_msg=1400,
                 base="video", vessels=None, profile=DEFAULT_PROFILE):
    """Replaces topic `base` of a DTP dict (SPEC.md format) by `layers` layer topics placed after every
    other topic in "order" (state first, then the telemetry, then base_1 .. base_layers) and adds the
    "video_layers" object. A needed `base` makes every layer needed."""
    _bits(layers)
    d = copy.deepcopy(dtp)
    if shares is None:
        shares = load_profile(profile) if layers == 16 else [1.0 / layers] * layers
    names = layer_names(base, layers)
    d["topics"] = [t for t in d["topics"] if t["name"] != base and t["name"] not in names] + \
        [{"name": n} for n in names]
    order = [n for n in d.get("order", [t["name"] for t in d["topics"]]) if n != base and n not in names]
    d["order"] = order + names
    if base in d.get("needed", []):
        d["needed"] = [n for n in d["needed"] if n != base] + names
    d["video_layers"] = {"base": base, "layers": layers, "fps": float(fps), "max_msg": int(max_msg),
                         "frame_cap": int(frame_cap), "profile": profile, "shares": [float(w) for w in shares],
                         "vessels": dict(vessels or {})}
    return set_layers(d, config(d))


def with_video(dtp, fps=None, frame_cap=None):
    """Copy of a layered DTP with another frame rate and/or frame cap (layer topics recomputed)."""
    d = copy.deepcopy(dtp)
    if fps is not None:
        d["video_layers"]["fps"] = float(fps)
    if frame_cap is not None:
        d["video_layers"]["frame_cap"] = int(frame_cap)
    return set_layers(d, config(d))


def resolve_vessel(dtp, vessel):
    """Per-vessel DTP: the layer topics with the size_scale and fps_scale of the vessel (rate and bound
    of each layer); everything else, the topic list and the order included, stays the same. The result
    records the vessel in video_layers.resolved_for."""
    c = config(dtp)
    d = copy.deepcopy(dtp)
    if c is None:
        return d
    set_layers(d, c, vessel)
    d["video_layers"]["resolved_for"] = int(vessel)
    return d


def overload_steps(seed, steps, fraction=0.1, warm_steps=240):
    """E4 overload: the seeded set of run steps (0-based relative to the first step S0 of the run) in
    which the video source of the overloaded vessel exceeds its demand table: round(fraction n) distinct
    steps drawn uniformly from the n = steps - warm_steps steps after the warm-up, sorted. fdt_source.py
    and emu/fleet.py (result.json) call it with the same arguments."""
    steps, warm_steps = int(steps), int(warm_steps)
    if not 0.0 <= float(fraction) <= 1.0 or warm_steps < 0:
        raise ValueError("overload: 0 <= fraction <= 1 and warm_steps >= 0")
    pool = list(range(min(warm_steps, max(steps, 0)), max(steps, 0)))
    k = int(round(float(fraction) * len(pool)))
    return sorted(random.Random("overload:%s" % seed).sample(pool, k))


def check_layers(dtp):
    """Raises ValueError when the layer topics of a common DTP differ from the configuration."""
    c = config(dtp)
    if c is None:
        return
    want = set_layers(copy.deepcopy(dtp), c)
    for t, w in zip(dtp["topics"], want["topics"]):
        if t != w:
            raise ValueError("layer topic %s differs from video_layers: %s != %s" % (t["name"], t, w))


def msgs_per_step(rate, Ts):
    return int(math.ceil(rate * Ts - 1e-9))


def _varint_bytes(v):
    return 1 if v < 128 else 2 if v < 16384 else 3 if v < 2097152 else 4


def publish_header(V, name, s):
    """h_P,l: PUBLISH header (QoS 0, no packet identifier) of a message of s content bytes plus the 6-byte step
    header on boat/b<V>/tx/<name> (common/fdt_paper.c, fleetdt/as_model.py)."""
    tl = len(("boat/b%d/tx/%s" % (max(1, int(V)), name)).encode())
    return 1 + _varint_bytes(2 + tl + 6 + int(s)) + 2 + tl


def cost_bound(dtp, name=None):
    """(c_b, c_m,l) of topic `name` as the readers of the paper stack compute them (common/fdt_paper.c,
    fleetdt/as_model.py; eq. cons of the paper): "c_b" and "c_m" of the file when present, else
    c_b = 1 + 2 (h_T + h_W) / MSS and c_m,l = c_b h_P,l + c_k, c_k = 2 (h_T + h_W) ("c_k" of the file when
    present), with MSS
    and h_T of "tcp" (1448, 80), h_W of the file (0) and h_P,l the PUBLISH header of
    boat/b<V>/tx/<name> (V the largest index of "vessels"). The bound s of a v2 topic stands for its
    "bound"."""
    tcp, mqtt = dtp.get("tcp") or {}, dtp.get("mqtt") or {}
    h_W = float(dtp.get("h_W") or 0.0)
    h_T, mss, h_A = float(tcp.get("h_T", 80)), float(tcp.get("mss", 1448)), float(mqtt.get("h_A", 4))
    c_b = float(dtp["c_b"]) if dtp.get("c_b") is not None else 1.0 + 2.0 * (h_T + h_W) / mss
    if dtp.get("c_m") is not None:
        return c_b, float(dtp["c_m"])
    if name is None:
        raise ValueError("c_m,l of the paper cost bound needs the topic")
    t = [x for x in dtp["topics"] if x["name"] == name][0]
    V = max([int(v) for v in dtp.get("vessels") or []] or [1])
    h_P = publish_header(V, name, int(t["bound"] if "bound" in t else t["s"]))
    c_k = float(dtp["c_k"]) if dtp.get("c_k") is not None else 2.0 * (h_T + h_W)
    return c_b, c_b * h_P + c_k


def channel_demand(dtp, name, vessel_bound=None, jitter_s=0.0):
    """Demand of one step of a topic as the DTI program computes it (SPEC.md Section 5):
    ceil(c_b (x + 6 n) + c_m,l n), n = ceil(r (Ts + J)), x = n s, with J the "jitter" of the topic, else
    jitter_s (the --jitter-ms of fdt_dti; 0 gives n = ceil(r Ts)); (c_b, c_m,l) of cost_bound."""
    t = [x for x in dtp["topics"] if x["name"] == name][0]
    J = float(t["jitter"]) if t.get("jitter") is not None else float(jitter_s)
    n = msgs_per_step(float(t["rate"]), float(dtp["Ts"]) + J)
    s = int(vessel_bound if vessel_bound is not None else t["bound"])
    c_b, c_m = cost_bound(dtp, name)
    return int(math.ceil(c_b * (n * s + 6 * n) + c_m * n - 1e-9))


def message_cost(dtp, x, name=None):
    """Channel bytes that the policing of the regulator charges one message of x content bytes of topic
    `name`: c_b (x + 6) + c_m,l (fdt_p_cost of common/fdt_paper.c). Without `name`, the first layer topic
    of the DTP (the layer topics of one DTP differ in c_m,l only through the length of the topic name and
    of the PUBLISH remaining length, at most a few bytes)."""
    if name is None and dtp.get("c_m") is None:
        lay = layer_topics(dtp)
        name = lay[0] if lay else dtp["topics"][0]["name"]
    c_b, c_m = cost_bound(dtp, name)
    return c_b * (x + 6) + c_m


def split_sizes(size, bound):
    """Content sizes of the messages of one stripe of `size` JPEG bytes (8-byte header each)."""
    room = int(bound) - CONTENT_HEADER
    m = max(1, int(math.ceil(size / float(room))))
    out, left = [], int(size)
    for _ in range(m):
        part = min(room, left)
        left -= part
        out.append(part + CONTENT_HEADER)
    return out


def vessel_demand(dtp, jitter_s=0.0):
    """Sum of the demands of all topics of a (resolved) DTP per step."""
    return sum(channel_demand(dtp, t["name"], jitter_s=jitter_s) for t in dtp["topics"])


def plan_for_rho(dtp, M, rho, C, frame_max, fps_options=(8, 16, 24, 32), jitter_s=0.0):
    """Video setting of a cell: the frame rate (one of fps_options, frames per second with an integer
    number of frames per step preferred) and the frame cap for which the demand tables of vessels 1..M
    sum to rho C per step. Picks the smallest frame rate whose frame cap does not exceed frame_max (the
    largest trace frame), so the frames keep the largest size the trace allows; the source then scales
    the trace sizes by scale = frame_cap / frame_max (no stripe exceeds its cap).
    Returns {"fps", "frame_cap", "scale", "total", "rho"} and the planned DTP under "dtp"."""
    best = None
    for fps in sorted(fps_options):
        lo, hi = 1, int(64 * frame_max)

        def total(fc):
            d = with_video(dtp, fps=fps, frame_cap=fc)
            return sum(vessel_demand(resolve_vessel(d, i), jitter_s) for i in range(1, M + 1)), d
        if total(lo)[0] > rho * C:
            continue
        while hi - lo > 1:
            mid = (lo + hi) // 2
            lo, hi = (mid, hi) if total(mid)[0] <= rho * C else (lo, mid)
        tot, d = total(lo)
        rec = {"fps": float(fps), "frame_cap": lo, "scale": lo / float(frame_max), "total": tot,
               "rho": tot / float(C), "dtp": d}
        if best is None:
            best = rec
        if lo <= frame_max:
            return rec
        best = rec
    if best is None:
        raise ValueError("rho %g with M %d: the telemetry and one message per layer topic exceed rho C" % (rho, M))
    return best


def _load(path):
    with open(path) as f:
        return json.load(f)


def _dump(d, out):
    txt = json.dumps(d, indent=1)
    if out:
        with open(out, "w") as f:
            f.write(txt + "\n")
    else:
        print(txt)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd")
    r = sub.add_parser("resolve", help="write the per-vessel DTP (layer topics with its size and frame-rate scales)")
    r.add_argument("--dtp", required=True)
    r.add_argument("--vessel", type=int, required=True)
    r.add_argument("--out", default=None)
    s = sub.add_parser("show", help="print the stripe of each layer and its per-step demand")
    s.add_argument("--dtp", required=True)
    s.add_argument("--vessel", type=int, default=None)
    m = sub.add_parser("make", help="split topic video of a DTP into stripe layer topics")
    m.add_argument("--dtp", required=True)
    m.add_argument("--layers", type=int, default=16)
    m.add_argument("--fps", type=float, default=DEFAULTS["fps"])
    m.add_argument("--frame-cap", type=int, default=DEFAULTS["frame_cap"])
    m.add_argument("--profile", default=DEFAULT_PROFILE)
    m.add_argument("--max-msg", type=int, default=1400)
    m.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    if a.cmd == "resolve":
        _dump(resolve_vessel(_load(a.dtp), a.vessel), a.out)
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
            print("%-9s stripe %2d of %d  cap %6d B/stripe  %d msg/stripe  demand %6d B/step" % (
                name, stripe_of_layer(j, L), L, stripe_cap(d, name), msgs_per_stripe(d, name),
                channel_demand(d, name)))
    elif a.cmd == "make":
        shares = load_profile(a.profile) if a.layers == 16 else None
        _dump(make_layered(_load(a.dtp), a.layers, a.fps, a.frame_cap, shares, a.max_msg, profile=a.profile), a.out)
    else:
        ap.print_help()
        sys.exit(2)


if __name__ == "__main__":
    main()
