#!/usr/bin/env python3
"""Stripe share profile of the spatially layered video (vessel/publishers/video_layers.py) from real JPEG
stripe encodes of representative frames of the water sequence (Pohang Canal Dataset, the source of the
traces water-sbs-q<Q>-s<S>.txt; emu/traces/README.md).

  python3 emu/traces/make_stripe_profile.py --out emu/traces/stripe-shares.json [--pairs 24] [--threads 8]

For each of --pairs stereo pairs spread evenly over the sequence of the traces (pohang00, pairs 012000 to
016499) the program builds the side-by-side frame exactly as make_trace_water.py does (centre crop of each
eye to 672:376, INTER_AREA resize to 672 S x 376 S), encodes the whole frame (the size that the traces
hold) and each of the 16 horizontal stripes of video_layers.stripe_rows as its own JPEG (OpenCV
imencode, quality Q, the encoder of zed_publisher.py), for Q in {50, 70, 90} and S in {0.5, 1}.

Output (JSON): per profile "q<Q>-s<S>":
  shares       w_s = mean over the frames of (JPEG bytes of stripe s) / (JPEG bytes of the whole frame),
               s = 0 (top) .. 15 (bottom); sum_s w_s > 1 because every stripe carries its own JPEG
               headers (quantization and Huffman tables, about 600 bytes) and restarts the DC prediction
  share_cv     coefficient of variation of each ratio over the frames
  share_max    largest ratio of each stripe over the frames
  sum          sum of the shares (expansion of the frame by the stripe encoding)
  frames       the pair indices used; whole_mean, whole_cv of the whole-frame sizes

The source replay (fdt_source.py) gives stripe s of frame k the size round(F(k) w_s), F(k) the trace size
of the frame, so the stripes keep the time variation of the trace and the spatial profile of the scene.
"""
import argparse
import concurrent.futures as cf
import io
import json
import os
import sys
import time
import zipfile

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "vessel", "publishers"))
import make_trace_water as mtw  # noqa: E402
import video_layers  # noqa: E402

QUALITIES = (50, 70, 90)
SCALES = (0.5, 1.0)


def tag(q, s):
    return "q%d-s%s" % (q, ("%g" % s).replace(".", "p"))


def encode(img, q):
    ok, enc = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(q)])
    if not ok:
        raise RuntimeError("imencode failed")
    return len(enc)


def stripe_sizes(sbs, q, layers=16):
    """(whole-frame JPEG bytes, [JPEG bytes of stripe s for s = 0 .. layers - 1])"""
    rows = video_layers.stripe_rows(sbs.shape[0], layers)
    return encode(sbs, q), [encode(np.ascontiguousarray(sbs[r0:r1]), q) for r0, r1 in rows]


def profile(per_frame):
    """per_frame: list of (whole, [stripes]) -> shares, cv, max, sum"""
    w = np.array([f[0] for f in per_frame], float)
    s = np.array([f[1] for f in per_frame], float)
    r = s / w[:, None]
    return {"shares": [round(float(x), 6) for x in r.mean(axis=0)],
            "share_cv": [round(float(x), 4) for x in r.std(axis=0) / r.mean(axis=0)],
            "share_max": [round(float(x), 6) for x in r.max(axis=0)],
            "sum": round(float(r.mean(axis=0).sum()), 4),
            "whole_mean": round(float(w.mean()), 1), "whole_cv": round(float(w.std() / w.mean()), 4)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--seq", default="pohang00")
    ap.add_argument("--first", type=int, default=12000)
    ap.add_argument("--last", type=int, default=16499)
    ap.add_argument("--pairs", type=int, default=24)
    ap.add_argument("--layers", type=int, default=16)
    ap.add_argument("--threads", type=int, default=8)
    a = ap.parse_args()
    url = "%s/%s/stereo.zip" % (mtw.BUCKET, a.seq)
    z = zipfile.ZipFile(io.BufferedReader(mtw.RangeFile(url), buffer_size=1 << 20))
    info = {i.filename: i for i in z.infolist()}
    ks = sorted(set(int(round(a.first + i * (a.last - a.first) / float(a.pairs - 1))) for i in range(a.pairs)))

    def work(k):
        names = ("stereo/left_images/%06d.png" % k, "stereo/right_images/%06d.png" % k)
        imgs = [cv2.imdecode(np.frombuffer(mtw.member(url, info[n]), np.uint8), cv2.IMREAD_COLOR) for n in names]
        out = {}
        for s in SCALES:
            sbs = np.hstack([mtw.eye(imgs[0], s), mtw.eye(imgs[1], s)])
            for q in QUALITIES:
                out[tag(q, s)] = stripe_sizes(sbs, q, a.layers)
        return k, out

    t0 = time.time()
    res = {}
    with cf.ThreadPoolExecutor(a.threads) as ex:
        for n, (k, out) in enumerate(ex.map(work, ks)):
            res[k] = out
            print("%d/%d pairs, %.0f s" % (n + 1, len(ks), time.time() - t0), flush=True)
    prof = {}
    for q in QUALITIES:
        for s in SCALES:
            key = tag(q, s)
            prof[key] = dict(profile([res[k][key] for k in ks]), quality=q, scale=s,
                             rows=video_layers.stripe_rows(int(round(376 * s)), a.layers))
    doc = {"dataset": "Pohang Canal Dataset (Chung et al., IJRR 2023, doi:10.1177/02783649231191145), CC BY-NC 4.0",
           "url": url, "sequence": a.seq, "pairs": ks, "layers": a.layers, "opencv": cv2.__version__,
           "method": "whole side-by-side frame and each horizontal stripe (video_layers.stripe_rows) encoded "
                     "as its own JPEG with cv2.imencode at quality Q; share = stripe bytes / whole-frame bytes, "
                     "mean over the pairs",
           "profiles": prof,
           "per_frame": {str(k): {key: {"whole": v[0], "stripes": v[1]} for key, v in res[k].items()} for k in ks}}
    with open(a.out, "w") as f:
        json.dump(doc, f, indent=1)
    print("done in %.0f s -> %s" % (time.time() - t0, a.out))


if __name__ == "__main__":
    main()
