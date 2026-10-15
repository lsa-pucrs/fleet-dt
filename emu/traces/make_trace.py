#!/usr/bin/env python3
"""Frame-size traces of the video topic (paper Section VI-C, Sources) from still frames of the ZED camera.

  python3 emu/traces/make_trace.py --left L.png --right R.png --out emu/traces [--seconds 150 --fps 15 --seed 1]
          [--sigma 0.15 --phi 0.95]

Step 1 (still): sizes of the camera frames under synthetic motion (below) -> still-q<Q>-s<S>.txt and the
statistics in stats.json ("still"). The scene does not change, so these sizes vary little (CV about 0.02).
Step 2 (model): the traces of the experiments, zed-sbs-q<Q>-s<S>.txt: a lognormal AR(1) series of the
same length, log size_k = log(median_still) + y_k with y_k = phi y_(k-1) + sqrt(1 - phi^2) sigma e_k,
e_k ~ N(0, 1), so that the median comes from the camera and the variation (sigma of the log size, lag-1
correlation phi) from an assumed MJPEG model of a moving camera (defaults sigma 0.15, phi 0.95).

Input: one left and one right frame of the camera (HD720, 1280x720 per eye). The program synthesizes a
sequence of --seconds at --fps: each frame warps both eyes with the motion of a small vessel (roll
3 deg * sin(2 pi t / P) with the wave period P = 3 s plus a random walk of 0.3 deg, pitch as a vertical
shift of 12 px * sin(2 pi t / 2.3 s) plus a random walk, yaw as a horizontal random walk of the pan with
40 px of spread), crops the central 90 %, scales each eye to the WVGA eye (672x376) times --scales, applies
a slow exposure drift (gain random walk within +-8 %) and fresh sensor noise (Gaussian, sigma 1.5 DN),
joins the eyes side by side (1344x376 at scale 1) and encodes the result as baseline JPEG (OpenCV/libjpeg,
4:2:0) at each quality of --qualities. Output: zed-sbs-q<Q>-s<S>.txt with one size in bytes per frame and
a summary (stats.json): mean, median, quantiles, coefficient of variation, lognormal fit (mu, sigma of the
log size) and lag-1 autocorrelation of the log size.
"""
import argparse
import json
import math
import os

import cv2
import numpy as np


def warp(img, roll_deg, dx, dy, crop=0.9):
    h, w = img.shape[:2]
    m = cv2.getRotationMatrix2D((w / 2, h / 2), roll_deg, 1.0)
    m[0, 2] += dx
    m[1, 2] += dy
    out = cv2.warpAffine(img, m, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    cw, ch = int(w * crop), int(h * crop)
    x0, y0 = (w - cw) // 2, (h - ch) // 2
    return out[y0:y0 + ch, x0:x0 + cw]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--left", required=True)
    ap.add_argument("--right", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=150)
    ap.add_argument("--fps", type=float, default=15)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--qualities", type=int, nargs="+", default=[50, 70, 90])
    ap.add_argument("--scales", type=float, nargs="+", default=[1.0, 0.5])
    ap.add_argument("--sigma", type=float, default=0.15, help="sigma of the log size of the model")
    ap.add_argument("--phi", type=float, default=0.95, help="lag-1 correlation of the log size of the model")
    a = ap.parse_args()
    L, R = cv2.imread(a.left), cv2.imread(a.right)
    rng = np.random.default_rng(a.seed)
    n = int(round(a.seconds * a.fps))
    sizes = {(q, s): [] for q in a.qualities for s in a.scales}
    rw_roll = rw_dy = 0.0
    pan = 0.0
    gain = 1.0
    for k in range(n):
        t = k / a.fps
        rw_roll = 0.98 * rw_roll + rng.normal(0, 0.06)
        rw_dy = 0.98 * rw_dy + rng.normal(0, 0.8)
        pan = 0.995 * pan + rng.normal(0, 4.0)
        gain = float(np.clip(gain + rng.normal(0, 0.004), 0.92, 1.08))
        roll = 3.0 * math.sin(2 * math.pi * t / 3.0) + rw_roll
        dy = 12.0 * math.sin(2 * math.pi * t / 2.3) + rw_dy
        dx = pan
        eyes = [warp(L, roll, dx, dy), warp(R, roll, dx, dy)]
        for s in a.scales:
            ew, eh = int(round(672 * s)), int(round(376 * s))
            sbs = np.hstack([cv2.resize(e, (ew, eh), interpolation=cv2.INTER_AREA) for e in eyes]).astype(np.float32)
            sbs = sbs * gain + rng.normal(0, 1.5, sbs.shape)
            sbs = np.clip(sbs, 0, 255).astype(np.uint8)
            for q in a.qualities:
                ok, b = cv2.imencode(".jpg", sbs, [cv2.IMWRITE_JPEG_QUALITY, q])
                sizes[(q, s)].append(len(b))
    def summary(x, q, s):
        x = np.asarray(x, dtype=float)
        lx = np.log(x)
        return {"quality": q, "scale": s, "frames": len(x), "fps": a.fps, "mean": float(x.mean()),
                "median": float(np.median(x)), "p05": float(np.quantile(x, 0.05)),
                "p95": float(np.quantile(x, 0.95)), "max": float(x.max()), "min": float(x.min()),
                "cv": float(x.std() / x.mean()), "lognormal": {"mu": float(lx.mean()), "sigma": float(lx.std())},
                "acf1_log": float(np.corrcoef(lx[:-1], lx[1:])[0, 1])}

    def write(name, v, what):
        with open(os.path.join(a.out, name), "w") as f:
            f.write("# JPEG sizes (bytes), %d frames at %g fps, quality %d, scale %g (%dx%d side by side), %s; "
                    "emu/traces/make_trace.py seed %d\n" % (n, a.fps, q, s, 2 * int(round(672 * s)),
                                                           int(round(376 * s)), what, a.seed))
            f.write("\n".join(str(int(x)) for x in v) + "\n")

    stats = {"still": {}, "model": {}, "model_params": {"sigma": a.sigma, "phi": a.phi, "seed": a.seed}}
    os.makedirs(a.out, exist_ok=True)
    for (q, s), v in sorted(sizes.items()):
        tag = "q%d-s%s" % (q, ("%g" % s).replace(".", "p"))
        write("still-%s.txt" % tag, v, "still frames of the camera with synthetic motion")
        stats["still"][tag] = summary(v, q, s)
        # model: the same median, lognormal AR(1) variation (seeded per quality and scale)
        mrng = np.random.default_rng([a.seed, q, int(s * 100)])
        y = np.zeros(n)
        y[0] = mrng.normal(0, a.sigma)
        for k in range(1, n):
            y[k] = a.phi * y[k - 1] + math.sqrt(1 - a.phi ** 2) * a.sigma * mrng.normal()
        m = np.round(np.exp(math.log(float(np.median(v))) + y)).astype(int)
        write("zed-sbs-%s.txt" % tag, m, "lognormal AR(1) model (sigma %g, phi %g) at the median of the still" % (
            a.sigma, a.phi))
        stats["model"][tag] = summary(m, q, s)
        print(tag, "still median %.0f cv %.3f | model mean %.0f p95 %.0f max %.0f cv %.3f acf1 %.3f" % (
            stats["still"][tag]["median"], stats["still"][tag]["cv"], stats["model"][tag]["mean"],
            stats["model"][tag]["p95"], stats["model"][tag]["max"], stats["model"][tag]["cv"],
            stats["model"][tag]["acf1_log"]))
    with open(os.path.join(a.out, "stats.json"), "w") as f:
        json.dump(stats, f, indent=1)


if __name__ == "__main__":
    main()
