#!/usr/bin/env python3
"""Frame-size traces of the video topic from a recorded stereo sequence over water (Pohang Canal Dataset).

  python3 emu/traces/make_trace_water.py --out emu/traces [--seq pohang00 --first 12000 --frames 4500]
          [--threads 24]

Input: the stereo camera of the Pohang Canal Dataset (Chung, Kim, Lee and Kim, "Pohang Canal Dataset: A
Multimodal Maritime Dataset for Autonomous Navigation in Restricted Waters", IJRR 2023,
doi:10.1177/02783649231191145; license CC BY-NC 4.0; public bucket s3://pohang-canal-dataset, us-west-2):
<seq>/stereo.zip holds left_images/NNNNNN.png and right_images/NNNNNN.png (2048x1080 per eye, 8-bit BGR,
10 frames/s) and timestamp.txt. The program reads the members of the ZIP through HTTP range requests
(no full download of the 112 GB archive), checks the CRC-32 of every member, and for each pair k of
[first, first + frames):

  each eye: centre crop to the aspect of the ZED WVGA eye (672:376), resize (INTER_AREA) to
  (672 S) x (376 S); side by side [left | right] (1344x376 at S = 1, 672x188 at S = 0.5); JPEG of OpenCV
  cv2.imencode(".jpg", img, [IMWRITE_JPEG_QUALITY, Q]) as vessel/publishers/zed_publisher.py encodes the
  side-by-side frame of the ZED (4:2:0, baseline).

Output: water-sbs-q<Q>-s<S>.txt (one size in bytes per frame, in the order of the sequence; the sources
replay the sequence from a seeded start, emu/traces/README.md), water-members.txt (the member names and
CRC-32 of the input), and the entry "water" of stats.json (mean, median, quantiles, CV, lag-1
autocorrelation of the log size, input description).
"""
import argparse
import concurrent.futures as cf
import io
import json
import os
import struct
import time
import urllib.request
import zipfile
import zlib

import cv2
import numpy as np

BUCKET = "https://pohang-canal-dataset.s3.us-west-2.amazonaws.com"
EYE = (672, 376)


class RangeFile(io.RawIOBase):
    """seekable file over HTTP range requests (for the central directory of the ZIP)"""

    def __init__(self, url):
        self.url, self.pos = url, 0
        with urllib.request.urlopen(urllib.request.Request(url, method="HEAD"), timeout=60) as r:
            self.size = int(r.headers["Content-Length"])

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, off, whence=0):
        self.pos = off if whence == 0 else self.pos + off if whence == 1 else self.size + off
        return self.pos

    def readinto(self, b):
        n = min(len(b), self.size - self.pos)
        if n <= 0:
            return 0
        req = urllib.request.Request(self.url, headers={"Range": "bytes=%d-%d" % (self.pos, self.pos + n - 1)})
        with urllib.request.urlopen(req, timeout=120) as r:
            d = r.read()
        b[:len(d)] = d
        self.pos += len(d)
        return len(d)


def member(url, zi):
    """bytes of one ZIP member (local header + data by one range request), CRC-32 checked"""
    for k in range(6):
        try:
            req = urllib.request.Request(url, headers={"Range": "bytes=%d-%d" % (
                zi.header_offset, zi.header_offset + 30 + 1024 + zi.compress_size)})
            with urllib.request.urlopen(req, timeout=180) as r:
                b = r.read()
            n, e = struct.unpack("<HH", b[26:30])
            data = b[30 + n + e:30 + n + e + zi.compress_size]
            out = zlib.decompress(data, -15) if zi.compress_type == zipfile.ZIP_DEFLATED else data
            if zlib.crc32(out) & 0xffffffff != zi.CRC:
                raise IOError("CRC mismatch in %s" % zi.filename)
            return out
        except Exception:
            if k == 5:
                raise
            time.sleep(2 * (k + 1))


def eye(img, scale):
    h, w = img.shape[:2]
    cw = int(round(h * EYE[0] / EYE[1]))
    if cw < w:
        x0 = (w - cw) // 2
        img = img[:, x0:x0 + cw]
    return cv2.resize(img, (int(EYE[0] * scale), int(EYE[1] * scale)), interpolation=cv2.INTER_AREA)


def stats(v):
    v = np.asarray(v, float)
    lg = np.log(v)
    return {"frames": len(v), "mean": float(v.mean()), "median": float(np.median(v)),
            "p05": float(np.percentile(v, 5)), "p95": float(np.percentile(v, 95)), "min": float(v.min()),
            "max": float(v.max()), "cv": float(v.std() / v.mean()),
            "lognormal": {"mu": float(lg.mean()), "sigma": float(lg.std())},
            "acf1_log": float(np.corrcoef(lg[:-1], lg[1:])[0, 1])}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--seq", default="pohang00")
    ap.add_argument("--first", type=int, default=12000)
    ap.add_argument("--frames", type=int, default=4500)
    ap.add_argument("--qualities", type=int, nargs="+", default=[50, 70, 90])
    ap.add_argument("--scales", type=float, nargs="+", default=[1.0, 0.5])
    ap.add_argument("--threads", type=int, default=24)
    a = ap.parse_args()
    url = "%s/%s/stereo.zip" % (BUCKET, a.seq)
    z = zipfile.ZipFile(io.BufferedReader(RangeFile(url), buffer_size=1 << 20))
    info = {i.filename: i for i in z.infolist()}
    ts = [ln.split() for ln in z.read("stereo/timestamp.txt").decode().splitlines() if ln.strip()]
    tsd = {s: float(t) for t, s in ts}
    ks = list(range(a.first, a.first + a.frames))
    names = [("stereo/left_images/%06d.png" % k, "stereo/right_images/%06d.png" % k) for k in ks]
    keys = [(q, s) for q in a.qualities for s in a.scales]

    def work(pair):
        imgs = [cv2.imdecode(np.frombuffer(member(url, info[n]), np.uint8), cv2.IMREAD_COLOR) for n in pair]
        out = {}
        for s in a.scales:
            sbs = np.hstack([eye(imgs[0], s), eye(imgs[1], s)])
            for q in a.qualities:
                ok, enc = cv2.imencode(".jpg", sbs, [int(cv2.IMWRITE_JPEG_QUALITY), q])
                out[(q, s)] = len(enc)
        return out

    t0 = time.time()
    res = [None] * len(names)
    with cf.ThreadPoolExecutor(a.threads) as ex:
        futs = {ex.submit(work, p): i for i, p in enumerate(names)}
        for n, f in enumerate(cf.as_completed(futs)):
            res[futs[f]] = f.result()
            if n % 250 == 0:
                print("%d/%d pairs, %.0f s" % (n, len(names), time.time() - t0), flush=True)
    os.makedirs(a.out, exist_ok=True)
    tag = lambda s: ("%g" % s).replace(".", "p")   # noqa: E731
    st = {}
    for q, s in keys:
        v = [r[(q, s)] for r in res]
        fn = "water-sbs-q%d-s%s.txt" % (q, tag(s))
        with open(os.path.join(a.out, fn), "w") as f:
            f.write("# JPEG sizes (bytes) of the side-by-side stereo frames of the Pohang Canal Dataset (CC BY-NC 4.0),\n"
                    "# %s pairs %06d-%06d (10 frames/s), re-encoded like zed_publisher: eyes %dx%d, quality %d\n"
                    "# emu/traces/make_trace_water.py\n" % (a.seq, ks[0], ks[-1], EYE[0] * s, EYE[1] * s, q))
            f.write("\n".join(str(x) for x in v) + "\n")
        st["q%d-s%s" % (q, tag(s))] = dict(stats(v), quality=q, scale=s)
    with open(os.path.join(a.out, "water-members.txt"), "w") as f:
        f.write("# member CRC-32 timestamp of %s\n" % url)
        for pair in names:
            for n in pair:
                f.write("%s %08x %.6f\n" % (n, info[n].CRC, tsd.get(n.rsplit("/", 1)[1][:-4], float("nan"))))
    sp = os.path.join(a.out, "stats.json")
    allst = json.load(open(sp)) if os.path.exists(sp) else {}
    allst["water"] = {"dataset": "Pohang Canal Dataset (Chung et al., IJRR 2023, doi:10.1177/02783649231191145)",
                      "license": "CC BY-NC 4.0", "url": url, "sequence": a.seq, "first": ks[0], "last": ks[-1],
                      "fps": 10, "eye": "centre crop to 672:376, INTER_AREA to 672S x 376S", "opencv": cv2.__version__,
                      "files": st}
    with open(sp, "w") as f:
        json.dump(allst, f, indent=1)
    print("done in %.0f s" % (time.time() - t0))


if __name__ == "__main__":
    main()
