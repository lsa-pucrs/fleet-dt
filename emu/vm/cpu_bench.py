#!/usr/bin/env python3
"""Single-thread CPU benchmark for the calibration of the vessel vCPU against the Raspberry Pi 4 of the
Jundiá vessels (design item I9; emu/vm/cpu_profile.py).

Fixed work units that resemble the work of the vessel software in phases 1 and 2 (Python publishers,
the DTI and the AS: JSON and binary encoding, dictionaries, hashing of message payloads, small integer
arithmetic), each timed over --seconds of repetitions; the score of a kernel is units per second and the
summary is their geometric mean. Python 3 standard library only, so the same file runs on Raspberry Pi OS
and in the Debian guest.

  python3 emu/vm/cpu_bench.py [--seconds 3] [--out result.json]
"""
import argparse
import hashlib
import json
import math
import os
import platform
import struct
import time
import zlib

TOPICS = [{"name": "video_%d" % i, "rate": 64, "bound": 1400} for i in range(1, 17)]


def k_json():
    d = {"t": 123456, "vessel": 3, "demand": [[t["name"], t["rate"] * t["bound"]] for t in TOPICS]}
    s = json.dumps(d)
    return len(json.loads(s)["demand"])


def k_struct():
    b = b"".join(struct.pack("!IB", k, k & 3) for k in range(64))
    return sum(struct.unpack_from("!IB", b, 5 * k)[1] for k in range(64))


def k_dict():
    d = {}
    for k in range(400):
        d[(k % 22, k % 4)] = d.get((k % 22, k % 4), 0) + k
    return len(d)


PAYLOAD = bytes(range(256)) * 6


def k_hash():
    h = hashlib.sha256()
    for _ in range(8):
        h.update(PAYLOAD)
    return h.digest()[0]


def k_zlib():
    return len(zlib.compress(PAYLOAD * 2, 1))


def k_arith():
    x = 0
    for k in range(1000):
        x = (x * 31 + k) & 0xFFFFFFFF
    return x


KERNELS = {"json": k_json, "struct": k_struct, "dict": k_dict, "sha256": k_hash, "zlib": k_zlib, "arith": k_arith}


def run(seconds=3.0):
    res = {}
    for name, fn in KERNELS.items():
        for _ in range(50):                       # warm-up
            fn()
        n, t0 = 0, time.perf_counter()
        while True:
            for _ in range(20):
                fn()
            n += 20
            dt = time.perf_counter() - t0
            if dt >= seconds:
                break
        res[name] = n / dt
    geo = math.exp(sum(math.log(v) for v in res.values()) / len(res))
    return {"kernels": res, "score": geo, "python": platform.python_version(), "machine": platform.machine(),
            "node": platform.node(), "cpus": os.cpu_count(), "seconds": seconds, "t_unix": time.time()}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    r = run(a.seconds)
    txt = json.dumps(r, indent=1)
    if a.out:
        with open(a.out, "w") as f:
            f.write(txt + "\n")
    print(txt)


if __name__ == "__main__":
    main()
