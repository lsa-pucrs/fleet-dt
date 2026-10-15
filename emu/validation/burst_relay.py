#!/usr/bin/env python3
"""Relay load test of fdt-emu without VMs: plays the VMs of M vessels on the UDP ports of the emulator and
sends, every period, a burst of K IPv4/TCP frames (port 1884, size bytes) from every vessel to the GS at
once (the release of the data window of a step); the frames that the emulator relays are read and dropped.
The emulator writes its real-time lag (lag.txt, summary.json) as in a run with VMs.

  burst_relay.py --emu BIN --scenario S.json --out DIR [--base 45000] [--k 11] [--size 1500] [--period 0.125]
                 [--duration 30] [--cpus 14,15]
"""
import argparse
import os
import select
import socket
import struct
import subprocess
import time


def frame(h, size, seq):
    src = bytes.fromhex("525400fd00%02x" % h)
    dst = bytes.fromhex("525400fd0000")
    ip = struct.pack("!BBHHHBBH4s4s", 0x45, 0, size, seq & 0xFFFF, 0x4000, 64, 6, 0,
                     bytes([10, 10, 0, 10 + h]), bytes([10, 10, 0, 1]))
    tcp = struct.pack("!HHIIBBHHH", 1884, 40000 + h, seq, 0, 0x50, 0x18, 65535, 0, 0)
    return dst + src + b"\x08\x00" + ip + tcp + bytes(size - 40)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--emu", required=True)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--base", type=int, default=45000)
    ap.add_argument("--k", type=int, default=11)
    ap.add_argument("--size", type=int, default=1500)
    ap.add_argument("--period", type=float, default=0.125)
    ap.add_argument("--duration", type=float, default=30)
    ap.add_argument("--cpus", default="14,15")
    ap.add_argument("--extra", nargs="*", default=[])
    a = ap.parse_args()
    import json
    M = json.load(open(a.scenario))["M"]
    socks = []
    for h in range(M + 1):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        s.bind(("127.0.0.1", a.base + 2 * h))
        s.setblocking(False)
        socks.append(s)
    os.makedirs(a.out, exist_ok=True)
    p = subprocess.Popen(["taskset", "-c", a.cpus, a.emu, "--scenario=" + a.scenario, "--outdir=" + a.out,
                          "--basePort=%d" % a.base, "--monitor=none", "--rt=HardLimit", "--gt=1",
                          "--duration=%g" % (a.duration + 3)] + a.extra,
                         stdout=open(os.path.join(a.out, "emu.out"), "w"), stderr=subprocess.STDOUT)
    t0 = time.time()
    while "READY" not in open(os.path.join(a.out, "emu.out")).read():
        time.sleep(0.05)
        if time.time() - t0 > 15:
            raise SystemExit("not ready")
    seq, nxt, rx = 0, time.time() + 1.0, 0
    end = time.time() + a.duration
    while time.time() < end:
        r, _, _ = select.select(socks, [], [], max(0.0, nxt - time.time()))
        for s in r:
            try:
                while True:
                    s.recv(4096)
                    rx += 1
            except BlockingIOError:
                pass
        if time.time() >= nxt:
            for h in range(1, M + 1):
                for k in range(a.k):
                    socks[h].sendto(frame(h, a.size, seq), ("127.0.0.1", a.base + 2 * h + 1))
                    seq += 1
            nxt += a.period
    p.wait()
    print(open(os.path.join(a.out, "emu.out")).read().strip().splitlines()[-1], "| relayed back", rx)


if __name__ == "__main__":
    main()
