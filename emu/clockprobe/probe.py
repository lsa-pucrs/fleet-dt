#!/usr/bin/env python3
"""Clock probe of the emulated hosts (SPEC_v2.md Section 7, Clock).

Every --period seconds, reads CLOCK_REALTIME of each VM over vsock (emu/vm/guest/fdt-clockd.py, port
5000, CID 100+i) or over the virtio-serial port fdt.clock of the VM (unix socket of the QEMU chardev,
--socks; hosts without access to /dev/vhost-vsock) and logs, per VM, the offset of the VM clock to the
host clock (vm - (t_send + t_recv)/2) and the round-trip time.

  probe.py --cids 100,101,102 --names gs,v1,v2 --out clock.csv [--period 0.1]
  probe.py --socks VMS/gs/clock.sock,VMS/v1/clock.sock --names gs,v1 --out clock.csv
  probe.py summary clock.csv [--after S]      per host: first offset, time until |offset| < 1 ms, and the
                                             |offset| statistics (us) from 10 s after that

CSV columns: host_ns (host CLOCK_REALTIME at the send), host, offset_ns, rtt_ns.
"""
import argparse
import socket
import struct
import sys
import time

PORT = 5000


def connect(cid):
    if isinstance(cid, str):            # unix socket of the virtio-serial chardev
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect(cid)
        return s
    s = socket.socket(socket.AF_VSOCK, socket.SOCK_STREAM)
    s.settimeout(0.5)
    s.connect((cid, PORT))
    return s


def probe(a):
    cids = a.socks.split(",") if a.socks else [int(x) for x in a.cids.split(",")]
    names = a.names.split(",") if a.names else [str(c) for c in cids]
    socks = {c: None for c in cids}
    out = open(a.out, "a", buffering=1 << 16)
    if out.tell() == 0:
        out.write("host_ns,host,offset_ns,rtt_ns\n")
    nxt = time.monotonic()
    while True:
        for c, name in zip(cids, names):
            try:
                if socks[c] is None:
                    socks[c] = connect(c)
                s = socks[c]
                t0 = time.clock_gettime_ns(time.CLOCK_REALTIME)
                s.sendall(b"x")
                b = b""
                while len(b) < 8:
                    k = s.recv(8 - len(b))
                    if not k:
                        raise OSError("closed")
                    b += k
                t1 = time.clock_gettime_ns(time.CLOCK_REALTIME)
                (g,) = struct.unpack("<q", b)
                out.write("%d,%s,%d,%d\n" % (t0, name, g - (t0 + t1) // 2, t1 - t0))
            except OSError:
                if socks[c] is not None:
                    socks[c].close()
                socks[c] = None
        out.flush()
        nxt += a.period
        time.sleep(max(0.0, nxt - time.monotonic()))


def summary(a):
    """per host: first offset, time until |offset| < 1 ms, and |offset| statistics from 10 s after that"""
    rows = {}
    t_first = None
    with open(a.csv) as f:
        f.readline()
        for line in f:
            t, h, off, rtt = line.strip().split(",")
            t = int(t)
            t_first = t if t_first is None else t_first
            if (t - t_first) / 1e9 < a.after:
                continue
            rows.setdefault(h, []).append((t, int(off), int(rtt)))
    print("%-4s %7s %9s %8s %10s %10s %10s %8s %9s" % ("host", "n", "off0_ms", "t<1ms_s", "|off|_p50", "|off|_p99",
                                                       "|off|_max", "n>1ms", "rtt_p50"))
    q = lambda arr, p: arr[min(len(arr) - 1, int(p * len(arr)))] if arr else float("nan")
    for h in sorted(rows, key=lambda x: (x != "gs", x)):
        v = rows[h]
        t0 = v[0][0]
        k = next((i for i, x in enumerate(v) if abs(x[1]) < 1e6), None)
        tc = (v[k][0] - t0) / 1e9 if k is not None else float("nan")
        after = [x for x in v if k is not None and x[0] >= v[k][0] + 10e9]
        ab = sorted(abs(x[1]) / 1e3 for x in after)
        rt = sorted(x[2] / 1e3 for x in v)
        print("%-4s %7d %9.3f %8.1f %10.1f %10.1f %10.1f %8d %9.1f" % (
            h, len(v), v[0][1] / 1e6, tc, q(ab, .5), q(ab, .99), ab[-1] if ab else float("nan"),
            sum(1 for x in ab if x > 1000), q(rt, .5)))


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "summary":
        ap = argparse.ArgumentParser()
        ap.add_argument("cmd")
        ap.add_argument("csv")
        ap.add_argument("--after", type=float, default=0)
        summary(ap.parse_args())
        return
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--cids")
    g.add_argument("--socks", help="unix sockets of the virtio-serial clock ports, one per VM")
    ap.add_argument("--names", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--period", type=float, default=0.1)
    probe(ap.parse_args())


if __name__ == "__main__":
    main()
