#!/usr/bin/env python3
"""Reader and offline analysis of the ground truth gt.bin of fdt-emu (format: emu/README.md).

  python3 emu/gt.py summary RUN/gt.bin
  python3 emu/gt.py windows RUN/gt.bin --from 10 --to 70 --len 0.125 [--ports 1884] [--csv out.csv]
  python3 emu/gt.py collisions RUN/gt.bin [--from S --to S]
  python3 emu/gt.py throughput RUN/gt.bin [--from S --to S]   IP bytes of the acknowledged data frames

`windows` recomputes, for consecutive windows of `--len` seconds (simulator time), the realized
capacity kappa (sum of T_succ of the successful exchanges of the frames of the given TCP ports, divided
by the window length; paper Def. capacity with e(t) = window end) and the external busy fraction b
(AP busy time minus the airtime of the successful exchanges of the AP with the vessels, their ACKs and
the beacons, divided by the window length; paper Def. busy). Times are simulator seconds; add
header["origin_ns"] for the host clock.
"""
import argparse
import json
import struct
import sys

import numpy as np

DT = np.dtype([("t_ns", "<i8"), ("dur_ns", "<u4"), ("uid", "<u4"), ("rate_kbps", "<u4"), ("node", "<u2"),
               ("size", "<u2"), ("ip_len", "<u2"), ("src", "<i2"), ("dst", "<i2"), ("sport", "<u2"),
               ("dport", "<u2"), ("ev", "u1"), ("kind", "u1"), ("ip_proto", "u1"), ("flags", "u1"),
               ("reason", "u1"), ("pad", "u1")])
assert DT.itemsize == 40
EV = {"TX": 0, "RX_OK": 1, "RX_ERR": 2, "RX_DROP": 3, "STATE": 4, "JAM": 5}
KIND = {"data": 0, "beacon": 1, "mgmt": 2, "ack": 3, "ctl": 4, "jam": 5, "null": 6}
STATE = {"IDLE": 0, "CCA_BUSY": 1, "TX": 2, "RX": 3}


def load(path):
    with open(path, "rb") as f:
        magic = f.read(8)
        if magic[:6] != b"FDTGT1":
            raise SystemExit("%s: not a gt.bin" % path)
        (n,) = struct.unpack("<I", f.read(4))
        hdr = json.loads(f.read(n))
        data = f.read()
    k = len(data) // DT.itemsize
    return hdr, np.frombuffer(data[:k * DT.itemsize], dtype=DT)


def exchanges(r, sifs_ns=10000, slack_ns=30000):
    """Unicast data/management TX events with the matching ACK TX of the receiver.
    Returns (tx records, ack index or -1)."""
    tx = r[r["ev"] == EV["TX"]]
    acks = tx[tx["kind"] == KIND["ack"]]
    frames = tx[((tx["kind"] == KIND["data"]) | (tx["kind"] == KIND["mgmt"]) | (tx["kind"] == KIND["null"]))
                & (tx["dst"] >= 0)]
    fend = frames["t_ns"] + frames["dur_ns"].astype(np.int64)
    match = np.full(len(frames), -1, dtype=np.int64)
    if len(acks):
        order = np.argsort(acks["t_ns"], kind="stable")
        at = acks["t_ns"][order]
        lo = np.searchsorted(at, fend + sifs_ns - 3000)
        hi = np.searchsorted(at, fend + sifs_ns + slack_ns)
        for i in np.nonzero(hi > lo)[0]:
            for j in range(lo[i], hi[i]):
                a = acks[order[j]]
                # the ACK comes from the receiver of the frame and goes to its transmitter (dst of ACK
                # is the transmitter node; src of ACK TX is the node that sends it)
                if a["node"] == frames["dst"][i] and a["dst"] == frames["node"][i]:
                    match[i] = order[j]
                    break
    return frames, acks, match


def merge(iv):
    if not iv:
        return []
    iv.sort()
    out = [list(iv[0])]
    for s, e in iv[1:]:
        if s <= out[-1][1]:
            out[-1][1] = max(out[-1][1], e)
        else:
            out.append([s, e])
    return out


def overlap(ivs, a, b):
    """total length of the (merged) intervals ivs inside [a, b)"""
    tot = 0
    for s, e in ivs:
        if e <= a or s >= b:
            continue
        tot += min(e, b) - max(s, a)
    return tot


def cmd_summary(hdr, r, a):
    names = [n["name"] for n in hdr["nodes"]]
    span = (r["t_ns"].max() - r["t_ns"].min()) / 1e9 if len(r) else 0
    print("origin_ns %d  records %d  span %.1f s  nodes %s" % (hdr["origin_ns"], len(r), span, names))
    print("%-6s %8s %8s %8s %8s %8s %8s" % ("node", "TX", "RX_OK", "RX_ERR", "RX_DROP", "STATE", "JAM"))
    for n, name in enumerate(names):
        m = r["node"] == n
        c = [int(np.sum(m & (r["ev"] == EV[k]))) for k in ("TX", "RX_OK", "RX_ERR", "RX_DROP", "STATE", "JAM")]
        print("%-6s %8d %8d %8d %8d %8d %8d" % (name, *c))
    frames, acks, match = exchanges(r)
    for n, name in enumerate(names):
        m = frames["node"] == n
        d = m & (frames["kind"] == KIND["data"])
        if np.sum(d):
            print("%-6s data frames %d acked %.4f retries %.4f" % (
                name, np.sum(d), np.mean(match[d] >= 0), np.mean((frames["flags"][d] & 1) > 0)))


def cmd_windows(hdr, r, a):
    M = hdr["M"]
    ports = set(int(p) for p in a.ports.split(","))
    aifs_sta = (a.aifsn_sta * a.slot_us + a.sifs_us) * 1000
    frames, acks, match = exchanges(r)
    ok = match >= 0
    ackdur = np.zeros(len(frames), dtype=np.int64)
    ackdur[ok] = acks["dur_ns"][match[ok]].astype(np.int64)
    ackend = np.where(ok, acks["t_ns"][np.maximum(match, 0)] + ackdur, 0)
    tsucc = frames["dur_ns"].astype(np.int64) + a.sifs_us * 1000 + ackdur + aifs_sta
    bridge = np.isin(frames["sport"], list(ports)) | np.isin(frames["dport"], list(ports))
    fleet = ok & bridge & (((frames["node"] >= 1) & (frames["node"] <= M) & (frames["dst"] == 0)) |
                           ((frames["node"] == 0) & (frames["dst"] >= 1) & (frames["dst"] <= M)))
    # own airtime of the AP (Def. busy): successful exchanges AP<->vessels (frame + ACK) and beacons
    own_mask = ok & (((frames["node"] >= 1) & (frames["node"] <= M) & (frames["dst"] == 0)) |
                     ((frames["node"] == 0) & (frames["dst"] >= 1) & (frames["dst"] <= M)))
    own = [(int(t), int(t) + int(d)) for t, d in zip(frames["t_ns"][own_mask], frames["dur_ns"][own_mask])]
    own += [(int(t), int(e)) for t, e in zip(acks["t_ns"][match[own_mask]], acks["t_ns"][match[own_mask]] +
                                                acks["dur_ns"][match[own_mask]].astype(np.int64))]
    tx = r[r["ev"] == EV["TX"]]
    bea = tx[(tx["node"] == 0) & (tx["kind"] == KIND["beacon"])]
    own += [(int(t), int(t) + int(d)) for t, d in zip(bea["t_ns"], bea["dur_ns"])]
    own = merge(own)
    st = r[(r["ev"] == EV["STATE"]) & (r["node"] == 0) & (r["reason"] >= 1) & (r["reason"] <= 3)]
    busy = merge([(int(t), int(t) + int(d)) for t, d in zip(st["t_ns"], st["dur_ns"])])
    if not busy:
        print("no AP state records (run fdt-emu with --gtStates=ap or all)", file=sys.stderr)
    t0, t1, L = int(a.t_from * 1e9), int(a.t_to * 1e9), int(a.len * 1e9)
    fs, fe = frames["t_ns"][fleet], (frames["t_ns"][fleet] + tsucc[fleet])
    rows = []
    for w in range(t0, t1 - L + 1, L):
        sel = (fs >= w) & (fs < w + L)
        tf = float(np.sum(np.minimum(fe[sel], w + L) - fs[sel]))
        bt = overlap(busy, w, w + L)
        ot = overlap(own, w, w + L)
        rows.append((w / 1e9, tf / L, bt / L, max(0.0, (bt - ot) / L), int(np.sum(sel))))
    out = open(a.csv, "w") if a.csv else sys.stdout
    out.write("t_s,kappa,busy,b_ext,n_exch\n")
    for row in rows:
        out.write("%.4f,%.5f,%.5f,%.5f,%d\n" % row)
    if rows:
        k = np.array([x[1] for x in rows])
        b = np.array([x[3] for x in rows])
        print("windows %d  kappa mean %.4f  b_ext mean %.4f" % (len(rows), k.mean(), b.mean()), file=sys.stderr)


def cmd_collisions(hdr, r, a):
    """Per vessel: data frames sent to the GS, fraction acknowledged, retries, and the RX errors and
    drops at the AP (the collisions that the AP sees)."""
    names = [n["name"] for n in hdr["nodes"]]
    t0, t1 = int(a.t_from * 1e9), int(a.t_to * 1e9)
    r = r[(r["t_ns"] >= t0) & (r["t_ns"] < t1)]
    frames, acks, match = exchanges(r)
    print("%-5s %8s %8s %8s %10s %10s" % ("node", "data_tx", "acked", "retry", "ap_rx_err", "ap_rx_drop"))
    for n in range(1, hdr["M"] + 1):
        d = (frames["node"] == n) & (frames["kind"] == KIND["data"]) & (frames["dst"] == 0)
        ap = r[(r["node"] == 0) & (r["src"] == n)]
        print("%-5s %8d %8.4f %8.4f %10d %10d" % (
            names[n], np.sum(d), np.mean(match[d] >= 0) if np.sum(d) else float("nan"),
            np.mean((frames["flags"][d] & 1) > 0) if np.sum(d) else float("nan"),
            np.sum(ap["ev"] == EV["RX_ERR"]), np.sum(ap["ev"] == EV["RX_DROP"])))
    allerr = r[(r["node"] == 0) & (r["ev"] == EV["RX_ERR"])]
    print("AP RX errors (all transmitters): %d; AP PPDU drops: %d" % (
        len(allerr), int(np.sum((r["node"] == 0) & (r["ev"] == EV["RX_DROP"])))))


def cmd_throughput(hdr, r, a):
    names = [n["name"] for n in hdr["nodes"]]
    t0, t1 = int(a.t_from * 1e9), int(a.t_to * 1e9)
    r = r[(r["t_ns"] >= t0) & (r["t_ns"] < t1)]
    frames, acks, match = exchanges(r)
    d = (frames["kind"] == KIND["data"]) & (match >= 0)
    tot = 0.0
    for n, name in enumerate(names):
        b = float(np.sum(frames["ip_len"][d & (frames["node"] == n)]))
        if b:
            print("%-5s %8.3f Mb/s" % (name, 8 * b / ((t1 - t0) / 1e9) / 1e6))
            tot += b
    print("total %8.3f Mb/s" % (8 * tot / ((t1 - t0) / 1e9) / 1e6))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=["summary", "windows", "collisions", "throughput"])
    ap.add_argument("path")
    ap.add_argument("--from", dest="t_from", type=float, default=0)
    ap.add_argument("--to", dest="t_to", type=float, default=1e9)
    ap.add_argument("--len", type=float, default=0.125)
    ap.add_argument("--ports", default="1884")
    ap.add_argument("--aifsn-sta", type=int, default=3)
    ap.add_argument("--slot-us", type=int, default=9)
    ap.add_argument("--sifs-us", type=int, default=10)
    ap.add_argument("--csv")
    a = ap.parse_args(argv)
    hdr, r = load(a.path)
    if a.t_to > 1e8 and len(r):
        a.t_to = r["t_ns"].max() / 1e9
    {"summary": cmd_summary, "windows": cmd_windows, "collisions": cmd_collisions,
     "throughput": cmd_throughput}[a.cmd](hdr, r, a)


if __name__ == "__main__":
    main()
