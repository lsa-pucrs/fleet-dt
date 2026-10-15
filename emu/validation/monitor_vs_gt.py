#!/usr/bin/env python3
"""Compare the external busy fraction b computed from the AP monitor stream (fleet/monitor, SPEC_v2 4.3,
captured by emu/tools/mon_capture.py) with b computed from the ground truth gt.bin (emu/gt.py), over
the same windows of the host clock.

  monitor_vs_gt.py RUN/monitor.jsonl RUN/emu/gt.bin [--len 1.0] [--sifs-us 10]

From the monitor: busy = union of the busy intervals; own = sum over the records with ok = 1 of the
vessels (sta >= 1) and of the beacons of the AP (sta = 0) of (end - start), minus SIFS for the acknowledged exchanges
(the record spans frame, SIFS and ACK; Def. busy counts the frame and its ACK).
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import gt  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("monitor")
    ap.add_argument("gt")
    ap.add_argument("--len", type=float, default=1.0)
    ap.add_argument("--sifs-us", type=int, default=10)
    a = ap.parse_args()
    busy, own, frm, to, ages, n = [], [], None, None, [], 0
    for line in open(a.monitor):
        j = json.loads(line)
        b = j["b"]
        n += 1
        frm = b["from_us"] if frm is None else frm
        to = b["to_us"]
        ages.append(j["rx_us"] - b["to_us"])
        busy += b["busy"]
        for s, e, sta, d, ipl, rate, ok, kind in b["fr"]:
            if ok and (sta >= 1 or (kind == 1 and sta == 0)):
                own.append((s, e - (a.sifs_us if (kind != 1) else 0)))
    hdr, r = gt.load(a.gt)
    org_us = hdr["origin_ns"] // 1000

    class A:
        pass
    ga = A()
    ga.ports, ga.aifsn_sta, ga.slot_us, ga.sifs_us, ga.csv = "1884", 3, 9, 10, os.devnull
    L = int(a.len * 1e6)
    t0 = max(frm, org_us + 2_000_000)
    t1 = min(to, org_us + int(r["t_ns"].max() // 1000))
    t0 = t0 + (L - (t0 - org_us) % L) % L        # align to the simulator time grid
    ga.t_from, ga.t_to, ga.len = (t0 - org_us) / 1e6, (t1 - org_us) / 1e6, a.len
    import io
    import contextlib
    buf = io.StringIO()
    ga.csv = None
    with contextlib.redirect_stdout(buf):
        gt.cmd_windows(hdr, r, ga)
    rows = [l.split(",") for l in buf.getvalue().strip().split("\n")[1:]]
    busy_m = gt.merge([tuple(x) for x in busy])
    own_m = gt.merge(own)
    print("batches %d, host-side age of the batches (rx - to_us) p50 %.0f us p99 %.0f us" % (
        n, np.percentile(ages, 50), np.percentile(ages, 99)))
    print("t_s,b_monitor,b_gt,busy_monitor,busy_gt")
    d = []
    for row in rows:
        w = org_us + int(float(row[0]) * 1e6)
        bm = gt.overlap(busy_m, w, w + L) / L
        om = gt.overlap(own_m, w, w + L) / L
        bmon = max(0.0, bm - om)
        print("%s,%.4f,%s,%.4f,%s" % (row[0], bmon, row[3], bm, row[2]))
        d.append((bmon - float(row[3]), bm - float(row[2])))
    d = np.array(d)
    print("windows %d: |b_monitor - b_gt| mean %.4f max %.4f; |busy_monitor - busy_gt| max %.4f" % (
        len(d), np.mean(np.abs(d[:, 0])), np.max(np.abs(d[:, 0])), np.max(np.abs(d[:, 1]))))


if __name__ == "__main__":
    main()
