#!/usr/bin/env python3
"""Validate y(t) computed from the nl80211 counter export of the emulator (fdt-emu --cnt, by the logic of
fdt_chmon.c in fleetdt/monitor_adapter.py) against the ground truth gt.bin of the same run.

  counters_vs_gt.py RUN/emu/counters.jsonl RUN/emu/gt.bin --dtp RUN/dtp.json
                    [--y RUN/throughput.jsonl] [--csv out.csv] [--skip-s 3] [--json]

For every pair of consecutive counter reads (a, b) that the adapter turns into y(t), the tool computes
over the same interval, scaled to Ts like fdt_chmon scales it:

  y_cnt     y(t) of the counters (monitor_adapter.compute, the arithmetic of fdt_chmon.c)
  y_model   R_PHY (Ts - Tb_gt + To_model) / 8: the exact busy time of the AP (PHY states CCA_BUSY, TX, RX
            of gt.bin) and, for To, 8 (MPDU bytes without FCS + h_W) / R_PHY of every unique MPDU between the
            AP and a vessel that reached its receiver (acknowledged; the first acknowledged copy of a
            packet), at the end of its exchange. y_cnt - y_model is the error of the counters themselves:
            whole-ms survey, the estimate B_tx N_acked / N_tx, the instants at which the driver counts
            (first transmission, RX end, drop at the retry limit), the read jitter
  y_phy     R_PHY (Ts - Tb_gt + To_phy) / 8 with To_phy the measured airtime (frame and ACK) of the same
            exchanges inside the interval: Section V-B with the true durations. y_model - y_phy is the
            error of the airtime model 8 (B + N h_W) / R_PHY
  y_step    y_phy over the nominal step [start(t), start(t + 1)) instead of the read interval

and reports bias, mean and 99th percentile of the relative error |e| / y_phy, and the maximum, for
y_cnt - y_model, y_model - y_phy, y_cnt - y_phy and y_cnt - y_step, with the parts of Tb and To.
With --y (the log of the adapter), it also checks that the published y(t) equal y_cnt (sigma = 0) or
reports the realized noise (sigma > 0). The first --skip-s seconds of simulator time are left out.
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", ".."))
import gt  # noqa: E402
from fleetdt import monitor_adapter as ma  # noqa: E402


class Measure:
    """Total length of a union of intervals in (-inf, x], vectorized: overlap(a, b) = F(b) - F(a)."""

    def __init__(self, iv):
        m = gt.merge([tuple(x) for x in iv]) if len(iv) else []
        self.s = np.array([x[0] for x in m], dtype=np.int64)
        self.e = np.array([x[1] for x in m], dtype=np.int64)
        self.c = np.concatenate([[0], np.cumsum(self.e - self.s)]) if len(m) else np.zeros(1, dtype=np.int64)

    def F(self, x):
        x = np.asarray(x, dtype=np.int64)
        if not len(self.s):
            return np.zeros_like(x)
        k = np.searchsorted(self.s, x, side="right")         # intervals that start at or before x
        full = self.c[np.maximum(k - 1, 0)]                    # complete intervals before the k-th
        last = np.where(k > 0, np.minimum(x, self.e[np.maximum(k - 1, 0)]) - self.s[np.maximum(k - 1, 0)], 0)
        return np.where(k > 0, full + np.maximum(last, 0), 0)

    def overlap(self, a, b):
        return self.F(b) - self.F(a)


def pairs(d, lines):
    """The pairs of reads that the adapter turns into y(t): (t, a, b, result)."""
    p = ma.StreamPairer(d)
    out = []
    for line in lines:
        s = ma.parse_sample(line)
        if s is None:
            continue
        prev = p.prev
        r = p.feed(s)
        if r is not None:
            out.append((r[0], prev, s, r[1]))
    return out


def ground_truth(hdr, rec, M, h_W, R):
    """Busy intervals of the AP, airtime intervals and modelled airtime events of the fleet exchanges."""
    st = rec[(rec["ev"] == gt.EV["STATE"]) & (rec["node"] == 0) & (rec["reason"] >= 1) & (rec["reason"] <= 3)]
    busy = Measure(np.stack([st["t_ns"], st["t_ns"] + st["dur_ns"].astype(np.int64)], axis=1) if len(st)
                   else np.zeros((0, 2), dtype=np.int64))
    frames, acks, match = gt.exchanges(rec)
    fleet = (match >= 0) & (((frames["node"] == 0) & (frames["dst"] >= 1) & (frames["dst"] <= M)) |
                            ((frames["node"] >= 1) & (frames["node"] <= M) & (frames["dst"] == 0)))
    f = frames[fleet]
    a = acks[match[fleet]]
    fs = f["t_ns"].astype(np.int64)
    fe = fs + f["dur_ns"].astype(np.int64)
    as_ = a["t_ns"].astype(np.int64)
    ae = as_ + a["dur_ns"].astype(np.int64)
    air = Measure(np.concatenate([np.stack([fs, fe], 1), np.stack([as_, ae], 1)]) if len(f)
                  else np.zeros((0, 2), dtype=np.int64))
    # unique MPDUs: the first acknowledged copy of each (transmitter, packet uid)
    order = np.argsort(fs, kind="stable")
    seen, keep = set(), np.zeros(len(f), dtype=bool)
    for i in order:
        k = (int(f["node"][i]), int(f["uid"][i]))
        if k not in seen:
            seen.add(k)
            keep[i] = True
    ev_t = ae[keep]
    ev_s = 8.0 * (f["size"][keep].astype(np.float64) - 4 + h_W) / R
    o = np.argsort(ev_t, kind="stable")
    ev_t, ev_c = ev_t[o], np.concatenate([[0.0], np.cumsum(ev_s[o])])
    model = lambda x0, x1: ev_c[np.searchsorted(ev_t, x1, side="left")] - ev_c[np.searchsorted(ev_t, x0, side="left")]
    return busy, air, model, int(np.sum(keep)), int(len(f))


def stats(e, ref):
    rel = np.abs(e) / np.maximum(ref, 1.0)
    return {"n": int(len(e)), "bias": float(np.mean(e)), "bias_rel": float(np.mean(e / np.maximum(ref, 1.0))),
            "mean_abs_rel": float(np.mean(rel)), "p99_abs_rel": float(np.percentile(rel, 99)),
            "max_abs_rel": float(np.max(rel)), "max_abs": float(np.max(np.abs(e)))}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("counters")
    ap.add_argument("gt")
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--y", default=None, help="log of the adapter (throughput.jsonl)")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--skip-s", type=float, default=3.0)
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    d = ma.ChmonDTP.load(a.dtp)
    hdr, rec = gt.load(a.gt)
    org = int(hdr["origin_ns"])
    M = int(hdr["M"])
    with open(a.counters) as f:
        P = pairs(d, f)
    if not P:
        raise SystemExit("no pair of consecutive reads")
    busy, air, model, n_unique, n_exch = ground_truth(hdr, rec, M, d.h_W, d.R_PHY)
    t = np.array([p[0] for p in P], dtype=np.int64)
    A = np.array([p[1].at_us for p in P], dtype=np.int64) * 1000 - org     # simulator ns
    B = np.array([p[2].at_us for p in P], dtype=np.int64) * 1000 - org
    sel = A >= int(a.skip_s * 1e9)
    sel &= B <= int(rec["t_ns"].max()) if len(rec) else sel
    t, A, B = t[sel], A[sel], B[sel]
    P = [p for p, s in zip(P, sel) if s]
    Ts, R = d.Ts, d.R_PHY
    ymax = R * Ts / 8.0
    clip = lambda y: np.clip(y, 0.0, ymax)
    w = (B - A).astype(np.float64)
    sc = Ts * 1e9 / w
    Tb_gt = busy.overlap(A, B) / 1e9 * sc
    To_phy = air.overlap(A, B) / 1e9 * sc
    To_mod = model(A, B) * sc
    y_cnt = np.array([p[3].y for p in P])
    Tb_cnt = np.array([p[3].Tb for p in P])
    To_cnt = np.array([p[3].To for p in P])
    y_model = clip(R * (Ts - Tb_gt + To_mod) / 8.0)
    y_phy = clip(R * (Ts - Tb_gt + To_phy) / 8.0)
    S = (d.epoch_us + t * d.Ts_us) * 1000 - org
    E = S + d.Ts_us * 1000
    y_step = clip(R * (Ts - busy.overlap(S, E) / 1e9 + air.overlap(S, E) / 1e9) / 8.0)
    out = {"pairs": int(len(t)), "exchanges": n_exch, "unique_mpdus": n_unique,
           "read_interval_us": {"min": float(w.min() / 1e3), "max": float(w.max() / 1e3)},
           "y_phy_mean": float(np.mean(y_phy)), "y_cnt_mean": float(np.mean(y_cnt)),
           "cnt_vs_model": stats(y_cnt - y_model, y_phy), "model_vs_phy": stats(y_model - y_phy, y_phy),
           "cnt_vs_phy": stats(y_cnt - y_phy, y_phy), "cnt_vs_step": stats(y_cnt - y_step, y_step),
           "Tb_cnt_minus_gt_ms": {"mean": float(np.mean(Tb_cnt - Tb_gt) * 1e3),
                                  "max_abs": float(np.max(np.abs(Tb_cnt - Tb_gt)) * 1e3)},
           "To_cnt_minus_model_ms": {"mean": float(np.mean(To_cnt - To_mod) * 1e3),
                                     "max_abs": float(np.max(np.abs(To_cnt - To_mod)) * 1e3)},
           "To_model_minus_phy_ms": {"mean": float(np.mean(To_mod - To_phy) * 1e3),
                                     "max_abs": float(np.max(np.abs(To_mod - To_phy)) * 1e3)}}
    if a.y:
        pub = {}
        with open(a.y) as f:
            for line in f:
                o = json.loads(line)
                pub[int(o["t"])] = o
        common = [k for k, tt in enumerate(t) if int(tt) in pub]
        if common:
            yp = np.array([float(pub[int(t[k])]["y"]) for k in common])
            y0 = np.array([float(pub[int(t[k])].get("y0", pub[int(t[k])]["y"])) for k in common])
            yc = np.round(y_cnt[common], 1)
            out["published"] = {"n": len(common), "of_pairs": int(len(t)),
                                "y0_equals_y_cnt": bool(np.all(np.abs(y0 - yc) <= 0.05 + 1e-9)),
                                "noise_ratio_sd": float(np.std(yp / np.maximum(y0, 1.0) - 1.0)) if len(common) > 1 else 0.0,
                                "noisy_vs_phy": stats(yp - y_phy[common], y_phy[common])}
    if a.csv:
        with open(a.csv, "w") as f:
            f.write("t,a_us,b_us,y_cnt,y_model,y_phy,y_step,Tb_cnt,Tb_gt,To_cnt,To_model,To_phy\n")
            for k in range(len(t)):
                f.write("%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.6f,%.6f,%.6f,%.6f,%.6f\n" % (
                    t[k], (A[k] + org) // 1000, (B[k] + org) // 1000, y_cnt[k], y_model[k], y_phy[k], y_step[k],
                    Tb_cnt[k], Tb_gt[k], To_cnt[k], To_mod[k], To_phy[k]))
    if a.json:
        print(json.dumps(out, indent=1))
        return out
    print("pairs %d (read interval %.0f..%.0f us), fleet exchanges %d (unique MPDUs %d)" % (
        out["pairs"], out["read_interval_us"]["min"], out["read_interval_us"]["max"], n_exch, n_unique))
    print("mean y_phy %.0f B, mean y_cnt %.0f B" % (out["y_phy_mean"], out["y_cnt_mean"]))
    print("%-14s %8s %10s %10s %10s %10s" % ("comparison", "n", "bias_rel", "mean|rel|", "p99|rel|", "max|rel|"))
    for k in ("cnt_vs_model", "model_vs_phy", "cnt_vs_phy", "cnt_vs_step"):
        s = out[k]
        print("%-14s %8d %10.5f %10.5f %10.5f %10.5f" % (k, s["n"], s["bias_rel"], s["mean_abs_rel"],
                                                        s["p99_abs_rel"], s["max_abs_rel"]))
    print("Tb_cnt - Tb_gt: mean %.3f ms, max |.| %.3f ms" % (out["Tb_cnt_minus_gt_ms"]["mean"],
                                                             out["Tb_cnt_minus_gt_ms"]["max_abs"]))
    print("To_cnt - To_model: mean %.3f ms, max |.| %.3f ms; To_model - To_phy: mean %.3f ms" % (
        out["To_cnt_minus_model_ms"]["mean"], out["To_cnt_minus_model_ms"]["max_abs"],
        out["To_model_minus_phy_ms"]["mean"]))
    if "published" in out:
        print("published y(t): %s" % json.dumps(out["published"]))
    return out


if __name__ == "__main__":
    main()
