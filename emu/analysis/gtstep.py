#!/usr/bin/env python3
"""Ground truth of a Fleet-DT run per step (paper Section VI-D): kappa(t), e(t), b(t), the work X(t) of
Lemma 2, and the steps with interference or emulator lag, from RUN/emu/gt.bin (emu/README.md).

  python3 emu/analysis/gtstep.py RUN [--out RUN/gt_steps.jsonl]

Definitions (the same as the AS, fleetdt/as2/monitor.py, so that H2 compares like with like):
* frames of the data bridges: IPv4/TCP frames with the port of a data bridge (1884; mode b2 also 1885 and
  1886) between a vessel and the AP; a successful exchange is a frame and the ACK of its receiver that
  starts SIFS after it, for a frame of the AP only when the AP receives the ACK (what the monitor of the
  AP records); its extent is [frame start, ACK end] and its airtime
  T_succ = frame + SIFS + ACK + AIFS of the AC_BE of the sender (the PHY timing of the emulator from
  RUN/emu/summary.json; with ERP-OFDM the frame and the ACK durations hold the signal extension);
* W(t) = [w, wend) with w = start(t) + T1 + T2 + eps and wend = end(t) - eps - d_rx (host clock, which the
  GS follows through ptp_kvm);
* a data bridge holds bytes at the end of W(t) when the AP records (receives, also with an FCS error) a
  data-bridge frame of a vessel that starts in [wend, end(t)) or the vessel reports B_i(t+1) > 0 (demand
  of step t+1 in the DTA); then e(t) = wend,
  otherwise e(t) = the end of the last successful exchange that starts in W(t) (at most wend), or w;
* kappa(t) = T_fleet([w, e)) / (e - w), an exchange that crosses a boundary counted in proportion;
* b(t) = (T_busy(Q) - T_own(Q)) / |Q| over Q(t) = [end of W(t-1), start(t) + T1): T_busy from the PHY
  states of the AP (CCA busy, TX, RX), T_own the successful frames of the AP with the vessels with their
  ACKs and the group-addressed frames of the AP and of the vessels (beacons, CF-End);
* X(t): the sum of T_succ over the successful data-bridge exchanges that start in [w(t), v(t+1) - eps),
  the instant before which no vessel releases a message of step t + 1 (v(t+1) = start(t+1) + T1 + T2);
  "empty" when no data-bridge frame starts in [wend, v(t+1) - eps), no vessel reports held bytes, and no
  data-bridge frame that the MAC dropped (its receiver never received it) waits at v(t+1) - eps for its
  TCP retransmission (pending_losses; the retransmission timer can back off past v(t+1) without any
  frame in between), so that X(t) is the work of Lemma 2; otherwise X(t) is a lower bound;
* interference: a data frame of the external BSS or a jammer burst in [end of W(t-1), end(t)) (the
  beacons of the external AP alone do not count);
* lag: the largest real-time lag of the emulator (lag.txt, every 10 ms) in [start(t) - Ts, end(t)).
"""
import argparse
import io
import json
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import gt as gtmod  # noqa: E402

DEFAULT_EPOCH = 1767225600.0


def open_maybe_zst(path):
    """bytes of path, or of path + '.zst' (zstandard module, else the zstd program)"""
    if os.path.exists(path):
        with open(path, "rb") as f:
            return f.read()
    z = path + ".zst"
    try:
        import zstandard
        with open(z, "rb") as f:
            return zstandard.ZstdDecompressor().stream_reader(f).read()
    except ImportError:
        return subprocess.run(["zstd", "-dc", z], check=True, stdout=subprocess.PIPE).stdout


def load_gt(path):
    data = open_maybe_zst(path)
    b = io.BytesIO(data)
    magic = b.read(8)
    if magic[:6] != b"FDTGT1":
        raise SystemExit("%s: not a gt.bin" % path)
    n = int.from_bytes(b.read(4), "little")
    hdr = json.loads(b.read(n))
    rest = data[12 + n:]
    k = len(rest) // gtmod.DT.itemsize
    return hdr, np.frombuffer(rest[:k * gtmod.DT.itemsize], dtype=gtmod.DT)


def load_jsonl(path):
    out = []
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        pass
    except OSError:
        pass
    return out


def merge_np(s, e):
    """sorted disjoint union of [s, e) intervals"""
    if len(s) == 0:
        return np.zeros(0, np.int64), np.zeros(0, np.int64)
    o = np.argsort(s, kind="stable")
    s, e = s[o], e[o]
    em = np.maximum.accumulate(e)
    new = np.ones(len(s), bool)
    new[1:] = s[1:] > em[:-1]
    idx = np.flatnonzero(new)
    return s[idx], np.append(em[idx[1:] - 1], em[-1])


def intersect_np(a, b):
    out_s, out_e = [], []
    i = j = 0
    As, Ae = a
    Bs, Be = b
    while i < len(As) and j < len(Bs):
        lo, hi = max(As[i], Bs[j]), min(Ae[i], Be[j])
        if hi > lo:
            out_s.append(lo)
            out_e.append(hi)
        if Ae[i] < Be[j]:
            i += 1
        else:
            j += 1
    return np.array(out_s, np.int64), np.array(out_e, np.int64)


class Measure:
    """measure of a sorted disjoint union inside [a, b), by prefix sums"""

    def __init__(self, iv):
        self.s, self.e = iv
        self.cum = np.concatenate([[0], np.cumsum(self.e - self.s)])

    def __call__(self, a, b):
        if len(self.s) == 0 or b <= a:
            return 0
        i = np.searchsorted(self.e, a, side="right")     # first interval that ends after a
        j = np.searchsorted(self.s, b, side="left")      # intervals starting before b
        if j <= i:
            return 0
        tot = self.cum[j] - self.cum[i]
        tot -= max(0, a - self.s[i])
        tot -= max(0, self.e[j - 1] - b)
        return int(tot)


def phy_of(summary, M):
    conf = summary.get("conf", {})
    gs = conf.get("gs", {})
    sifs = int(gs.get("sifs_us", 10)) * 1000
    slot = int(gs.get("slot_us", 9)) * 1000
    aifsn = np.zeros(64, np.int64)
    aifsn[0] = int(gs.get("be_aifsn", 1))
    for i in range(1, M + 1):
        aifsn[i] = int(conf.get("v%d" % i, {}).get("be_aifsn", 3))
    return sifs, slot, aifsn


def pending_losses(r, frames, bridge, origin):
    """Intervals [loss, clear) in which a data bridge holds bytes that the MAC dropped: a data-bridge frame
    (packet uid) that its receiver never receives (all retries failed) leaves its bytes in the TCP send
    queue of the bridge until TCP retransmits them, possibly after a backoff of the retransmission timer
    without any frame in the meantime. The loss is the start of the last attempt; the clear is the first
    later frame of the same TCP flow and sender that its receiver receives and that is at least as long
    (the retransmission, possibly collapsed with new data), or never. Conservative without the TCP
    sequence numbers: a lost pure ACK opens an interval too."""
    f = frames[bridge]
    none = np.zeros(0, np.int64)
    if not len(f):
        return none, none
    rx = r[r["ev"] == gtmod.EV["RX_OK"]]
    rkey = (rx["node"].astype(np.int64) << 32) | rx["uid"].astype(np.int64)
    fkey = (f["dst"].astype(np.int64) << 32) | f["uid"].astype(np.int64)
    got = np.isin(fkey, rkey)
    t = f["t_ns"].astype(np.int64) + origin
    flow = (f["node"].astype(np.int64) << 32) | (f["sport"].astype(np.int64) << 16) | f["dport"].astype(np.int64)
    ipl = f["ip_len"].astype(np.int64)
    lost_uid = np.unique(f["uid"][~got])
    if not len(lost_uid):
        return none, none
    ok_by_flow = {}
    for fl in np.unique(flow[np.isin(f["uid"], lost_uid)]):
        m = (flow == fl) & got
        o = np.argsort(t[m], kind="stable")
        ok_by_flow[int(fl)] = (t[m][o], ipl[m][o])
    S, E = [], []
    big = np.iinfo(np.int64).max
    li = np.nonzero(np.isin(f["uid"], lost_uid))[0]
    li = li[np.lexsort((t[li], f["uid"][li]))]                 # by uid, then time
    last = li[np.r_[f["uid"][li][1:] != f["uid"][li][:-1], True]]   # the last attempt of each lost uid
    for k in last:
        ts, ls = ok_by_flow[int(flow[k])]
        j = np.searchsorted(ts, t[k], side="right")
        hit = np.nonzero(ls[j:] >= ipl[k])[0]
        S.append(int(t[k]))
        E.append(int(ts[j + hit[0]]) if len(hit) else big)
    return np.array(S, np.int64), np.array(E, np.int64)


def analyze_run(rd, ports=None):
    with open(os.path.join(rd, "result.json")) as f:
        res = json.load(f)
    with open(os.path.join(rd, "dtp.json")) as f:
        dtp = json.load(f)
    with open(os.path.join(rd, "experiment.json")) as f:
        spec = json.load(f)
    with open(os.path.join(rd, "emu", "summary.json")) as f:
        summ = json.load(f)
    M = int(res["M"])
    if ports is None:
        ports = [1884, 1885, 1886] if spec.get("method") == "b2" else [1884]
    hdr, r = load_gt(os.path.join(rd, "emu", "gt.bin"))
    origin = int(hdr["origin_ns"])
    names = [n["name"] for n in hdr["nodes"]]
    sifs, slot, aifsn = phy_of(summ, M)
    # successful exchanges (frame + ACK of its receiver)
    frames, acks, match = gtmod.exchanges(r, sifs_ns=sifs)
    ok = match >= 0
    # what the monitor of the AP sees: a frame of the AP succeeds when the AP receives its ACK; a frame of a
    # vessel is recorded when the AP receives it (also with an FCS error)
    rx_ap = r[(r["node"] == 0) & ((r["ev"] == gtmod.EV["RX_OK"]) | (r["ev"] == gtmod.EV["RX_ERR"]))]
    ap_ok_uids = set(rx_ap["uid"][rx_ap["ev"] == gtmod.EV["RX_OK"]].tolist())
    is_ap = frames["node"] == 0
    ack_uid = np.where(ok, acks["uid"][np.maximum(match, 0)], 0)
    ok = ok & (~is_ap | np.isin(ack_uid, list(ap_ok_uids)))
    fs = frames["t_ns"].astype(np.int64) + origin
    fdur = frames["dur_ns"].astype(np.int64)
    adur = np.zeros(len(frames), np.int64)
    adur[ok] = acks["dur_ns"][match[ok]].astype(np.int64)
    aend = np.where(ok, acks["t_ns"][np.maximum(match, 0)].astype(np.int64) + origin + adur, fs + fdur)
    node = frames["node"].astype(np.int64)
    dst = frames["dst"].astype(np.int64)
    fleet_pair = (((node >= 1) & (node <= M) & (dst == 0)) | ((node == 0) & (dst >= 1) & (dst <= M)))
    bridge = fleet_pair & (frames["kind"] == gtmod.KIND["data"]) & (frames["ip_proto"] == 6) & \
        (np.isin(frames["sport"], ports) | np.isin(frames["dport"], ports))
    tsucc = fdur + sifs + adur + sifs + aifsn[np.clip(node, 0, 63)] * slot
    sel = bridge & ok
    o = np.argsort(fs[sel], kind="stable")
    XS, XE, XT = fs[sel][o], aend[sel][o], tsucc[sel][o]
    XN = node[sel][o]
    XR = frames["rate_kbps"][sel][o].astype(np.int64)
    # every data-bridge frame (any outcome) for "empty"; the uplink data-bridge frames that the AP records
    # (received, also with an FCS error) for the held test of Def. capacity
    tx_all = fs[bridge]
    tx_all.sort()
    rb = rx_ap[(rx_ap["kind"] == gtmod.KIND["data"]) & (rx_ap["src"] >= 1) & (rx_ap["src"] <= M) &
               (rx_ap["dst"] == 0) & (rx_ap["ip_proto"] == 6) &
               (np.isin(rx_ap["sport"], ports) | np.isin(rx_ap["dport"], ports))]
    up_all = np.sort(rb["t_ns"].astype(np.int64) + origin)
    pend_s, pend_e = pending_losses(r, frames, bridge, origin)
    # AP busy (PHY states CCA busy, TX, RX) and own airtime
    stt = r[(r["ev"] == gtmod.EV["STATE"]) & (r["node"] == 0) & (r["reason"] >= 1) & (r["reason"] <= 3)]
    busy = merge_np(stt["t_ns"].astype(np.int64) + origin, stt["t_ns"].astype(np.int64) + origin +
                    stt["dur_ns"].astype(np.int64))
    own_m = fleet_pair & ok
    os_ = np.concatenate([fs[own_m], acks["t_ns"][match[own_m]].astype(np.int64) + origin])
    oe_ = np.concatenate([fs[own_m] + fdur[own_m], acks["t_ns"][match[own_m]].astype(np.int64) + origin +
                          adur[own_m]])
    # group-addressed frames of the AP and of the vessels (beacons, CF-End, broadcast)
    tx = r[r["ev"] == gtmod.EV["TX"]]
    grp = tx[(tx["node"] <= M) & (tx["dst"] == -2) & (tx["kind"] != gtmod.KIND["ack"]) &
             ((tx["node"] == 0) | np.isin(tx["uid"], list(ap_ok_uids)))]
    os_ = np.concatenate([os_, grp["t_ns"].astype(np.int64) + origin])
    oe_ = np.concatenate([oe_, grp["t_ns"].astype(np.int64) + origin + grp["dur_ns"].astype(np.int64)])
    own = merge_np(os_, oe_)
    m_busy = Measure(busy)
    m_own_busy = Measure(intersect_np(busy, own))
    # interference: frames of the nodes outside the fleet (external BSS) and jammer bursts
    ext_nodes = [k for k, n in enumerate(names) if n in ("ap2", "sta2", "jam")]
    ext = r[np.isin(r["node"], ext_nodes) & (((r["ev"] == gtmod.EV["TX"]) & (r["kind"] == gtmod.KIND["data"])) |
                                              (r["ev"] == gtmod.EV["JAM"]))]
    ext_t = np.sort(ext["t_ns"].astype(np.int64) + origin)
    # lag of the real-time scheduler
    lag_t, lag_v = [], []
    try:
        with open(os.path.join(rd, "emu", "lag.txt")) as f:
            for line in f:
                a, b = line.split()
                lag_t.append(origin + int(a) * 1000)
                lag_v.append(float(b))
    except OSError:
        pass
    lag_t, lag_v = np.array(lag_t, np.int64), np.array(lag_v)
    # held bytes reported in step t+1 (demand B of the DTA, AS record of step t+1)
    dta = [x for x in load_jsonl(os.path.join(rd, "vms", "gs", "dta.jsonl")) if x.get("type") == "as"]
    held_next = {}
    for x in dta:
        hb = []
        for si, B in (x.get("Bbar") or {}).items():
            if int(si) in (x.get("M") or []) and (B.get("x0", 0) + B.get("x2", 0) > 0 or B.get("n0", 0) + B.get("n2", 0) > 0):
                hb.append(int(si))
        held_next[int(x["t"]) - 1] = hb
    epoch = dtp.get("epoch")
    epoch_ns = int(round((DEFAULT_EPOCH if epoch is None else float(epoch)) * 1e9))
    Ts = int(round(float(dtp["Ts"]) * 1e9))
    T1, T2 = int(round(dtp["T1"] * 1e9)), int(round(dtp["T2"] * 1e9))
    eps, drx = int(round(dtp["eps"] * 1e9)), int(round(dtp["d_rx"] * 1e9))
    S0, N = int(res["S0"]), int(res["steps"])
    out = []

    def start(t):
        return epoch_ns + (t - 1) * Ts

    for t in range(S0, S0 + N):
        st, en = start(t), start(t + 1)
        w, wend = st + T1 + T2 + eps, en - eps - drx
        vnext = start(t + 1) + T1 + T2 - eps
        q0, q1 = st - eps - drx, st + T1
        # held at the end of W(t)
        a = np.searchsorted(up_all, wend)
        b = np.searchsorted(up_all, en)
        frame_after = b > a
        rep = held_next.get(t, [])
        hold = bool(frame_after or rep)
        lo = np.searchsorted(XS, w - 5_000_000)
        hi = np.searchsorted(XS, wend + 5_000_000)
        S_, E_, T_ = XS[lo:hi], XE[lo:hi], XT[lo:hi]
        inw = (S_ >= w) & (S_ < wend)
        if hold:
            e = wend
        elif inw.any():
            e = int(min(E_[inw].max(), wend))
        else:
            e = w
        kappa = Tf = None
        n_up = 0
        wm = (S_ >= w) & (S_ < wend)
        RR, NN = XR[lo:hi][wm], XN[lo:hi][wm]
        r_up = float(RR[NN >= 1].mean()) if np.any(NN >= 1) else None
        r_dn = float(RR[NN == 0].mean()) if np.any(NN == 0) else None
        if e > w:
            m = (E_ > w) & (S_ < e)
            frac = (np.minimum(E_[m], e) - np.maximum(S_[m], w)) / np.maximum(1, E_[m] - S_[m])
            Tf = float((np.clip(frac, 0, 1) * T_[m]).sum())
            kappa = Tf / (e - w)
            n_up = int(len(set(XN[lo:hi][m & (XN[lo:hi] >= 1)].tolist())))
        # X(t)
        xl = np.searchsorted(XS, w)
        xh = np.searchsorted(XS, vnext)
        X = float(XT[xl:xh].sum()) / 1e9
        busy_after = np.searchsorted(tx_all, vnext) > np.searchsorted(tx_all, wend)
        lost_open = bool(np.any((pend_s < vnext) & (pend_e >= vnext)))
        empty = not busy_after and not rep and not lost_open
        # b(t)
        tb = m_busy(q0, q1)
        to = m_own_busy(q0, q1)
        bq = (tb - to) / (q1 - q0)
        # interference and lag
        ia, ib = np.searchsorted(ext_t, q0), np.searchsorted(ext_t, en)
        la, lb = np.searchsorted(lag_t, st - Ts), np.searchsorted(lag_t, en)
        lagmax = float(lag_v[la:lb].max()) if lb > la else None
        out.append({"t": t, "w": w, "wend": wend, "e": e, "hold": hold, "hold_frame": bool(frame_after),
                    "hold_rep": rep, "kappa": kappa, "T_fleet": (Tf / 1e9 if Tf is not None else None), "n": n_up,
                    "X": X, "empty": bool(empty), "lost_open": lost_open, "b": bq, "T_busy": tb / 1e9, "T_own": to / 1e9,
                    "interf": bool(ib > ia), "lag_max_us": lagmax, "R_up_kbps": r_up, "R_down_kbps": r_dn})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    rows = analyze_run(a.run)
    out = a.out or os.path.join(a.run, "gt_steps.jsonl")
    with open(out, "w") as f:
        for x in rows:
            f.write(json.dumps(x, separators=(",", ":")) + "\n")
    k = [x["kappa"] for x in rows if x["kappa"] is not None]
    print("%s: %d steps, kappa mean %.4f, b mean %.4f, held %d, empty %d" % (
        out, len(rows), float(np.mean(k)) if k else float("nan"), float(np.mean([x["b"] for x in rows])),
        sum(x["hold"] for x in rows), sum(x["empty"] for x in rows)))


if __name__ == "__main__":
    main()
