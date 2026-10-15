"""Monitor of the AP (SPEC Section 4.3): b(t) (Def. busy), e(t) and kappa(t) (Def. capacity), n(t),
R_i(t) and R_0i(t) (Section V-B).

Times in integer microseconds on the GS clock. Frame record (columns of `fr`):
  0 start_us, 1 end_us (includes the MAC ACK), 2 sta (vessel, 0 = AP broadcast, -1 = outside the fleet),
  3 dir (0 uplink, 1 downlink), 4 ip_len, 5 rate_kbps, 6 ok, 7 kind (0 data, 1 beacon, 2 mgmt/ctl),
  8 ac (optional; access category of the frame, 0 = AC_BE, -1 = unknown).
A frame of a data bridge is a data frame (kind 0, ip_len > 0) of a vessel (sta >= 1) of AC_BE; when the
record carries no access category, every data frame of a vessel counts (the control bridge sends in W(t)
only the goals and the keepalives).
"""
from __future__ import annotations

import threading
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np

from .airtime import Phy

NCOL = 9
S, E, STA, DIR, IPL, RATE, OK, KIND, AC = range(NCOL)


# ---------------------------------------------------------------------- interval arithmetic
def union(iv: np.ndarray) -> np.ndarray:
    """Sorted disjoint union of half-open intervals (k x 2)."""
    if iv.size == 0:
        return iv.reshape(0, 2)
    iv = iv[iv[:, 1] > iv[:, 0]]
    if iv.size == 0:
        return iv.reshape(0, 2)
    iv = iv[np.argsort(iv[:, 0], kind="stable")]
    starts, ends = iv[:, 0], np.maximum.accumulate(iv[:, 1])
    # a new block starts where the start exceeds the running maximum of the previous ends
    new = np.ones(len(iv), dtype=bool)
    new[1:] = starts[1:] > ends[:-1]
    idx = np.flatnonzero(new)
    out_s = starts[idx]
    out_e = np.append(ends[idx[1:] - 1], ends[-1])
    return np.stack([out_s, out_e], axis=1)


def clip(iv: np.ndarray, a: int, b: int) -> np.ndarray:
    if iv.size == 0:
        return iv.reshape(0, 2)
    c = np.stack([np.maximum(iv[:, 0], a), np.minimum(iv[:, 1], b)], axis=1)
    return c[c[:, 1] > c[:, 0]]


def measure(iv: np.ndarray) -> int:
    return int((iv[:, 1] - iv[:, 0]).sum()) if iv.size else 0


def intersect(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Intersection of two sorted disjoint unions."""
    out = []
    i = j = 0
    while i < len(u) and j < len(v):
        a = max(u[i, 0], v[j, 0])
        b = min(u[i, 1], v[j, 1])
        if b > a:
            out.append((a, b))
        if u[i, 1] < v[j, 1]:
            i += 1
        else:
            j += 1
    return np.array(out, dtype=np.int64).reshape(-1, 2)


# ---------------------------------------------------------------------- store
class MonitorStore:
    """Holds the batches of fleet/monitor and answers the queries of the AS. Thread-safe."""

    def __init__(self, phy: Phy, keep_us: int = 1_000_000):
        self.phy = phy
        self.keep_us = keep_us
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.batches: List[Tuple[int, int, np.ndarray, np.ndarray]] = []   # (from, to, busy, fr)
        self.last_to = None
        self.last_rate: Dict[int, List[Optional[float]]] = {}             # i -> [R_i, R_0i] in bit/s
        self.rate_seen_us: Dict[int, List[int]] = {}
        self.has_ac = False

    def add(self, batch: Dict) -> None:
        f, t = int(batch["from_us"]), int(batch["to_us"])
        busy = np.array(batch.get("busy") or [], dtype=np.int64).reshape(-1, 2)
        raw = batch.get("fr") or []
        fr = np.full((len(raw), NCOL), -1, dtype=np.int64)
        for k, r in enumerate(raw):
            n = min(len(r), NCOL)
            if n < 8:
                raise ValueError("frame record with %d fields" % len(r))
            fr[k, :n] = [int(round(v)) for v in r[:n]]
        with self.cond:
            if fr.size and (fr[:, AC] >= 0).any():
                self.has_ac = True
            self.batches.append((f, t, busy, fr))
            self.last_to = t if self.last_to is None else max(self.last_to, t)
            self._update_rates(fr)
            lim = t - self.keep_us
            if self.batches and self.batches[0][1] < lim:
                self.batches = [b for b in self.batches if b[1] >= lim]
            self.cond.notify_all()

    def wait_until(self, to_us: int, timeout: float) -> bool:
        """Waits up to `timeout` seconds for a batch that reaches to_us."""
        with self.cond:
            return self.cond.wait_for(lambda: self.last_to is not None and self.last_to >= to_us, timeout)

    def _update_rates(self, fr: np.ndarray) -> None:
        if fr.size == 0:
            return
        m = self._bridge_mask(fr) & (fr[:, OK] == 1)
        for row in fr[m][np.argsort(fr[m][:, S], kind="stable")]:
            i, d = int(row[STA]), int(row[DIR])
            seen = self.rate_seen_us.setdefault(i, [-1, -1])
            if row[S] >= seen[d]:
                seen[d] = int(row[S])
                self.last_rate.setdefault(i, [None, None])[d] = float(row[RATE]) * 1e3

    def _bridge_mask(self, fr: np.ndarray) -> np.ndarray:
        m = (fr[:, KIND] == 0) & (fr[:, STA] >= 1) & (fr[:, IPL] > 0)
        if self.has_ac:
            m &= (fr[:, AC] == 0) | (fr[:, AC] < 0)
        return m

    def _select(self, a: int, b: int, margin: int = 20000):
        """Batches that overlap [a, b), plus `margin` us on both sides: a record joins the batch of the
        end of its exchange, so its start can precede the batch."""
        with self.lock:
            sel = [x for x in self.batches if x[1] > a - margin and x[0] < b + margin]
        cov = union(np.array([(x[0], x[1]) for x in sel], dtype=np.int64).reshape(-1, 2))
        busy = np.concatenate([x[2] for x in sel]) if sel else np.zeros((0, 2), dtype=np.int64)
        fr = np.concatenate([x[3] for x in sel]) if sel else np.zeros((0, NCOL), dtype=np.int64)
        return clip(cov, a, b), busy, fr

    # ------------------------------------------------------------------ Def. busy
    def busy_fraction(self, a: int, b: int, min_cover: float = 0.5) -> Dict:
        """b(t) = (T_busy(Q) - T_own(Q)) / |Q| over the part of Q = [a, b) that the batches cover.
        T_own: the time in which the channel is busy with a successful exchange of the AP with a vessel
        (frame and MAC acknowledgment) or with a beacon of the AP."""
        cov, busy, fr = self._select(a, b)
        c = measure(cov)
        out = {"cover": c / (b - a) if b > a else 0.0, "b": None}
        if b <= a or c < min_cover * (b - a):
            return out
        bu = intersect(union(clip(busy, a, b)), cov)
        own_m = ((fr[:, STA] >= 1) & (fr[:, OK] == 1)) | (fr[:, KIND] == 1) if fr.size else np.zeros(0, bool)
        own = union(clip(fr[own_m][:, [S, E]], a, b)) if fr.size else np.zeros((0, 2), np.int64)
        tb = measure(bu)
        to = measure(intersect(bu, own))
        out.update({"b": (tb - to) / c, "T_busy": tb, "T_own": to})
        return out

    # ------------------------------------------------------------------ Def. capacity
    def capacity(self, w: int, wend: int, step_end: int, held: Iterable[int] = (), min_cover: float = 0.95) -> Dict:
        """e(t), kappa(t) and n(t) of W(t) = [w, wend). `held`: vessels that report B_i(t+1) > 0.
        A data bridge holds bytes at the end of W(t) when an uplink frame of the data bridge starts in
        [wend, step_end) or its vessel is in `held`."""
        cov, _, fr = self._select(w, step_end)
        phy = self.phy
        out = {"e": None, "kappa": None, "n": 0, "T_fleet": 0.0, "hold": [], "cover": measure(cov) / (step_end - w)}
        if out["cover"] < min_cover:      # frames of W(t) missing: no e(t), no score (Table exchanges)
            return out
        bm = self._bridge_mask(fr) if fr.size else np.zeros(0, bool)
        after = fr[bm & (fr[:, DIR] == 0) & (fr[:, S] >= wend) & (fr[:, S] < step_end)] if fr.size else fr
        hold = set(int(i) for i in after[:, STA]) if after.size else set()
        hold |= set(int(i) for i in held)
        ok = fr[bm & (fr[:, OK] == 1)] if fr.size else fr
        inw = ok[(ok[:, S] >= w) & (ok[:, S] < wend)] if ok.size else ok
        if hold:
            e = wend
        elif inw.size:
            e = int(min(int(inw[:, E].max()), wend))
        else:
            e = w
        up = fr[bm & (fr[:, DIR] == 0) & (fr[:, S] >= w) & (fr[:, S] < e)] if fr.size else fr
        out["n"] = len(set(int(i) for i in up[:, STA])) if up.size else 0
        out["hold"] = sorted(hold)
        out["e"] = e
        if e <= w:
            return out
        sel = ok[(ok[:, E] > w) & (ok[:, S] < e)] if ok.size else ok
        tf = 0.0
        if sel.size:
            s0, s1 = sel[:, S], sel[:, E]
            frac = (np.minimum(s1, e) - np.maximum(s0, w)) / np.maximum(1, s1 - s0)
            tf = float((np.clip(frac, 0.0, 1.0) * self.t_succ_rows(sel)).sum())
        out["T_fleet"] = tf
        out["kappa"] = tf / ((e - w) * 1e-6)
        return out

    def t_succ_rows(self, rows: np.ndarray) -> np.ndarray:
        """Vectorized t_succ_record over the rows of a frame array."""
        out = np.empty(len(rows))
        keys = rows[:, RATE] * 2 + (rows[:, DIR] == 1)
        for key in np.unique(keys):
            m = keys == key
            sub = rows[m]
            R = float(sub[0, RATE]) * 1e3
            phy = self.phy
            aifsn = phy.aifsn_ap if int(sub[0, DIR]) == 1 else phy.aifsn_sta
            if phy.is_ofdm(R):
                nd = phy.ndbps(R)
                bits = 22 + 8 * (phy.h_mac + sub[:, IPL])
                out[m] = (phy.t_pre + phy.t_sig + phy.t_se + phy.t_sym * np.ceil(bits / nd)
                          + phy.sifs + phy.t_ack(R) + phy.aifs(aifsn))
            else:
                out[m] = (sub[:, E] - sub[:, S]) * 1e-6 + phy.aifs(aifsn)
        return out

    def t_succ_record(self, row) -> float:
        """T_succ (s) of a recorded successful exchange, exact, with the AIFS of the sender."""
        phy = self.phy
        R = float(row[RATE]) * 1e3
        aifsn = phy.aifsn_ap if int(row[DIR]) == 1 else phy.aifsn_sta
        if phy.is_ofdm(R):
            return phy.t_succ(int(row[IPL]), R, aifsn)
        return (int(row[E]) - int(row[S])) * 1e-6 + phy.aifs(aifsn)

    # ------------------------------------------------------------------ Section V-B
    def rates(self, i: int, default: float) -> Tuple[float, float]:
        """R_i(t), R_0i(t): rates of the last successful uplink and downlink data-bridge frames of i."""
        with self.lock:
            r = self.last_rate.get(i, [None, None])
        return (r[0] or default, r[1] or default)
