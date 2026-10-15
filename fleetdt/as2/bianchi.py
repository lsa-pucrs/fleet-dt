"""Bianchi's saturation model: eq. (bianchi) for tau and p_c, eq. (eff) for eta(n).

eq. (bianchi): tau = 2 (1 - 2p) / ((1 - 2p)(CWmin + 2) + p (CWmin + 1)(1 - (2p)^m)),
               p = 1 - (1 - tau)^(n - 1).
The solver uses the equivalent form tau = 2 / (1 + W + p W sum_{i<m} (2p)^i), W = CWmin + 1, which has
no removable singularity at p = 1/2, and a vectorized bisection on tau - tau_B(p(tau)), which is
strictly increasing in tau (tau_B decreases in p, p increases in tau), so the root is unique.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

import numpy as np

from .airtime import Phy


def tau_of_p(p, cwmin: int, m: int):
    p = np.asarray(p, dtype=float)
    W = cwmin + 1.0
    s = np.zeros_like(p)
    term = np.ones_like(p)
    for _ in range(m):
        s = s + term
        term = term * 2 * p
    return 2.0 / (1.0 + W + p * W * s)


def solve(n, cwmin: int, m: int, iters: int = 100) -> Tuple[np.ndarray, np.ndarray]:
    """Fixed point of eq. (bianchi) for each n (array). Returns (tau, p_c)."""
    n = np.atleast_1d(np.asarray(n, dtype=float))
    lo, hi = np.zeros_like(n), np.ones_like(n)
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        p = 1.0 - (1.0 - mid) ** (n - 1)
        g = mid - tau_of_p(p, cwmin, m)
        pos = g > 0
        hi = np.where(pos, mid, hi)
        lo = np.where(pos, lo, mid)
    tau = 0.5 * (lo + hi)
    return tau, 1.0 - (1.0 - tau) ** (n - 1)


def probs(n, tau):
    n = np.asarray(n, dtype=float)
    Ptr = 1.0 - (1.0 - tau) ** n
    Ps = n * tau * (1.0 - tau) ** (n - 1) / Ptr
    return Ptr, Ps


def eta(n, T_s: float, T_c: float, slot: float, cwmin: int, m: int) -> np.ndarray:
    """eq. (eff): fraction of time in which the channel carries successful transmissions."""
    tau, _ = solve(n, cwmin, m)
    Ptr, Ps = probs(n, tau)
    return Ptr * Ps * T_s / ((1 - Ptr) * slot + Ptr * Ps * T_s + Ptr * (1 - Ps) * T_c)


def mean_frame(dtp, R: Optional[float] = None, R0: Optional[float] = None) -> Dict:
    """Mean frame of the data bridges from the DTP: in one step each vessel sends every message of
    every topic (r Ts outputs of n messages of s bytes), one segment per message (two when the
    message exceeds MSS), one TCP acknowledgment per segment from the AP and, for QoS 2, the PUBREC,
    PUBREL, PUBCOMP and their acknowledgments. Returns the frame count, the mean exact T_succ with
    the AIFS of AC_BE of the stations, and the mean collision time (frame + AIFS)."""
    phy = Phy(dtp.phy)
    R = R or phy.rate_init
    R0 = R0 or R
    frames = []    # (ip bytes, rate)
    for tp in dtp.topics:
        msgs = tp.rate * dtp.Ts * tp.n
        size = tp.h_P + tp.s
        nseg = math.ceil(size / dtp.mss)
        last = size - (nseg - 1) * dtp.mss
        for k in range(nseg):
            seg = dtp.mss if k < nseg - 1 else last
            frames.append((dtp.h_T + seg, R, msgs))
        frames.append((dtp.h_T, R0, msgs * nseg))
        if tp.qos == 2:
            frames.append((dtp.h_T + dtp.h_A, R0, 2 * msgs))
            frames.append((dtp.h_T, R0, msgs))
            frames.append((dtp.h_T + dtp.h_A, R, msgs))
            frames.append((dtp.h_T, R, 2 * msgs))
    cnt = sum(w for _, _, w in frames)
    Ts = sum(w * phy.t_succ(s, r) for s, r, w in frames) / cnt
    Tc = sum(w * phy.t_frame_aifs(s, r) for s, r, w in frames) / cnt
    sbar = sum(w * s for s, _, w in frames) / cnt
    return {"count": cnt, "T_s": Ts, "T_c": Tc, "s_mean": sbar, "R": R, "R0": R0}


class EtaTable:
    """eta(n) for n = 1..nmax with the EDCA parameters of AC_BE of the stations and the mean frame."""

    def __init__(self, phy: Phy, T_s: float, T_c: float, nmax: int = 32):
        self.phy, self.T_s, self.T_c = phy, T_s, T_c
        self.nmax = nmax
        n = np.arange(1, nmax + 1)
        self.table = eta(n, T_s, T_c, phy.slot, phy.cwmin_sta, phy.m_b)

    @classmethod
    def from_dtp(cls, dtp, nmax: int = 32, R: Optional[float] = None) -> "EtaTable":
        mf = mean_frame(dtp, R)
        return cls(Phy(dtp.phy), mf["T_s"], mf["T_c"], max(nmax, dtp.M + 1))

    def __call__(self, n: int) -> float:
        n = int(n)
        if n < 1:
            return 0.0
        if n > self.nmax:
            return float(eta(np.array([n]), self.T_s, self.T_c, self.phy.slot, self.phy.cwmin_sta, self.phy.m_b)[0])
        return float(self.table[n - 1])


# ---------------------------------------------------------------------- ns-3 reference (Bianchi11g.m)
def ns3_11g(n, data_rate: float, ack_rate: float, grid: bool = False) -> np.ndarray:
    """Saturation throughput (Mbit/s) of ns-3's reference script Bianchi11g.m (DIFS variant), with the
    exact fixed point (grid=False) or the script's grid search over 1e4 values of tau (grid=True)."""
    n = np.atleast_1d(np.asarray(n, dtype=float))
    CWmin, CWmax = 15, 1023
    L_DATA, L_ACK = 1500 * 8, 14 * 8
    B = 1.0 / (CWmin + 1)
    EP = L_DATA / (1 - B)
    T_SYM, T_LPHY, T_SE = 4e-6, 20e-6, 6e-6
    L_MAC, L_APP = 28 * 8, 6 * 8
    SIFS, DIFS, SLOT = 10e-6, 28e-6, 9e-6
    nd = data_rate * T_SYM
    T_DATA = T_LPHY + T_SYM * math.ceil((16 + L_MAC + L_DATA + L_APP + 6) / nd) + T_SE
    nd = ack_rate * T_SYM
    T_ACK = T_LPHY + T_SYM * math.ceil((16 + L_ACK + 6) / nd) + T_SE
    T_s = T_DATA + SIFS + T_ACK + DIFS
    T_C = T_DATA + DIFS
    T_S = T_s / (1 - B) + SLOT
    m = int(round(math.log2((CWmax + 1) / (CWmin + 1))))
    if grid:
        tau = []
        t1 = np.linspace(0, 1, 10000)
        for nn in n:
            p = 1 - (1 - t1) ** (nn - 1)
            tp = tau_of_p(p, CWmin, m)
            tau.append(tp[np.argmin(np.abs(t1 - tp))])
        tau = np.array(tau)
    else:
        tau, _ = solve(n, CWmin, m)
    Ptr, Ps = probs(n, tau)
    return Ps * Ptr * EP / ((1 - Ptr) * SLOT + Ptr * Ps * T_S + Ptr * (1 - Ps) * T_C) / 1e6


def parse_reference(text: str) -> Dict[float, Dict[int, float]]:
    """Parses ns-3's bianchi_11g_difs.txt: {rate Mbit/s: {n: throughput Mbit/s}}."""
    import re
    out: Dict[float, Dict[int, float]] = {}
    cur = None
    for line in text.splitlines():
        m = re.match(r"\s*//\s*([\d.]+)\s*Mbps", line)
        if m:
            cur = float(m.group(1))
            out[cur] = {}
            continue
        m = re.match(r"\s*\{(\d+),\s*([\d.]+)\}", line)
        if m and cur is not None:
            out[cur][int(m.group(1))] = float(m.group(2))
    return out
