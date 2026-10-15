"""Airtime of IEEE 802.11 ERP-OFDM exchanges (eq. tsucc) and the airtime cost a_i (eq. cost).

Sizes in bytes, rates in bit/s, times in seconds. `s` is the size of the IP packet of a frame.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Sequence, Tuple

OFDM_MANDATORY = (6e6, 12e6, 24e6)          # mandatory ERP-OFDM rates (IEEE 802.11-2020, 18.1 and 18.3)
OFDM_RATES = (6e6, 9e6, 12e6, 18e6, 24e6, 36e6, 48e6, 54e6)
ACK_BYTES = 14                              # ACK frame: FC 2, duration 2, RA 6, FCS 4
SERVICE_TAIL_BITS = 22                      # SERVICE 16 + tail 6


class Phy:
    """ERP-OFDM timing of the DTP (SPEC Section 2, "phy")."""

    def __init__(self, p: Dict):
        self.slot = float(p["slot"])
        self.sifs = float(p["sifs"])
        self.t_pre = float(p["t_pre"])
        self.t_sig = float(p["t_sig"])
        self.t_sym = float(p["t_sym"])
        self.t_se = float(p["t_se"])
        self.h_mac = int(p["h_mac"])
        self.aifsn_sta = int(p["aifsn_sta"])
        self.aifsn_ap = int(p["aifsn_ap"])
        self.cwmin_sta = int(p["cwmin_sta"])
        self.cwmax_sta = int(p["cwmax_sta"])
        self.cwmin_ap = int(p["cwmin_ap"])
        self.m_b = int(p["m_b"])
        self.basic = sorted(float(r) for r in p["basic_rates"])
        self.rate_init = float(p["rate_init"])
        self._oh: Dict[float, float] = {}
        self._ack: Dict[float, float] = {}

    # ------------------------------------------------------------------ PHY
    def ndbps(self, R: float) -> int:
        if not self.is_ofdm(R):
            raise ValueError("%g bit/s is not an ERP-OFDM rate" % R)
        return int(round(R * self.t_sym))

    def is_ofdm(self, R: float) -> bool:
        return any(abs(R - r) < 1.0 for r in OFDM_RATES)

    def ctrl_rate(self, R: float) -> float:
        """Control response rate of a frame at R (IEEE 802.11-2020, 10.6.6.5.2): the highest rate of the
        BSS basic rate set that does not exceed R; if none, the highest mandatory OFDM rate that does not
        exceed R; if none, the lowest basic rate."""
        b = [r for r in self.basic if r <= R + 1e-6]
        if b:
            return b[-1]
        m = [r for r in OFDM_MANDATORY if r <= R + 1e-6]
        return m[-1] if m else self.basic[0]

    def t_ppdu(self, mpdu: int, R: float) -> float:
        """T_PRE + T_SIG + T_SYM ceil((22 + 8 mpdu)/N_DBPS) + T_SE (ceiling form, exact)."""
        nd = self.ndbps(R)
        return self.t_pre + self.t_sig + self.t_sym * math.ceil((SERVICE_TAIL_BITS + 8 * mpdu) / nd) + self.t_se

    def t_ack(self, R: float) -> float:
        a = self._ack.get(R)
        if a is None:
            a = self._ack[R] = self.t_ppdu(ACK_BYTES, self.ctrl_rate(R))
        return a

    def aifs(self, aifsn: Optional[int] = None) -> float:
        return self.sifs + (self.aifsn_sta if aifsn is None else aifsn) * self.slot

    # ------------------------------------------------------------------ eq. (tsucc)
    def t_succ(self, s: float, R: float, aifsn: Optional[int] = None) -> float:
        """Exact airtime of a successful exchange of an IP packet of s bytes at R: the frame, SIFS, the
        ACK at the control response rate, and AIFS (of the stations unless aifsn is given)."""
        return self.t_ppdu(self.h_mac + int(math.ceil(s)), R) + self.sifs + self.t_ack(R) + self.aifs(aifsn)

    def t_frame_aifs(self, s: float, R: float, aifsn: Optional[int] = None) -> float:
        """Frame plus AIFS (the duration of a collision of this frame, eq. eff)."""
        return self.t_ppdu(self.h_mac + int(math.ceil(s)), R) + self.aifs(aifsn)

    def t_oh(self, R: float) -> float:
        """T_oh(R) of eq. (tsucc)."""
        v = self._oh.get(R)
        if v is None:
            v = (self.t_pre + self.t_sig + self.t_sym + (SERVICE_TAIL_BITS + 8 * self.h_mac) / R + self.t_se
                 + self.sifs + self.t_ack(R) + self.aifs())
            self._oh[R] = v
        return v

    def t_succ_bound(self, s: float, R: float) -> float:
        """Right side of eq. (tsucc): T_oh(R) + 8 s / R."""
        return self.t_oh(R) + 8.0 * s / R


class Cost:
    """Airtime cost a_i(x, n, q) of eq. (cost) with per-station rates R_i (vessel) and R_0i (AP)."""

    def __init__(self, phy: Phy, mss: int, h_T: int, h_A: int):
        self.phy, self.mss, self.h_T, self.h_A = phy, int(mss), int(h_T), int(h_A)

    @classmethod
    def from_dtp(cls, dtp) -> "Cost":
        return cls(Phy(dtp.phy), dtp.mss, dtp.h_T, dtp.h_A)

    def n_seg(self, x: float, n: float, h_P: float, s: Optional[float] = None) -> float:
        """N_seg(x, n) = min{n + (x + n h_P)/MSS, n ceil((s + h_P)/MSS)} (Section V-C), with s the largest
        content of one message of the topic (s_l); without s, the first term alone."""
        v = n + (x + n * h_P) / self.mss
        if s is not None:
            v = min(v, n * math.ceil((s + h_P) / self.mss))
        return v

    def per_seg(self, Ri: float, R0: float) -> float:
        p = self.phy
        return p.t_oh(Ri) + 8.0 * self.h_T / Ri + p.t_oh(R0) + 8.0 * self.h_T / R0

    def ctl(self, Ri: float, R0: float) -> float:
        """QoS 2 term of eq. (cost) per message: PUBREC and PUBCOMP of the GS, PUBREL of the vessel and
        the TCP acknowledgment of each."""
        p, hT, hA = self.phy, self.h_T, self.h_A
        return 3 * p.t_oh(R0) + 3 * p.t_oh(Ri) + 8.0 * (3 * hT + 2 * hA) / R0 + 8.0 * (3 * hT + hA) / Ri

    def a(self, x: float, n: float, q: int, Ri: float, R0: float, h_P: float, s: Optional[float] = None) -> float:
        """a_i(x, n, q) of eq. (cost) for n messages with x bytes of content and the PUBLISH header h_P, each
        message with at most s bytes of content (s_l of the topic; None: the first term of N_seg alone)."""
        if n <= 0 and x <= 0:
            return 0.0
        v = self.n_seg(x, n, h_P, s) * self.per_seg(Ri, R0) + 8.0 * (x + n * h_P) / Ri
        if q == 2:
            v += n * self.ctl(Ri, R0)
        return v

    def held(self, B: Dict, Ri: float, R0: float, n_pubrel: int = 0, s_max: Optional[float] = None) -> float:
        """c^B_i: a_i applied to the held bytes with the QoS of each part, plus the QoS 2 control packets
        of n_pubrel messages whose PUBREL has not reached the GS broker. B holds socket bytes (MQTT
        headers included), so h_P = 0 here, and s_max bounds one message with its header (the largest
        s_l + h_P of the topics)."""
        x0, n0 = float(B.get("x0", 0)), float(B.get("n0", 0))
        x2, n2 = float(B.get("x2", 0)), float(B.get("n2", 0))
        if x0 > 0 and n0 <= 0:
            n0 = 1.0      # bytes without a whole message: at least one segment
        if x2 > 0 and n2 <= 0:
            n2 = 1.0
        return (self.a(x0, n0, 0, Ri, R0, 0, s_max) + self.a(x2, n2, 2, Ri, R0, 0, s_max)
                + max(0, int(n_pubrel)) * self.ctl(Ri, R0))


# ---------------------------------------------------------------------- frame-by-frame reference
def segments(sizes: Sequence[int], mss: int, cuts: Optional[Sequence[bool]] = None) -> list:
    """TCP segmentation of a byte stream made of writes of the given sizes. A segment ends when it
    reaches MSS or, where cuts[j] is True, at the end of write j (TCP without Nagle sends what it has).
    cuts=None cuts at the end of every write (an idle connection)."""
    out, cur = [], 0
    for j, sz in enumerate(sizes):
        rem = sz
        while rem > 0:
            take = min(rem, mss - cur)
            cur += take
            rem -= take
            if cur == mss:
                out.append(cur)
                cur = 0
        if cur > 0 and (cuts is None or cuts[j]):
            out.append(cur)
            cur = 0
    if cur > 0:
        out.append(cur)
    return out


def exact_airtime(phy: Phy, contents: Sequence[int], q: int, h_P: int, mss: int, h_T: int, h_A: int,
                  Ri: float, R0: float, cuts=None, acks_per_seg: float = 1.0) -> float:
    """Sum of the exact T_succ over the frames of n messages (content sizes `contents`): the data
    segments (vessel, AIFS of the stations), one TCP acknowledgment per segment (AP, AIFS of the AP),
    and for QoS 2 the PUBREC and PUBCOMP of the GS, the PUBREL of the vessel and their acknowledgments."""
    segs = segments([c + h_P for c in contents], mss, cuts)
    v = sum(phy.t_succ(h_T + sg, Ri) for sg in segs)
    v += math.ceil(acks_per_seg * len(segs)) * phy.t_succ(h_T, R0, phy.aifsn_ap)
    if q == 2:
        n = len(contents)
        v += n * (2 * phy.t_succ(h_T + h_A, R0, phy.aifsn_ap) + phy.t_succ(h_T, R0, phy.aifsn_ap))
        v += n * (phy.t_succ(h_T + h_A, Ri) + 2 * phy.t_succ(h_T, Ri))
    return v
