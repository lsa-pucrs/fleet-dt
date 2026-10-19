"""Computations of the AS that the VE reproduces when the AS does not publish them:
exponential smoothing (eq. 2, C_hat(t) = (1 - alpha) C_hat(t-1) + alpha c_y y(t-1)), predicted capacity of phase 3
K_hat(t) = C_hat(t) (T3 - T_m) / Ts, grant limit (eq. 3, b(t) = max(0, U K_hat(t) - Q(t)) / m(t)), Algorithm 1
(admission control), the per-vessel grants of baseline B7, and the cost bound of the channel bytes of the
forwarded messages."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# epoch of a DTP file with "epoch": null (2026-01-01T00:00:00Z); the C programs use the same value
DEFAULT_EPOCH = 1767225600.0
# bytes of the step header t:u32, s:u16 that the vessel regulator writes in front of each forwarded message
STEP_HEADER_BYTES = 6


def _varint_bytes(v: int) -> int:
    return 1 if v < 128 else 2 if v < 16384 else 3 if v < 2097152 else 4


def publish_len(topic_len: int, qos: int, payload_len: int) -> int:
    """Bytes of an MQTT 3.1.1 PUBLISH packet (fdt_p_publish_len of common/fdt_paper.c)."""
    rem = 2 + topic_len + (2 if qos > 0 else 0) + payload_len
    return 1 + _varint_bytes(rem) + rem


def publish_header(V: int, name: str, s: int) -> int:
    """h_P,l: PUBLISH header (QoS 0, no packet identifier) of a message of s content bytes plus the step header
    on boat/b<V>/tx/<name>, the packet minus the payload (common/fdt_paper.c)."""
    tl = len(("boat/b%d/tx/%s" % (max(1, V), name)).encode())
    return publish_len(tl, 0, STEP_HEADER_BYTES + s) - (STEP_HEADER_BYTES + s)


def cost_c_b(h_T: float, h_W: float, mss: float) -> float:
    """c_b of the cost bound of the paper (eq. cons): 1 + 2 (h_T + h_W) / MSS (a full segment of MSS content
    bytes carries h_T + h_W bytes of headers and fixed frame time, and its TCP acknowledgment as many)."""
    return 1.0 + 2.0 * (h_T + h_W) / mss


def cost_c_k(h_T: float, h_W: float, h_A: float = 0.0) -> float:
    """c_k of the cost bound of the paper (eq. cons), the expression of common/fdt_paper.c: 2 (h_T + h_W), the
    TCP/IP header and the frame of the last segment of a message and of its TCP acknowledgment (QoS 0 has no
    MQTT acknowledgment; h_A stays in the signature and does not enter the bound)."""
    return 2.0 * (h_T + h_W)


def cost_c_m(c_b: float, h_P: float, h_T: float, h_W: float, h_A: float) -> float:
    """c_m,l = c_b h_P,l + c_k of the cost bound of the paper (eq. cons), the expression of common/fdt_paper.c
    in the same order."""
    return c_b * h_P + cost_c_k(h_T, h_W, h_A)


def backlog_bytes(c_b: float, c_k: float, o: float, n: float) -> int:
    """Q_i = ceil(c_b o + c_k n) of o unsent or unacknowledged bytes of the data-bridge socket held by n PUBLISH
    packets (fdt_p_backlog of common/fdt_paper.c); 0 when o = n = 0."""
    o, n = max(0.0, float(o)), max(0.0, float(n))
    if o == 0.0 and n == 0.0:
        return 0
    return int(min(4294967295.0, math.ceil(c_b * o + c_k * n - 1e-9)))


def h_W_busy(h_W: float, R_PHY: float, T_DIFS: float, T_bo: float) -> float:
    """h_W^busy = h_W - R_PHY (T_DIFS + T_bo) / 8 (C7), the expression of fdt_dtp_parse in common/fdt_common.c."""
    return h_W - R_PHY * (T_DIFS + T_bo) / 8.0


@dataclass
class TopicSpec:
    """Uplink topic of the DTP: source rate (messages per second), bound on the content of one message
    (bytes), jitter J_l of the source (s, None when the DTP gives none) and c_m,l of the cost bound
    (None: the c_m of the DTP)."""
    name: str
    rate: float = 0.0
    bound: int = 0
    jitter: Optional[float] = None
    c_m: Optional[float] = None


@dataclass
class DTP:
    """Parameters of the DTP shared by the AS, the VE, the DTI program, the vessel regulator and the channel monitor.
    The file format is described in SPEC.md (dtp.json)."""
    topics: List[str]
    Ts: float = 0.125
    T1: float = 0.020
    T2: float = 0.010
    alpha: float = 0.125
    U: float = 0.8
    C0: float = 300_000.0   # initial prediction, bytes per step
    c_b: float = 1.0        # c(x, n) = c_b x + c_m n (eq. 5)
    c_m: float = 0.0
    c_k: float = 0.0        # per-message term of the cost bound without the PUBLISH header (eq. cons)
    epoch: Optional[float] = None   # Unix time of the start of step 0 of the operation
    R_PHY: float = 24e6             # bit rate of the radios, bit/s
    h_W: float = 0.0                # bytes that the PHY sends at R_PHY in the fixed time of one frame
    T_DIFS: float = 0.0             # DIFS (AIFS of AC_BE under EDCA) inside h_W, s (channel monitor, C7)
    T_bo: float = 0.0               # mean backoff inside h_W, CWmin/2 slots, s (channel monitor, C7)
    T_m: float = 0.0                # margin of phase 3: clock offset bound eps + GS delivery time delta_GS, s
    c_y: float = 1.0                # smallest C(t) / y(t) of the saturation test, in (0, 1] (eq. 2)
    B_ctl_bar: float = 0.0          # bytes of control traffic outside the data bridges in phase 3 (fidelity)
    order: List[str] = field(default_factory=list)    # initial order sigma of the AS
    needed: List[str] = field(default_factory=list)   # initial needed topics of the AS
    vessels: List[int] = field(default_factory=list)
    specs: List[TopicSpec] = field(default_factory=list)

    @property
    def t0(self) -> float:
        """Start of step 0: the DTP epoch, or DEFAULT_EPOCH when the file has none."""
        return DEFAULT_EPOCH if self.epoch is None else float(self.epoch)

    @property
    def T3(self) -> float:
        return self.Ts - self.T1 - self.T2

    @property
    def T3m(self) -> float:
        """T3 - T_m: the part of phase 3 that the grant counts on (K_hat(t) = C_hat(t) (T3 - T_m) / Ts)."""
        return self.T3 - self.T_m

    def K_hat(self, C_hat: float) -> float:
        """Predicted capacity of phase 3, K_hat(t) = C_hat(t) (T3 - T_m) / Ts."""
        return C_hat * self.T3m / self.Ts

    def K_y(self, y: float) -> float:
        """Measured capacity of phase 3 for the fidelity metrics, K_y(t) = c_y y(t) (T3 - T_m) / Ts - B_ctl_bar."""
        return self.c_y * y * self.T3m / self.Ts - self.B_ctl_bar

    @property
    def L(self) -> int:
        return len(self.topics)

    @property
    def h_W_busy(self) -> float:
        """Busy part of h_W (C7): h_W - R_PHY (T_DIFS + T_bo) / 8, the bytes of the fixed time of a frame
        during which the medium is busy (preamble, headers, SIFS, MAC ACK); radio/channel_monitor/fdt_chmon.c
        charges this per frame in T_fleet(t)."""
        return h_W_busy(self.h_W, self.R_PHY, self.T_DIFS, self.T_bo)

    def channel_bytes(self, x: int, n: int) -> float:
        return self.c_b * x + self.c_m * n

    def topic_c_m(self, l: int) -> float:
        """c_m,l of the 1-based topic l (the c_m of the DTP unless the topic has its own)."""
        if 1 <= l <= len(self.specs) and self.specs[l - 1].c_m is not None:
            return float(self.specs[l - 1].c_m)
        return self.c_m

    def message_bytes(self, l: int, x: int, n: int = 1) -> float:
        """Channel bytes of n forwarded messages of topic l with x content bytes in all:
        c_b (x + 6 n) + c_m,l n, the cost that the demand d_il (fdt_dti) and the policing of the vessel
        regulator (fdt_p_cost of common/fdt_paper.c) charge per message."""
        return self.c_b * (x + STEP_HEADER_BYTES * n) + self.topic_c_m(l) * n

    def index(self, name: str) -> int:
        """1-based index of a topic."""
        return self.topics.index(name) + 1

    @staticmethod
    def from_dict(d: Dict) -> "DTP":
        """DTP of SPEC.md (topics with "rate" in messages per second and "bound") or DTP v2 of SPEC_v2.md
        (topics with "rate" in outputs per second, "n" messages per output and "s"; rate becomes n rate
        and bound s). Without "c_b" (or "c_m") in the file, the cost bound of the paper (eq. cons) applies,
        the rule of common/fdt_paper.c: c_b = 1 + 2 (h_T + h_W) / MSS, c_k = 2 (h_T + h_W) (unless the file
        gives "c_k") and, per topic, c_m,l = c_b h_P,l + c_k, with MSS, h_T of "tcp" (1448, 80), h_W of the
        file (0) and h_P,l the PUBLISH header of boat/b<V>/tx/<name> (QoS 0, V the largest vessel index)."""
        d = dict(d)
        specs = []
        topics = d.get("topics", [])
        if topics and isinstance(topics[0], dict):
            for t in topics:
                kw = {k: v for k, v in t.items() if k in ("name", "rate", "bound", "jitter")}
                if "bound" not in t and "s" in t:          # DTP v2
                    kw["bound"] = int(t["s"])
                    kw["rate"] = float(t.get("rate", 0.0)) * int(t.get("n", 1))
                specs.append(TopicSpec(**kw))
            d["topics"] = [t.name for t in specs]
        tcp, mqtt = d.get("tcp") or {}, d.get("mqtt") or {}
        h_W = float(d.get("h_W") or 0.0)
        h_T, mss, h_A = float(tcp.get("h_T", 80)), float(tcp.get("mss", 1448)), float(mqtt.get("h_A", 4))
        if d.get("c_b") is None:
            d["c_b"] = cost_c_b(h_T, h_W, mss)
        if d.get("c_k") is None:
            d["c_k"] = cost_c_k(h_T, h_W, h_A)
        if d.get("c_m") is None:
            d.pop("c_m", None)
            V = max([int(v) for v in d.get("vessels") or []] or [1])
            for t in specs:
                t.c_m = float(d["c_b"]) * publish_header(V, t.name, int(t.bound)) + float(d["c_k"])
        keys = {k: v for k, v in d.items() if k in DTP.__dataclass_fields__ and k != "specs"}
        for k in ("T_m", "c_y", "B_ctl_bar"):          # null in the file: the default
            if k in keys and keys[k] is None:
                del keys[k]
        dtp = DTP(**keys)
        if not 0.0 < float(dtp.c_y) <= 1.0:
            raise ValueError("DTP: c_y must lie in (0, 1]")
        if dtp.T_m < 0 or dtp.T_m >= dtp.T3 or dtp.B_ctl_bar < 0:
            raise ValueError("DTP: 0 <= T_m < T3 and B_ctl_bar >= 0")
        dtp.specs = specs or [TopicSpec(name=n) for n in dtp.topics]
        return dtp

    @staticmethod
    def load(path) -> "DTP":
        import json
        with open(path, encoding="utf-8") as f:
            return DTP.from_dict(json.load(f))


def smooth(C_prev: float, y_prev: Optional[float], alpha: float, c_y: float = 1.0) -> float:
    """Eq. (2): C_hat(t) = (1 - alpha) C_hat(t-1) + alpha c_y y(t-1); keeps the previous prediction when y(t-1)
    is missing (did not arrive in phase 1)."""
    if y_prev is None:
        return C_prev
    return (1.0 - alpha) * C_prev + alpha * c_y * y_prev


def grant_cap(C_hat: float, U: float, T3: float, Ts: float, Q: float = 0.0, T_m: float = 0.0) -> float:
    """Bytes that the fleet may admit in step t: max(0, U K_hat(t) - Q(t)), K_hat(t) = C_hat(t) (T3 - T_m) / Ts."""
    return max(0.0, U * (C_hat * (T3 - T_m) / Ts) - Q)


def grant_limit(C_hat: float, U: float, T3: float, Ts: float, m: int, Q: float = 0.0,
                T_m: float = 0.0) -> Optional[float]:
    """Eq. (3): b(t) = max(0, U K_hat(t) - Q(t)) / m(t) with K_hat(t) = C_hat(t) (T3 - T_m) / Ts and Q(t) the sum
    of the backlogs Q_i(t) of the vessels of M(t). Returns None for m = 0."""
    if m == 0:
        return None
    return grant_cap(C_hat, U, T3, Ts, Q, T_m) / m


class BacklogBook:
    """Backlog of the vessels outside M(t) (eq. 3). For each vessel i the book keeps the step s_i of its last
    demand table that reached the AS in phase 1, the Q_i(s_i) of that table and the final a_i(s_i) that
    Algorithm 1 granted to i in s_i. A vessel outside M(t) still sends its bytes of s_i and earlier in phase 3
    of t, and those bytes stay within Qbar_i(t) = Q_i(s_i) + a_i(s_i) (0 for a vessel that never reported).
    Q(t) = sum_{i in M(t)} Q_i(t) + sum_{i not in M(t)} Qbar_i(t)."""

    def __init__(self) -> None:
        self.last: Dict[int, Tuple[int, int, int]] = {}   # i -> (s_i, Q_i(s_i), a_i(s_i))

    def qbar(self, i: int) -> int:
        s = self.last.get(int(i))
        return 0 if s is None else int(s[1] + s[2])

    def split(self, M: Sequence[int], Qi: Dict[int, int]) -> Tuple[int, int, Dict[int, int]]:
        """(sum of Q_i(t) over M(t), sum of Qbar_i(t) outside M(t), Qbar_i(t) per vessel outside M(t) that has
        reported before)."""
        inM = set(int(i) for i in M)
        qbar = {i: self.qbar(i) for i in sorted(self.last) if i not in inM}
        return int(sum(int(Qi.get(i, 0)) for i in inM)), int(sum(qbar.values())), qbar

    def reset(self, i: int) -> bool:
        """The data bridge of vessel i disconnected or reconnected: the new TCP connection (clean session) starts
        with an empty send queue, so Qbar_i = 0 until the next table of i. Returns True when i had an entry."""
        return self.last.pop(int(i), None) is not None

    def record(self, t: int, M: Sequence[int], Qi: Dict[int, int], a: Dict[int, int]) -> None:
        """After the decision of step t: s_i = t, Q_i(t) and a_i(t) for every vessel of M(t)."""
        for i in M:
            self.last[int(i)] = (int(t), int(Qi.get(i, 0)), int(a.get(i, 0)))


def admission_b7(D: Dict[int, Sequence[int]], order: Sequence[int], needed: Sequence[int],
                 b: float) -> Tuple[Dict[int, List[int]], Dict[int, float]]:
    """Baseline B7: per-vessel grants with redistribution of the unused grant (IPACT limited service with
    excess distribution) under the grant limit b = max(0, U K_hat - Q) / m.

    Each vessel of D starts with the grant g_i = b and gets its own table p_i = Algorithm 1 over its own
    demand table with the limit g_i. A vessel whose table admits every topic keeps g_i = a_i (its
    admitted demand) and gives the excess g_i - a_i to the pool; the pool is divided equally among the
    vessels with a topic at level 0, whose tables are computed again with the larger grant; the passes
    repeat until no vessel becomes fully admitted (at most m passes). The grants sum to m b = max(0, U K_hat - Q),
    so the admitted demand of the fleet stays within it. Returns (p_i per vessel, g_i per vessel)."""
    vs = sorted(D)
    g = {i: float(b) for i in vs}
    p_by: Dict[int, List[int]] = {}
    final = set()
    while True:
        a = {}
        for i in vs:
            if i not in final:
                p_by[i] = admission({i: D[i]}, order, needed, g[i])
            a[i] = sum(D[i][l - 1] for l in range(1, len(p_by[i]) + 1) if p_by[i][l - 1] > 0)
        full = [i for i in vs if i not in final and all(v > 0 for v in p_by[i])]
        rest = [i for i in vs if i not in final and i not in full]
        if not full or not rest:
            break
        pool = sum(g[i] - a[i] for i in full)
        for i in full:
            g[i] = float(a[i])
            final.add(i)
        for i in rest:
            g[i] += pool / len(rest)
    return p_by, g


def admission(D: Dict[int, Sequence[int]], order: Sequence[int], needed: Sequence[int], b: float) -> List[int]:
    """Algorithm 1. D maps vessel i to (d_{i,1}, ..., d_{i,L}); order and needed hold 1-based topics.
    Returns p = (p_1, ..., p_L)."""
    L = len(order)
    p = [0] * L
    a = {i: 0 for i in D}
    need = set(needed)
    for l in order:
        if all(a[i] + D[i][l - 1] <= b for i in D):
            p[l - 1] = 2 if l in need else 1
            for i in D:
                a[i] += D[i][l - 1]
        else:
            p[l - 1] = 0
    return p
