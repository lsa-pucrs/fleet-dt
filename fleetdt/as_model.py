"""Computations of the AS that the VE reproduces when the AS does not publish them:
exponential smoothing (eq. 2), grant limit (eq. 3), and Algorithm 1 (admission control)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

# epoch of a DTP file with "epoch": null (2026-01-01T00:00:00Z); the C programs use the same value
DEFAULT_EPOCH = 1767225600.0


@dataclass
class TopicSpec:
    """Uplink topic of the DTP: source rate (messages per second) and bound on the content of one message (bytes)."""
    name: str
    rate: float = 0.0
    bound: int = 0


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
    epoch: Optional[float] = None   # Unix time of the start of step 0 of the operation
    R_PHY: float = 24e6             # bit rate of the radios, bit/s
    h_W: float = 0.0                # bytes that the PHY sends at R_PHY in the fixed time of one frame
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
    def L(self) -> int:
        return len(self.topics)

    def channel_bytes(self, x: int, n: int) -> float:
        return self.c_b * x + self.c_m * n

    def index(self, name: str) -> int:
        """1-based index of a topic."""
        return self.topics.index(name) + 1

    @staticmethod
    def from_dict(d: Dict) -> "DTP":
        d = dict(d)
        specs = []
        topics = d.get("topics", [])
        if topics and isinstance(topics[0], dict):
            specs = [TopicSpec(**{k: v for k, v in t.items() if k in TopicSpec.__dataclass_fields__}) for t in topics]
            d["topics"] = [t.name for t in specs]
        keys = {k: v for k, v in d.items() if k in DTP.__dataclass_fields__ and k != "specs"}
        dtp = DTP(**keys)
        dtp.specs = specs or [TopicSpec(name=n) for n in dtp.topics]
        return dtp

    @staticmethod
    def load(path) -> "DTP":
        import json
        with open(path, encoding="utf-8") as f:
            return DTP.from_dict(json.load(f))


def smooth(C_prev: float, y_prev: Optional[float], alpha: float) -> float:
    """Eq. (2); keeps the previous prediction when the measurement is missing."""
    if y_prev is None:
        return C_prev
    return (1.0 - alpha) * C_prev + alpha * y_prev


def grant_limit(C_hat: float, U: float, T3: float, Ts: float, m: int) -> Optional[float]:
    """Eq. (3): K_hat = U C_hat T3 / Ts and b = K_hat / m. Returns None for m = 0."""
    if m == 0:
        return None
    return U * C_hat * T3 / Ts / m


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
