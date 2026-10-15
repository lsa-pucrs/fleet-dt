"""DTP v2 (SPEC_v2.md Section 2) and the step clock (SPEC Section 3).

Topic indices are 0-based in the DTP order (the index of d[l] in the demand). Times in seconds on the
GS clock. Step t >= 1 is [epoch + (t-1) Ts, epoch + t Ts).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

DEFAULT_EPOCH = 1767225600.0     # 2026-01-01T00:00:00Z, used when "epoch" is null
STEP_HEADER = 6                  # t:u32, seq:u16 written by the regulator in front of the content
OUTPUT_HEADER = 8                # out:u32, k:u16, n:u16 at the start of the content of an output topic

PHY_KEYS = ("slot", "sifs", "t_pre", "t_sig", "t_sym", "t_se", "h_mac", "aifsn_sta", "cwmin_sta",
            "cwmax_sta", "m_b", "aifsn_ap", "cwmin_ap", "cwmax_ap", "basic_rates", "rate_init")


class DTPError(ValueError):
    pass


def varint_len(v: int) -> int:
    """Bytes of the MQTT remaining-length field of value v."""
    if v < 0 or v > 268435455:
        raise DTPError("remaining length %d out of range" % v)
    n = 1
    while v >= 128:
        v //= 128
        n += 1
    return n


def publish_header(topic: str, qos: int, content: int) -> int:
    """h_P of SPEC Section 2: PUBLISH fixed header (1 + remaining-length bytes) + 2 + len(topic)
    + (2 if qos > 0) + 6 (step header), for a message with `content` bytes of content."""
    var = 2 + len(topic.encode()) + (2 if qos > 0 else 0)
    rl = var + STEP_HEADER + content
    return 1 + varint_len(rl) + var + STEP_HEADER


@dataclass
class Topic:
    name: str
    rate: float
    jitter: float
    n: int
    s: int
    kind: str          # "stream" or "output"
    qos: int           # 0 or 2
    index: int = 0
    h_P: int = 0

    @property
    def output(self) -> bool:
        return self.kind == "output"

    def outputs_bound(self, duration: float) -> int:
        """Arrival-curve bound: ceil(r (duration + J)) outputs in an interval of `duration`."""
        return int(math.ceil(self.rate * (duration + self.jitter) - 1e-12))


@dataclass
class DTP:
    raw: Dict
    Ts: float
    T1: float
    T2: float
    eps: float
    d_rx: float
    d_m: float
    T_tick: float
    L_w: int
    B_cap: int
    T_min: float
    beta: float
    gamma: float
    alpha1: float
    lam: float
    phy: Dict
    mss: int
    h_T: int
    h_A: int
    vessels: List[int]
    topics: List[Topic]
    required: List[int]           # topic indices of R
    order: List[int]              # sigma (default), topic indices
    needed: List[int]             # N (default), topic indices
    goals_rate: float = 1.0
    goals_s: int = 200
    epoch: float = DEFAULT_EPOCH
    names: Dict[str, int] = field(default_factory=dict)

    # ------------------------------------------------------------------ derived values
    @property
    def L(self) -> int:
        return len(self.topics)

    @property
    def M(self) -> int:
        return len(self.vessels)

    @property
    def T3(self) -> float:
        return self.Ts - self.T1 - self.T2

    @property
    def T_W(self) -> float:
        return self.T3 - 2 * self.eps - self.d_rx

    @property
    def g(self) -> float:
        return self.eps + self.d_rx

    @property
    def R(self) -> frozenset:
        return frozenset(self.required)

    def h_P(self, l: int) -> int:
        return self.topics[l].h_P

    def vpos(self, i: int) -> int:
        """Position of vessel i in the DTP list (0-based)."""
        p = self.__dict__.get("_vpos")
        if p is None or len(p) != len(self.vessels):
            p = self.__dict__["_vpos"] = {v: k for k, v in enumerate(self.vessels)}
        return p[i]

    def rot(self, i: int, t: int) -> int:
        """(i - t) mod M of the key of pi_t, with i the 1-based position of the vessel in the DTP list
        (equal to the vessel index when the list is 1..M)."""
        return (self.vpos(i) + 1 - t) % self.M

    # ------------------------------------------------------------------ step clock (SPEC Section 3)
    def start(self, t: int) -> float:
        return self.epoch + (t - 1) * self.Ts

    def end(self, t: int) -> float:
        return self.epoch + t * self.Ts

    def step_of(self, at: float) -> int:
        return int(math.floor((at - self.epoch) / self.Ts)) + 1

    def phase2(self, t: int) -> float:
        return self.start(t) + self.T1

    def phase3(self, t: int) -> float:
        return self.start(t) + self.T1 + self.T2

    def window(self, t: int):
        """W(t) = [w(t), end(t) - eps - d_rx)."""
        return self.start(t) + self.T1 + self.T2 + self.eps, self.end(t) - self.g

    def quiet(self, t: int):
        """Q(t) = [end of W(t-1), start(t) + T1)."""
        return self.start(t) - self.g, self.start(t) + self.T1

    def demand_open(self, t: int, at: float) -> bool:
        """A demand of step t arriving at `at` enters M(t): start(t) - eps <= at < start(t) + T1."""
        return self.start(t) - self.eps <= at < self.phase2(t)

    def topic_string(self, i: int, l: int) -> str:
        return "boat/b%d/tx/%s" % (i, self.topics[l].name)

    # ------------------------------------------------------------------ loading
    @staticmethod
    def load(path: str) -> "DTP":
        with open(path, encoding="utf-8") as f:
            return DTP.from_dict(json.load(f))

    @staticmethod
    def from_dict(d: Dict) -> "DTP":
        return _build(d)

    def topic_indices(self, names: Sequence) -> List[int]:
        out = []
        for v in names:
            if isinstance(v, int) and not isinstance(v, bool):
                if not 0 <= v < self.L:
                    raise DTPError("topic index %r out of range" % v)
                out.append(v)
            else:
                if str(v) not in self.names:
                    raise DTPError("unknown topic %r" % v)
                out.append(self.names[str(v)])
        return out


def _num(d: Dict, k: str, pos: bool = True) -> float:
    if k not in d:
        raise DTPError("missing key %r" % k)
    v = d[k]
    if not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v):
        raise DTPError("%r must be a number" % k)
    if pos and v <= 0:
        raise DTPError("%r must be positive" % k)
    return float(v)


def _build(d: Dict) -> DTP:
    if not isinstance(d, dict):
        raise DTPError("DTP must be a JSON object")
    Ts, T1, T2 = _num(d, "Ts"), _num(d, "T1"), _num(d, "T2")
    eps, d_rx, d_m = _num(d, "eps", False), _num(d, "d_rx", False), _num(d, "d_m", False)
    if min(eps, d_rx, d_m) < 0:
        raise DTPError("eps, d_rx and d_m must be nonnegative")
    T_tick, T_min = _num(d, "T_tick"), _num(d, "T_min")
    L_w, B_cap = int(_num(d, "L_w")), int(_num(d, "B_cap"))
    beta, gamma, alpha1, lam = _num(d, "beta"), _num(d, "gamma"), _num(d, "alpha1", False), _num(d, "lambda")
    if not 0 < beta < 1:
        raise DTPError("beta must be in (0, 1)")
    if not 0 < gamma < 1:
        raise DTPError("gamma must be in (0, 1)")
    if not 0 <= alpha1 <= 1:
        raise DTPError("alpha1 must be in [0, 1]")
    if not 0 < lam <= 1:
        raise DTPError("lambda must be in (0, 1]")
    T3 = Ts - T1 - T2
    if T3 <= 0:
        raise DTPError("T3 = Ts - T1 - T2 must be positive")
    if T3 - 2 * eps - d_rx <= 0:
        raise DTPError("T_W = T3 - 2 eps - d_rx must be positive")
    if T_min >= T3 - 2 * eps - d_rx:
        raise DTPError("T_min must be shorter than T_W")
    phy = d.get("phy")
    if not isinstance(phy, dict):
        raise DTPError("missing phy")
    for k in PHY_KEYS:
        if k not in phy:
            raise DTPError("missing phy.%s" % k)
    if not phy["basic_rates"]:
        raise DTPError("phy.basic_rates is empty")
    ofdm = (6e6, 9e6, 12e6, 18e6, 24e6, 36e6, 48e6, 54e6)
    for r in list(phy["basic_rates"]) + [phy["rate_init"]]:
        if not any(abs(r - o) < 1.0 for o in ofdm):
            raise DTPError("rate %g is not an ERP-OFDM rate" % r)
    tcp, mq = d.get("tcp") or {}, d.get("mqtt") or {}
    mss, h_T, h_A = int(_num(tcp, "mss")), int(_num(tcp, "h_T")), int(_num(mq, "h_A"))
    vessels = d.get("vessels")
    if not isinstance(vessels, list) or not vessels or any(not isinstance(v, int) or isinstance(v, bool)
                                                           or v < 1 for v in vessels):
        raise DTPError("vessels must be a nonempty list of positive integers")
    if len(set(vessels)) != len(vessels):
        raise DTPError("repeated vessel index")
    tl = d.get("topics")
    if not isinstance(tl, list) or not tl:
        raise DTPError("topics must be a nonempty list")
    topics: List[Topic] = []
    names: Dict[str, int] = {}
    for k, td in enumerate(tl):
        name = td.get("name")
        if not isinstance(name, str) or not name or "/" in name or "+" in name or "#" in name:
            raise DTPError("invalid topic name %r" % name)
        if name in names:
            raise DTPError("repeated topic %r" % name)
        kind = td.get("kind")
        if kind not in ("stream", "output"):
            raise DTPError("topic %s: kind must be stream or output" % name)
        qos = td.get("qos")
        if qos not in (0, 2):
            raise DTPError("topic %s: qos must be 0 or 2" % name)
        n = int(_num(td, "n"))
        s = int(_num(td, "s"))
        if kind == "output" and s <= OUTPUT_HEADER:
            raise DTPError("topic %s: s must exceed the output header" % name)
        tp = Topic(name=name, rate=_num(td, "rate"), jitter=_num(td, "jitter", False), n=n, s=s, kind=kind,
                   qos=qos, index=k)
        if tp.jitter < 0:
            raise DTPError("topic %s: negative jitter" % name)
        topics.append(tp)
        names[name] = k
    imax = max(vessels)
    for tp in topics:
        tp.h_P = publish_header("boat/b%d/tx/%s" % (imax, tp.name), tp.qos, tp.s)

    def idx(lst, key):
        if not isinstance(lst, list):
            raise DTPError("%s must be a list" % key)
        out = []
        for v in lst:
            if v not in names:
                raise DTPError("%s: unknown topic %r" % (key, v))
            out.append(names[v])
        if len(set(out)) != len(out):
            raise DTPError("%s: repeated topic" % key)
        return out

    required = idx(d.get("required", []), "required")
    order = idx(d.get("order", [t.name for t in topics]), "order")
    needed = idx(d.get("needed", d.get("required", [])), "needed")
    for tp in topics:
        if (tp.index in required) != (tp.qos == 2):
            raise DTPError("topic %s: the topics of required have qos 2 and the others qos 0" % tp.name)
    if sorted(order) != list(range(len(topics))):
        raise DTPError("order must be a permutation of the topics")
    if set(order[:len(required)]) != set(required):
        raise DTPError("order must start with the topics of required")
    if not set(required) <= set(needed):
        raise DTPError("needed must hold the topics of required")
    goals = d.get("goals") or {}
    epoch = d.get("epoch")
    epoch = DEFAULT_EPOCH if epoch is None else float(epoch)
    return DTP(raw=d, Ts=Ts, T1=T1, T2=T2, eps=eps, d_rx=d_rx, d_m=d_m, T_tick=T_tick, L_w=L_w, B_cap=B_cap,
               T_min=T_min, beta=beta, gamma=gamma, alpha1=alpha1, lam=lam, phy=dict(phy), mss=mss, h_T=h_T,
               h_A=h_A, vessels=list(vessels), topics=topics, required=required, order=order, needed=needed,
               goals_rate=float(goals.get("rate", 1.0)), goals_s=int(goals.get("s", 200)), epoch=epoch,
               names=names)


SPEC_EXAMPLE = {
    "epoch": None,
    "Ts": 0.125, "T1": 0.020, "T2": 0.010,
    "eps": 0.002, "d_rx": 0.002, "d_m": 0.001,
    "T_tick": 0.001, "L_w": 16384, "B_cap": 1048576, "T_min": 0.005,
    "beta": 0.05, "gamma": 0.01, "alpha1": 0.5, "lambda": 0.25,
    "phy": {"slot": 9e-6, "sifs": 10e-6, "t_pre": 16e-6, "t_sig": 4e-6, "t_sym": 4e-6, "t_se": 6e-6,
            "h_mac": 38, "aifsn_sta": 3, "cwmin_sta": 15, "cwmax_sta": 1023, "m_b": 6,
            "aifsn_ap": 1, "cwmin_ap": 7, "cwmax_ap": 1023,
            "basic_rates": [6e6, 12e6, 24e6], "rate_init": 24e6},
    "tcp": {"mss": 1448, "h_T": 80},
    "mqtt": {"h_A": 4},
    "vessels": [1, 2, 3, 4],
    "topics": [
        {"name": "state", "rate": 8, "jitter": 0.002, "n": 1, "s": 48, "kind": "stream", "qos": 2},
        {"name": "imu", "rate": 32, "jitter": 0.002, "n": 1, "s": 48, "kind": "stream", "qos": 0},
        {"name": "gps", "rate": 8, "jitter": 0.002, "n": 1, "s": 80, "kind": "stream", "qos": 0},
        {"name": "baro", "rate": 8, "jitter": 0.002, "n": 1, "s": 32, "kind": "stream", "qos": 0},
        {"name": "power", "rate": 8, "jitter": 0.002, "n": 1, "s": 40, "kind": "stream", "qos": 0},
        {"name": "mag", "rate": 16, "jitter": 0.002, "n": 1, "s": 40, "kind": "stream", "qos": 0},
        {"name": "video", "rate": 15, "jitter": 0.010, "n": 16, "s": 1400, "kind": "output", "qos": 0},
    ],
    "required": ["state"],
    "goals": {"rate": 1, "s": 200},
    "order": ["state", "imu", "gps", "video", "baro", "power", "mag"],
    "needed": ["state", "imu", "gps"],
}


def example(**over) -> DTP:
    """The DTP of SPEC Section 2 with the given keys replaced (tests and what-if)."""
    import copy
    d = copy.deepcopy(SPEC_EXAMPLE)
    for k, v in over.items():
        d[k] = v
    return DTP.from_dict(d)
