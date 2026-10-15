"""Graph G(t) of the network (Definition 7 of the Fleet-DT paper) and the wire formats of Section V-C.

Wire formats (network byte order):
  demand table  : t (u32), L (u8), d_{i,1}(t) .. d_{i,L}(t) (u32 each)
  priority table: t (u32), L (u8), p_1(t) .. p_L(t) (u8 each)
  forwarded msg : the first 6 bytes of the payload hold t (u32) and the sequence number s (u16)
"""
from __future__ import annotations

import socket
import struct
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple

STEP_HEADER = struct.Struct("!IH")  # t, s


def tcp_nodelay(client):
    """Disables Nagle's algorithm on every socket that a paho-mqtt client opens (R8). paho leaves Nagle
    on; a small PUBLISH that follows a PUBACK then waits for the delayed ACK of the broker (about 40 ms
    on Linux), which pushes the priority table past the end of phase 2."""
    def on_open(c, userdata, sock):
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except (OSError, AttributeError):
            pass
    client.on_socket_open = on_open
    return client


def encode_demand(t: int, d: List[int]) -> bytes:
    return struct.pack("!IB", t, len(d)) + struct.pack("!%dI" % len(d), *d)


def decode_demand(buf: bytes) -> Tuple[int, List[int]]:
    t, n = struct.unpack_from("!IB", buf, 0)
    if len(buf) != 5 + 4 * n:
        raise ValueError("demand table of %d bytes for L=%d" % (len(buf), n))
    return t, list(struct.unpack_from("!%dI" % n, buf, 5))


def encode_levels(t: int, p: List[int]) -> bytes:
    return struct.pack("!IB", t, len(p)) + bytes(p)


def decode_levels(buf: bytes) -> Tuple[int, List[int]]:
    t, n = struct.unpack_from("!IB", buf, 0)
    if len(buf) != 5 + n:
        raise ValueError("priority table of %d bytes for L=%d" % (len(buf), n))
    p = list(buf[5:])
    if any(v > 2 for v in p):
        raise ValueError("level above 2")
    return t, p


def encode_step_header(t: int, s: int) -> bytes:
    return STEP_HEADER.pack(t, s)


def decode_step_header(payload: bytes) -> Tuple[int, int]:
    if len(payload) < STEP_HEADER.size:
        raise ValueError("payload shorter than the step header")
    return STEP_HEADER.unpack_from(payload, 0)


@dataclass
class Edge:
    """Edge (i, l): messages of step t of uplink topic l of vessel i on the data bridge of vessel i."""
    i: int
    l: int                 # 1-based index of the topic in the DTP
    d: int = 0             # demand d_{i,l}(t), bytes in the channel
    p: Optional[int] = None  # level p_l(t)
    r: int = 0             # received bytes within step t, in channel bytes c(x, n)
    x: int = 0             # received content bytes
    n: int = 0             # received distinct messages
    dup: int = 0           # repeated messages (discarded by the GS)
    late: int = 0          # messages of step t that arrived after step t (discarded)
    fwd: Optional[int] = None   # messages that the vessel regulator forwarded (boat/b<i>/regulator)
    held: Optional[int] = None  # messages that the vessel regulator withheld


@dataclass
class Graph:
    """G(t) = (V(t), E(t)); vertex 0 is the GS, V(t) = {0} U M(t)."""
    t: int
    L: int
    topics: List[str]
    vessels: List[int] = field(default_factory=list)       # M(t), sorted
    edges: List[Edge] = field(default_factory=list)
    # attributes of vertex 0
    y: Optional[float] = None       # measured throughput of step t (bytes per T_s)
    C_hat: Optional[float] = None   # prediction of step t
    K_hat: Optional[float] = None   # predicted capacity of phase 3
    b: Optional[float] = None       # grant limit
    U: Optional[float] = None
    alpha: Optional[float] = None
    needed: List[int] = field(default_factory=list)  # N(t), 1-based topic indices
    order: List[int] = field(default_factory=list)   # sigma_t

    @property
    def m(self) -> int:
        return len(self.vessels)

    def edge(self, i: int, l: int) -> Edge:
        for e in self.edges:
            if e.i == i and e.l == l:
                return e
        e = Edge(i=i, l=l)
        self.edges.append(e)
        return e

    def admitted(self, i: int) -> int:
        """Final value a_i of Algorithm 1: demand of vessel i over the admitted topics."""
        return sum(e.d for e in self.edges if e.i == i and (e.p or 0) > 0)

    def to_dict(self) -> Dict:
        d = asdict(self)
        d["m"] = self.m
        return d

    @staticmethod
    def from_dict(d: Dict) -> "Graph":
        g = Graph(t=d["t"], L=d["L"], topics=list(d["topics"]))
        g.vessels = list(d.get("vessels", []))
        g.edges = [Edge(**{k: v for k, v in e.items() if k in Edge.__dataclass_fields__}) for e in d.get("edges", [])]
        for k in ("y", "C_hat", "K_hat", "b", "U", "alpha"):
            setattr(g, k, d.get(k))
        g.needed = list(d.get("needed", []))
        g.order = list(d.get("order", []))
        return g
