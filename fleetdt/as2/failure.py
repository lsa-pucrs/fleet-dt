"""Accepted messages of the data bridges and the failure indicator f(t) (Def. aci).

The AS subscribes to boat/+/tx/# with QoS 2, so the GS broker delivers a QoS 2 message to the AS after
the PUBREL of the vessel. A message is accepted within step t when its step header carries t and the AS
receives it no later than the end of step t on the GS clock. Repeated messages (same vessel, topic,
t, seq) count once.
"""
from __future__ import annotations

import re
import struct
import threading
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Set, Tuple

TX_RE = re.compile(r"^boat/b(\d+)/tx/([^/]+)$")
HDR = struct.Struct("!IH")
OUT_HDR = struct.Struct("!IHH")     # out:u32, k:u16, n:u16 at the start of the content of an output topic


@dataclass
class EdgeAcc:
    seqs: Set[int] = field(default_factory=set)
    n_in: int = 0      # messages accepted within step t
    x_in: int = 0      # content bytes accepted within step t (d_{i,l}(t))
    n_late: int = 0
    x_late: int = 0
    n_dup: int = 0


class Acceptor:
    """Counts the accepted messages per (vessel, topic, step). `log` receives one tuple per accepted
    message of the topics in `log_topics` (for the age of information):
    (i, l, t, seq, at, first `prefix` bytes of the content as hex)."""

    def __init__(self, dtp, log_topics: Optional[Iterable[int]] = None,
                 log: Optional[Callable[[Tuple], None]] = None, prefix: int = 16,
                 out_log: Optional[Callable[[Dict], None]] = None):
        self.dtp = dtp
        self.lock = threading.Lock()
        self.e: Dict[Tuple[int, int, int], EdgeAcc] = {}
        self.log_topics = set(dtp.required if log_topics is None else log_topics)
        self.log = log
        self.prefix = prefix
        self.bad = 0
        self.unknown = 0
        # outputs of the output topics, by (vessel, topic, output index of the source): one "out" record
        # per output when its last message arrives (or, incomplete, when it is pruned), so that the
        # analysis finds the outputs that the GS accepts complete, independent of the step header
        self.out_log = out_log
        self.out_topics = set(tp.index for tp in dtp.topics if tp.output)
        self.outs: Dict[Tuple[int, int, int], Dict] = {}

    def on_message(self, topic: str, payload: bytes, at: float) -> Optional[Tuple]:
        m = TX_RE.match(topic)
        if not m or len(payload) < HDR.size:
            self.bad += 1
            return None
        i, name = int(m.group(1)), m.group(2)
        l = self.dtp.names.get(name)
        if l is None:
            self.unknown += 1
            return None
        t, seq = HDR.unpack_from(payload, 0)
        x = len(payload) - HDR.size
        intime = at <= self.dtp.end(t)
        with self.lock:
            e = self.e.get((i, l, t))
            if e is None:
                e = self.e[(i, l, t)] = EdgeAcc()
            if seq in e.seqs:
                e.n_dup += 1
                return None
            e.seqs.add(seq)
            if intime:
                e.n_in += 1
                e.x_in += x
            else:
                e.n_late += 1
                e.x_late += x
        if self.log is not None and l in self.log_topics:
            self.log((i, l, t, seq, at, bytes(payload[HDR.size:HDR.size + self.prefix]).hex()))
        if self.out_log is not None and l in self.out_topics and len(payload) >= HDR.size + OUT_HDR.size:
            oid, k, n = OUT_HDR.unpack_from(payload, HDR.size)
            done = None
            with self.lock:
                o = self.outs.get((i, l, oid))
                if o is None:
                    o = self.outs[(i, l, oid)] = {"n": n, "k": set(), "x": 0, "t": t, "t_max": t, "first": at,
                                                  "last": at}
                if k not in o["k"]:
                    o["k"].add(k)
                    o["x"] += x
                    o["last"] = max(o["last"], at)
                    o["t"] = min(o["t"], t)
                    o["t_max"] = max(o["t_max"], t)
                if len(o["k"]) >= o["n"]:
                    done = self.outs.pop((i, l, oid))
            if done is not None:
                self._emit_out(i, l, oid, done, True)
        return (i, l, t, seq, intime)

    def _emit_out(self, i: int, l: int, oid: int, o: Dict, complete: bool) -> None:
        self.out_log({"type": "out", "i": i, "l": l, "o": oid, "n": o["n"], "got": len(o["k"]), "x": o["x"],
                      "t": o["t"], "t_max": o["t_max"], "first": o["first"], "at": o["last"],
                      "complete": complete})

    def get(self, i: int, l: int, t: int) -> EdgeAcc:
        with self.lock:
            e = self.e.get((i, l, t))
            if e is None:
                return EdgeAcc()
            return EdgeAcc(set(), e.n_in, e.x_in, e.n_late, e.x_late, e.n_dup)

    def step(self, t: int) -> Dict[int, Dict[int, EdgeAcc]]:
        """{i: {l: EdgeAcc}} of step t (copies without the sets)."""
        out: Dict[int, Dict[int, EdgeAcc]] = {}
        with self.lock:
            for (i, l, s), e in self.e.items():
                if s == t:
                    out.setdefault(i, {})[l] = EdgeAcc(set(), e.n_in, e.x_in, e.n_late, e.x_late, e.n_dup)
        return out

    def prune(self, t_min: int) -> None:
        old = []
        with self.lock:
            for k in [k for k in self.e if k[2] < t_min]:
                del self.e[k]
            for k in [k for k, o in self.outs.items() if o["t_max"] < t_min]:
                old.append((k, self.outs.pop(k)))
        if self.out_log is not None:
            for (i, l, oid), o in old:
                self._emit_out(i, l, oid, o, False)


@dataclass
class StepFailure:
    f: int
    reasons: List[Tuple] = field(default_factory=list)


def indicator(t: int, Mt: Iterable[int], reports: Dict[int, Optional[Dict]], acc: Dict[int, Dict[int, EdgeAcc]],
              L: int, sent_tables: Optional[Iterable[int]] = None, count_tab_unsent: bool = True) -> StepFailure:
    """f(t) of Def. aci, evaluated in phase 2 of step t+1.

    Mt: M(t). reports: the reports of step t of the vessels of M(t+1) ({i: rep} with rep["t"] == t; a
    report of another step counts as missing). acc: accepted messages of step t within step t.
    f(t) = 1 when, for some vessel, the GS accepts within step t fewer messages of an admitted unit
    than the vessel reports as released, the vessel reports a discarded message of an admitted unit,
    or the vessel reports that its table of step t did not reach it; or a vessel of M(t) sends no
    report of step t. Accepted messages are at most the released messages of their unit, so the test
    compares the sums over the units of each topic. count_tab_unsent=False ignores tab = false of the
    vessels to which the AS sent no table (outside M(t))."""
    out = StepFailure(0)
    Mt = set(Mt)
    sent = Mt if sent_tables is None else set(sent_tables)
    for i in sorted(Mt):
        r = reports.get(i)
        if r is None or r.get("t") != t:
            out.reasons.append((i, "noreport"))
    for i, r in sorted(reports.items()):
        if r is None or r.get("t") != t:
            continue
        rel = r.get("rel") or []
        for l in range(min(L, len(rel))):
            nrel = sum(rel[l])
            nacc = acc.get(i, {}).get(l, EdgeAcc()).n_in
            if nacc < nrel:
                out.reasons.append((i, "miss", l, nrel, nacc))
        if r.get("disc", 0) > 0:
            out.reasons.append((i, "disc", r["disc"]))
        if not r.get("tab", False) and (count_tab_unsent or i in sent):
            out.reasons.append((i, "notab"))
    out.f = 1 if out.reasons else 0
    return out
