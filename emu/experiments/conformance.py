#!/usr/bin/env python3
"""E0 conformance check of one paper-stack run directory (design v2, Section 6, experiment E0).

  python3 emu/experiments/conformance.py RUN [--json OUT] [--first T] [--last T] [--warm 0]

The script checks the run against the model of the paper from the logs that emu/fleet.py collects:

  alg1        offline replay of the AS: eq. (2) from the C_hat of the previous step and the y(t - 1) that
              the AS used (equal to the y(t - 1) that the channel monitor published), the grant limit
              b(t) = max(0, U K_hat(t) - Q(t)) / m(t) with the logged Q(t), Algorithm 1 (B6: the per-vessel
              grants) from the logged D_i(t), sigma_t and N(t) gives the published p(t) in every step; the
              tables equal the D_i(t) that each regulator policed and the p(t) that it received; sigma_t and
              N(t) equal the mode of the JMCS stub of step t
  bytes       forwarded channel bytes per edge <= d_il(t): the cost of the messages that each regulator
              forwarded (step log) and the r_il(t) that the GS received (G(t) of the DTA)
  qos         every forwarded message of a topic of level 1 or 2 crosses with MQTT QoS 0 (the model forwards
              both levels with QoS 0 and the data bridges subscribe with QoS 0, so the baselines cross with
              QoS 0 too) and no message of a level-0 topic crosses: the PUBLISH packets of the data bridges in the
              pcap of the AP (fdt-emu --pcap, "pcap": true of the experiment); skipped without a pcap
  no_p        no uplink message of step t from a vessel without p(t) (step log, DTA and pcap)
  logged      every message of step t that the regulator did not forward is in the vessel log with a
              reason (rx - fwd = log lines = held + drop per step and topic), and the regulator received
              every message that the source published (source log against the step log, steps whose
              releases lie within --boundary-ms of a collection boundary excepted)
  dt          G(t) of the DTA against the ground truth of the emulator: content bytes received at the GS
              within step t at most the IP bytes that the AP received from the vessel in step t (and t + 1,
              for the clock offset of the GS), with the relative error of r_il against the channel bytes of
              stepbytes.csv; M(t) of the DTA equal to the M(t) of the AS; the lag of the DTI (emission of
              G(t) after the end of step t) at most one step

Inputs (RUN/): dtp.json, result.json (S0, steps, method), throughput.jsonl (y(t) of the monitor adapter),
vms/gs/as.jsonl (AS decisions with D and y_used), vms/gs/dta.jsonl (G(t) with "emitted"),
vms/gs/order.jsonl (JMCS modes), vms/v<i>/regsteps.jsonl (regulator step log), vms/v<i>/regulator.jsonl
(vessel log), vms/v<i>/src.jsonl (source log), emu/stepbytes.csv, emu/ap.pcap (optional).

Each check returns {"status": "pass" | "fail" | "skipped", "n": items checked, "fail": failures,
"examples": the first failures, ...}; the run conforms when no check fails and alg1, bytes, no_p,
logged and dt pass. Exit status 0 when the run conforms, 1 otherwise.
"""
import argparse
import csv
import json
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "emu"))

from fleetdt.as_model import DTP, admission, admission_b6, grant_limit, smooth  # noqa: E402

REASONS = {"no_table", "no_demand", "level0", "police", "overflow", "resync", "eos", "shutdown", "budget", "slot"}
TABLE_METHODS = {"fleetdt", "B4", "B5a", "B5b", "B6"}
POLICING_METHODS = {"fleetdt", "B4", "B5b", "B6"}
OTHER_METHODS = {"B1", "B2", "B3", "B7", "B8", "B9"}
MAX_EXAMPLES = 8


# ------------------------------------------------------------------------------------------------ inputs

def load_json(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_jsonl(p):
    out = []
    try:
        with open(p) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    except OSError:
        return None
    return out


# labels of the code before the paper labels of Table S3 that no paper label reuses (emu/fleet.py PAPER_RENAMED)
RENAMED = {"b10": "B9"}


def method_key(m):
    m = str(m or "fleetdt")
    if m.lower() in RENAMED:
        raise SystemExit("method %s: the baseline labels follow Table S3 of the paper; the former %s is now %s "
                         "(and the former B7, B8, B9 are B6, B7, B8)" % (m, m, RENAMED[m.lower()]))
    for k in TABLE_METHODS | OTHER_METHODS:
        if k.lower() == m.lower():
            return k
    return m


class Run(object):
    """The logs of one run directory."""

    def __init__(self, rd, first=None, last=None, warm=0):
        self.rd = rd
        self.res = load_json(os.path.join(rd, "result.json")) or {}
        self.dtp_raw = load_json(os.path.join(rd, "dtp.json"))
        if self.dtp_raw is None:
            raise SystemExit("%s: no dtp.json" % rd)
        self.dtp = DTP.from_dict(self.dtp_raw)
        self.epoch = self.dtp.t0
        self.method = method_key(self.res.get("method"))
        self.M = int(self.res.get("M") or len(self.dtp.vessels) or 0)
        self.vessels = list(range(1, self.M + 1))
        gs = os.path.join(rd, "vms", "gs")
        self.as_log = {int(r["t"]): r for r in (load_jsonl(os.path.join(gs, "as.jsonl")) or []) if "t" in r}
        self.dta = {int(g["t"]): g for g in (load_jsonl(os.path.join(gs, "dta.jsonl")) or []) if "t" in g}
        self.order_log = {int(r["t"]): r for r in (load_jsonl(os.path.join(gs, "order.jsonl")) or []) if "t" in r}
        self.y = {}
        for r in load_jsonl(os.path.join(rd, "throughput.jsonl")) or []:
            if "t" in r and r.get("y") is not None:
                self.y[int(r["t"])] = float(r["y"])
        self.steps_log, self.vlog, self.src = {}, {}, {}
        for i in self.vessels:
            vd = os.path.join(rd, "vms", "v%d" % i)
            st = load_jsonl(os.path.join(vd, "regsteps.jsonl"))
            self.steps_log[i] = None if st is None else {int(r["t"]): r for r in st if "t" in r}
            self.vlog[i] = load_jsonl(os.path.join(vd, "regulator.jsonl"))
            self.src[i] = load_jsonl(os.path.join(vd, "src.jsonl"))
        S0 = self.res.get("S0")
        if S0 is None:
            ts = sorted(self.as_log) or sorted(self.dta)
            S0 = ts[0] if ts else 0
        self.S0 = int(S0)
        steps = int(self.res.get("steps") or 0)
        if not steps:
            ts = sorted(set(self.as_log) | set(self.dta))
            steps = (ts[-1] - self.S0 + 1) if ts else 0
        self.first = self.S0 + int(warm) if first is None else int(first)
        self.last = self.S0 + steps - 1 if last is None else int(last)
        self.stepbytes = self._stepbytes(os.path.join(rd, "emu", "stepbytes.csv"))
        self.pcap_path = os.path.join(rd, "emu", "ap.pcap")

    def steps(self):
        return range(self.first, self.last + 1)

    def _stepbytes(self, path):
        """{(t, i): (frames, bytes)} of the whole step (scenario step k = t - S0)"""
        out = {}
        try:
            with open(path) as f:
                for r in csv.DictReader(f):
                    out[(self.S0 + int(r["k"]), int(r["i"]))] = (int(r["frames"]), int(r["bytes"]))
        except (OSError, KeyError, ValueError):
            return None
        return out

    def start(self, t):
        return self.epoch + t * self.dtp.Ts

    def v(self, t):
        return self.start(t) + self.dtp.T1 + self.dtp.T2

    def collect_step(self, at):
        """step of a raw message that reaches the regulator at `at`: [v(t-1), v(t)) belongs to step t"""
        return int(math.floor((at - self.epoch - self.dtp.T1 - self.dtp.T2) / self.dtp.Ts)) + 1

    def table_of(self, i, t):
        """levels that vessel i applied in step t: the p of its step log, else the table of the AS
        (B6: the table of the vessel); None when the vessel had no table"""
        st = self.steps_log.get(i)
        if st is not None:
            r = st.get(t)
            if r is not None:
                return list(r["p"]) if r.get("have_p") and r.get("p") is not None else None
        a = self.as_log.get(t)
        if not a or not a.get("published") or i not in (a.get("M") or []):
            return None
        if a.get("p_by"):
            p = a["p_by"].get(str(i), a["p_by"].get(i))
            return list(p) if p is not None else None
        return list(a["p"]) if a.get("p") is not None else None


class Check(object):
    def __init__(self, name):
        self.name, self.n, self.nfail, self.ex, self.extra = name, 0, 0, [], {}
        self.skip = None

    def ok(self, cond, what=None):
        self.n += 1
        if not cond:
            self.nfail += 1
            if len(self.ex) < MAX_EXAMPLES and what is not None:
                self.ex.append(what)
        return cond

    def skipped(self, why):
        self.skip = why
        return self

    def result(self):
        if self.skip is not None and self.n == 0:
            out = {"status": "skipped", "reason": self.skip}
        else:
            out = {"status": "fail" if self.nfail else ("pass" if self.n else "skipped"), "n": self.n,
                   "fail": self.nfail, "examples": self.ex}
            if self.n == 0:
                out["reason"] = self.skip or "nothing to check"
        out.update(self.extra)
        return out


def _rel(a, b):
    return abs(a - b) <= 1e-6 * max(1.0, abs(a), abs(b))


# ------------------------------------------------------------------------------------------------ alg1

def jmcs_modes(run):
    import jmcs_stub
    m = jmcs_stub.build_modes(run.dtp_raw)
    idx = {n: k + 1 for k, n in enumerate(run.dtp.topics)}
    return {k: ([idx[n] for n in v["order"]], [idx[n] for n in v["needed"]]) for k, v in m.items()}


def check_alg1(run):
    c = Check("alg1")
    if run.method not in TABLE_METHODS:
        return c.skipped("method %s takes no priority table" % run.method).result()
    if not run.as_log:
        return c.skipped("no vms/gs/as.jsonl").result()
    d = run.dtp
    modes = jmcs_modes(run) if run.order_log else None
    replayed = recv_bad = d_bad = order_bad = eq2_bad = y_bad = 0
    for t in run.steps():
        r = run.as_log.get(t)
        if r is None or not r.get("published"):
            continue
        if "D" not in r:
            c.ok(False, {"t": t, "why": "the AS log has no D (older AS)"})
            continue
        D = {int(i): [int(x) for x in v] for i, v in r["D"].items()}
        M = sorted(D)
        sigma, N = [int(x) for x in r["order"]], [int(x) for x in r["needed"]]
        # eq. (2) and the y(t - 1) of the monitor
        prev = run.as_log.get(t - 1)
        if prev is not None and prev.get("C_hat") is not None and not r.get("skipped"):
            want = smooth(float(prev["C_hat"]), r.get("y_used"), float(r["alpha"]), float(r.get("c_y", d.c_y)))
            if not _rel(want, float(r["C_hat"])):
                eq2_bad += 1
                c.ok(False, {"t": t, "why": "eq. (2)", "C_hat": r["C_hat"], "replay": want})
                continue
        if r.get("y_used") is not None and (t - 1) in run.y and not _rel(float(r["y_used"]), run.y[t - 1]):
            y_bad += 1
            c.ok(False, {"t": t, "why": "y_used differs from y(t-1) of the monitor", "y_used": r["y_used"],
                         "y": run.y[t - 1]})
            continue
        # b(t) = max(0, U K_hat(t) - Q(t)) / m(t), Q(t) of the logged demand tables (0 for an older AS)
        b = grant_limit(float(r["C_hat"]), float(r["U"]), d.T3, d.Ts, len(M), Q=float(r.get("Q") or 0),
                        T_m=float(r.get("T_m", d.T_m)))
        if r.get("b") is None or not _rel(b, float(r["b"])):
            c.ok(False, {"t": t, "why": "grant limit", "b": r.get("b"), "replay": b})
            continue
        if r.get("p_by"):
            p_by, _ = admission_b6(D, sigma, N, b)
            got = {int(i): list(v) for i, v in r["p_by"].items()}
            good = all(got.get(i) == p_by[i] for i in M)
            c.ok(good, None if good else {"t": t, "why": "B6 grants", "p_by": got, "replay": p_by})
        else:
            p = admission(D, sigma, N, b)
            good = list(r["p"]) == p
            c.ok(good, None if good else {"t": t, "why": "Algorithm 1", "p": r["p"], "replay": p})
        replayed += 1
        # the tables of the vessels: the D_i(t) they policed, the p(t) they received
        for i in M:
            st = (run.steps_log.get(i) or {}).get(t)
            if st is None:
                continue
            if st.get("have_d") and st.get("d") is not None and list(st["d"]) != D[i]:
                d_bad += 1
                c.ok(False, {"t": t, "i": i, "why": "D_i(t) of the AS differs from the table the regulator policed"})
            want = (r["p_by"].get(str(i)) if r.get("p_by") else r["p"])
            if st.get("have_p") and list(st.get("p") or []) != list(want):
                recv_bad += 1
                c.ok(False, {"t": t, "i": i, "why": "p(t) received differs from p(t) published"})
        # sigma_t and N(t) of the JMCS mode of step t
        if modes is not None and t in run.order_log:
            mo = modes.get(run.order_log[t].get("mode"))
            if mo is not None and (mo[0] != sigma or mo[1] != N):
                order_bad += 1
                c.ok(False, {"t": t, "why": "sigma/N differ from the JMCS mode %s" % run.order_log[t].get("mode")})
    c.extra.update(replayed=replayed, eq2_mismatch=eq2_bad, y_mismatch=y_bad, table_received_mismatch=recv_bad,
                   demand_mismatch=d_bad, order_mismatch=order_bad)
    if replayed == 0 and c.n == 0:
        c.skipped("no published decision in the steps")
    return c.result()


# ------------------------------------------------------------------------------------------------ bytes

def check_bytes(run):
    c = Check("bytes")
    if run.method not in POLICING_METHODS:
        return c.skipped("method %s does not police to d_il(t)" % run.method).result()
    vessel_edges = gs_edges = 0
    for i in run.vessels:
        st = run.steps_log.get(i)
        if not st:
            continue
        for t in run.steps():
            r = st.get(t)
            if r is None or not r.get("have_d") or r.get("d") is None:
                continue
            for l, (cost, dl) in enumerate(zip(r.get("cost") or [], r["d"]), start=1):
                vessel_edges += 1
                c.ok(float(cost) <= float(dl) + 1.0,
                     {"t": t, "i": i, "l": l, "side": "vessel", "cost": cost, "d": dl})
    for t in run.steps():
        g = run.dta.get(t)
        if g is None:
            continue
        vs = set(g.get("vessels") or [])
        for e in g.get("edges") or []:
            if e.get("i") not in vs or not (e.get("n") or 0):
                continue
            gs_edges += 1
            c.ok(float(e.get("r") or 0) <= float(e.get("d") or 0) + 1.0,
                 {"t": t, "i": e["i"], "l": e["l"], "side": "gs", "r": e.get("r"), "d": e.get("d")})
    c.extra.update(vessel_edges=vessel_edges, gs_edges=gs_edges)
    if c.n == 0:
        c.skipped("no step log and no DTA in the steps")
    return c.result()


# ------------------------------------------------------------------------------------------------ pcap

def read_pcap(path):
    """[(time s, IPv4 packet bytes)] of a pcap with LINKTYPE_RAW (101) or LINKTYPE_IPV4 (228)"""
    out = []
    with open(path, "rb") as f:
        gh = f.read(24)
        if len(gh) < 24:
            return out
        magic = struct.unpack("<I", gh[:4])[0]
        if magic in (0xa1b2c3d4, 0xa1b23c4d):
            e = "<"
        else:
            magic = struct.unpack(">I", gh[:4])[0]
            if magic not in (0xa1b2c3d4, 0xa1b23c4d):
                raise ValueError("%s: not a pcap file" % path)
            e = ">"
        frac = 1e-9 if magic == 0xa1b23c4d else 1e-6
        link = struct.unpack(e + "I", gh[20:24])[0]
        if link not in (101, 228):
            raise ValueError("%s: link type %d (need RAW 101 or IPV4 228)" % (path, link))
        while True:
            h = f.read(16)
            if len(h) < 16:
                break
            sec, sub, incl, orig = struct.unpack(e + "IIII", h)
            data = f.read(incl)
            if len(data) < incl:
                break
            out.append((sec + sub * frac, data))
    return out


def tcp_segments(packets):
    """{(src, sport, dst, dport): [(time, seq, flags, payload)]} of the TCP segments of IPv4 packets"""
    flows = {}
    for ts, p in packets:
        if len(p) < 20 or p[0] >> 4 != 4 or p[9] != 6:
            continue
        ihl = (p[0] & 0x0f) * 4
        tot = struct.unpack("!H", p[2:4])[0]
        p = p[:tot] if tot >= ihl else p
        if len(p) < ihl + 20:
            continue
        src, dst = p[12:16], p[16:20]
        sport, dport, seq = struct.unpack("!HHI", p[ihl:ihl + 8])
        doff = (p[ihl + 12] >> 4) * 4
        flags = p[ihl + 13]
        flows.setdefault((src, sport, dst, dport), []).append((ts, seq, flags, p[ihl + doff:]))
    return flows


def reassemble(segs):
    """in-order byte runs of one TCP direction: [(chunks, synced)], chunks = [(time of the segment, bytes)].
    The first run starts after the SYN (synced) or at the first segment; a gap in the sequence space (a
    segment missing from the capture) starts a new run that the parser resynchronizes."""
    runs = []
    out = []
    nxt = None
    pending = {}
    synced = False
    for ts, seq, flags, data in segs:
        if flags & 0x02:            # SYN
            nxt = (seq + 1) & 0xffffffff
            synced = True
            continue
        if not data:
            continue
        if nxt is None:
            nxt = seq
        off = (seq - nxt) & 0xffffffff
        if off >= 0x80000000:       # retransmission of bytes already in the stream
            back = (nxt - seq) & 0xffffffff
            if back >= len(data):
                continue
            data, seq, off = data[back:], nxt, 0
        if off:
            pending[seq] = (ts, data)
            continue
        out.append((ts, data))
        nxt = (nxt + len(data)) & 0xffffffff
        while nxt in pending:
            ts2, d2 = pending.pop(nxt)
            out.append((ts2, d2))
            nxt = (nxt + len(d2)) & 0xffffffff
    runs.append((out, synced))
    # segments after a gap: contiguous runs in sequence order, each resynchronized
    while pending:
        seq = min(pending, key=lambda q: (q - nxt) & 0xffffffff)
        out = []
        while seq in pending:
            ts2, d2 = pending.pop(seq)
            out.append((ts2, d2))
            seq = (seq + len(d2)) & 0xffffffff
        nxt = seq
        runs.append((out, False))
    return runs


def _varint(buf, k):
    v, mult = 0, 1
    for _ in range(4):
        if k >= len(buf):
            return None, k
        b = buf[k]
        k += 1
        v += (b & 0x7f) * mult
        if not b & 0x80:
            return v, k
        mult *= 128
    return None, k


def mqtt_publishes(chunks, synced=True, prefix=b"boat/"):
    """[(time, topic, qos, dup, payload)] of the PUBLISH packets of an MQTT byte stream; without the start
    of the connection the parser first searches a PUBLISH whose topic starts with prefix"""
    buf = b"".join(c for _, c in chunks)
    ends, pos = [], 0
    for ts, ch in chunks:
        pos += len(ch)
        ends.append((pos, ts))
    out = []
    k = 0
    if not synced:
        k = _resync(buf, 0, prefix)
    e = 0
    while k is not None and k < len(buf):
        typ = buf[k] >> 4
        rl, h = _varint(buf, k + 1)
        if rl is None or typ == 0 or h + rl > len(buf):
            break
        if typ == 3:
            qos = (buf[k] >> 1) & 3
            dup = bool(buf[k] & 0x08)
            tl = struct.unpack("!H", buf[h:h + 2])[0]
            topic = buf[h + 2:h + 2 + tl].decode("utf-8", "replace")
            body = h + 2 + tl + (2 if qos else 0)
            while e < len(ends) and ends[e][0] < h + rl:
                e += 1
            out.append((ends[min(e, len(ends) - 1)][1], topic, qos, dup, buf[body:h + rl]))
        k = h + rl
    return out


def _resync(buf, k, prefix):
    while True:
        j = buf.find(prefix, k)
        if j < 0:
            return None
        for start in range(max(0, j - 7), j - 3):
            if buf[start] >> 4 == 3:
                rl, h = _varint(buf, start + 1)
                if rl is not None and h + 2 == j and struct.unpack("!H", buf[h:h + 2])[0] < rl:
                    return start
        k = j + 1


def uplink_publishes(path):
    """[(time, vessel, topic name, qos, dup, t, s)] of the PUBLISH packets on boat/b<i>/tx/<topic> in a pcap"""
    out = []
    for key, segs in tcp_segments(read_pcap(path)).items():
        for chunks, synced in reassemble(segs):
            out += _uplink(chunks, synced)
    return out


def _uplink(chunks, synced):
    """the uplink PUBLISH packets (boat/b<i>/tx/<topic>, B2: tx2/<class>/<topic>) of one byte run"""
    out = []
    for ts, topic, qos, dup, payload in mqtt_publishes(chunks, synced):
        parts = topic.split("/")
        if len(parts) < 4 or parts[0] != "boat" or not parts[1].startswith("b") or parts[2] not in ("tx", "tx2"):
            continue
        if len(payload) < 6:
            continue
        t, s = struct.unpack("!IH", payload[:6])
        out.append((ts, int(parts[1][1:]), parts[-1], qos, dup, t, s))
    return out


def check_qos(run, pubs=None):
    c = Check("qos")
    if pubs is None:
        if not os.path.exists(run.pcap_path):
            return c.skipped("no emu/ap.pcap (run the experiment with \"pcap\": true)").result()
        pubs = uplink_publishes(run.pcap_path)
    idx = {n: k for k, n in enumerate(run.dtp.topics)}
    table = run.method in TABLE_METHODS
    hist = {}
    for ts, i, name, qos, dup, t, s in pubs:
        if not run.first <= t <= run.last or name not in idx:
            continue
        hist[qos] = hist.get(qos, 0) + 1
        if not table:
            c.ok(qos == 0, {"t": t, "i": i, "topic": name, "s": s, "qos": qos, "want": 0})
            continue
        p = run.table_of(i, t)
        if p is None:
            c.ok(False, {"t": t, "i": i, "topic": name, "s": s, "qos": qos, "why": "no table"})
            continue
        lv = int(p[idx[name]])
        c.ok(lv in (1, 2) and qos == 0, {"t": t, "i": i, "topic": name, "s": s, "qos": qos, "level": lv})
    c.extra["qos_histogram"] = {str(k): v for k, v in sorted(hist.items())}
    if c.n == 0:
        c.skipped("no uplink PUBLISH in the steps")
    return c.result()


# ------------------------------------------------------------------------------------------------ no p(t)

def check_no_p(run, pubs=None):
    c = Check("no_p")
    if run.method not in TABLE_METHODS:
        return c.skipped("method %s forwards without tables" % run.method).result()
    without = set()
    for t in run.steps():
        for i in run.vessels:
            sl = run.steps_log.get(i)
            if sl is not None:
                st = sl.get(t)
                if st is None or st.get("have_p"):
                    continue
                without.add((t, i))
                c.ok(sum(st.get("fwd") or []) == 0 and sum(st.get("late_fwd") or []) == 0,
                     {"t": t, "i": i, "side": "vessel", "fwd": st.get("fwd")})
            else:
                if run.table_of(i, t) is not None:
                    continue
                without.add((t, i))
            g = run.dta.get(t)
            if g is not None:
                n = sum((e.get("n") or 0) + (e.get("late") or 0) + (e.get("dup") or 0)
                        for e in g.get("edges") or [] if e.get("i") == i)
                c.ok(n == 0, {"t": t, "i": i, "side": "gs", "messages": n})
    if pubs is None and os.path.exists(run.pcap_path):
        pubs = uplink_publishes(run.pcap_path)
    for ts, i, name, qos, dup, t, s in pubs or []:
        if (t, i) in without:
            c.ok(False, {"t": t, "i": i, "side": "pcap", "topic": name, "s": s})
    c.extra["pairs_without_p"] = len(without)
    if c.n == 0 and not without:
        c.n = 1     # every vessel had p(t) in every step: nothing could cross without a table
        c.extra["note"] = "every (t, i) had p(t)"
    return c.result()


# ------------------------------------------------------------------------------------------------ logged

def check_logged(run, boundary_ms=5.0):
    c = Check("logged")
    names = run.dtp.topics
    src_bad = src_steps = 0
    have = False
    for i in run.vessels:
        st = run.steps_log.get(i)
        if st is None:
            continue
        have = True
        cnt, bad_reason = {}, 0
        for r in run.vlog.get(i) or []:
            if not run.first <= int(r.get("t", -1)) <= run.last:
                continue
            key = (int(r["t"]), int(r["l"]))
            cnt[key] = cnt.get(key, 0) + 1
            if r.get("reason") not in REASONS:
                bad_reason += 1
                c.ok(False, {"t": r["t"], "i": i, "l": r["l"], "why": "reason %r" % r.get("reason")})
        for t in run.steps():
            r = st.get(t)
            if r is None:
                continue
            for l in range(1, len(names) + 1):
                rx, fwd = int(r["rx"][l - 1]), int(r["fwd"][l - 1])
                held, drop = int(r["held"][l - 1]), int(r["drop"][l - 1])
                lines = cnt.get((t, l), 0)
                c.ok(rx - fwd == lines == held + drop,
                     {"t": t, "i": i, "l": l, "rx": rx, "fwd": fwd, "held": held, "drop": drop, "log": lines})
        # the source log against the received messages of the regulator
        src = run.src.get(i)
        if src:
            per, near = {}, set()
            for r in src:
                if r.get("topic") not in names or "t_us" not in r:
                    continue
                at = float(r["t_us"]) / 1e6
                tc = run.collect_step(at)
                l = names.index(r["topic"]) + 1
                per[(tc, l)] = per.get((tc, l), 0) + int(r.get("n", 1))
                # releases near a collection boundary may reach the regulator in the next step
                if at - run.v(tc - 1) < boundary_ms / 1e3 or run.v(tc) - at < boundary_ms / 1e3:
                    near.update({(tc, l), (tc + 1, l), (tc - 1, l)})
            for t in range(run.first + 1, run.last):
                r = st.get(t)
                if r is None:
                    continue
                for l in range(1, len(names) + 1):
                    if (t, l) in near:
                        continue
                    src_steps += 1
                    if per.get((t, l), 0) != int(r["rx"][l - 1]):
                        src_bad += 1
                        c.ok(False, {"t": t, "i": i, "l": l, "why": "source messages != received",
                                     "src": per.get((t, l), 0), "rx": r["rx"][l - 1]})
    c.extra.update(src_steps_checked=src_steps, src_mismatch=src_bad)
    if not have:
        c.skipped("no regulator step log (vms/v<i>/regsteps.jsonl)")
    return c.result()


# ------------------------------------------------------------------------------------------------ G(t)

def check_dt(run):
    c = Check("dt")
    d = run.dtp
    m_n = m_bad = 0
    lag_max = None
    lag_bad = 0
    for t in run.steps():
        g = run.dta.get(t)
        if g is None:
            continue
        a = run.as_log.get(t)
        if a is not None:
            m_n += 1
            same = sorted(g.get("vessels") or []) == sorted(a.get("M") or [])
            if not same:
                m_bad += 1
            c.ok(same, {"t": t, "why": "M(t)", "dta": g.get("vessels"), "as": a.get("M")})
        if g.get("emitted") is not None:
            lag = (float(g["emitted"]) - run.start(t + 1)) / d.Ts
            lag_max = lag if lag_max is None else max(lag_max, lag)
            good = lag <= 1.0
            if not good:
                lag_bad += 1
            c.ok(good, {"t": t, "why": "DTI lag %.3f steps" % lag})
    c.extra.update(M_agreement=(1.0 - m_bad / m_n) if m_n else None, M_steps=m_n, dti_lag_max_steps=lag_max,
                   dti_lag_over=lag_bad)
    # r_il against the ground truth of the AP (stepbytes.csv)
    if run.stepbytes is None:
        c.extra["ground_truth"] = "no emu/stepbytes.csv"
    else:
        errs, strict_bad, gt_n = [], 0, 0
        for t in run.steps():
            g = run.dta.get(t)
            if g is None:
                continue
            for i in g.get("vessels") or []:
                es = [e for e in g.get("edges") or [] if e.get("i") == i]
                x = sum(int(e.get("x") or 0) + 6 * int(e.get("n") or 0) for e in es)
                R = sum(float(e.get("r") or 0) for e in es)
                fr, by = run.stepbytes.get((t, i), (0, 0))
                fr1, by1 = run.stepbytes.get((t + 1, i), (0, 0))
                gt_n += 1
                if x > by:
                    strict_bad += 1
                c.ok(x <= by + by1, {"t": t, "i": i, "why": "content bytes at the GS above the IP bytes at the AP",
                                     "x": x, "ap_bytes": by, "ap_bytes_next": by1})
                B = by + d.h_W * fr
                if B > 0 and R > 0:
                    errs.append((R - B) / B)
        errs.sort()
        q = lambda p: errs[min(len(errs) - 1, int(p * (len(errs) - 1)))] if errs else None   # noqa: E731
        c.extra["r_vs_ground_truth"] = {"pairs": gt_n, "with_bytes": len(errs),
                                        "rel_err_mean": sum(errs) / len(errs) if errs else None,
                                        "rel_err_p05": q(0.05), "rel_err_p50": q(0.5), "rel_err_p95": q(0.95),
                                        "content_above_ap_bytes_same_step": strict_bad}
    if c.n == 0:
        c.skipped("no DTA in the steps")
    return c.result()


# ------------------------------------------------------------------------------------------------ main

REQUIRED = ("alg1", "bytes", "no_p", "logged", "dt")


def conformance(rd, first=None, last=None, warm=0, boundary_ms=5.0):
    run = Run(rd, first, last, warm)
    pubs = None
    pcap_err = None
    if os.path.exists(run.pcap_path):
        try:
            pubs = uplink_publishes(run.pcap_path)
        except (OSError, ValueError, struct.error) as e:
            pcap_err = str(e)
    checks = {"alg1": check_alg1(run), "bytes": check_bytes(run),
              "qos": check_qos(run, pubs) if pubs is not None else
              {"status": "skipped", "reason": pcap_err or "no emu/ap.pcap (run the experiment with \"pcap\": true)"},
              "no_p": check_no_p(run, pubs), "logged": check_logged(run, boundary_ms), "dt": check_dt(run)}
    failed = [k for k, v in checks.items() if v["status"] == "fail"]
    applicable = [k for k in REQUIRED if not (k in ("alg1", "no_p") and run.method not in TABLE_METHODS)
                  and not (k == "bytes" and run.method not in POLICING_METHODS)]
    missing = [k for k in applicable if checks[k]["status"] != "pass"]
    return {"run": rd, "method": run.method, "M": run.M, "steps": [run.first, run.last],
            "conforms": not failed and not missing, "failed": failed, "not_verified": [k for k in missing if k not in failed],
            "checks": checks}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir")
    ap.add_argument("--json", default=None, help="write the report here (default: RUN/conformance.json)")
    ap.add_argument("--first", type=int, default=None, help="first step (default: S0 + warm)")
    ap.add_argument("--last", type=int, default=None, help="last step (default: S0 + steps - 1)")
    ap.add_argument("--warm", type=int, default=0)
    ap.add_argument("--boundary-ms", type=float, default=5.0)
    a = ap.parse_args(argv)
    rep = conformance(a.run_dir, a.first, a.last, a.warm, a.boundary_ms)
    with open(a.json or os.path.join(a.run_dir, "conformance.json"), "w") as f:
        json.dump(rep, f, indent=1)
    for k, v in rep["checks"].items():
        print("%-7s %-7s %s" % (k, v["status"], "" if v["status"] == "skipped" and "reason" not in v
                                else (v.get("reason") or "n=%d fail=%d" % (v.get("n", 0), v.get("fail", 0)))))
    print("conforms: %s" % rep["conforms"])
    return 0 if rep["conforms"] else 1


if __name__ == "__main__":
    sys.exit(main())
