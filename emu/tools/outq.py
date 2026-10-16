#!/usr/bin/env python3
"""Unsent bytes of the bridge sockets of a vessel per step (SIOCOUTQ through sock_diag) and the
saturation decision of the paper (Section VI, "Saturation throughput").

The vessel-side sampler is emu/tools/outq_sampler.c (fdt_outq, built in the VM); this module has the same
sampler in Python (fallback, about 10x the CPU of the C program) and the analysis of the logs.

  python3 emu/tools/outq.py sample --epoch E --ts 0.125 --t1 0.02 --t2 0.01 --port 1884 --port 1883 \\
          --out /tmp/fdt-run/outq.jsonl [--period-ms 1] [--steps N]
  python3 emu/tools/outq.py saturation RUN_DIR [--port 1884] [--min-cover 0.95] [--json]

Log format: one JSON line per step, see the header of outq_sampler.c.

Saturation rule (paper Section VI): step t is saturated when every vessel with forwarded bytes in phase 3
(bytes acknowledged during phase 3 of the data bridge, or unsent bytes at a sample of phase 3) holds
unsent bytes in the send queue of its data bridge at every sample of phase 3, at least one vessel has
forwarded bytes, and every such vessel has samples covering at least `min_cover` of phase 3 (a sampler
that was late or stopped cannot show a full phase).
"""
import argparse
import glob
import json
import os
import signal
import socket
import struct
import sys
import time

NETLINK_SOCK_DIAG = 4
SOCK_DIAG_BY_FAMILY = 20
NLM_F_REQUEST, NLM_F_DUMP = 0x1, 0x300
NLMSG_ERROR, NLMSG_DONE = 2, 3
INET_DIAG_INFO = 2
TCPI_BYTES_ACKED, TCPI_NOTSENT = 120, 144


class SockDiag:
    """dump of the established IPv4 TCP sockets: (local port) -> (wqueue sum, notsent sum, acked sum, n)"""

    def __init__(self):
        self.s = socket.socket(socket.AF_NETLINK, socket.SOCK_DGRAM, NETLINK_SOCK_DIAG)
        self.seq = 0

    def sample(self, ports):
        self.seq += 1
        # inet_diag_req_v2: family, protocol, ext, pad, states, sockid (48 bytes)
        req = struct.pack("=BBBBI", socket.AF_INET, socket.IPPROTO_TCP, 1 << (INET_DIAG_INFO - 1), 0, 1 << 1)
        req += b"\0" * 48
        hdr = struct.pack("=IHHII", 16 + len(req), SOCK_DIAG_BY_FAMILY, NLM_F_REQUEST | NLM_F_DUMP, self.seq, 0)
        self.s.send(hdr + req)
        out = {p: [0, 0, 0, 0] for p in ports}
        while True:
            data = self.s.recv(65536)
            off = 0
            while off + 16 <= len(data):
                ln, typ, _fl, seq, _pid = struct.unpack_from("=IHHII", data, off)
                if ln < 16:
                    return out
                if seq == self.seq:
                    if typ == NLMSG_DONE:
                        return out
                    if typ == NLMSG_ERROR:
                        raise OSError("sock_diag: netlink error")
                    m = off + 16
                    sport = struct.unpack_from("!H", data, m + 4)[0]
                    if sport in out:
                        wq = struct.unpack_from("=I", data, m + 60)[0]   # idiag_wqueue
                        o = out[sport]
                        o[0] += wq
                        o[3] += 1
                        a = m + 72                                         # rtattrs after inet_diag_msg
                        while a + 4 <= off + ln:
                            alen, atyp = struct.unpack_from("=HH", data, a)
                            if alen < 4:
                                break
                            if atyp == INET_DIAG_INFO:
                                pl = alen - 4
                                if pl >= TCPI_BYTES_ACKED + 8:
                                    o[2] += struct.unpack_from("=Q", data, a + 4 + TCPI_BYTES_ACKED)[0]
                                if pl >= TCPI_NOTSENT + 4:
                                    o[1] += struct.unpack_from("=I", data, a + 4 + TCPI_NOTSENT)[0]
                            a += (alen + 3) & ~3
                off += (ln + 3) & ~3


def steal_ms():
    try:
        with open("/proc/stat") as f:
            v = f.readline().split()
        return int(v[8]) * 1000.0 / os.sysconf("SC_CLK_TCK")
    except (OSError, IndexError, ValueError):
        return -1.0


def sample_loop(epoch, ts, t1, t2, ports, out, period=0.001, steps=None, start_step=None):
    """Python version of fdt_outq (same output)"""
    stop = []
    signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
    sd = SockDiag()
    nper = int(round(ts / period))
    t = start_step if start_step is not None else int((time.time() - epoch) // ts) + 1
    done = 0
    with open(out, "w", buffering=1 << 20) as f:
        f.write(json.dumps({"hdr": {"epoch": epoch, "ts_us": int(ts * 1e6), "t1_us": int(t1 * 1e6),
                                    "t2_us": int(t2 * 1e6), "period_us": int(period * 1e6), "first_step": t,
                                    "ports": list(ports), "source": "sock_diag idiag_wqueue (= SIOCOUTQ), python"}})
                + "\n")
        st0 = steal_ms()
        while not stop and (steps is None or done < steps):
            st = epoch + t * ts
            ps = {p: {"n": 0, "nz": 0, "q0": 0, "nsd0": 0, "qmin": 0, "qmax": 0, "acked": 0, "sock": 0, "n12": 0,
                      "nz12": 0, "_a0": None, "_a1": 0} for p in ports}
            late_max, miss = 0.0, 0
            for j in range(nper):
                if stop:
                    break
                at = st + j * period
                now = time.time()
                if now > at + period:
                    miss += 1
                    continue
                if now < at:
                    time.sleep(at - now)
                late_max = max(late_max, time.time() - at)
                s = sd.sample(ports)
                ph3 = j * period >= t1 + t2 - 1e-9
                for p in ports:
                    q, nsd, acked, n = s[p]
                    r = ps[p]
                    r["sock"] = max(r["sock"], n)
                    if not ph3:
                        r["n12"] += 1
                        r["nz12"] += q > 0
                        continue
                    if r["n"] == 0:
                        r["q0"], r["nsd0"], r["qmin"], r["qmax"] = q, nsd, q, q
                    r["n"] += 1
                    r["nz"] += q > 0
                    r["qmin"], r["qmax"] = min(r["qmin"], q), max(r["qmax"], q)
                    if r["_a0"] is None:
                        r["_a0"] = acked
                    r["_a1"] = acked
            if stop:
                break
            for r in ps.values():
                r["acked"] = max(0, r.pop("_a1") - (r.pop("_a0") or 0))
            st1 = steal_ms()
            f.write(json.dumps({"t": t, "ports": {str(p): ps[p] for p in ports},
                                "steal_ms": round(st1 - st0, 1) if st0 >= 0 and st1 >= 0 else -1.0,
                                "late_us": int(late_max * 1e6), "miss": miss}) + "\n")
            st0 = st1
            done += 1
            t += 1


# ------------------------------------------------------------------------------------------------
# analysis

def load(path):
    """(header, {t: record}) of one sampler log"""
    hdr, steps = {}, {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except ValueError:
                continue            # a line cut by the end of the run
            if "hdr" in r:
                hdr = r["hdr"]
            elif "t" in r:
                steps[int(r["t"])] = r
    return hdr, steps


def vessel_logs(run_dir, name="outq.jsonl"):
    """{vessel index: path} of RUN/vms/v<i>/<name>"""
    out = {}
    for p in glob.glob(os.path.join(run_dir, "vms", "v*", name)):
        h = os.path.basename(os.path.dirname(p))
        if h[1:].isdigit():
            out[int(h[1:])] = p
    return out


def vessel_state(rec, port="1884", min_cover=0.95, expected=None):
    """(forwarded, saturated, covered) of one vessel in one step from its sampler record"""
    if rec is None:
        return False, False, False
    p = rec["ports"].get(str(port))
    if not p:
        return False, False, False
    n = int(p["n"])
    exp = expected if expected else n
    covered = exp > 0 and n >= min_cover * exp
    forwarded = int(p.get("acked", 0)) > 0 or int(p["nz"]) > 0
    saturated = covered and n > 0 and int(p["nz"]) == n
    return forwarded, saturated, covered


def saturation(logs, port="1884", min_cover=0.95):
    """{t: {"saturated": bool, "vessels": [i with forwarded bytes], "sat_vessels": [...], "uncovered": [...]}}
    over the steps that every log covers; logs = {vessel: (hdr, steps)}"""
    if not logs:
        return {}
    ts = set.intersection(*[set(s.keys()) for _, s in logs.values()])
    out = {}
    for t in sorted(ts):
        fwd, sat, unc = [], [], []
        for i, (hdr, steps) in sorted(logs.items()):
            exp = None
            if hdr.get("ts_us") and hdr.get("period_us"):
                exp = int((hdr["ts_us"] - hdr["t1_us"] - hdr["t2_us"]) // hdr["period_us"])
            f, s, c = vessel_state(steps.get(t), port, min_cover, exp)
            if f:
                fwd.append(i)
                if s:
                    sat.append(i)
                if not c:
                    unc.append(i)
        out[t] = {"saturated": bool(fwd) and len(sat) == len(fwd), "vessels": fwd, "sat_vessels": sat,
                  "uncovered": unc}
    return out


def buffers_empty(logs, port="1884"):
    """{t: True when every vessel's data bridge held no unsent byte at the first sample of phase 3}"""
    if not logs:
        return {}
    ts = set.intersection(*[set(s.keys()) for _, s in logs.values()])
    out = {}
    for t in sorted(ts):
        ok = True
        for _, steps in logs.values():
            p = steps[t]["ports"].get(str(port))
            if p is None or int(p["n"]) == 0 or int(p["q0"]) > 0:
                ok = False
        out[t] = ok
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sample")
    s.add_argument("--epoch", type=float, required=True)
    s.add_argument("--ts", type=float, default=0.125)
    s.add_argument("--t1", type=float, default=0.020)
    s.add_argument("--t2", type=float, default=0.010)
    s.add_argument("--port", type=int, action="append", required=True)
    s.add_argument("--out", required=True)
    s.add_argument("--period-ms", type=float, default=1.0)
    s.add_argument("--steps", type=int, default=None)
    s.add_argument("--start-step", type=int, default=None)
    z = sub.add_parser("saturation")
    z.add_argument("run_dir")
    z.add_argument("--port", default="1884")
    z.add_argument("--min-cover", type=float, default=0.95)
    z.add_argument("--json", action="store_true")
    a = ap.parse_args()
    if a.cmd == "sample":
        sample_loop(a.epoch, a.ts, a.t1, a.t2, a.port, a.out, a.period_ms / 1000.0, a.steps, a.start_step)
        return
    logs = {i: load(p) for i, p in vessel_logs(a.run_dir).items()}
    if not logs:
        sys.exit("no RUN/vms/v*/outq.jsonl")
    sat = saturation(logs, a.port, a.min_cover)
    if a.json:
        print(json.dumps({str(t): v for t, v in sat.items()}))
    else:
        n = len(sat)
        k = sum(v["saturated"] for v in sat.values())
        print("vessels %s, steps %d, saturated %d (%.3f)" % (sorted(logs), n, k, k / n if n else 0.0))


if __name__ == "__main__":
    main()
