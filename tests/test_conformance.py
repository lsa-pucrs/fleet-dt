"""E0 conformance check (emu/experiments/conformance.py) on synthetic run directories: a run built with the
AS core of the paper stack, the regulator step logs and vessel logs, the source logs, the DTA, the bytes per
step of the AP and a pcap of the data bridges conforms; each injected defect fails the check that covers it."""
import json
import math
import os
import random
import struct
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "emu"))
sys.path.insert(0, os.path.join(ROOT, "emu", "experiments"))

import conformance as CF  # noqa: E402
import jmcs_stub  # noqa: E402
from fleetdt.as_model import DTP  # noqa: E402
from fleetdt.as_service import AS  # noqa: E402

TS, T1, T2 = 0.125, 0.020, 0.010
S0, STEPS, M = 1000, 24, 2
DTP_RAW = {"epoch": 0.0, "Ts": TS, "T1": T1, "T2": T2, "alpha": 0.125, "U": 0.8, "C0": 60000, "c_b": 1.06,
           "c_m": 180, "R_PHY": 24000000, "h_W": 150, "vessels": [1, 2],
           "topics": [{"name": "state", "rate": 8, "bound": 48}, {"name": "imu", "rate": 32, "bound": 48},
                      {"name": "video_1", "rate": 48, "bound": 1300, "jitter": 0},
                      {"name": "video_2", "rate": 64, "bound": 1300, "jitter": 0}],
           "order": ["state", "imu", "video_1", "video_2"], "needed": ["state", "imu"]}
NAMES = [t["name"] for t in DTP_RAW["topics"]]
# messages per step and content bytes of each topic (video_2 sends one message more than its demand allows
# in some steps, so the policing holds it)
MSGS = {"state": 1, "imu": 4, "video_1": 6, "video_2": 8}
SIZE = {"state": 48, "imu": 48, "video_1": 1200, "video_2": 1250}
NO_P = (S0 + 7, 2)        # vessel 2 does not receive p(t) in this step


def start(t):
    return t * TS


def cost(x):
    return 1.06 * (x + 6) + 180


def demand_of(d, name, size_scale=1.0):
    """d_il of the DTI program (SPEC.md Section 5) with the video bounds scaled for the vessel"""
    t = [x for x in d["topics"] if x["name"] == name][0]
    n = int(math.ceil(t["rate"] * d["Ts"] - 1e-9))
    s = int(round(t["bound"] * size_scale)) if name.startswith("video") else t["bound"]
    return int(math.ceil(d["c_b"] * (n * s + 6 * n) + d["c_m"] * n - 1e-9))


# ------------------------------------------------------------------------------------------------ pcap writer

def ip4(src, dst, payload):
    tot = 20 + len(payload)
    return struct.pack("!BBHHHBBH4s4s", 0x45, 0, tot, 0, 0, 64, 6, 0, bytes(src), bytes(dst)) + payload


def tcp(sport, dport, seq, flags, data=b""):
    return struct.pack("!HHIIBBHHH", sport, dport, seq, 0, 5 << 4, flags, 65535, 0, 0) + data


def mqtt_publish(topic, qos, pid, payload, dup=False):
    tb = topic.encode()
    var = struct.pack("!H", len(tb)) + tb + (struct.pack("!H", pid) if qos else b"")
    body = var + payload
    rl, n = b"", len(body)
    while True:
        b = n % 128
        n //= 128
        rl += bytes([b | (0x80 if n else 0)])
        if not n:
            break
    return bytes([0x30 | (qos << 1) | (0x08 if dup else 0)]) + rl + body


def write_pcap(path, pkts):
    with open(path, "wb") as f:
        f.write(struct.pack("<IHHiIII", 0xa1b23c4d, 2, 4, 0, 0, 65535, 101))
        for ts, p in pkts:
            ns = int(round(ts * 1e9))
            f.write(struct.pack("<IIII", ns // 1000000000, ns % 1000000000, len(p), len(p)) + p)


# ------------------------------------------------------------------------------------------------ synthetic run

def jl(path, recs):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        for r in recs:
            f.write(json.dumps(r) + "\n")


def build_run(rd, seed=3):
    rng = random.Random(seed)
    dtp = DTP.from_dict(DTP_RAW)
    for d in ("vms/gs", "vms/v1", "vms/v2", "emu"):
        os.makedirs(os.path.join(rd, d), exist_ok=True)
    json.dump(DTP_RAW, open(os.path.join(rd, "dtp.json"), "w"))
    json.dump({"status": "ok", "S0": S0, "steps": STEPS, "M": M, "method": "fleetdt"},
              open(os.path.join(rd, "result.json"), "w"))
    modes = jmcs_stub.build_modes(DTP_RAW)
    core = AS(dtp, epoch=0.0, now=lambda: 0.0)
    as_log, y_log, order_log, dta = [], [], [], []
    regsteps = {1: [], 2: []}
    vlog = {1: [], 2: []}
    src = {1: [], 2: []}
    stepbytes = []
    pkts = []
    seqs = {1: 1000, 2: 5000}
    pid = {1: 1, 2: 1}
    gs_ip, vip = [10, 10, 0, 1], {1: [10, 10, 0, 11], 2: [10, 10, 0, 12]}
    for i in (1, 2):   # the data bridges connect before the run
        pkts.append((start(S0) - 1.0, ip4(vip[i], gs_ip, tcp(1884, 40000 + i, seqs[i] - 1, 0x12))))
    for t in range(S0, S0 + STEPS):
        mode = "survey" if (t - S0) // 12 == 0 else "transit"
        order_log.append({"t": t, "mode": mode, "pub": start(t) + 0.002})
        core.on_order(modes[mode]["order"], modes[mode]["needed"], at=start(t) + 0.002)
        y = 50000.0 + rng.uniform(-8000, 8000)
        if t > S0:
            core.on_throughput(t - 1, y_prev, at=start(t) + 0.0005)
        D = {i: [demand_of(DTP_RAW, n, size_scale=(1.0 if i == 1 else 0.9)) for n in NAMES] for i in (1, 2)}
        for i in (1, 2):
            core.on_demand(i, t, D[i], at=start(t) + 0.003)
        dec = core.close_phase1(t)
        as_log.append(dict(core.last, published=dec is not None))
        y_log.append({"t": t, "y": y, "pub": start(t + 1) + 0.002})
        y_prev = y
        p = dec["p"]
        g = {"t": t, "L": 4, "topics": NAMES, "vessels": [1, 2], "edges": [], "y": y, "C_hat": dec["C_hat"],
             "K_hat": dec["K_hat"], "b": dec["b"], "U": dec["U"], "alpha": dec["alpha"],
             "needed": dec["needed"], "order": dec["order"], "emitted": start(t + 1) + 0.0625 + 0.004}
        for i in (1, 2):
            have_p = (t, i) != NO_P
            rec = {"t": t, "v": i, "mode": "paper", "have_d": True, "have_p": have_p, "p": p if have_p else None,
                   "d": D[i], "rx": [], "fwd": [], "held": [], "police": [], "drop": [], "x": [], "cost": [],
                   "late_fwd": [0] * 4, "carried": [0] * 4}
            frames = bytes_ = 0
            stream = b""
            for l, n in enumerate(NAMES, start=1):
                k = MSGS[n] + (1 if n.startswith("video") and t % 3 == 0 else 0)
                size = SIZE[n] if i == 1 else int(SIZE[n] * 0.9)
                # source releases in the middle of the collection window [v(t-1), v(t))
                at = start(t - 1) + T1 + T2 + 0.04 + 0.001 * l
                src[i].append({"topic": n, "out": t, "n": k, "bytes": k * size, "t_us": int(at * 1e6)})
                lv = p[l - 1] if have_p else 0
                fwd, c, x = 0, 0.0, 0
                for s in range(k):
                    if have_p and lv > 0 and c + cost(size) <= D[i][l - 1]:
                        fwd += 1
                        c += cost(size)
                        x += size + 6
                        payload = struct.pack("!IH", t, s) + bytes(size)
                        stream += mqtt_publish("boat/b%d/tx/%s" % (i, n), lv, pid[i], payload)
                        pid[i] += 1
                    else:
                        reason = "no_table" if not have_p else ("level0" if lv == 0 else "police")
                        vlog[i].append({"t": t, "v": i, "l": l, "topic": n, "seq": s, "bytes": size,
                                        "reason": reason, "at": t})
                rec["rx"].append(k)
                rec["fwd"].append(fwd)
                rec["held"].append(k - fwd)
                rec["police"].append(sum(1 for r in vlog[i] if r["t"] == t and r["l"] == l and r["reason"] == "police"))
                rec["drop"].append(0)
                rec["x"].append(x)
                rec["cost"].append(round(c))
                g["edges"].append({"i": i, "l": l, "d": D[i][l - 1], "p": lv if have_p else p[l - 1],
                                   "r": int(round(dtp.message_bytes(l, x - 6 * fwd, fwd))) if fwd else 0,
                                   "x": x - 6 * fwd, "n": fwd, "dup": 0, "late": 0, "fwd": fwd, "held": k - fwd})
            regsteps[i].append(rec)
            # TCP segments of the data bridge of vessel i in phase 3: MSS chunks, one retransmission and one
            # reordered pair
            seg = [stream[k:k + 1448] for k in range(0, len(stream), 1448)]
            ts = start(t) + T1 + T2 + 0.002
            sent = []
            for k, chunk in enumerate(seg):
                sent.append((ts + 0.0005 * k, seqs[i], chunk))
                seqs[i] += len(chunk)
            if len(sent) >= 3:
                sent[1], sent[2] = sent[2], sent[1]
                sent.append((sent[-1][0] + 0.001, sent[0][1], sent[0][2]))
            for ts_, sq, chunk in sent:
                pkts.append((ts_, ip4(vip[i], gs_ip, tcp(1884, 40000 + i, sq, 0x18, chunk))))
                frames += 1
                bytes_ += 40 + len(chunk)
            stepbytes.append((t - S0, i, frames, bytes_, frames + 3, bytes_ + 300))
        dta.append(g)
    jl(os.path.join(rd, "vms/gs/as.jsonl"), as_log)
    jl(os.path.join(rd, "vms/gs/dta.jsonl"), dta)
    jl(os.path.join(rd, "vms/gs/order.jsonl"), order_log)
    jl(os.path.join(rd, "throughput.jsonl"), y_log)
    for i in (1, 2):
        jl(os.path.join(rd, "vms/v%d/regsteps.jsonl" % i), regsteps[i])
        jl(os.path.join(rd, "vms/v%d/regulator.jsonl" % i), vlog[i])
        jl(os.path.join(rd, "vms/v%d/src.jsonl" % i), src[i])
    with open(os.path.join(rd, "emu", "stepbytes.csv"), "w") as f:
        f.write("k,i,frames3,bytes3,frames,bytes\n")
        for r in stepbytes:
            f.write("%d,%d,%d,%d,%d,%d\n" % r)
    pkts.sort(key=lambda z: z[0])
    write_pcap(os.path.join(rd, "emu", "ap.pcap"), pkts)
    return rd


@pytest.fixture
def run_dir(tmp_path):
    return build_run(str(tmp_path / "run"))


def rewrite(path, fn):
    recs = [json.loads(l) for l in open(path) if l.strip()]
    recs = fn(recs)
    jl(path, recs)


def status(rd):
    rep = CF.conformance(rd)
    return rep, {k: v["status"] for k, v in rep["checks"].items()}


# ------------------------------------------------------------------------------------------------ tests

def test_synthetic_run_conforms(run_dir):
    rep, st = status(run_dir)
    assert st == {"alg1": "pass", "bytes": "pass", "qos": "pass", "no_p": "pass", "logged": "pass", "dt": "pass"}, rep
    assert rep["conforms"]
    c = rep["checks"]
    assert c["alg1"]["replayed"] == STEPS and c["alg1"]["order_mismatch"] == 0
    assert c["no_p"]["pairs_without_p"] == 1
    assert set(c["qos"]["qos_histogram"]) <= {"1", "2"} and c["qos"]["n"] > STEPS
    assert c["dt"]["M_agreement"] == 1.0 and c["dt"]["dti_lag_max_steps"] < 1.0
    assert c["logged"]["src_steps_checked"] > 0 and c["logged"]["src_mismatch"] == 0
    # the pcap parser sees every forwarded message once (retransmission and reordering included)
    pubs = CF.uplink_publishes(os.path.join(run_dir, "emu", "ap.pcap"))
    fwd = sum(sum(json.loads(l)["fwd"]) for i in (1, 2) for l in open(os.path.join(run_dir, "vms/v%d/regsteps.jsonl" % i)))
    assert len(pubs) == fwd and len({(p[1], p[2], p[5], p[6]) for p in pubs}) == fwd


def test_main_writes_the_report(run_dir):
    assert CF.main([run_dir]) == 0
    rep = json.load(open(os.path.join(run_dir, "conformance.json")))
    assert rep["conforms"] and rep["steps"] == [S0, S0 + STEPS - 1]


def test_wrong_table_fails_alg1(run_dir):
    def f(recs):
        r = recs[5]
        r["p"] = [2, 2, 1, 1] if r["p"] != [2, 2, 1, 1] else [2, 2, 0, 0]
        return recs
    rewrite(os.path.join(run_dir, "vms/gs/as.jsonl"), f)
    rep, st = status(run_dir)
    assert st["alg1"] == "fail" and not rep["conforms"]
    assert any(e.get("why") == "Algorithm 1" for e in rep["checks"]["alg1"]["examples"])


def test_wrong_y_fails_eq2(run_dir):
    def f(recs):
        recs[9]["y"] += 1000.0
        return recs
    rewrite(os.path.join(run_dir, "throughput.jsonl"), f)
    rep, st = status(run_dir)
    assert st["alg1"] == "fail" and rep["checks"]["alg1"]["y_mismatch"] == 1


def test_forwarded_above_demand_fails_bytes(run_dir):
    def f(recs):
        recs[3]["cost"][3] = recs[3]["d"][3] + 50
        return recs
    rewrite(os.path.join(run_dir, "vms/v1/regsteps.jsonl"), f)

    def g(recs):
        e = [e for e in recs[4]["edges"] if e["i"] == 2 and e["l"] == 3][0]
        e["r"] = e["d"] + 200
        return recs
    rewrite(os.path.join(run_dir, "vms/gs/dta.jsonl"), g)
    rep, st = status(run_dir)
    assert st["bytes"] == "fail" and rep["checks"]["bytes"]["fail"] == 2
    assert {e["side"] for e in rep["checks"]["bytes"]["examples"]} == {"vessel", "gs"}


def test_wrong_qos_fails(run_dir, monkeypatch):
    pubs = CF.uplink_publishes(os.path.join(run_dir, "emu", "ap.pcap"))
    bad = [list(p) for p in pubs]
    k = next(n for n, p in enumerate(bad) if p[2] == "state")
    bad[k][3] = 1                                   # state is needed: level 2, so QoS 2
    monkeypatch.setattr(CF, "uplink_publishes", lambda path: [tuple(p) for p in bad])
    rep, st = status(run_dir)
    assert st["qos"] == "fail" and rep["checks"]["qos"]["fail"] == 1


def test_uplink_without_table_fails(run_dir):
    t, i = NO_P

    def f(recs):
        g = [r for r in recs if r["t"] == t][0]
        e = [e for e in g["edges"] if e["i"] == i and e["l"] == 1][0]
        e["n"], e["late"] = 1, 0
        return recs
    rewrite(os.path.join(run_dir, "vms/gs/dta.jsonl"), f)
    rep, st = status(run_dir)
    assert st["no_p"] == "fail" and rep["checks"]["no_p"]["examples"][0]["side"] == "gs"


def test_unlogged_message_fails(run_dir):
    def f(recs):
        return recs[1:]                              # one withheld message missing from the vessel log
    rewrite(os.path.join(run_dir, "vms/v1/regulator.jsonl"), f)
    rep, st = status(run_dir)
    assert st["logged"] == "fail"

    def g(recs):
        recs[10]["n"] += 1                           # the source published a message the regulator never saw
        return recs
    rewrite(os.path.join(run_dir, "vms/v2/src.jsonl"), g)
    rep, st = status(run_dir)
    assert rep["checks"]["logged"]["src_mismatch"] == 1


def test_dt_fidelity_failures(run_dir):
    def f(recs):
        recs[2]["vessels"] = [1]                     # M(t) of the DTA differs from the AS
        recs[6]["emitted"] = start(recs[6]["t"] + 2) + 0.01   # G(t) more than one step late
        return recs
    rewrite(os.path.join(run_dir, "vms/gs/dta.jsonl"), f)
    rep, st = status(run_dir)
    c = rep["checks"]["dt"]
    assert st["dt"] == "fail" and c["M_agreement"] < 1.0 and c["dti_lag_over"] == 1


def test_content_above_ground_truth_fails(run_dir):
    p = os.path.join(run_dir, "emu", "stepbytes.csv")
    rows = open(p).read().splitlines()
    k, i = rows[5].split(",")[:2]
    rows[5] = "%s,%s,1,10,1,10" % (k, i)
    nxt = [n for n, r in enumerate(rows) if r.startswith("%d,%s," % (int(k) + 1, i))][0]
    rows[nxt] = "%d,%s,1,10,1,10" % (int(k) + 1, i)
    open(p, "w").write("\n".join(rows) + "\n")
    rep, st = status(run_dir)
    assert st["dt"] == "fail" and rep["checks"]["dt"]["r_vs_ground_truth"]["content_above_ap_bytes_same_step"] >= 1


def test_no_pcap_skips_qos(run_dir):
    os.remove(os.path.join(run_dir, "emu", "ap.pcap"))
    rep, st = status(run_dir)
    assert st["qos"] == "skipped" and rep["conforms"]


def test_baseline_skips_table_checks(run_dir):
    r = json.load(open(os.path.join(run_dir, "result.json")))
    r["method"] = "B1"
    json.dump(r, open(os.path.join(run_dir, "result.json"), "w"))
    rep, st = status(run_dir)
    assert st["alg1"] == "skipped" and st["no_p"] == "skipped" and st["bytes"] == "skipped"
    assert st["qos"] == "fail"                       # B1 forwards with QoS 1; the synthetic run used the levels


def test_pcap_resync_without_syn():
    """a capture that starts in the middle of a connection: the parser finds the first PUBLISH"""
    msgs = [mqtt_publish("boat/b3/tx/imu", 1, 7 + k, struct.pack("!IH", 50, k) + bytes(40)) for k in range(5)]
    stream = b"\x40\x02\x00\x05" + msgs[0][9:] + b"".join(msgs[1:])   # a PUBACK and the tail of a PUBLISH first
    chunks = [(1.0, stream[:100]), (1.1, stream[100:])]
    pubs = [p for p in CF._uplink(chunks, synced=False)]
    assert [(p[1], p[2], p[3], p[5], p[6]) for p in pubs] == [(3, "imu", 1, 50, k) for k in range(1, 5)]
