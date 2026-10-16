"""Tests of the emulation additions of the paper design v2 (items I8, I9, Q, E2):
paper scenarios (emu/scenarios/gen.py), emulator options (fdt-emu --stepBytes, --selftestSet, --shiftS,
--phaseLag, --abortLagUs, --probeLoss, channel models), the subset replays of C(t)
(emu/replay_subsets.py), the vessel sampler of SIOCOUTQ (emu/tools/outq*.{c,py}), the clock and CPU
tooling of the guests (emu/vm/*) and the qualification analysis (emu/experiments/qualify.py).

The emulator tests need the fdt-emu binary: FDT_EMU_BIN, or FDT_NS3 (ns-3 tree with the build), else they
are skipped. The C tests need a C compiler (cc)."""
import cmath
import csv
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import threading
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in ("emu/scenarios", "emu/tools", "emu/vm", "emu/vm/guest", "emu/experiments", "emu"):
    sys.path.insert(0, os.path.join(ROOT, p))

import gen  # noqa: E402
import outq  # noqa: E402
import fdt_clock_schedule as CS  # noqa: E402
import chrony_conf  # noqa: E402
import cpu_profile  # noqa: E402
import qualify  # noqa: E402
import replay_subsets as RS  # noqa: E402


def emu_bin():
    b = os.environ.get("FDT_EMU_BIN")
    if not b:
        ns3 = os.path.expanduser(os.environ.get("FDT_NS3", "~/fleetdt-deploy/ns3/ns-3.48"))
        b = os.path.join(ns3, "build/scratch/fdt-emu/ns3.48-fdt-emu-optimized")
    return b if os.path.exists(b) else None


EMU = emu_bin()
needs_emu = pytest.mark.skipif(EMU is None, reason="fdt-emu binary not found (FDT_EMU_BIN / FDT_NS3)")
CC = shutil.which("cc") or shutil.which("gcc")
needs_cc = pytest.mark.skipif(CC is None, reason="no C compiler")


# ------------------------------------------------------------------------------------------------
# scenarios

def test_old_scenarios_unchanged():
    for M in (2, 4, 8):
        for sid in gen.SCENARIOS:
            with open(os.path.join(ROOT, "emu", "scenarios", "%s-m%d.json" % (sid, M))) as f:
                assert json.load(f) == json.loads(json.dumps(gen.make(sid, M, 1, 180.0))), (sid, M)


def test_paper_files_match_generator():
    for M in (2, 4):
        for sid in gen.PAPER_SCENARIOS:
            p = os.path.join(ROOT, "emu", "scenarios", gen.paper_file(sid, M))
            with open(p) as f:
                assert json.load(f) == json.loads(json.dumps(gen.make_paper(sid, M, 1, 180.0))), p
        for sid in gen.QUEUE_SCENARIOS:
            for q in gen.QUEUE_VARIANTS:
                p = os.path.join(ROOT, "emu", "scenarios", gen.paper_file(sid, M, q))
                with open(p) as f:
                    assert json.load(f) == json.loads(json.dumps(gen.make_paper(sid, M, 1, 180.0, q))), p


@pytest.mark.parametrize("M", [2, 4])
def test_paper_scenarios_parameters(M):
    P = gen.PAPER_PARAMS
    s = {sid: gen.make_paper(sid, M, 11) for sid in gen.PAPER_SCENARIOS}
    for sid, sc in s.items():
        assert sc["rate"]["manager"] == "constant" and sc["rate"]["data"] == "ErpOfdmRate24Mbps"
        assert sc["phy"] == ("spectrum" if sid in ("S3", "S8") else "yans")
        assert sc["M"] == M and sc["scenario_id"] == sid and sc["mac_queue"]["packets"] == 100
        assert "standard" not in sc        # 11g ERP-OFDM everywhere (S5 no longer 11a + Minstrel)
    # S2 / S9: same schedule, beacons at DSSS 1 Mb/s, S9 at -82 dBm with Nakagami fading
    assert s["S2"]["external_bss"]["schedule"] == s["S9"]["external_bss"]["schedule"]
    assert s["S2"]["external_bss"]["basic_modes"][0] == "DsssRate1Mbps"
    for t0, t1, rate, size in s["S2"]["external_bss"]["schedule"]:
        assert 1.0 <= t1 - t0 <= 6.0 + 1e-6 or t1 == 180.0
        assert 2e6 <= rate <= 8e6 and size == 1400
    l9 = {(a, b): v for a, b, v in s["S9"]["channel"]["loss"]}
    assert all(P["tx_dbm"] - l9[(x, f)] == -82.0 for x in ("ap2", "sta2") for f in ["gs"] + ["v%d" % i for i in range(1, M + 1)])
    assert len(s["S9"]["channel"]["fading"]) == 2 * (M + 1)
    # S3 / S8: same schedule, injector above / below the energy-detection threshold
    assert s["S3"]["jammer"]["schedule"] == s["S8"]["jammer"]["schedule"]
    for sid, rx in (("S3", -50.0), ("S8", -72.0)):
        jl = [v for a, b, v in s[sid]["channel"]["loss"] if a == "jam"]
        assert len(jl) == M + 1 and all(P["tx_dbm"] - v == rx for v in jl)
    assert P["s8_injector_rx_dbm"] < P["cca_ed_dbm"] < P["s3_injector_rx_dbm"]
    apv = [v for a, b, v in s["S8"]["channel"]["loss"] if a == "gs"]
    assert apv == [P["s8_fleet_loss_db"]] * M
    for t0, t1, per, duty, pw in s["S3"]["jammer"]["schedule"]:
        assert 500 <= per <= 5000 and 0.05 <= duty <= 0.3 and pw == 20
    # S4 hidden pairs
    assert [p[:2] for p in s["S4"]["channel"]["loss"]] == ([["v1", "v2"]] if M == 2 else [["v1", "v2"], ["v3", "v4"]])
    # S5 / S7: same trajectories; S7 motion within the drafted ranges
    assert s["S5"]["channel"]["positions"] == s["S7"]["channel"]["positions"]
    assert s["S5"]["channel"]["model"] == "tworay" and s["S5"]["channel"]["nakagami"]["m0"] == 5
    ch7 = s["S7"]["channel"]
    assert ch7["model"] == "sea" and ch7["polarization"] == "V" and ch7["eps_r"] == 70
    for v, m in ch7["motion"].items():
        assert m["heave_m"] == 0.3 and m["roll_deg"] == 10 and 2.0 <= m["period_s"] <= 8.0
    # S6 walk: bounds, +-1 dB per second, preamble limit lowered
    lo, hi = P["s6_range_db"]
    for a, b, v in s["S6"]["channel"]["walk"]["pairs"]:
        assert a == "gs" and len(v) >= 182
        assert all(lo - 1e-9 <= x <= hi + 1e-9 for x in v)
        assert all(abs(abs(v[k + 1] - v[k]) - 1.0) < 1e-6 or abs(v[k + 1] - v[k]) < 1.0 for k in range(len(v) - 1))
    assert s["S6"]["preamble_min_rssi_dbm"] == -90.0


def test_queue_variants():
    q1 = gen.make_paper("S1", 4, 3, queue="q1000")
    assert q1["fleet_queue"] == {"packets": 1000, "max_delay_ms": 60000}
    c = gen.make_paper("S3", 4, 3, queue="codel")
    assert c["aqm"]["type"] == "codel" and c["aqm"]["nodes"] == "stations" and c["station_queue"]["packets"] == 4
    base = gen.make_paper("S1", 4, 3)
    q100 = gen.make_paper("S1", 4, 3, queue="q100")
    assert {k: v for k, v in q100.items() if k not in ("name", "queue_variant")} == \
        {k: v for k, v in base.items() if k not in ("name", "queue_variant")}
    with pytest.raises(ValueError):
        gen.make_paper("S2", 4, 3, queue="fifo")


# ------------------------------------------------------------------------------------------------
# emulator

def run_emu(args, cwd=None, check=True):
    r = subprocess.run([EMU] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=cwd)
    if check and r.returncode != 0:
        raise AssertionError(r.stdout[-3000:])
    return r


def read_csv(p):
    with open(p) as f:
        return list(csv.DictReader(f))


@needs_emu
def test_emu_walk_probe(tmp_path):
    sc = gen.make_paper("S6", 2, 5)
    sp = tmp_path / "s6.json"
    sp.write_text(json.dumps(sc))
    run_emu(["--scenario=%s" % sp, "--outdir=%s" % tmp_path, "--vms=0", "--rt=none", "--monitor=none", "--gt=0",
             "--duration=8", "--shiftS=1", "--probeLoss=gs-v1,v1-v2:250"])
    w = sc["channel"]["walk"]["pairs"][0][2]
    for r in read_csv(tmp_path / "lossprobe.csv"):
        t = float(r["t_s"])
        if r["b"] == "v2":
            assert float(r["rx_dbm"]) == pytest.approx(-50.0, abs=1e-6)
            continue
        i = int(t)
        exp = w[i] + (t - i) * (w[i + 1] - w[i])
        assert float(r["rx_dbm"]) == pytest.approx(20.0 - exp, abs=2e-3), t


def wp_state(wp, t):
    """position and velocity of a waypoint trajectory (times relative to the scenario start)"""
    if t <= wp[0][0]:
        return wp[0][1:], (0.0, 0.0)
    for a, b in zip(wp, wp[1:]):
        if a[0] <= t < b[0]:
            f = (t - a[0]) / (b[0] - a[0])
            pos = [a[k] + f * (b[k] - a[k]) for k in (1, 2, 3)]
            v = ((b[1] - a[1]) / (b[0] - a[0]), (b[2] - a[2]) / (b[0] - a[0]))
            return pos, v
    return wp[-1][1:], (0.0, 0.0)


def sea_ref_dbm(ch, pa, pb, ma, mb, ha_, hb_, t, txp=20.0):
    """independent implementation of the sea two-ray model (fdt-emu SeaTwoRayLoss)"""
    from scipy.special import i0
    lam = 299792458.0 / ch["frequency_hz"]
    eps = complex(ch["eps_r"], -60.0 * lam * ch["sigma_s_m"])

    def height(p, m):
        return p[2] + (m["heave_m"] * math.sin(2 * math.pi * t / m["period_s"] + m["phase_rad"]) if m else 0.0)

    def roll(m):
        return math.radians(m["roll_deg"]) * math.sin(2 * math.pi * t / m["period_s"] + m["roll_phase_rad"]) if m else 0.0

    def pattern(m, head, e, az):
        r = roll(m)
        c = -math.cos(e) * math.sin(az - head) * math.sin(r) + math.sin(e) * math.cos(r)
        c = max(-1.0, min(1.0, c))
        s = math.sqrt(1 - c * c)
        return 0.0 if s < 1e-9 else abs(math.cos(math.pi / 2 * c)) / s

    ha, hb = max(0.05, height(pa, ma)), max(0.05, height(pb, mb))
    dx, dy = pb[0] - pa[0], pb[1] - pa[1]
    d = max(ch.get("min_distance_m", 1), math.hypot(dx, dy))
    azab, azba = math.atan2(dy, dx), math.atan2(-dy, -dx)
    r1, r2 = math.hypot(d, ha - hb), math.hypot(d, ha + hb)
    psi, eab = math.atan2(ha + hb, d), math.atan2(hb - ha, d)
    sp, cp = math.sin(psi), math.cos(psi)
    root = cmath.sqrt(eps - cp * cp)
    G = (eps * sp - root) / (eps * sp + root)
    g = 2 * math.pi * ch["rough_sigma_m"] * sp / lam
    rho = math.exp(-2 * g * g) * i0(2 * g * g)
    k = 2 * math.pi / lam
    e1 = pattern(ma, ha_, eab, azab) * pattern(mb, hb_, -eab, azba) / r1 * cmath.exp(-1j * k * r1)
    e2 = G * rho * pattern(ma, ha_, -psi, azab) * pattern(mb, hb_, -psi, azba) / r2 * cmath.exp(-1j * k * r2)
    amp = abs(e1 + e2) * lam / (4 * math.pi)
    return txp + 20 * math.log10(amp)


@needs_emu
def test_emu_sea_model_against_reference(tmp_path):
    pytest.importorskip("scipy")
    sc = gen.make_paper("S7", 2, 9)
    sp = tmp_path / "s7.json"
    sp.write_text(json.dumps(sc))
    shift = 1.0
    run_emu(["--scenario=%s" % sp, "--outdir=%s" % tmp_path, "--vms=0", "--rt=none", "--monitor=none", "--gt=0",
             "--duration=%g" % (shift + 12), "--shiftS=%g" % shift, "--probeLoss=gs-v1,v1-v2,gs-v2:37"])
    ch = sc["channel"]
    pos, mo = ch["positions"], ch["motion"]
    heading = {}
    bad, n, span = 0, 0, {}
    for r in read_csv(tmp_path / "lossprobe.csv"):
        t = float(r["t_s"])
        st = {}
        for name in (r["a"], r["b"]):
            if name == "gs":
                st[name] = (pos["gs"], 0.0, None)
                continue
            p, v = wp_state(pos[name], t)
            if v[0] * v[0] + v[1] * v[1] > 1e-4:
                heading[name] = math.atan2(v[1], v[0])
            st[name] = (p, heading.get(name, 0.0), mo[name])
        (pa, ha, ma), (pb, hb, mb) = st[r["a"]], st[r["b"]]
        ref = sea_ref_dbm(ch, pa, pb, ma, mb, ha, hb, t)
        got = float(r["rx_dbm"])
        n += 1
        bad += abs(ref - got) > 0.02
        span.setdefault(r["a"] + r["b"], []).append(got)
    assert n > 900 and bad <= n * 0.01, (bad, n)
    # the heave makes the received power move by more than a dB within the 12 s
    assert max(max(v) - min(v) for v in span.values()) > 1.0


@needs_emu
def test_emu_stepbytes_selftestset(tmp_path):
    sc = gen.make_paper("S1", 2, 4)
    sp = tmp_path / "s1.json"
    sp.write_text(json.dumps(sc))
    run_emu(["--scenario=%s" % sp, "--outdir=%s" % tmp_path, "--vms=0", "--rt=none", "--monitor=none",
             "--selftest=30000000,1500", "--selftestSet=1", "--shiftS=2", "--duration=4.5",
             "--stepBytes=125000:30000"])
    rows = read_csv(tmp_path / "stepbytes.csv")
    assert rows and all(r["i"] == "1" for r in rows)
    ks = sorted(int(r["k"]) for r in rows)
    assert ks[0] == 0 and ks[-1] == 19
    for r in rows:
        fr3, by3, fr, by = (int(r[k]) for k in ("frames3", "bytes3", "frames", "bytes"))
        assert by3 == 1500 * fr3 and by == 1500 * fr and fr3 <= fr
        if 0 < int(r["k"]) < 19:
            # one saturated vessel at 24 Mb/s: about 1700 IP packets per second
            assert 150 <= fr <= 260 and 0.70 <= fr3 / fr <= 0.82
    # the same count from the ground truth (unique UIDs of data frames from v1 received by the AP)
    import gt as G
    hdr, rec = G.load(str(tmp_path / "gt.bin"))
    ok = rec[(rec["node"] == 0) & (rec["ev"] == 1) & (rec["kind"] == 0) & (rec["src"] == 1) & (rec["ip_len"] > 0)]
    seen, per = set(), {}
    for x in ok:
        if int(x["uid"]) in seen:
            continue
        seen.add(int(x["uid"]))
        s = int(x["t_ns"]) + int(x["dur_ns"]) - 2_000_000_000
        if s < 0:
            continue
        k = s // 125_000_000
        per[k] = per.get(k, 0) + 1
    for r in rows:
        assert abs(per.get(int(r["k"]), 0) - int(r["frames"])) <= 1


@needs_emu
def test_emu_phase_lag_and_abort(tmp_path):
    sc = gen.make_paper("S1", 2, 4)
    sp = tmp_path / "s1.json"
    sp.write_text(json.dumps(sc))
    d1 = tmp_path / "a"
    run_emu(["--scenario=%s" % sp, "--outdir=%s" % d1, "--vms=0", "--rt=BestEffort", "--monitor=none", "--gt=0",
             "--selftest=2000000,1000", "--shiftS=0.3", "--duration=1.6", "--phaseLag=125000:20000:10000"])
    rows = read_csv(d1 / "steplag.csv")
    assert len(rows) >= 8 and rows[0]["k"] == "0"
    for r in rows[:-1]:
        assert float(r["lag_p3_us"]) >= 0 and float(r["lag_end_us"]) >= 0 and int(r["n"]) > 100
        assert int(r["catchup"]) in (0, 1)
    s = json.load(open(d1 / "summary.json"))
    assert s["phase_lag"]["on"] and not s["phase_lag"]["aborted"]
    d2 = tmp_path / "b"
    r = run_emu(["--scenario=%s" % sp, "--outdir=%s" % d2, "--vms=0", "--rt=BestEffort", "--monitor=none", "--gt=0",
                 "--shiftS=0.3", "--duration=1.6", "--phaseLag=125000:20000:10000", "--abortLagUs=0.001"], check=False)
    assert r.returncode == 3 and "ABORT lag" in r.stdout
    s = json.load(open(d2 / "summary.json"))
    assert s["phase_lag"]["aborted"] and s["phase_lag"]["abort_step"] == 0


@needs_emu
def test_emu_scenario_features(tmp_path):
    """S2: external beacons at 1 Mb/s; codel: queue discs on the stations; q1000: queues of the fleet"""
    out = {}
    for sid, q in (("S2", None), ("S1", "codel"), ("S1", "q1000")):
        sc = gen.make_paper(sid, 2, 6, queue=q)
        sp = tmp_path / ("%s-%s.json" % (sid, q))
        sp.write_text(json.dumps(sc))
        od = tmp_path / ("%s-%s" % (sid, q))
        run_emu(["--scenario=%s" % sp, "--outdir=%s" % od, "--vms=0", "--rt=none", "--monitor=none",
                 "--selftest=20000000,1500", "--duration=3"])
        out[(sid, q)] = json.load(open(od / "summary.json"))
    c = out[("S2", None)]["conf"]
    assert c["ap2"]["basic0"] == "DsssRate1Mbps" and c["gs"]["basic0"] == "ErpOfdmRate6Mbps"
    assert all(v["rts_threshold"] >= 65535 for v in c.values())
    c = out[("S1", "codel")]
    assert c["conf"]["v1"]["qdisc"] == "mq+codel" and c["conf"]["v1"]["be_queue_p"] == 4
    assert c["conf"]["gs"]["qdisc"] is None and c["conf"]["gs"]["be_queue_p"] == 100
    assert c["qdisc"]["v1"]["dropped"] > 0 and c["qdisc"]["v1"]["sent"] > 0
    c = out[("S1", "q1000")]["conf"]
    assert c["gs"]["be_queue_p"] == 1000 and c["v2"]["be_queue_p"] == 1000 and c["v2"]["be_queue_delay_ms"] == 60000


# ------------------------------------------------------------------------------------------------
# subset replays

def test_subsets_and_combine():
    s = RS.subsets(4)
    assert len(s) == 15 and s[0] == (1,) and s[-1] == (1, 2, 3, 4) and len(set(s)) == 15
    sb = {(0, 1): (10, 15000, 12, 18000), (0, 2): (8, 12000, 9, 13500), (1, 1): (0, 0, 0, 0)}
    c1 = RS.c_subset(sb, (1,), 2, 0.125, 0.02, 0.01, 150)
    assert c1[0] == pytest.approx((15000 + 1500) * 0.125 / 0.095) and c1[1] == 0.0
    c12 = RS.c_subset(sb, (1, 2), 2, 0.125, 0.02, 0.01, 150)
    comb = RS.combine({(1,): c1, (2,): RS.c_subset(sb, (2,), 2, 0.125, 0.02, 0.01, 150), (1, 2): c12}, 2)
    assert comb[0][1] == (2,) and comb[0][0] == pytest.approx((12000 + 1200) * 0.125 / 0.095)


def write_outq(path, steps, sat, period_us=1000):
    """synthetic sampler log: sat(t) -> (n, nz, acked)"""
    with open(path, "w") as f:
        f.write(json.dumps({"hdr": {"epoch": 0, "ts_us": 125000, "t1_us": 20000, "t2_us": 10000,
                                    "period_us": period_us, "first_step": steps[0], "ports": [1884, 1883]}}) + "\n")
        for t in steps:
            n, nz, acked = sat(t)
            f.write(json.dumps({"t": t, "ports": {"1884": {"n": n, "nz": nz, "q0": 100 if nz else 0, "nsd0": 0,
                                                           "qmin": 0, "qmax": 0, "acked": acked, "sock": 1,
                                                           "n12": 30, "nz12": 0},
                                                  "1883": {"n": n, "nz": 0, "q0": 0, "nsd0": 0, "qmin": 0, "qmax": 0,
                                                           "acked": 10, "sock": 1, "n12": 30, "nz12": 0}},
                                "steal_ms": 0.5, "late_us": 30, "miss": 0}) + "\n")


def test_outq_saturation_rule(tmp_path):
    steps = list(range(100, 110))
    write_outq(tmp_path / "v1.jsonl", steps, lambda t: (95, 95, 5000))
    # v2: idle in t = 101 (no forwarded bytes), backlog lost in t = 102, short phase in t = 103
    def v2(t):
        return {101: (95, 0, 0), 102: (95, 60, 4000), 103: (50, 50, 3000)}.get(t, (95, 95, 6000))
    write_outq(tmp_path / "v2.jsonl", steps, v2)
    logs = {1: outq.load(str(tmp_path / "v1.jsonl")), 2: outq.load(str(tmp_path / "v2.jsonl"))}
    sat = outq.saturation(logs)
    assert sat[100]["saturated"] and sat[100]["vessels"] == [1, 2]
    assert sat[101]["saturated"] and sat[101]["vessels"] == [1]
    assert not sat[102]["saturated"] and sat[102]["sat_vessels"] == [1]
    assert not sat[103]["saturated"] and sat[103]["uncovered"] == [2]
    be = outq.buffers_empty(logs)
    assert be[101] is False   # v1 holds bytes at the start of phase 3


@needs_emu
def test_replay_tool_short_run(tmp_path):
    rd = tmp_path / "run"
    (rd / "vms" / "v1").mkdir(parents=True)
    (rd / "vms" / "v2").mkdir(parents=True)
    sc = gen.make_paper("S3", 2, 8, duration=40)
    (rd / "scenario.json").write_text(json.dumps(sc))
    shutil.copy(os.path.join(ROOT, "dtp.json"), rd / "dtp.json")
    (rd / "result.json").write_text(json.dumps({"S0": 5000, "steps": 24}))
    steps = list(range(5000, 5024))
    write_outq(rd / "vms" / "v1" / "outq.jsonl", steps, lambda t: (95, 95, 5000))
    write_outq(rd / "vms" / "v2" / "outq.jsonl", steps, lambda t: (95, 95 if t % 2 else 10, 5000))
    r = subprocess.run([sys.executable, os.path.join(ROOT, "emu", "replay_subsets.py"), str(rd), "--jobs", "2",
                        "--emu", EMU], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    assert r.returncode == 0, r.stdout
    lines = [json.loads(x) for x in open(rd / "csub" / "c_t.jsonl")]
    assert len(lines) == 24 and lines[0]["t"] == 5000
    for x in lines:
        assert x["C"] == pytest.approx(min(x["C_S"].values()), abs=0.1)
        assert set(x["C_S"]) == {"1", "2", "3"} and x["C"] > 0
        assert x["saturated"] == bool(x["t"] % 2)
    rep = json.load(open(rd / "csub" / "replay.json"))
    assert rep["M"] == 2 and rep["saturated_steps"] == 12 and rep["h_W"] == 150
    # single-vessel saturation throughput near the 802.11g figure (about 19 Mb/s with h_W per frame)
    c1 = [x["C_S"]["1"] * 8 / 0.125 / 1e6 for x in lines]
    assert 12.0 < sum(c1) / len(c1) < 22.0


# ------------------------------------------------------------------------------------------------
# vessel sampler

def sock_diag_ok():
    try:
        outq.SockDiag().sample([1])
        return True
    except OSError:
        return False


def backlog_pair(port, hold_s):
    """server socket on `port` with unsent bytes for hold_s seconds (the client reads nothing), then drained"""
    ls = socket.socket()
    ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ls.bind(("127.0.0.1", port))
    ls.listen(1)
    c = socket.socket()
    c.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    c.connect(("127.0.0.1", port))
    s, _ = ls.accept()
    s.setblocking(False)

    def run():
        t0 = time.time()
        while time.time() - t0 < hold_s:
            try:
                s.send(b"x" * 65536)
            except BlockingIOError:
                time.sleep(0.005)
        c.setblocking(False)
        while time.time() - t0 < hold_s + 1.0:
            try:
                c.recv(1 << 20)
            except BlockingIOError:
                time.sleep(0.001)
        for x in (c, s, ls):
            x.close()
    th = threading.Thread(target=run)
    th.start()
    return th


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def check_sampler_log(path, port):
    hdr, steps = outq.load(path)
    assert hdr["ports"][0] == port
    v = [steps[t]["ports"][str(port)] for t in sorted(steps)]
    assert v[0]["nz"] == v[0]["n"] > 50 and v[0]["q0"] > 0
    assert v[-1]["nz"] == 0 and v[-1]["sock"] == 1


@pytest.mark.skipif(not sock_diag_ok(), reason="no sock_diag")
def test_python_sampler_loopback(tmp_path):
    port = free_port()
    th = backlog_pair(port, 0.4)
    time.sleep(0.05)
    outq.sample_loop(time.time(), 0.1, 0.02, 0.01, [port], str(tmp_path / "p.jsonl"), steps=12)
    th.join()
    check_sampler_log(str(tmp_path / "p.jsonl"), port)


@needs_cc
@pytest.mark.skipif(not sock_diag_ok(), reason="no sock_diag")
def test_c_sampler_loopback(tmp_path):
    exe = tmp_path / "fdt_outq"
    subprocess.run([CC, "-O2", "-Wall", "-Werror", "-o", str(exe), os.path.join(ROOT, "emu", "tools", "outq_sampler.c"),
                    "-lm"], check=True)
    port = free_port()
    th = backlog_pair(port, 0.4)
    time.sleep(0.05)
    subprocess.run([str(exe), "--epoch", "%.6f" % time.time(), "--ts-us", "100000", "--t1-us", "20000", "--t2-us",
                    "10000", "--port", str(port), "--out", str(tmp_path / "c.jsonl"), "--steps", "12"], check=True)
    th.join()
    check_sampler_log(str(tmp_path / "c.jsonl"), port)


# ------------------------------------------------------------------------------------------------
# clock and CPU of the guests

def test_clock_schedule_properties():
    sc = CS.make(7, 1800, ppm0=12.0, t0_unix=1000.0)
    A, lo, hi = 20.0, 60.0, 300.0
    segs = sc["freq"]
    assert segs[0][0] == 0.0 and segs[-1][1] >= 1800
    for a, b in zip(segs, segs[1:]):
        assert a[1] == pytest.approx(b[0]) and a[3] == pytest.approx(b[2])      # continuous
    for a, b, fa, fb in segs:
        assert -8.0 - 1e-9 <= fa <= 32.0 + 1e-9 and -8.0 - 1e-9 <= fb <= 32.0 + 1e-9
        if fa != fb:
            assert lo - 1e-6 <= b - a <= hi + 1e-6
    assert any(s[2] != s[3] for s in segs)
    assert CS.max_slope(sc) <= 2 * A / lo + 1e-9
    s = CS.Schedule(sc)
    assert s.ppm(-5) == 12.0 and s.ppm(1e6) == segs[-1][3]
    a, b, fa, fb = next(x for x in segs if x[2] != x[3])
    assert s.ppm((a + b) / 2) == pytest.approx((fa + fb) / 2)
    assert s.at_unix(1000.0 + (a + b) / 2)[0] == pytest.approx((fa + fb) / 2)
    loss = sc["ntp_loss"]
    assert len(loss) == 3 and all(60.0 <= x[0] and 10.0 - 1e-6 <= x[1] - x[0] <= 60.0 + 1e-6 for x in loss)
    assert all(p[1] <= q[0] for p, q in zip(loss, loss[1:]))
    assert s.ntp_blocked((loss[0][0] + loss[0][1]) / 2) and not s.ntp_blocked(loss[0][1] + 1e-3)
    assert CS.make(7, 1800, ppm0=12.0, t0_unix=1000.0) == sc
    w = CS.make(8, 600, loss_windows=[[100, 130], [300, 400]])
    assert len(w["ntp_loss"]) == 2
    assert 100 <= w["ntp_loss"][0][0] <= w["ntp_loss"][0][1] <= 130 and 300 <= w["ntp_loss"][1][0] < w["ntp_loss"][1][1] <= 400


def load_clockerr():
    import importlib.util
    spec = importlib.util.spec_from_file_location("fdt_clockerr", os.path.join(ROOT, "emu", "vm", "guest", "fdt-clockerr.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_injector_tracks_chrony(tmp_path):
    ce = load_clockerr()
    shm = ce.Shm(str(tmp_path / "freq.shm"))
    k = ce.FakeClock(f=3 * 65536)           # kernel frequency before chronyd (e.g. left by an earlier boot)
    inj = ce.Injector(shm, k, ce.Nft(dry=True))
    base, kern = inj.apply(5.0, False)
    assert base == 3 * 65536 and k.f == 8 * 65536 and shm.get("applied_err") == 5 * 65536
    # chronyd writes its correction through the shim (as fdt-freqshim.c does)
    def chrony_write(f):
        shm.set("gen", shm.get("gen") + 1)
        shm.set("chrony_freq", f)
        shm.set("chrony_valid", 1)
        e = shm.get("err")
        k.write_freq(f + e)
        shm.set("applied_err", e)
        shm.set("gen", shm.get("gen") + 1)
    chrony_write(-2 * 65536)
    assert k.f == 3 * 65536
    base, kern = inj.apply(-7.5, True)
    assert base == -2 * 65536 and k.f == int(round(-9.5 * 65536)) and inj.nft.on
    n = len(k.writes)
    inj.apply(-7.5, False)                  # no change of the error: no kernel write
    assert len(k.writes) == n and not inj.nft.on
    inj.apply(0.0, False)
    assert k.f == -2 * 65536


@needs_cc
def test_freqshim_with_fake_libc(tmp_path):
    """the shim adds the injected error to the frequency that the caller writes and hides it on reads"""
    fake = tmp_path / "fake.c"
    fake.write_text(r'''
#include <sys/timex.h>
#include <string.h>
static long kfreq = 0, ktick = 10000;
int adjtimex(struct timex *t) {
    if (t->modes & ADJ_FREQUENCY) kfreq = t->freq;
    if (t->modes & ADJ_TICK) ktick = t->tick;
    t->freq = kfreq; t->tick = ktick; return 0;
}
long fake_kernel_freq(void) { return kfreq; }
''')
    prog = tmp_path / "prog.c"
    prog.write_text(r'''
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/timex.h>
#include <unistd.h>
#include <stdlib.h>
long fake_kernel_freq(void);
int main(int argc, char **argv) {
    int fd = open(argv[1], O_RDWR); int64_t *p = mmap(0, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    p[1] = 5L * 65536;                                         /* err = +5 ppm (offset 8) */
    struct timex t = {0}; t.modes = ADJ_FREQUENCY | ADJ_TICK; t.freq = -2L * 65536; t.tick = 10001;
    adjtimex(&t);
    struct timex r = {0}; r.modes = 0; adjtimex(&r);
    printf("%ld %ld %ld %ld %ld %ld\n", fake_kernel_freq(), r.freq, (long)p[2], (long)p[3], (long)p[5], (long)p[4]);
    return 0;
}
''')
    subprocess.run([CC, "-O2", "-shared", "-fPIC", "-o", str(tmp_path / "libfake.so"), str(fake)], check=True)
    subprocess.run([CC, "-O2", "-Wall", "-Werror", "-shared", "-fPIC", "-o", str(tmp_path / "shim.so"),
                    os.path.join(ROOT, "emu", "vm", "guest", "fdt-freqshim.c"), "-ldl"], check=True)
    subprocess.run([CC, "-O2", "-o", str(tmp_path / "prog"), str(prog), "-L", str(tmp_path), "-lfake"], check=True)
    shm = tmp_path / "freq.shm"
    with open(shm, "wb") as f:
        f.write(b"\0" * 4096)
    env = dict(os.environ, LD_PRELOAD=str(tmp_path / "shim.so"), LD_LIBRARY_PATH=str(tmp_path),
               FDT_FREQ_SHM=str(shm))
    out = subprocess.run([str(tmp_path / "prog"), str(shm)], env=env, stdout=subprocess.PIPE, text=True,
                         check=True).stdout.split()
    kern, seen, chrony_f, chrony_tick, applied, gen_ = map(int, out)
    assert kern == 3 * 65536            # -2 ppm of the caller + 5 ppm injected
    assert seen == -2 * 65536           # the caller reads back its own value
    assert chrony_f == -2 * 65536 and chrony_tick == 10001 and applied == 5 * 65536 and gen_ == 2


def test_guest_scripts_syntax():
    for p in ("emu/vm/provision-base.sh", "emu/vm/build_image.sh"):
        subprocess.run(["bash", "-n", os.path.join(ROOT, p)], check=True)
    for p in ("emu/vm/guest/fdt-clockerr.py", "emu/vm/guest/fdt-clocksource.py", "emu/vm/guest/fdt-clockinit.py",
              "emu/vm/guest/fdt_clock_schedule.py", "emu/vm/chrony_conf.py", "emu/vm/cpu_profile.py",
              "emu/vm/cpu_bench.py"):
        subprocess.run([sys.executable, "-m", "py_compile", os.path.join(ROOT, p)], check=True)
    txt = open(os.path.join(ROOT, "emu", "vm", "provision-base.sh")).read()
    assert "clocksource=tsc tsc=reliable" in txt and "fdt-freqshim.so" in txt and "fdt-clockerr.service" in txt


def test_chrony_conf_and_warmup(tmp_path):
    v = chrony_conf.render("vessel")
    assert "server 10.10.0.1 iburst minpoll 0 maxpoll 2" in v and "makestep" not in v
    assert "makestep 0.005 1" in chrony_conf.render("vessel", initial_step_ms=5)
    g = chrony_conf.render("gs")
    assert "refclock PHC /dev/ptp0" in g and "makestep 0.1 3" in g and "allow 10.10.0.0/24" in g
    p = tmp_path / "clock.csv"
    with open(p, "w") as f:
        f.write("host_ns,host,offset_ns,rtt_ns\n")
        for k in range(600):                        # 60 s at 10 Hz
            t = 1_000_000_000_000 + k * 100_000_000
            off = 40e6 * math.exp(-k / 20.0) if k < 150 else 50e3 * math.sin(k)
            f.write("%d,v1,%d,120000\n" % (t, off))
            f.write("%d,gs,%d,120000\n" % (t, 10000))
    data = chrony_conf.read_clock(str(p))
    c = chrony_conf.convergence(data["v1"], 500.0, 10.0)
    assert 8.0 < c < 15.1
    ok, w, need = chrony_conf.warmup_ok(c, 0.125, 240)
    assert ok and w == 30.0 and need == math.ceil(c / 0.125)
    assert not chrony_conf.warmup_ok(c, 0.125, 40)[0]
    assert chrony_conf.convergence(data["v1"], 1.0, 10.0) is None


def test_cpu_profile():
    assert cpu_profile.profile(0.25) == (1000, 4000)
    assert cpu_profile.profile(0.5) == (1000, 2000)
    q, p = cpu_profile.profile(0.3)
    assert q >= 1000 and p >= 1000 and abs(q / p - 0.3) < 0.01
    assert cpu_profile.systemd_props(0.25) == ["CPUQuota=25%", "CPUQuotaPeriodSec=4ms"]
    w = cpu_profile.wrap_cmd(["qemu", "-x"], 0.25, unit="fdt-v1")
    assert w[:5] == ["systemd-run", "--user", "--scope", "--quiet", "--collect"] and w[-3:] == ["--", "qemu", "-x"]
    r, per = cpu_profile.ratio_of({"score": 10.0, "kernels": {"a": 5.0, "b": 20.0}},
                                  {"score": 40.0, "kernels": {"a": 20.0, "b": 80.0}})
    assert r == 0.25 and per == {"a": 0.25, "b": 0.25}
    with pytest.raises(ValueError):
        cpu_profile.profile(0.0)


# ------------------------------------------------------------------------------------------------
# qualification

def test_qualify_manifest(tmp_path):
    man = qualify.write_manifest(str(tmp_path), steps=400)
    runs = man["runs"]
    assert len(runs) == 60
    cells = {(r["scenario"], r["M"], r["K"]) for r in runs}
    assert len(cells) == 20
    assert all(r.get("max_K") == 1 for r in runs if r["K"] == 1) and all("max_K" not in r for r in runs if r["K"] == 3)
    k3 = [i for i, r in enumerate(runs) if r["K"] == 3]
    assert max(k3) < min(i for i, r in enumerate(runs) if r["K"] == 1)
    seeds = {}
    for r in runs:
        seeds.setdefault((r["scenario"], r["M"], r["rep"]), set()).add(r["seed"])
    assert all(len(v) == 1 for v in seeds.values()) and len({min(v) for v in seeds.values()}) == 30
    s = json.load(open(tmp_path / runs[0]["spec"]))
    assert s["stack"] == "paper" and s["steps"] == 400 and s["scenario"]["scenario_set"] == "paper-v2"
    assert any(a.startswith("--phaseLag=125000:20000:10000") for a in s["emu_args"])


def make_run(root, rid, steps, lag, outcomes, status="ok", aborted=False):
    rd = os.path.join(root, rid)
    os.makedirs(os.path.join(rd, "emu"))
    with open(os.path.join(rd, "result.json"), "w") as f:
        json.dump({"status": status, "S0": 1000, "steps": steps}, f)
    with open(os.path.join(rd, "emu", "summary.json"), "w") as f:
        json.dump({"phase_lag": {"aborted": aborted}}, f)
    with open(os.path.join(rd, "emu", "steplag.csv"), "w") as f:
        f.write("k,max1_us,max2_us,max3_us,lag_p3_us,lag_end_us,catchup,n\n")
        for k in range(steps):
            m1, m2, m3, cu = lag(k)
            f.write("%d,%.1f,%.1f,%.1f,100.0,%.1f,%d,700\n" % (k, m1, m2, m3, 100.0 + 1200.0 * cu, cu))
    if outcomes is not None:
        with open(os.path.join(rd, "outcomes.json"), "w") as f:
            json.dump(outcomes, f)


def test_qualify_analysis_synthetic(tmp_path):
    root = str(tmp_path)
    man = {"steps": 300, "runs": []}
    for prof in ("S1", "S3"):
        for rep in range(3):
            for K in (1, 3):
                rid = "Q-%s-m4-k%d-r%d" % (prof, K, rep)
                man["runs"].append({"id": rid, "M": 4, "scenario": prof, "K": K, "rep": rep, "method": "fleetdt"})
                base = {"gamma": 0.990 + 0.001 * rep, "phi": 0.980 + 0.001 * rep, "Pi": 0.60 + 0.01 * rep}
                if prof == "S1":
                    o = {k: v + (0.0005 * (-1) ** rep if K == 3 else 0.0) for k, v in base.items()}
                    lag = (lambda k: (150.0, 200.0, 250.0, 0)) if K == 1 else \
                        (lambda k: (300.0, 450.0 if k % 50 else 700.0, 400.0, 0))
                else:
                    # S3: K = 3 lowers phi by 0.02 (outside the margin) and one K = 3 run lags past 1 ms
                    o = dict(base, phi=base["phi"] - (0.02 if K == 3 else 0.0))
                    lag = (lambda k: (150.0, 200.0, 250.0, 0)) if not (K == 3 and rep == 2) else \
                        (lambda k: (150.0, 200.0, 2500.0 if k == 10 else 300.0, 1 if k == 10 else 0))
                make_run(root, rid, 300, lag, o)
    with open(os.path.join(root, "manifest.json"), "w") as f:
        json.dump(man, f)
    # delta 0.5 ms (step rule) below the HardLimit of 1 ms (run rule)
    runs = qualify.collect(root, os.path.join(root, "manifest.json"), warm_steps=100, delta_us=500.0)
    rep = qualify.analyze(runs, warm_steps=100, delta_us=500.0)
    valid = {r["id"]: r["valid"] for r in rep["runs"]}
    assert not valid["Q-S3-m4-k3-r2"] and sum(valid.values()) == 11
    cell = rep["lag_by_cell"]["S1-m4-k3"]
    assert cell["steps"] == 600 and cell["step_exclusion"] == pytest.approx(4 / 200, abs=1e-9)
    assert cell["lag_us"]["max2"]["max"] == 700.0 and cell["lag_us"]["max1"]["p50"] == 300.0
    assert rep["lag_by_cell"]["S1-m4-k1"]["step_exclusion"] == 0.0
    assert rep["invalid_runs"]["K:3"]["invalid"] == 1 and rep["invalid_runs"]["K:1"]["invalid"] == 0
    assert rep["fisher_invalid"]["K1 vs K3"] == pytest.approx(1.0)
    t = rep["tost_K1_vs_K3"]
    assert t["S1-m4"]["gamma"]["n"] == 3 and t["S1-m4"]["gamma"]["equivalent"]
    assert t["S1-m4"]["all_equivalent"]
    assert t["S3-m4"]["phi"]["n"] == 2 and not t["S3-m4"]["phi"]["equivalent"]
    assert rep["cells_kept"] == ["S1-m4"] and rep["cells_dropped"] == ["S3-m4"]


def test_tost_paired():
    eq = qualify.tost_paired([0.001, -0.001, 0.0005, -0.0005, 0.0], 0.005)
    assert eq["equivalent"] and eq["ci90"][0] > -0.005 and eq["ci90"][1] < 0.005
    ne = qualify.tost_paired([0.006, 0.007, 0.0065, 0.0062], 0.005)
    assert not ne["equivalent"] and ne["mean"] > 0.005
    assert not qualify.tost_paired([0.0], 0.005)["equivalent"]
