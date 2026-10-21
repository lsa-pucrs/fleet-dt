"""Calibration Cal of Section VI-E of the paper on synthetic logs: w1(t) from the arrival times of the AS, w2(t)
from the step log of the vessel regulator with the probe offsets, e_i(t) of the chrony sampler, the delay of the
regulator, delta_GS of the GS broker plugin, B_ctl(t) from the ground truth, c_y, C_ref, alpha* and U* per left-out
scenario (fleetdt/calibration.py), the script emu/experiments/calibrate_paper.py and its DTP mode, the chrony
sampler (emu/tools/chrony_sampler.py), the GS broker plugin fdt_gsstamp.so and the hooks of emu/fleet.py."""
import json
import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import time
import types

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "emu"))
sys.path.insert(0, os.path.join(ROOT, "emu", "tools"))
sys.path.insert(0, os.path.join(ROOT, "emu", "experiments"))
sys.path.insert(0, os.path.join(ROOT, "tests"))

from fleetdt import calibration as CAL  # noqa: E402
from fleetdt import outer as O  # noqa: E402
from fleetdt.as_model import DTP  # noqa: E402
from fleetdt.as_service import AS  # noqa: E402
from fleetdt.gsconf import gs_conf  # noqa: E402
import gt as GT  # noqa: E402
import chrony_sampler as CS  # noqa: E402
import calibrate_paper as CP  # noqa: E402

EPOCH = 1767225600.0
TS, T1, T2 = 0.125, 0.020, 0.010
BASE_DTP = os.path.join(ROOT, "dtp.json")


# --------------------------------------------------------------------------- AS arrival times (w1)

def small_dtp():
    return DTP.from_dict({"Ts": TS, "T1": T1, "T2": T2, "topics": [{"name": "state", "rate": 8, "bound": 48},
                                                                    {"name": "video", "rate": 16, "bound": 1400}],
                          "vessels": [1, 2], "order": ["state", "video"], "needed": ["state"]})


def test_as_logs_the_arrival_of_each_demand_table():
    a = AS(small_dtp(), epoch=0.0, now=lambda: 0.0)
    assert a.on_demand(1, 0, [100, 0], at=0.0031)
    assert a.on_demand(2, 0, [100, 0], at=0.0174)
    dec = a.close_phase1(0)
    assert dec["d_at"] == {"1": pytest.approx(0.0031), "2": pytest.approx(0.0174)}
    # a table at or after start(t) + T1 stays out of M(t) and becomes a demand_late record
    assert a.on_demand(1, 1, [100, 0], at=TS + 0.003)
    assert not a.on_demand(2, 1, [100, 0], at=TS + T1 + 0.0021)
    dec = a.close_phase1(1)
    assert dec["M"] == [1] and list(dec["d_at"]) == ["1"]
    assert not a.on_demand(2, 1, [100, 0], at=TS + 0.040)      # after the decision too
    late = a.take_late_demand()
    assert [(r["event"], r["t"], r["i"]) for r in late] == [("demand_late", 1, 2), ("demand_late", 1, 2)]
    assert late[0]["w"] == pytest.approx(T1 + 0.0021) and late[1]["w"] == pytest.approx(0.040)
    assert a.take_late_demand() == []
    w1 = CAL.w1_series([dict(dec, published=True)] + late)
    assert w1[1]["w1"] == pytest.approx(0.040) and w1[1]["n"] == 1 and w1[1]["late"] == 2


def test_w1_series_and_members():
    recs = [{"t": 10, "M": [1, 2], "d_at": {"1": 0.004, "2": 0.009}, "published": True},
            {"t": 11, "M": [1], "d_at": {"1": 0.003}, "published": True},
            {"event": "demand_late", "t": 11, "i": 2, "w": 0.0215},
            {"event": "outer", "t": 11, "alpha_star": 0.1},
            {"t": 12, "M": [], "d_at": {}, "published": False}]
    w1 = CAL.w1_series(recs)
    assert w1[10] == {"w1": 0.009, "n": 2, "late": 0}
    assert w1[11] == {"w1": 0.0215, "n": 1, "late": 1}
    assert w1[12]["w1"] is None
    assert CAL.members(recs) == {10: [1, 2], 11: [1]}


# --------------------------------------------------------------------------- probe offsets and w2

def probe_csv(rows):
    return ["host_ns,host,offset_ns,rtt_ns\n"] + ["%d,%s,%d,40000\n" % (int(t * 1e9), h, int(o * 1e9))
                                                  for t, h, o in rows]


def test_offsets_interpolate_in_host_time():
    off = CAL.Offsets.from_csv_lines(probe_csv([(100.0, "gs", 0.0001), (101.0, "gs", 0.0003),
                                                (100.0, "v1", -0.002), (101.0, "v1", -0.004)]))
    assert off.has(0) and off.has(1) and not off.has(2)
    assert float(off.theta(0, 100.5)) == pytest.approx(0.0002)
    assert float(off.theta(1, 100.25)) == pytest.approx(-0.0025)
    assert float(off.theta(2, 100.25)) == 0.0
    assert float(off.theta(1, 50.0)) == pytest.approx(-0.002)       # held at the ends


def test_w2_maps_the_vessel_clock_to_the_gs_clock():
    t = 800
    at = EPOCH + t * TS
    # vessel 1 runs 2 ms ahead of the host, vessel 2 1 ms behind, the GS 0.5 ms ahead
    off = CAL.Offsets({0: ([at - 10, at + 10], [0.0005, 0.0005]), 1: ([at - 10, at + 10], [0.002, 0.002]),
                       2: ([at - 10, at + 10], [-0.001, -0.001])})
    regs = {1: [{"t": t, "p_rx_us": 26000, "p_late_us": None}],
            2: [{"t": t, "p_rx_us": None, "p_late_us": 31000}, {"t": t + 1, "p_rx_us": 24000}]}
    w2 = CAL.w2_series(regs, {t: [1, 2], t + 1: [1, 2]}, T1, EPOCH, TS, off)
    # vessel 1: 26 ms - 20 ms - (2 - 0.5) ms = 4.5 ms; vessel 2 (late): 31 - 20 - (-1 - 0.5) = 12.5 ms
    assert w2[t]["w2"] == pytest.approx(0.0125) and w2[t]["n"] == 2 and w2[t]["missing"] == 0
    # step t + 1: vessel 1 has no record (missing), vessel 2: 24 - 20 + 1.5 = 5.5 ms
    assert w2[t + 1]["w2"] == pytest.approx(0.0055) and w2[t + 1]["missing"] == 1
    w2n = CAL.w2_series(regs, {t: [1, 2]}, T1, EPOCH, TS, None)
    assert w2n[t]["w2"] == pytest.approx(0.011)


# --------------------------------------------------------------------------- e_i, regulator delay, delta_GS

def test_chrony_regulator_and_delivery_logs():
    assert CAL.chrony_errors([{"t": 5, "e": 0.0004}, {"t": 6, "error": "x"}, {"t": 7, "e": 0.0002}]) == \
        {5: 0.0004, 7: 0.0002}
    d = CAL.regulator_delays([{"t": 1, "late_us": 120, "late_p1_us": -1}, {"t": 2, "late_us": 80, "late_p1_us": 300},
                              {"t": 3, "late_us": -1, "late_p1_us": -1}])
    assert d == {1: pytest.approx(120e-6), 2: pytest.approx(300e-6)}
    g = CAL.delivery_times([{"t": 4, "max_us": 210.0}, {"t": 4, "max_us": 90.0}, {"t": 5, "max_us": None}])
    assert g == {4: pytest.approx(210e-6)}


# --------------------------------------------------------------------------- B_ctl from the ground truth

def gt_records(rows, origin_ns):
    """rows: (host time s, node, ip_len, sport, dport, ev, kind)"""
    r = np.zeros(len(rows), dtype=GT.DT)
    for k, (at, node, ip, sp, dp, ev, kind) in enumerate(rows):
        r[k]["t_ns"] = int(round(at * 1e9)) - origin_ns
        r[k]["dur_ns"] = 100000
        r[k]["node"], r[k]["ip_len"], r[k]["sport"], r[k]["dport"] = node, ip, sp, dp
        r[k]["ev"], r[k]["kind"], r[k]["size"] = ev, kind, ip + 36
    return r


def test_ctl_bytes_count_frames_outside_the_data_bridges_in_phase_3():
    t = 900
    s = EPOCH + t * TS
    origin = int((EPOCH + 100.0) * 1e9)
    rows = [(s + 0.040, 1, 120, 50000, 1883, 0, 0),    # control bridge, vessel 1 -> GS: counts
            (s + 0.050, 0, 52, 1883, 50000, 0, 0),     # its TCP ACK from the AP: counts
            (s + 0.060, 2, 76, 123, 123, 0, 0),        # NTP of vessel 2: counts
            (s + 0.070, 1, 1500, 1884, 40000, 0, 0),   # data bridge: not counted
            (s + 0.071, 0, 52, 40000, 1884, 0, 0),     # its ACK: not counted
            (s + 0.072, 1, 1500, 1885, 40000, 0, 0),   # data bridge of B2: not counted
            (s + 0.010, 1, 120, 50000, 1883, 0, 0),    # phase 1: not counted in step t
            (s + 0.080, 3, 500, 5000, 5000, 0, 0),     # node 3 > M (interferer): not counted
            (s + 0.081, 1, 120, 50000, 1883, 1, 0),    # an RX record: not counted
            (s + 0.082, 1, 0, 0, 0, 0, 3),             # a MAC ACK: not counted
            (s + 0.0299, 1, 100, 50000, 1883, 0, 0)]   # 0.1 ms before phase 3 on the host clock
    r = gt_records(rows, origin)
    hW = 570.25
    b = CAL.ctl_bytes_phase3(r, origin, EPOCH, TS, T1, T2, 2, hW, None, steps=[t, t + 1])
    assert b[t] == pytest.approx(120 + 52 + 76 + 3 * hW) and b[t + 1] == 0.0
    # the GS clock is 0.2 ms ahead of the host: the frame at 29.9 ms (host) starts in phase 3 (GS clock)
    off = CAL.Offsets({0: ([s - 1, s + 1], [0.0002, 0.0002])})
    b2 = CAL.ctl_bytes_phase3(r, origin, EPOCH, TS, T1, T2, 2, hW, off, steps=[t])
    assert b2[t] == pytest.approx(120 + 52 + 76 + 100 + 4 * hW)


# --------------------------------------------------------------------------- margins, c_y, C_ref

def meas(M, scen, **kw):
    m = {"M": M, "scenario": scen, "w1_max": None, "w2_max": None, "e_max": None, "reg_delay_max": None,
         "delta_GS": None, "B_ctl_max": None, "w2_missing": 0, "w1_incomplete": 0, "cy_min": None, "cy_n": 0,
         "C_sum": 0.0, "C_n": 0, "missing": []}
    m.update(kw)
    return m


def test_margins_follow_section_vi_e():
    ms = [meas(2, "S1", w1_max=0.0121, w2_max=0.0042, e_max=0.0003, reg_delay_max=0.0002, delta_GS=0.0007,
               B_ctl_max=2100.0),
          meas(2, "S3", w1_max=0.0135, w2_max=0.0038, e_max=0.0005, reg_delay_max=0.0004, delta_GS=0.0009,
               B_ctl_max=1800.0)]
    g = CAL.margins(ms)
    eps = 2 * 0.0005 + 0.001 + 0.0004
    assert g["eps"] == pytest.approx(eps)
    assert g["T1"] == pytest.approx(0.0135 + eps) and g["T2"] == pytest.approx(0.0042 + eps)
    assert g["delta_GS"] == pytest.approx(0.0009) and g["T_m"] == pytest.approx(eps + 0.0009)
    assert g["B_ctl_bar"] == 2100.0 and g["runs"] == 2
    # rounded up to the microsecond
    assert abs(g["T1"] * 1e6 - round(g["T1"] * 1e6)) < 1e-6 and g["T1"] >= 0.0135 + eps - 1e-12
    # without the chrony error bound no eps, so no T1, T2, T_m
    g2 = CAL.margins([meas(2, "S1", w1_max=0.01, w2_max=0.004, reg_delay_max=0.0001, delta_GS=0.0005)])
    assert g2["eps"] is None and g2["T1"] is None and g2["T_m"] is None and g2["delta_GS"] == 0.0005


def test_c_y_and_C_ref():
    ms = [meas(2, "S1", cy_min=0.97, cy_n=100, C_sum=3.0e7, C_n=100),
          meas(2, "S1", cy_min=0.91, cy_n=50, C_sum=1.6e7, C_n=50),
          meas(4, "S3", cy_min=1.2, cy_n=80, C_sum=2.0e7, C_n=80),
          meas(2, "S3")]
    cy = CAL.c_y_global(ms)
    assert cy["c_y"] == 0.91 and cy["n"] == 230
    assert CAL.c_y_global([meas(2, "S1", cy_min=1.3, cy_n=3)])["c_y"] == 1.0
    cr = CAL.c_ref(ms)
    assert cr[2]["S1"] == pytest.approx(4.6e7 / 150) and cr[4]["S3"] == pytest.approx(2.0e7 / 80)
    assert "S3" not in cr[2]


# --------------------------------------------------------------------------- alpha*, U* per left-out scenario

def dta(seed, n, level, noise, S0=0):
    rng = np.random.default_rng(seed)
    y = level + noise * rng.standard_normal(n)
    return [{"t": S0 + k, "y": float(y[k]), "C_hat": float(level), "vessels": [1, 2]} for k in range(n)]


def test_outer_fit_leaves_the_scenario_out():
    runs = {"S1": [dta(1, 400, 300000, 4000), dta(2, 400, 300000, 4000)],
            "S3": [dta(3, 400, 200000, 30000), dta(4, 400, 200000, 30000)]}
    fit_all = CAL.outer_fit(runs["S1"] + runs["S3"], 1.0, 0.0, 0.76, B=200)
    fit_wo3 = CAL.outer_fit(runs["S1"], 1.0, 0.0, 0.76, B=200)
    assert fit_wo3["runs"] == 2 and fit_all["runs"] == 4
    for f in (fit_all, fit_wo3):
        assert abs(f["alpha"] * 100 - round(f["alpha"] * 100)) < 1e-9 and 0.01 <= f["alpha"] <= 1.0   # grid only
    # the noisy scenario lowers U*
    assert fit_wo3["U"] is not None and (fit_all["U"] is None or fit_all["U"] < fit_wo3["U"])
    # U* is the largest U of the grid whose lower bound reaches 0.99
    low = fit_wo3["PICP_low"]
    assert low["%.2f" % fit_wo3["U"]] >= 0.99
    nxt = "%.2f" % (fit_wo3["U"] + 0.01)
    assert nxt not in low or low[nxt] < 0.99
    # the same as fleetdt.outer on the same runs (grid, no refinement)
    ser = [(O.series(r)[1], O.series(r)[2], O.fill_series(r, 0.76), O.backlog_series(r)) for r in runs["S1"]]
    ref = O._calibrate(ser, [O._C0(s[0], s[1]) for s in ser], 0.99, None, O.U_GRID, 0.95, 200, None, 0, 0.76,
                       refine=False)
    assert ref["alpha_star"] == fit_wo3["alpha"] and ref["U_star"] == fit_wo3["U"]


# --------------------------------------------------------------------------- calibrate_paper.py on run directories

def write_jsonl(p, recs):
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p, "w") as f:
        for r in recs:
            f.write(json.dumps(r) + "\n")


def write_gt(p, origin_ns, recs):
    os.makedirs(os.path.dirname(p), exist_ok=True)
    hdr = json.dumps({"origin_ns": origin_ns, "record_size": 40, "M": 2, "nodes": []}).encode()
    with open(p, "wb") as f:
        f.write(b"FDTGT1\0\0" + struct.pack("<I", len(hdr)) + hdr + recs.tobytes())


def make_run(root, name, scen, seed, S0=8000, steps=300, M=2, w1=0.010, w2=0.004, e=0.0003, late=150,
             dgs=500.0, ctl=300, cy=0.95, noise=5000.0, zst=False):
    rd = os.path.join(root, name)
    os.makedirs(rd)
    dtp = json.load(open(BASE_DTP))
    dtp["vessels"] = list(range(1, M + 1))
    json.dump(dtp, open(os.path.join(rd, "dtp.json"), "w"))
    json.dump({"stack": "paper", "status": "ok", "M": M, "S0": S0, "steps": steps},
              open(os.path.join(rd, "result.json"), "w"))
    json.dump({"scenario_id": scen}, open(os.path.join(rd, "scenario.json"), "w"))
    rng = np.random.default_rng(seed)
    ts = list(range(S0, S0 + steps))
    as_recs, gsd, C, y, dtarecs = [], [], [], [], []
    regs, chr_ = {i: [] for i in range(1, M + 1)}, {h: [] for h in range(M + 1)}
    for k, t in enumerate(ts):
        warm = k < 10   # larger values inside the warm-up of the test: never in the constants
        f = 3.0 if warm else 1.0
        as_recs.append({"t": t, "M": list(range(1, M + 1)), "published": True,
                        "d_at": {str(i): (w1 * f if i == M and k == 50 else 0.003 + 0.001 * i)
                                 for i in range(1, M + 1)}})
        for i in range(1, M + 1):
            regs[i].append({"t": t, "late_us": late * f if (i == 1 and k == 60) or warm else 20, "late_p1_us": -1,
                            "p_rx_us": int((T1 + (w2 * f if k == 70 and i == 2 else 0.002)) * 1e6), "p_late_us": None})
        gsd.append({"t": t, "n_in": 10, "n_out": 20, "unmatched": 0,
                    "max_us": dgs * f if k == 80 or warm else 100.0, "mean_us": 50.0})
        for h in range(M + 1):
            chr_[h].append({"t": t, "e": e * f if (h == 0 and k == 90) or warm else 0.0001})
        yy = 300000.0 + noise * rng.standard_normal()
        y.append({"t": t, "y": yy})
        C.append({"t": t, "C": yy * (cy if k == 100 else 1.02)})
        dtarecs.append({"t": t, "y": yy, "C_hat": 300000.0, "vessels": list(range(1, M + 1))})
    write_jsonl(os.path.join(rd, "vms", "gs", "as.jsonl"), as_recs)
    write_jsonl(os.path.join(rd, "vms", "gs", "gsdelivery.jsonl"), gsd)
    write_jsonl(os.path.join(rd, "vms", "gs", "dta.jsonl"), dtarecs)
    for i in range(1, M + 1):
        write_jsonl(os.path.join(rd, "vms", "v%d" % i, "regsteps.jsonl"), regs[i])
    for h in range(M + 1):
        write_jsonl(os.path.join(rd, "vms", "gs" if h == 0 else "v%d" % h, "chrony.jsonl"), chr_[h])
    write_jsonl(os.path.join(rd, "csub", "c_t.jsonl"), C)
    write_jsonl(os.path.join(rd, "throughput.jsonl"), y)
    t0 = EPOCH + S0 * TS
    with open(os.path.join(rd, "clock.csv"), "w") as f:
        f.writelines(probe_csv([(t0 - 5, "gs", 0.0), (t0 + 100, "gs", 0.0)] +
                               [(t0 - 5, "v%d" % i, 0.0) for i in range(1, M + 1)] +
                               [(t0 + 100, "v%d" % i, 0.0) for i in range(1, M + 1)]))
    origin = int((t0 - 2.0) * 1e9)
    tc = EPOCH + (S0 + 120) * TS
    recs = gt_records([(tc + 0.050, 1, ctl, 50000, 1883, 0, 0), (tc + 0.060, 1, 1500, 1884, 40000, 0, 0),
                       (EPOCH + (S0 + 2) * TS + 0.05, 1, 9000, 50000, 1883, 0, 0)], origin)   # warm-up: ignored
    p = os.path.join(rd, "emu", "gt.bin")
    write_gt(p, origin, recs)
    if zst:
        subprocess.run(["zstd", "-q", "--rm", "-f", p], check=True)
    return rd


def test_calibrate_paper_compute_and_dtp(tmp_path):
    root = str(tmp_path / "cal")
    os.makedirs(root)
    make_run(root, "r1", "S1", 1, w1=0.012, w2=0.005, e=0.0004, late=300, dgs=800.0, ctl=200, cy=0.93)
    make_run(root, "r2", "S3", 2, w1=0.009, w2=0.003, e=0.0002, late=100, dgs=600.0, ctl=400, cy=0.97,
             noise=40000.0, zst=bool(shutil.which("zstd")))
    make_run(root, "r3", "S1", 3, M=4)
    # a failed run is left out
    rd = make_run(root, "r4", "S1", 4, w1=0.05)
    json.dump({"stack": "paper", "status": "failed", "M": 2, "S0": 8000, "steps": 300},
              open(os.path.join(rd, "result.json"), "w"))
    # the reader of a decompressed gt.bin.zst gives the records of emu/gt.py
    p1 = os.path.join(root, "r1", "emu", "gt.bin")
    hdr, recs = GT.load(p1)
    origin, recs2 = CP.parse_gt(open(p1, "rb").read())
    assert origin == hdr["origin_ns"] and recs2.tobytes() == recs.tobytes()
    assert CP.parse_gt(b"nothing") is None
    out = str(tmp_path / "calibration.json")
    CP.main(["compute", root, "--out", out, "--warm", "10", "--B", "200"])
    cal = json.load(open(out))
    assert cal["version"] == 1 and sorted(cal["M"]) == ["2", "4"] and len(cal["skipped"]) == 1
    m2 = cal["M"]["2"]
    eps = 2 * 0.0004 + 0.001 + 300e-6
    assert m2["eps"] == pytest.approx(eps, abs=1e-12)
    assert m2["T1"] == pytest.approx(0.012 + eps, abs=1e-6) and m2["T2"] == pytest.approx(0.005 + eps, abs=1e-6)
    assert m2["delta_GS"] == pytest.approx(800e-6) and m2["T_m"] == pytest.approx(eps + 800e-6, abs=1e-6)
    assert m2["B_ctl_bar"] == pytest.approx(400 + json.load(open(BASE_DTP))["h_W"])
    assert m2["runs"] == 2 and m2["w2_missing"] == 0
    assert cal["c_y"] == pytest.approx(0.93)
    assert set(cal["C_ref"]["2"]) == {"S1", "S3"} and set(cal["C_ref"]["4"]) == {"S1"}
    o2 = cal["outer"]["2"]["left_out"]
    assert set(o2) == {"S1", "S3", "all"}
    assert o2["S1"]["scenarios"] == ["S3"] and o2["S3"]["scenarios"] == ["S1"]
    assert o2["all"]["scenarios"] == ["S1", "S3"]
    assert o2["S3"]["U"] is not None and o2["S3"]["alpha"] is not None
    assert cal["outer"]["2"]["T3_Ts"] == pytest.approx((TS - m2["T1"] - m2["T2"] - m2["T_m"]) / TS)
    assert cal["outer"]["2"]["c_y"] == cal["c_y"] and cal["outer"]["2"]["B_ctl_bar"] == m2["B_ctl_bar"]
    assert CAL.lookup_outer(cal, 2, "S3") == (o2["S3"]["alpha"], o2["S3"]["U"])
    assert CAL.lookup_outer(cal, 2, "S9") == (o2["all"]["alpha"], o2["all"]["U"])
    assert CAL.lookup_outer(cal, 2, None) == (o2["all"]["alpha"], o2["all"]["U"])
    # DTP of an evaluation run in S3 with M = 2
    dp = str(tmp_path / "dtp_eval.json")
    CP.main(["dtp", "--calibration", out, "--dtp", BASE_DTP, "--M", "2", "--scenario", "S3", "--out", dp])
    d = json.load(open(dp))
    for k in ("T1", "T2", "T_m", "B_ctl_bar"):
        assert d[k] == m2[k]
    assert d["c_y"] == cal["c_y"] and d["alpha"] == o2["S3"]["alpha"] and d["U"] == o2["S3"]["U"]
    assert d["calibration"] == {"M": 2, "scenario": "S3", "left_out": "S3"}
    dtp = DTP.from_dict(d)
    assert dtp.T_m == m2["T_m"] and dtp.T1 == m2["T1"]
    with pytest.raises(SystemExit):
        CP.main(["dtp", "--calibration", out, "--dtp", BASE_DTP, "--M", "8", "--out", dp])


def test_apply_to_dtp_keeps_uncalibrated_fields_and_refuses_no_phase_3():
    cal = {"M": {"2": {"T1": 0.03, "T2": None, "T_m": 0.002, "B_ctl_bar": 1000.0}}, "c_y": None,
           "outer": {"2": {"left_out": {"all": {"alpha": 0.2, "U": None}}}}}
    base = json.load(open(BASE_DTP))
    d, kept = CAL.apply_to_dtp(base, cal, 2, "S1")
    assert d["T1"] == 0.03 and d["T2"] == base["T2"] and d["alpha"] == 0.2 and d["U"] == base["U"]
    assert sorted(kept) == ["T2", "U", "c_y"] and d["calibration"]["left_out"] is None
    cal["M"]["2"]["T1"] = 0.12
    with pytest.raises(ValueError):
        CAL.apply_to_dtp(base, cal, 2, "S1")


# --------------------------------------------------------------------------- chrony sampler

LINE = "C0A80001,192.168.0.1,3,1767225600.123,-0.000012345,0.000003100,0.000020000,-1.234,0.001,0.050," \
       "0.000410000,0.000220000,64.2,Normal"


def test_parse_tracking():
    r = CS.parse_tracking(LINE)
    assert r["stratum"] == 3 and r["leap"] == "Normal"
    assert r["sys"] == pytest.approx(-0.000012345) and r["root_delay"] == pytest.approx(0.00041)
    assert r["e"] == pytest.approx(0.000012345 + 0.00022 + 0.00041 / 2)
    assert CS.parse_tracking("chronyc> " + LINE)["e"] == r["e"]
    assert CS.parse_tracking("506 Cannot talk to daemon") is None
    assert CS.parse_tracking("a,b,c,d,e,f,g,h,i,j,k,l,m,n") is None


class FakeSession:
    def __init__(self, lines):
        self.lines = list(lines)

    def query(self, command="tracking"):
        return self.lines.pop(0) if self.lines else None


def test_sampler_schedule_and_records():
    clock = {"now": EPOCH + 10 * TS}
    slept = []

    def sleep(d):
        slept.append(d)
        clock["now"] += d
    import io
    out = io.StringIO()
    n = CS.run(FakeSession([LINE, "garbage", LINE]), out, EPOCH, TS, 10, 3, 0.0625, clock=lambda: clock["now"],
               sleep=sleep)
    recs = [json.loads(x) for x in out.getvalue().splitlines()]
    assert n == 3 and [r["t"] for r in recs] == [10, 11, 12]
    assert [r["at"] for r in recs] == [pytest.approx(EPOCH + t * TS + 0.0625) for t in (10, 11, 12)]
    assert "e" in recs[0] and recs[1]["error"] == "garbage" and "e" in recs[2]
    # a late start skips the steps whose instant has passed by more than one step
    clock["now"] = EPOCH + 25 * TS + 0.07
    out = io.StringIO()
    CS.run(FakeSession([LINE] * 5), out, EPOCH, TS, 20, 8, 0.0625, clock=lambda: clock["now"], sleep=sleep)
    assert [json.loads(x)["t"] for x in out.getvalue().splitlines()] == [26, 27]


def test_persistent_chronyc_session(tmp_path):
    fake = tmp_path / "chronyc"
    fake.write_text("#!%s\nimport sys\nfor line in sys.stdin:\n    if line.strip() == 'tracking':\n"
                    "        print(%r, flush=True)\n" % (sys.executable, LINE))
    fake.chmod(0o755)
    s = CS.Chronyc([str(fake), "-c", "-n"], timeout=5.0)
    try:
        a, b = s.query(), s.query()
        assert a == LINE and b == LINE
        p = s.p
        s.p.kill()
        s.p.wait()
        assert s.query() == LINE and s.p is not p        # restarted after an exit
    finally:
        s.close()
    s = CS.Chronyc([sys.executable, "-c", "import time; time.sleep(5)"], timeout=0.2)
    assert s.query() is None and s.p is None             # no answer within the timeout
    once = tmp_path / "chronyc1"
    once.write_text("#!%s\nimport sys\nif sys.argv[-1] == 'tracking':\n    print(%r)\n" % (sys.executable, LINE))
    once.chmod(0o755)
    assert CS.OneShot([str(once), "-c", "-n"]).query() == LINE
    assert CS.OneShot([str(fake)], timeout=2.0).query() is None      # reads stdin, which is empty: no line
    assert CS.chronyc_cmd("chronyc", oneshot=True) == ["chronyc", "-c", "-n"]


# --------------------------------------------------------------------------- GS broker configuration and plugin

def test_gsconf_loads_the_stamp_plugin():
    c = gs_conf({1: "10.0.0.2"}, stamp="/tmp/fdt-run/gsdelivery.jsonl", stamp_plugin="/x/fdt_gsstamp.so")
    lines = c.splitlines()
    k = lines.index("plugin /x/fdt_gsstamp.so")
    assert lines[k + 1] == "plugin_opt_log /tmp/fdt-run/gsdelivery.jsonl"
    assert k < lines.index("connection b1-ctl")
    assert "fdt_gsstamp" not in gs_conf({1: "10.0.0.2"})


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_gsstamp_plugin_in_the_broker(tmp_path):
    mq = os.environ.get("MOSQ") or next((p for p in ("/opt/mosquitto-2.1", "/opt/mosquitto-2.1.2-fleetdt")
                                        if os.path.isdir(p)), None)
    mosq = os.path.join(mq, "sbin", "mosquitto") if mq else None
    if not mosq or not os.path.exists(mosq):
        pytest.skip("no patched Mosquitto 2.1 (MOSQ)")
    paho = pytest.importorskip("paho.mqtt.client")
    so = os.path.join(os.environ.get("FDT_BUILD", os.path.join(ROOT, "build")), "fdt_gsstamp.so")
    if not os.path.exists(so):
        r = subprocess.run(["make", "-s", "-C", ROOT, "build/fdt_gsstamp.so", "MOSQ=" + mq],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if r.returncode != 0:
            pytest.skip("fdt_gsstamp.so does not build: %s" % r.stdout[-300:])
        so = os.path.join(ROOT, "build", "fdt_gsstamp.so")
    port = _free_port()
    log = tmp_path / "gsdelivery.jsonl"
    # started as root, the broker would drop to nobody and could not open the log
    (tmp_path / "m.conf").write_text("listener %d 127.0.0.1\nallow_anonymous true\nlog_dest stdout\nuser root\n"
                                     "plugin %s\nplugin_opt_log %s\n" % (port, so, log))
    br = subprocess.Popen([mosq, "-c", str(tmp_path / "m.conf")], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        time.sleep(0.4)
        got = []
        sub = paho.Client(paho.CallbackAPIVersion.VERSION2, client_id="sub")
        sub.on_message = lambda c, u, m: got.append(m.topic)
        sub.connect("127.0.0.1", port)
        sub.subscribe("boat/#", 0)
        sub.loop_start()
        time.sleep(0.3)
        pub = paho.Client(paho.CallbackAPIVersion.VERSION2, client_id="pub")
        pub.connect("127.0.0.1", port)
        pub.loop_start()
        for t in (40, 41):
            for s in range(3):
                pub.publish("boat/b1/tx/state", struct.pack("!IH", t, s) + b"x" * 40, qos=0)
        pub.publish("boat/b1/demand", b"\0" * 20, qos=0)
        deadline = time.time() + 3
        while len(got) < 7 and time.time() < deadline:
            time.sleep(0.05)
        assert len(got) == 7
        pub.loop_stop()
        sub.loop_stop()
        pub.disconnect()
        sub.disconnect()
    finally:
        br.terminate()
        out = br.communicate(timeout=10)[0].decode(errors="replace")
    recs = {r["t"]: r for r in (json.loads(x) for x in log.read_text().splitlines())}
    assert sorted(recs) == [40, 41], out
    for t in (40, 41):
        assert recs[t]["n_in"] == 3 and recs[t]["n_out"] == 3 and recs[t]["unmatched"] == 0
        assert 0 <= recs[t]["max_us"] < 1e6
    assert "fdt_gsstamp" in out


# --------------------------------------------------------------------------- hooks of emu/fleet.py

def test_fleet_starts_the_samplers_and_the_gs_plugin(tmp_path):
    import fleet
    from test_paper_hooks import run, session, spec, ssh_cmds
    R, calls = session(tmp_path)
    res = run(R, spec(tmp_path))
    assert res["status"] == "ok", res.get("error")
    prep_gs = next(c for c in ssh_cmds(calls, 0) if "echo prepared" in c)
    assert "make -s build/fdt_gsstamp.so MOSQ=/opt/mosquitto-2.1" in prep_gs
    assert "--stamp /tmp/fdt-run/gsdelivery.jsonl --stamp-plugin $HOME/fleet-dt/build/fdt_gsstamp.so" in prep_gs
    started = [c for c in ssh_cmds(calls) if "setsid nohup" in c and "/chrony.log" in c]
    hosts = sorted(c[1] for c in calls if c[0] == "ssh" and "setsid nohup" in c[2] and "/chrony.log" in c[2])
    assert hosts == [0, 1, 2]
    assert all("emu/tools/chrony_sampler.py" in c and "--start-step %d" % res["S0"] in c and "--steps 8" in c
               for c in started)
    kills = [c for c in ssh_cmds(calls) if "chrony.pid" in c and "kill -TERM" in c]
    assert len(kills) == 3
    assert res["commands"]["chrony"].endswith("--out /tmp/fdt-run/chrony.jsonl")
    assert "emu/tools/chrony_sampler.py" in res["revision"]["stack_files"]
    assert "mosquitto/plugins/fdt_gsstamp.c" in res["revision"]["stack_files"]
    # both can be switched off
    R, calls = session(tmp_path)
    res = run(R, spec(tmp_path, chrony={"enabled": False}, gs_stamp={"enabled": False}))
    prep_gs = next(c for c in ssh_cmds(calls, 0) if "echo prepared" in c)
    assert "fdt_gsstamp" not in prep_gs and res["commands"]["chrony"] is None
    assert not [c for c in ssh_cmds(calls) if "setsid nohup" in c and "/chrony.log" in c]
    assert fleet.paper_chrony_cmd({"dtp": {"Ts": 0.125, "epoch": None}, "chrony": {"offset_s": 0.1, "oneshot": True}},
                                  5, 7).endswith("--offset 0.1 --oneshot")
