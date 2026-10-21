"""Tests of the tools around the AS and the VE: configuration generator, outer loop, Section VI
metrics, channel monitor (replay backend), synthetic load, and the Python 3.7 syntax of the vessel
programs (Raspbian 10)."""
import ast
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from fleetdt.analysis import analyze, block_bootstrap_ci, clopper_pearson_upper, jain, tau_int
from fleetdt.as_model import DEFAULT_EPOCH, DTP
from fleetdt.gsconf import gs_conf, vessel_conf
from fleetdt.mock import FleetSim
from fleetdt.outer import outer_loop
from fleetdt.ve.collector import Assembler, StepClock

ROOT = Path(__file__).resolve().parents[1]
BUILD = Path(os.environ.get("FDT_BUILD", ROOT / "build"))


def test_gsconf_bridges():
    c = gs_conf({1: "10.0.0.1", 2: "10.0.0.2,2883,2884"})
    assert "connection b1-ctl\naddress 10.0.0.1:1883" in c
    assert "connection b2-data\naddress 10.0.0.2:2884" in c
    assert "topic boat/b2/tx/# in 0" in c and "tx/# in 1" not in c and "in 2" not in c and "topic boat/b1/levels out 1" in c
    assert "topic boat/b1/regulator in 0" in c and "set_tcp_nodelay true" in c
    v = vessel_conf(3, "/etc/fleetdt/dtp.json")
    assert "plugin_opt_vessel 3" in v and "listener 1884" in v


def test_default_epoch():
    d = DTP.from_dict({"topics": ["a"], "epoch": None})
    assert d.t0 == DEFAULT_EPOCH
    # the step index of today fits in the u32 field of the wire formats
    assert (1.9e9 - DEFAULT_EPOCH) / 0.125 < 2 ** 32


def test_outer_loop_recovers_gain_and_U():
    rng = np.random.default_rng(3)
    level = 300_000 + np.cumsum(rng.normal(0, 3000, 4000))
    y = level + rng.normal(0, 20_000, 4000)
    recs = [{"t": t, "y": float(v), "C_hat": 300_000.0} for t, v in enumerate(y)]
    res = outer_loop(recs, target=0.95)
    assert 0.01 <= res["alpha_star"] <= 0.6
    g = {w["U"]: w["gamma"] for w in res["whatif"]}
    assert g[0.5] >= g[0.9] >= g[1.0]          # coverage falls as U grows
    assert res["U_star"] is not None and g[res["U_star"]] >= 0.95


def test_analysis_on_synthetic_fleet():
    sim = FleetSim(M=3, seed=5)
    recs = [g.to_dict() for g in sim.graphs(600)]
    res = analyze(recs, sim.dtp, criterion="bytes", B=100)
    assert 0 < res["MASE"]["value"] < 10
    assert 0 < res["gamma"]["value"] <= 1
    assert res["J"] > 0.9
    for k in ("gamma", "phi"):
        lo, hi = res[k]["ci"]
        assert lo <= res[k]["value"] <= hi


def test_statistics():
    rng = np.random.default_rng(0)
    assert abs(tau_int(rng.normal(size=5000)) - 0.5) < 0.2
    ar = np.zeros(5000)
    for k in range(1, 5000):
        ar[k] = 0.9 * ar[k - 1] + rng.normal()
    assert tau_int(ar) > 5          # tau_int = (1 + 0.9) / (2 (1 - 0.9)) = 9.5 for AR(1)
    lo, hi, blk = block_bootstrap_ci(ar, B=200)
    assert lo < 0.3 and hi > -0.3 and blk > 1
    assert clopper_pearson_upper(0, 1000) == pytest.approx(1 - 0.05 ** (1 / 1000), rel=1e-6)
    assert jain([1, 1, 1]) == pytest.approx(1.0)


def test_collector_reads_regulator_log():
    dtp = DTP.load(ROOT / "dtp.json")
    clock = StepClock(dtp.Ts, 1000.0, now=lambda: 1000.0)
    asm = Assembler(dtp, clock)
    log = {"t": 3, "fwd": [1, 4, 1, 1, 1, 2, 3], "held": [0, 1, 0, 0, 0, 0, 1]}
    asm.on_message("boat/b2/regulator", json.dumps(log).encode(), at=1000.0 + 4 * dtp.Ts + 0.001)
    g = asm.open[3]
    assert g.edge(2, 2).fwd == 4 and g.edge(2, 7).held == 1


@pytest.mark.skipif(not (BUILD / "fdt_chmon").exists(), reason="needs make")
def test_channel_monitor_replay(tmp_path):
    dtp = DTP.load(ROOT / "dtp.json")
    rows, at, tm, busy, rxb, txb, rxp, txp, txf = [], 0.0, 0, 0, 0, 0, 0, 0, 0
    steps = [(30, 2000, 10, 50_000, 40, 0), (60, 0, 0, 0, 0, 0), (110, 30_000, 25, 1000, 10, 2)]
    for b, drx, nrx, dtx, ntx, nf in [(0, 0, 0, 0, 0, 0)] + steps:
        tm += 125 if rows else 0
        busy += b
        rxb += drx; rxp += nrx; txb += dtx; txp += ntx; txf += nf
        rows.append({"at": at, "survey": 1, "time_ms": tm, "busy_ms": busy, "rx_bytes": rxb, "tx_bytes": txb,
                     "rx_packets": rxp, "tx_packets": txp, "tx_failed": txf})
        at += 0.125
    f = tmp_path / "s.jsonl"
    f.write_text("".join(json.dumps(r) + "\n" for r in rows))
    out = subprocess.run([str(BUILD / "fdt_chmon"), "--dtp", str(ROOT / "dtp.json"), "--replay", str(f), "--fast"],
                         capture_output=True, text=True, check=True).stdout.split("\n")
    got = [json.loads(l) for l in out if l]
    assert len(got) == 3
    R, Ts, h = dtp.R_PHY, dtp.Ts, dtp.h_W_busy      # C7: the busy part of h_W per frame
    for g, (b, drx, nrx, dtx, ntx, nf) in zip(got, steps):
        Tb = b / 1000.0
        acked = ntx - nf
        To = (8 * (drx + (dtx * acked / ntx if ntx else 0)) + (nrx + acked) * 8 * h) / R
        y = min(R * Ts / 8, max(0.0, R * (Ts - Tb + To) / 8))
        assert g["y"] == pytest.approx(y, abs=0.1)
        assert g["Tb"] == pytest.approx(Tb, abs=1e-6)


def test_raw_load_header():
    sys.path.insert(0, str(ROOT / "vessel" / "publishers"))
    import raw_load
    c = raw_load.content(3, 77, 40)
    assert len(c) == 40 and c[0] == 3 and int.from_bytes(c[2:6], "big") == 77
    assert len(raw_load.content(1, 1, 8)) == 8


def test_vessel_programs_parse_as_python37():
    for p in (ROOT / "vessel" / "publishers").glob("*.py"):
        ast.parse(p.read_text(), filename=str(p), feature_version=(3, 7))


def test_zed_split_frame():
    cv2 = pytest.importorskip("cv2")
    sys.path.insert(0, str(ROOT / "vessel" / "publishers"))
    import zed_publisher
    jpeg = bytes(range(256)) * 200
    msgs = zed_publisher.split_frame(jpeg, 5, 14000)
    assert all(len(m) <= 14000 for m in msgs)
    assert b"".join(m[8:] for m in msgs) == jpeg
    assert [int.from_bytes(m[4:6], "big") for m in msgs] == list(range(len(msgs)))


def test_mav_layouts_fit_the_bounds():
    pytest.importorskip("pymavlink")
    sys.path.insert(0, str(ROOT / "vessel" / "publishers"))
    import mav_publisher as mp
    from pymavlink.dialects.v20 import ardupilotmega as m
    dtp = DTP.load(ROOT / "dtp.json")
    bound = {s.name: s.bound for s in dtp.specs}
    for name, lay in mp.LAYOUTS.items():
        assert lay.size <= bound[name], name
    gpi = m.MAVLink_global_position_int_message(1000, -300000000, -510000000, 12000, 0, 150, -20, 0, 9000)
    att = m.MAVLink_attitude_message(1000, 0.1, -0.2, 1.5, 0.01, 0.02, 0.03)
    st = mp.decode("state", mp.encode_state(gpi, att))
    assert st[0] == -300000000 and st[1] == -510000000 and st[2] == pytest.approx(12.0)
    imu = m.MAVLink_scaled_imu2_message(2000, 1000, 0, -1000, 10, 20, 30, 1, 2, 3, 4500)
    v = mp.decode("imu", mp.encode_imu(imu, 1))
    assert v[2] == 2_000_000 and v[3] == pytest.approx(9.80665, rel=1e-6)
    gps = m.MAVLink_gps_raw_int_message(123456789, 3, -300000000, -510000000, 12000, 120, 200, 150, 9000, 11)
    assert mp.decode("gps", mp.encode_gps(gps))[1] == 123456789
