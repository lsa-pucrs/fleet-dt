"""End-to-end AS v2 over a local Mosquitto with fake vessels, the operation-control stub and a fake AP
monitor. Checks the tables, the DTA, f(t), kappa(t), the analysis, and the phase-2 delay."""
import json
import os
import subprocess
import sys
import time

import numpy as np
import pytest

from fleetdt.as2 import dtp as D
from fleetdt.as2.analysis import analyze, load
from fleetdt.as2.service import Service

from .harness import Broker, mosquitto_bin

pytestmark = pytest.mark.skipif(mosquitto_bin() is None, reason="no mosquitto binary")


def run_fleet(tmp_path, M, steps, mode="fleetdt", drop=None):
    """AS in this process; vessels, the operation-control stub and the monitor in a child process."""
    d = D.example(vessels=list(range(1, M + 1)))
    d.epoch = time.time() + 1.5          # step 1 starts in 1.5 s
    br = Broker()
    log = tmp_path / ("dta_%s_%d.jsonl" % (mode, M))
    out = tmp_path / ("fleet_%s_%d.json" % (mode, M))
    svc = Service(d, "127.0.0.1", br.port, mode=mode, log_path=str(log))
    child = None
    try:
        svc.connect()
        arg = json.dumps({"dtp": d.raw, "epoch": d.epoch, "port": br.port, "steps": steps, "out": str(out),
                          "drop": {str(k): list(v) for k, v in (drop or {}).items()}})
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        child = subprocess.Popen([sys.executable, "-m", "tests.as2.harness", arg], cwd=root)
        svc.run(steps=steps + 1, first=1)
        child.wait(30)
    finally:
        svc.stop()
        if child and child.poll() is None:
            child.kill()
        br.stop()
    fleet = json.loads(out.read_text())
    assert not fleet["errors"], fleet["errors"]
    return d, svc, fleet, load(str(log))


def test_e2e_four_vessels(tmp_path):
    steps = 40
    d, svc, fleet, recs = run_fleet(tmp_path, 4, steps, drop={2: (20,)})
    asr = [r for r in recs if r["type"] == "as"]
    assert [r["t"] for r in asr] == list(range(1, steps + 2))
    # every vessel got the table of (almost) every step before v_i(t), with L levels, counts and the order
    for i in d.vessels:
        got = fleet["tables_in_time"][str(i)]
        assert len(got) >= steps - 3, (i, len(got))
        tb = fleet["last_table"][str(i)]
        assert len(tb["p"]) == d.L and len(tb["k"]) == d.L and sorted(tb["o"]) == list(range(d.L))
        assert tb["p"][0] == 2 and tb["k"][0] == 1
        assert fleet["goals"][str(i)] >= 1
    # vessel 2 dropped its demand of step 20: outside M(20), its state is forced with the arrival bound,
    # f(19) = 1 (no report of step 19), and the next steps recover
    r20 = asr[19]
    assert 2 not in r20["M"] and [2, 0, 1] in r20["A"]
    assert asr[20]["f_reasons"] is not None
    assert asr[19]["f_prev"] == 1 and [2, "noreport"] in asr[19]["f_reasons"]
    # survey mode (steps 1-8) admits video for needed vessels; transit (9-16) ranks video last
    assert any(r["k"]["1"][6] > 0 for r in asr[1:8])
    assert asr[10]["rank"][6] == d.L - 1
    # kappa(t), z(t) from the monitor; f(t) mostly 0 on loopback
    kap = [r["kappa_prev"] for r in asr[2:] if r["kappa_prev"] is not None]
    assert len(kap) >= steps - 5 and 0.35 < np.median(kap) < 0.65
    f = [r["f_prev"] for r in asr[1:]]
    assert sum(f) <= 6, [r["f_reasons"] for r in asr if r["f_prev"]]
    # DTA: acceptances of the state (AoI) and the vessel logs
    acc = [r for r in recs if r["type"] == "acc"]
    vl = [r for r in recs if r["type"] == "vlog"]
    assert len(acc) >= 4 * (steps - 3) * 2 and len(vl) >= 4 * (steps - 3)
    res = analyze(recs, d, ts_offset=0, skip=2)
    assert res["theorem1"]["holds_all_prefixes"]
    assert res["deadline_miss"] is not None and res["deadline_miss"] < 0.05
    assert res["needed_delivery"] is not None and res["needed_delivery"] > 0.3
    for i in d.vessels:
        a = res["aoi"][str(i)]
        assert a["n"] > 50 and a["max"] < 2 * d.Ts + d.T3 + d.eps + 0.05
    # phase 2 within T2
    tm = svc.timing_summary()
    print("\nphase 2 (M=4):", json.dumps(tm))
    assert tm["pub_ms"]["p50"] < d.T2 * 1e3


@pytest.mark.parametrize("mode", ["slotted", "b4", "b1", "tdma", "ipact"])
def test_e2e_modes(tmp_path, mode):
    steps = 12
    d, svc, fleet, recs = run_fleet(tmp_path, 3, steps, mode=mode)
    asr = [r for r in recs if r["type"] == "as"]
    assert len(asr) == steps + 1 and all(r["mode"] == mode for r in asr)
    got = sum(len(v) for v in fleet["tables"].values())
    if mode == "b1":
        assert got == 0
    else:
        assert got >= 3 * (steps - 2)
    if mode in ("slotted", "tdma", "ipact"):
        sl = [(r["t"], s) for r in asr for s in r["slot"].values()]
        assert sl
        for t, (s0, s1) in sl:
            w, we = d.window(t)
            assert int(w * 1e6) - 1 <= s0 < s1 <= int(we * 1e6) + 1
        assert any("slot" in (x or {}) for x in fleet["last_table"].values())
    if mode == "b4":
        assert all(r["Z"] == 1.0 for r in asr)


def test_e2e_sixteen_vessels_phase2_timing(tmp_path):
    """Phase 2 for M = 16, L = 7 with video outputs (survey mode for every vessel)."""
    steps = 48
    d, svc, fleet, recs = run_fleet(tmp_path, 16, steps)
    asr = [r for r in recs if r["type"] == "as"]
    m = [len(r["M"]) for r in asr[:steps]]
    assert np.median(m) == 16 and sum(1 for x in m if x >= 14) >= 0.9 * steps
    assert max(r["n_cand"] for r in asr) >= 16 * 6
    tm = svc.timing_summary()
    comp = sorted(r["t_compute_ms"] for r in asr)
    print("\nphase 2 (M=16): compute ms p50 %.2f p99 %.2f max %.2f; deadline->published ms p50 %.2f p99 %.2f max %.2f;"
          " PUBACK ms max %s" % (comp[len(comp) // 2], comp[int(0.99 * (len(comp) - 1))], comp[-1],
                                 tm["pub_ms"]["p50"], tm["pub_ms"]["p99"], tm["pub_ms"]["max"],
                                 max((r.get("ack_ms_prev") or 0) for r in asr)))
    assert tm["pub_ms"]["p50"] < d.T2 * 1e3
    assert tm["over_T2"] <= 2
    assert comp[len(comp) // 2] < 0.5 * d.T2 * 1e3
