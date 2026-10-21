"""Campaign generator of the paper (emu/experiments/campaign.py): the cells, levels and run counts of Table S6, the
blocks (seed, slot, methods in random order), the seed ranges, the calibrated specs of E1 to E5 (DTP constants,
rho by C_ref, pilot U of B4, B6, B7, outer-loop variants of E5), the manifest fields of the hypothesis driver,
the validation of every spec with the checks of emu/fleet.py; the pilot U of fleetdt/calibration.py; the block
slots of emu/experiments/runner.py. No emulator, VM or run."""
import collections
import json
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (ROOT, os.path.join(ROOT, "emu"), os.path.join(ROOT, "emu", "experiments"),
          os.path.join(ROOT, "emu", "analysis"), os.path.join(ROOT, "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import campaign as C  # noqa: E402
import fleet  # noqa: E402
import runner  # noqa: E402
from fleetdt import calibration as CAL  # noqa: E402
from fleetdt import outer as O  # noqa: E402

SCEN = ("S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8", "S9")


def synthetic_calibration(pilot=True, U_S3=0.82):
    """calibration.json in the format of calibrate_paper.py compute"""
    M = {"2": {"T1": 0.0141, "T2": 0.0071, "eps": 0.0021, "delta_GS": 0.0008, "T_m": 0.0029, "B_ctl_bar": 970.25},
         "4": {"T1": 0.0162, "T2": 0.0083, "eps": 0.0023, "delta_GS": 0.0009, "T_m": 0.0032, "B_ctl_bar": 1210.5}}
    left = {}
    for m, base in (("2", 0.2), ("4", 0.3)):
        left[m] = {s: {"alpha": base, "U": 0.85, "scenarios": []} for s in ("S1", "S2", "S6", "S7", "S5", "all")}
        left[m]["S3"] = {"alpha": base + 0.1, "U": U_S3, "scenarios": []}
    cal = {"version": 1, "params": {"PICP_nom": 0.99, "warm_steps": 240}, "c_y": 0.93, "c_y_n": 1000, "M": M,
           "C_ref": {"2": {s: 310000.0 for s in ("S1", "S2", "S3", "S6", "S7")},
                     "4": dict({s: 290000.0 for s in ("S1", "S2", "S3", "S6", "S7")}, S3=250000.0, S5=270000.0)},
           "outer": {m: {"T3_Ts": 0.7, "c_y": 0.93, "B_ctl_bar": M[m]["B_ctl_bar"], "left_out": left[m]}
                     for m in M},
           "runs": [], "pilot": {}}
    if pilot:
        cal["pilot"] = {"4": {"S3": {"B4": {"U": 0.7}, "B6": {"U": 0.75}, "B7": {"U": 0.66}}}}
    return cal


@pytest.fixture(scope="module")
def cal_phase(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("cal"))
    mans = C.main(["cal", "--out", d])
    return d, mans


@pytest.fixture(scope="module")
def eval_phase(tmp_path_factory):
    d = str(tmp_path_factory.mktemp("eval"))
    cp = os.path.join(d, "calibration.json")
    json.dump(synthetic_calibration(), open(cp, "w"))
    mans = C.main(["eval", "--calibration", cp, "--out", os.path.join(d, "camp")])
    return os.path.join(d, "camp"), mans, cp


def specs(d, man):
    return {r["id"]: json.load(open(os.path.join(d, man["cmd"], r["spec"]))) for r in man["runs"]}


# ------------------------------------------------------------------------------------------------ counts


def test_counts_of_table_s6(capsys):
    r = C.count()
    for exp in C.EXPERIMENTS:
        assert r["experiments"][exp]["run_equiv"] == C.TABLE_S6[exp]
    assert {k: v["runs"] for k, v in r["experiments"].items()} == \
        {"E0": 26, "Cal": 210, "E1": 80, "E2": 290, "E3": 190, "E4": 676, "E5": 60}
    assert r["experiments"]["E5"]["run_equiv"] == pytest.approx(156.0)
    assert r["total"] == pytest.approx(1773) and r["Q"] == 60 and r["reruns"] == 85
    assert "Q" not in r["experiments"]
    C.main(["count"])
    out = capsys.readouterr().out
    assert "total 1773.0" in out and "E4    676 runs" in out


def test_count_follows_the_options():
    o = C.opts(m2_methods=("fleetdt", "B1"), low_methods=("fleetdt", "B1", "B3"))
    r = C.count(o)
    assert r["experiments"]["E4"]["runs"] == 440 + 2 * 20 + 3 * 2 * 12 + 40


# ------------------------------------------------------------------------------------------------ cal phase


def test_cal_phase_cells(cal_phase):
    d, mans = cal_phase
    e0, cal = mans["E0"], mans["Cal"]
    assert len(e0["runs"]) == 26 and len(cal["runs"]) == 210
    # E0: M x {S1, S3} x 5 and M x outages of 1, 3, 5 s x 1
    main = collections.Counter((r["M"], r["scenario"]) for r in e0["runs"] if r["part"] == "main")
    assert main == {(2, "S1"): 5, (2, "S3"): 5, (4, "S1"): 5, (4, "S3"): 5}
    sp = specs(d, e0)
    outs = sorted((r["M"], sp[r["id"]]["outage"]["dur_s"]) for r in e0["runs"] if r["part"] == "outage")
    assert outs == [(2, 1.0), (2, 3.0), (2, 5.0), (4, 1.0), (4, 3.0), (4, 5.0)]
    assert all(s["pcap"] for s in sp.values())
    # Cal: E1 cells and saturated E2 cells x 10 (S1 saturated in both), idle x 3, S5, pilot
    parts = collections.Counter(r["part"] for r in cal["runs"])
    assert parts == {"E1": 40, "E2": 100, "idle": 24, "S5": 13, "pilot": 33}
    s1sat = collections.Counter(r["M"] for r in cal["runs"] if r["scenario"] == "S1" and
                                r["factors"]["load"] == "saturated")
    assert s1sat == {2: 20, 4: 20}
    idle = collections.Counter((r["scenario"], r["M"]) for r in cal["runs"] if r["part"] == "idle")
    assert idle == {(s, M): 3 for s in ("S2", "S3", "S6", "S7") for M in (2, 4)}
    s5 = collections.Counter((r["M"], r["factors"]["load"]) for r in cal["runs"] if r["part"] == "S5")
    assert s5 == {(4, "saturated"): 10, (4, "idle"): 3}
    pil = [r for r in cal["runs"] if r["part"] == "pilot"]
    assert collections.Counter(r["method"] for r in pil) == {m: 3 for m in C.E4_METHODS}
    assert {(r["M"], r["scenario"]) for r in pil} == {(4, "S3")}
    assert len({r["block"]["seed"] for r in pil}) == 3
    csp = specs(d, cal)
    # the saturation replays of the Cal runs (not of the pilot), B4 with the nominal C, no calibration
    assert all(bool(csp[r["id"]].get("replay_subsets")) == (r["part"] != "pilot") for r in cal["runs"])
    b4 = [csp[r["id"]] for r in pil if r["method"] == "B4"]
    assert all(s["as"]["C_ref"] == 300000.0 and "U" not in s["as"] for s in b4)
    assert cal["calibration"] is None and e0["calibration"] is None


def test_load_levels(cal_phase):
    d, mans = cal_phase
    sp = specs(d, mans["Cal"])
    for r in mans["Cal"]["runs"]:
        s = sp[r["id"]]
        v = s["campaign"]["video"]
        if r["factors"]["load"] == "idle":
            assert s["vessels"] == {str(i): {"video": False} for i in range(1, r["M"] + 1)}
            assert v["demand"] == 23000 * r["M"]        # the six telemetry topics of a vessel
        elif r["factors"]["load"] == "saturated":
            assert s["dtp"]["video_layers"]["fps"] == 8.0 and s["dtp"]["video_layers"]["frame_cap"] == 98774
            assert s["source"]["scale"] == 1.0 and v["demand"] > 2 * 300000     # above K(t) in every step
        else:   # pilot: rho 1.5 of the nominal C, overload of vessel 1
            assert v["rho_plan"] == pytest.approx(1.5, rel=0.01) and s["overload"]["vessel"] == 1


def test_blocks_seeds_and_order(cal_phase, eval_phase):
    d, mans = cal_phase
    ed, emans, _ = eval_phase
    cal_seeds = {r["seed"] for m in mans.values() for r in m["runs"]}
    ev_seeds = {r["seed"] for m in emans.values() for r in m["runs"]}
    assert not cal_seeds & ev_seeds
    assert max(r["seed"] for r in mans["E0"]["runs"]) < min(r["seed"] for r in mans["Cal"]["runs"])
    for m in list(mans.values()) + list(emans.values()):
        assert m["order_seed"] == C.DEFAULTS["order_seed"] and m["K"] == 3
        by = collections.defaultdict(list)
        for k, r in enumerate(m["runs"]):
            assert r["block"]["seed"] == r["seed"] and 0 <= r["block"]["slot"] < 3
            by[r["seed"]].append((k, r))
        for seed, rr in by.items():
            # one block: one slot, one M, consecutive in the manifest, methods once each
            assert len({r["block"]["slot"] for _, r in rr}) == 1 and len({r["M"] for _, r in rr}) == 1
            ks = [k for k, _ in rr]
            assert ks == list(range(ks[0], ks[0] + len(ks)))
            assert len({(r["cell"], r["method"]) for _, r in rr}) == len(rr)
        # grouped by M
        Ms = [r["M"] for r in m["runs"]]
        assert Ms == sorted(Ms)
    # E4: the 11 methods of a cell in one block, in a random order of the block
    e4 = emans["E4"]
    blocks = collections.defaultdict(list)
    for r in e4["runs"]:
        blocks[r["seed"]].append(r)
    main = [b for b in blocks.values() if b[0]["part"] == "main"]
    assert len(main) == 40 and all(sorted(r["method"] for r in b) == sorted(C.E4_METHODS) for b in main)
    assert len({tuple(r["method"] for r in b) for b in main}) > 1
    # E3: the 8 U levels of a scenario in one block; the 3 ablations in one block
    e3 = collections.defaultdict(list)
    for r in emans["E3"]["runs"]:
        e3[r["seed"]].append(r)
    for b in e3.values():
        if b[0]["part"] == "U":
            assert sorted(r["factors"]["U"] for r in b) == list(C.U_LEVELS) and len({r["cell"] for r in b}) == 8
        else:
            assert sorted(r["factors"]["ablation"] for r in b) == ["R7", "R8", "R9"]


def test_generation_is_deterministic(tmp_path, cal_phase):
    d, mans = cal_phase
    m2 = C.main(["cal", "--out", str(tmp_path / "again"), "--only", "E0"])
    assert [r["id"] for r in m2["E0"]["runs"]] == [r["id"] for r in mans["E0"]["runs"]]
    m3 = C.main(["cal", "--out", str(tmp_path / "other"), "--only", "E0", "--order-seed", "5"])
    assert [r["id"] for r in m3["E0"]["runs"]] != [r["id"] for r in mans["E0"]["runs"]]
    assert sorted(r["id"] for r in m3["E0"]["runs"]) == sorted(r["id"] for r in mans["E0"]["runs"])
    # the seeds of Cal do not move when only E0 is written
    m4 = C.main(["cal", "--out", str(tmp_path / "calonly"), "--only", "Cal", "--no-validate"])
    assert sorted(r["seed"] for r in m4["Cal"]["runs"]) == sorted(r["seed"] for r in mans["Cal"]["runs"])
    with pytest.raises(SystemExit):
        C.main(["cal", "--out", str(tmp_path / "x"), "--seed-cal", "8010"])     # overlaps E0
    with pytest.raises(SystemExit):
        C.main(["cal", "--out", str(tmp_path / "x"), "--only", "E1"])


# ------------------------------------------------------------------------------------------------ eval phase


def test_eval_phase_cells(eval_phase):
    d, mans, _ = eval_phase
    n = {k: len(m["runs"]) for k, m in mans.items()}
    assert n == {"E1": 80, "E2": 290, "E3": 190, "E4": 676, "E5": 60}
    e1 = collections.Counter((r["M"], r["factors"]["load"]) for r in mans["E1"]["runs"])
    assert e1 == {(M, l): 20 for M in (2, 4) for l in ("idle", "saturated")}
    assert {r["scenario"] for r in mans["E1"]["runs"]} == {"S1"}
    e2 = collections.Counter(r["part"] for r in mans["E2"]["runs"])
    assert e2 == {"main": 200, "descriptive": 30, "queue": 40, "alpha": 20}
    main = collections.Counter((r["scenario"], r["M"]) for r in mans["E2"]["runs"] if r["part"] == "main")
    assert main == {(s, M): 20 for s in ("S1", "S2", "S3", "S6", "S7") for M in (2, 4)}
    q = collections.Counter((r["scenario"], r["factors"]["queue"], r["M"]) for r in mans["E2"]["runs"]
                            if r["part"] == "queue")
    assert q == {(s, x, 4): 10 for s in ("S1", "S3") for x in ("q1000", "codel")}
    al = collections.Counter(r["factors"]["alpha"] for r in mans["E2"]["runs"] if r["part"] == "alpha")
    assert al == {"alpha*": 10, 1.0: 10}
    e3 = collections.Counter((r["scenario"], r["factors"]["U"]) for r in mans["E3"]["runs"] if r["part"] == "U")
    assert e3 == {(s, u): 10 for s in ("S1", "S3") for u in C.U_LEVELS}
    e4 = collections.Counter((r["part"], r["M"], r["factors"]["rho"], r["method"]) for r in mans["E4"]["runs"])
    for m in C.E4_METHODS:
        assert e4[("main", 4, 1.5, m)] == 20 and e4[("main", 4, 2.0, m)] == 20
    for m in ("fleetdt", "B1", "B3", "B6", "B7"):
        assert e4[("M2", 2, 1.5, m)] == 20
    for m in ("fleetdt", "B1", "B3", "B6"):
        assert e4[("low", 4, 0.5, m)] == 12 and e4[("low", 4, 1.0, m)] == 12
    assert e4[("fairness", 4, 1.5, "fleetdt")] == 20 and e4[("fairness", 4, 1.5, "B6")] == 20
    e5 = collections.Counter(r["method"] for r in mans["E5"]["runs"])
    assert e5 == {v: 12 for v in C.VARIANTS}
    assert sum(r["run_equiv"] for r in mans["E5"]["runs"]) == pytest.approx(156.0)


def test_manifest_fields_of_the_hypothesis_driver(eval_phase):
    d, mans, cp = eval_phase
    cal = synthetic_calibration()
    for m in mans.values():
        c = m["calibration"]
        assert c["eps_s"] == {"2": 0.0021, "4": 0.0023} and c["T2_s"] == {"2": 0.0071, "4": 0.0083}
        assert c["PICP_nom"] == 0.99 and c["N_w"] == 480 and c["C_ref"]["4/S3"] == 250000.0
        assert c["pilot_U"] == {"B4": 0.7, "B6": 0.75, "B7": 0.66}
        for r in m["runs"]:
            assert set(r["factors"]) == {"M", "scenario", "rho", "U", "load", "alpha", "queue", "ablation",
                                         "fairness", "regimes", "regime_changes"}
            assert r["status"] == "pending" and set(r["block"]) == {"seed", "slot"}
            f = r["factors"]
            want = bool(f["scenario"] in ("S4", "S8", "S9") or f["queue"] or f["ablation"] or
                        (r["exp"] == "E4" and f["rho"] is not None and f["rho"] <= 1.0))
            assert r["descriptive"] == want
    assert sum(r["descriptive"] for r in mans["E2"]["runs"]) == 70
    assert sum(r["descriptive"] for r in mans["E3"]["runs"]) == 30
    assert sum(r["descriptive"] for r in mans["E4"]["runs"]) == 96
    e5 = mans["E5"]["runs"][0]["factors"]
    assert e5["regimes"] == ["S1", "S3", "S1"] and e5["regime_changes"] == [1040, 2080]
    assert cal["c_y"] == mans["E1"]["calibration"]["c_y"]


def test_calibrated_specs(eval_phase):
    d, mans, cp = eval_phase
    cal = synthetic_calibration()
    sp = {}
    for m in mans.values():
        sp.update({r["id"]: (r, s) for r, s in zip(m["runs"], [json.load(open(os.path.join(d, m["cmd"], r["spec"])))
                                                               for r in m["runs"]])})
    for rid, (r, s) in sp.items():
        M, sid = r["M"], r["scenario"]
        dtp = s["dtp"]
        g = cal["M"][str(M)]
        for k in ("T1", "T2", "T_m", "B_ctl_bar"):
            assert dtp[k] == g[k]
        assert dtp["c_y"] == 0.93 and s["steps"] == (3120 if r["exp"] == "E5" else 1200)
        key = "S1" if sid == "SEQ" else sid
        a, U = CAL.lookup_outer(cal, M, key)
        assert dtp["alpha"] == a and dtp["U"] == U
        m = s["method"]
        if r["exp"] == "E5":
            assert m == "fleetdt" and s["outer"]["variant"] == r["method"] and "as" not in s
            assert ("calibration_file" in s["outer"]) == (r["method"] in ("O2", "O4", "O5"))
            if r["method"] in ("O1", "O3"):
                assert (s["outer"]["alpha0"], s["outer"]["U0"]) == (0.125, 0.8)
        elif m == "B4":
            assert s["as"] == {"C_ref": cal["C_ref"][str(M)]["S3"], "U": 0.7}
        elif m == "B6":
            assert s["as"] == {"U": 0.75} and r["factors"]["alpha"] == "alpha*"
        elif m == "B7":
            assert s["feedback"] == {"U": 0.66} and "as" not in s
        elif r["exp"] == "E3" and r["part"] == "U":
            assert s["as"] == {"U": r["factors"]["U"]}
        elif r["part"] == "alpha" and r["factors"]["alpha"] == 1.0:
            assert s["as"] == {"alpha": 1.0}
        elif m in ("fleetdt", "B5a", "B5b"):
            assert "as" not in s and r["factors"]["U"] == U and r["factors"]["alpha"] == "alpha*"
        # rho of the cells with rho: the demand tables sum to rho C_ref within the planning step
        v = s["campaign"]["video"]
        if r["factors"]["rho"] is not None and v["floor"]:
            # rho 0.5 at M = 4 with C_ref 250000: below the telemetry and one message per stripe topic
            assert (r["factors"]["rho"], M) == (0.5, 4) and v["demand"] == 179800 and v["frame_cap"] == 1
            assert {"M": 4, "rho": 0.5, "C": 250000.0, "rho_floor": 179800 / 250000.0} in mans["E4"]["rho_infeasible"]
        elif r["factors"]["rho"] is not None:
            Cr = cal["C_ref"][str(M)][sid] if sid != "SEQ" else (2 * 290000.0 + 250000.0) / 3
            assert v["C"] == pytest.approx(Cr) and v["demand"] <= r["factors"]["rho"] * Cr
            assert v["demand"] == pytest.approx(r["factors"]["rho"] * Cr, rel=0.01)
            assert s["source"]["scale"] <= 1.0 and dtp["video_layers"]["frame_cap"] <= 98774
        # E4 overload at rho >= 1.5 in every method, the fairness cells included; the replays of E2
        ov = r["exp"] == "E4" and r["factors"]["rho"] >= 1.5
        assert ("overload" in s) == ov
        assert bool(s.get("replay_subsets")) == (r["exp"] == "E2")
        if r["factors"]["fairness"]:
            assert s["vessels"] == fleet.FAIRNESS_VESSELS and r["scenario"] == "S5"
            assert dtp["video_layers"]["vessels"]["4"]["video"] is False
        if r["factors"]["ablation"]:
            assert s["ablation"] == r["factors"]["ablation"] and r["factors"]["U"] == CAL.lookup_outer(cal, 4, "S3")[1]
        if r["factors"]["queue"]:
            assert s["scenario"]["queue_variant"] == r["factors"]["queue"]
        assert s["session"] == fleet.paper_session({"Ts": 0.125}) and s["scenario"]["M"] == M
    # E5: the scenario of the regimes and the calibration next to the manifest
    r, s = next(v for v in sp.values() if v[0]["exp"] == "E5")
    assert s["scenario"]["scenario_id"] == "SEQ" and [x["scenario"] for x in s["scenario"]["regimes"]] == \
        ["S1", "S3", "S1"]
    assert os.path.isfile(os.path.join(d, "E5", "calibration.json"))


def test_every_spec_passes_the_checks_of_fleet(eval_phase, cal_phase):
    for d, mans in ((eval_phase[0], eval_phase[1]), cal_phase):
        for m in mans.values():
            md = os.path.join(d, m["cmd"])
            for r in m["runs"][::7]:
                s = json.load(open(os.path.join(md, r["spec"])))
                chk = C.validate(s, md)
                assert chk["method"] == fleet.paper_method(s)[0]
    # a spec that fleet.py refuses
    s = json.load(open(os.path.join(eval_phase[0], "E4", next(r["spec"] for r in eval_phase[1]["E4"]["runs"]
                                                               if r["method"] == "B4"))))
    del s["as"]["C_ref"]
    with pytest.raises(SystemExit):
        C.validate(s)
    s = json.load(open(os.path.join(eval_phase[0], "E5", eval_phase[1]["E5"]["runs"][0]["spec"])))
    s["as"] = {"U": 0.5}
    with pytest.raises(SystemExit):
        C.validate(s, os.path.join(eval_phase[0], "E5"))


def test_eval_refuses_an_incomplete_calibration(tmp_path):
    cp = str(tmp_path / "c.json")
    json.dump(synthetic_calibration(pilot=False), open(cp, "w"))
    with pytest.raises(SystemExit, match="pilot"):
        C.main(["eval", "--calibration", cp, "--out", str(tmp_path / "o"), "--only", "E4"])
    cal = synthetic_calibration()
    del cal["C_ref"]["4"]["S3"]
    json.dump(cal, open(cp, "w"))
    with pytest.raises(SystemExit, match="C_ref"):
        C.main(["eval", "--calibration", cp, "--out", str(tmp_path / "o"), "--only", "E3"])
    json.dump(synthetic_calibration(U_S3=None), open(cp, "w"))
    with pytest.raises(SystemExit, match="U"):
        C.main(["eval", "--calibration", cp, "--out", str(tmp_path / "o"), "--only", "E3"])
    m = C.main(["eval", "--calibration", cp, "--out", str(tmp_path / "o"), "--only", "E3", "--allow-missing"])["E3"]
    assert m["calibration_kept"] == [{"M": 4, "scenario": "S3", "kept": ["U"]}]
    r = next(r for r in m["runs"] if r["part"] == "ablation")
    assert r["factors"]["U"] == 0.8 and r["factors"]["alpha"] == "alpha*"       # the U of the DTP
    with pytest.raises(SystemExit):
        C.main(["eval", "--out", str(tmp_path / "o")])


# ------------------------------------------------------------------------------------------------ merge, runner


def test_merge_for_the_hypothesis_driver(tmp_path, cal_phase, eval_phase):
    import hypotheses as H
    d, mans = cal_phase
    ed, emans, _ = eval_phase
    cm = json.load(open(os.path.join(d, "Cal", "manifest.json")))
    for r in cm["runs"][:3]:
        r["status"], r["run_dir"] = "done", "/res/Cal/" + r["id"]
    p1 = str(tmp_path / "cal.json")
    json.dump(cm, open(p1, "w"))
    out = str(tmp_path / "res" / "manifest.json")
    os.makedirs(os.path.dirname(out))
    man = C.merge([p1] + [os.path.join(ed, e, "manifest.json") for e in ("E1", "E5")], out)
    assert len(man["runs"]) == 210 + 80 + 60 and man["calibration"] == emans["E1"]["calibration"]
    assert man["runs"][0]["dir"] == "/res/Cal/" + cm["runs"][0]["id"]
    run = H.Run(str(tmp_path / "res"), man["runs"][0], 0)
    assert run.dir == "/res/Cal/" + cm["runs"][0]["id"] and run.exp == "Cal"
    assert json.loads(run.block) == cm["runs"][0]["block"] and run.M == cm["runs"][0]["M"]
    c = H.load_campaign(str(tmp_path / "res"), out)
    assert not c["runs"] and len(c["dropped"]) == 350          # pending runs and runs without metrics
    with pytest.raises(SystemExit):
        C.merge([p1, p1], out)


def test_runner_keeps_a_block_on_its_slot(tmp_path):
    runs = [{"id": "a", "M": 4, "status": "pending", "block": {"seed": 1, "slot": 1}},
            {"id": "b", "M": 4, "status": "pending", "block": {"seed": 2, "slot": 0}},
            {"id": "c", "M": 4, "status": "pending"}]
    json.dump({"name": "x", "runs": runs}, open(str(tmp_path / "manifest.json"), "w"))
    man = runner.Manifest(str(tmp_path))
    r, _ = man.next(4, 0, K=3)
    assert r["id"] == "b"
    r, _ = man.next(4, 2, K=3)
    assert r["id"] == "c"
    r, why = man.next(4, 2, K=3)
    assert r is None and why == "exit"
    r, _ = man.next(4, 1, K=3)
    assert r["id"] == "a"
    assert runner.block_slot({"block": {"slot": 2}}, 2) == 0 and runner.block_slot({"block": {"slot": 2}}, 3) == 2
    assert runner.block_slot({"block": {"slot": 2}, "max_K": 1}, 3) == 0 and runner.block_slot({}, 3) is None
    # without K (--no-pin) any slot takes the first run
    runs[0]["status"] = "pending"
    json.dump({"name": "x", "runs": runs}, open(str(tmp_path / "manifest.json"), "w"))
    assert runner.Manifest(str(tmp_path)).next(4, 0)[0]["id"] == "a"
    s = runner.run_spec({"stack": "paper", "Ts": 0.125, "outer": {"variant": "O2", "calibration_file": "calibration.json"}},
                        "/r", "/m/E5")
    assert s["outer"]["calibration_file"] == "/m/E5/calibration.json"
    assert runner.run_spec({"outer": {"calibration_file": "/abs/c.json"}}, "/r", "/m")["outer"]["calibration_file"] == \
        "/abs/c.json"


# ------------------------------------------------------------------------------------------------ pilot U


@pytest.mark.parametrize("method", ["B4", "B6", "B7"])
def test_pilot_fit_follows_the_rule_of_U_star(method):
    rng = np.random.default_rng(3)
    ys = [300000.0 + 20000.0 * rng.standard_normal(500) for _ in range(3)]
    c_y, bctl, T3_Ts, ratio = 0.93, 1200.0, 0.7, 1.04
    f = CAL.pilot_fit(ys, method, c_y, bctl, T3_Ts, C_ref=280000.0, alpha=0.3, T3_ratio=ratio, B=300)
    # the same bootstrap on the prediction of the method
    rep = []
    for y in ys:
        z = c_y * y
        if method == "B4":
            Ch = np.full(len(y), 280000.0)
        elif method == "B6":
            Ch = O.replay(z, [0.3], z[0])[:, 0]
        else:
            Ch = np.r_[np.nan, y[:-1] * ratio]
        rep.append((y, Ch))
    gb = O.gamma_bounds(rep, O.U_GRID, B=300, T3_Ts=T3_Ts, c_y=c_y, B_ctl_bar=bctl)
    assert f["U"] == O.largest_U(gb["gamma_low"], O.U_GRID, 0.99) and f["runs"] == 3
    assert f["U"] is not None and f["PICP_low"]["%.2f" % f["U"]] >= 0.99
    nxt = "%.2f" % (f["U"] + 0.01)
    assert nxt not in f["PICP_low"] or f["PICP_low"][nxt] < 0.99


def test_pilot_fit_on_a_constant_channel():
    y = [np.full(400, 300000.0)]
    c_y, bctl, T3_Ts = 0.9, 0.0, 0.75
    # B4: K_y >= U K_hat for U <= c_y y / C_ref = 0.9 * 300000 / 300000
    assert CAL.pilot_fit(y, "B4", c_y, bctl, T3_Ts, C_ref=300000.0, B=50)["U"] == pytest.approx(0.9)
    # B6: eq. (2) on c_y y converges to c_y y: U* = 1
    assert CAL.pilot_fit(y, "B6", c_y, bctl, T3_Ts, alpha=0.2, B=50)["U"] == pytest.approx(1.0)
    # B7: the budget U y(t-1) T3/Ts against c_y y (T3 - T_m)/Ts: U <= c_y / ratio
    assert CAL.pilot_fit(y, "B7", c_y, bctl, T3_Ts, T3_ratio=1.2, B=50)["U"] == pytest.approx(0.75)
    assert CAL.pilot_fit(y, "B4", c_y, bctl, T3_Ts, B=50)["U"] is None          # no C_ref
    assert CAL.pilot_fit(y, "B1", c_y, bctl, T3_Ts, B=50)["U"] is None


def test_calibrate_paper_computes_the_pilot(tmp_path):
    import calibrate_paper as CP
    import test_calibration_paper as T
    root = str(tmp_path / "cal")
    os.makedirs(root)

    def mk(name, scen, seed, method=None, **kw):
        rd = T.make_run(root, name, scen, seed, M=4, **kw)
        if method:
            p = os.path.join(rd, "result.json")
            res = json.load(open(p))
            res["method"] = method
            json.dump(res, open(p, "w"))
        return rd
    mk("f1", "S1", 1)
    mk("f2", "S3", 2, noise=20000.0)
    for k, m in enumerate(("B4", "B6", "B7")):
        mk("p%d" % k, "S3", 10 + k, m)
    mk("b1", "S3", 20, "B1", ctl=9000, w1=0.05)       # another method: stays out of the constants
    out = str(tmp_path / "calibration.json")
    CP.main(["compute", root, "--out", out, "--warm", "10", "--B", "200"])
    cal = json.load(open(out))
    assert cal["M"]["4"]["runs"] == 2 and cal["M"]["4"]["T1"] < 0.05
    assert cal["M"]["4"]["B_ctl_bar"] < 9000
    assert [x["method"] for x in cal["ignored"]] == ["B1"]
    p = cal["pilot"]["4"]["S3"]
    assert set(p) == {"B4", "B6", "B7"}
    assert p["B4"]["C_ref"] == cal["C_ref"]["4"]["S3"] and p["B6"]["alpha"] == CAL.lookup_outer(cal, 4, "S3")[0]
    for m in ("B4", "B6", "B7"):
        assert p[m]["runs"] == 1 and p[m]["U"] is not None
    assert CAL.lookup_pilot(cal, "B6", 4) == (p["B6"]["U"], 4)
    assert CAL.lookup_pilot(cal, "B6", 2) == (p["B6"]["U"], 4)       # the M = 2 cells take the pilot at M = 4
    assert CAL.lookup_pilot(cal, "B6", 4, "S1") == (None, None)
    assert sorted(x["method"] for x in cal["pilot_runs"]) == ["B4", "B6", "B7"]
    # the eval phase reads it (the U of the M = 2 cells too)
    ctx = C.Context(C.opts(), cal, out)
    assert ctx.pilot_U("B7", 4) == p["B7"]["U"] and ctx.pilot_U("B4", 2) == p["B4"]["U"]
