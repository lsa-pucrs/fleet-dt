"""Tests of the hypothesis-level driver (emu/analysis/hypotheses.py) on a small synthetic campaign: per run a DTA,
the regulator-free inputs of paper_post (steplag.csv, csub/c_t.jsonl, clock.csv, h1.jsonl), paper_post.post for
metrics.json, and a manifest with the fields of emu/README.md ("Hypothesis driver")."""
import json
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "emu", "analysis"))

import hypotheses as H  # noqa: E402
import paper_post  # noqa: E402
from fleetdt.as_model import DTP, admission  # noqa: E402

TS = 0.125
DTP_RAW = {"epoch": 0.0, "Ts": TS, "T1": 0.02, "T2": 0.01, "alpha": 0.125, "U": 0.8, "C0": 300000, "c_b": 1.0,
           "c_m": 0.0, "vessels": [1, 2, 3, 4],
           "topics": [{"name": "state", "rate": 8, "bound": 48}, {"name": "imu", "rate": 32, "bound": 48},
                      {"name": "video", "rate": 48, "bound": 1300, "jitter": 0}],
           "order": ["state", "imu", "video"], "needed": ["state", "imu"]}


def make_run(rd, seed, steps=300, M=4, U=0.8, level=300_000.0, fill=0.88, bad_imu=0.0, bad_state=0.0,
             r_scale=None, u_scale=1.0, absent=0.0):
    """One synthetic paper-stack run: y(t) of a local level model, C_hat(t) of eq. (2), the demand tables sized so
    that Algorithm 1 admits every topic with the fill ratio `fill`, the received messages complete except in a
    fraction bad_imu (imu of every vessel short: PCR 1/2) and bad_state (state of vessels 2 to 4 short) of the
    steps; r_scale: received bytes per vessel (BDR_i); u_scale: received bytes of every edge (u)."""
    rng = np.random.default_rng(seed)
    os.makedirs(os.path.join(rd, "vms", "gs"), exist_ok=True)
    os.makedirs(os.path.join(rd, "emu"), exist_ok=True)
    os.makedirs(os.path.join(rd, "csub"), exist_ok=True)
    json.dump(DTP_RAW, open(os.path.join(rd, "dtp.json"), "w"))
    json.dump({"status": "ok", "S0": 0, "steps": steps, "M": M}, open(os.path.join(rd, "result.json"), "w"))
    dtp = DTP.from_dict(DTP_RAW)
    r3 = dtp.T3m / dtp.Ts
    mu = level
    C = None
    recs, cs, h1 = [], [], []
    y_prev = None
    for t in range(steps):
        mu += rng.normal(0, 1500.0)
        y = mu + rng.normal(0, 3000.0)
        C = level if C is None else 0.875 * C + 0.125 * y_prev
        vessels = [i for i in range(1, M + 1) if not (absent and i == M and rng.random() < absent)]
        b = U * C * r3 / len(vessels)
        D = {i: [100, 600, max(0, int(fill * b) - 700)] for i in vessels}
        p = admission(D, [1, 2, 3], [1, 2], b)
        bi, bs = rng.random() < bad_imu, rng.random() < bad_state
        edges = []
        for i in vessels:
            for l in (1, 2, 3):
                n = {1: 1, 2: 4, 3: 6}[l]
                got = n
                if (l == 2 and bi) or (l == 1 and i >= 2 and bs):
                    got = n - 1
                rs = (r_scale or {}).get(i, 1.0) * u_scale
                edges.append({"i": i, "l": l, "d": D[i][l - 1], "p": p[l - 1], "fwd": n, "held": 0, "n": got,
                              "r": int(D[i][l - 1] * rs * got / n), "dup": 0, "late": 0})
        recs.append({"t": t, "y": y, "C_hat": C, "K_hat": C * r3, "U": U, "alpha": 0.125, "b": b, "Q": 0,
                     "vessels": vessels, "needed": [1, 2], "order": [1, 2, 3], "edges": edges})
        cs.append({"t": t, "C": y})
        h1.append({"t": t, "w1": 0.004 + 0.001 * rng.random(), "w2": 0.001 + 0.002 * rng.random()})
        y_prev = y
    with open(os.path.join(rd, "vms", "gs", "dta.jsonl"), "w") as f:
        f.write("".join(json.dumps(r) + "\n" for r in recs))
    with open(os.path.join(rd, "csub", "c_t.jsonl"), "w") as f:
        f.write("".join(json.dumps(r) + "\n" for r in cs))
    with open(os.path.join(rd, "h1.jsonl"), "w") as f:
        f.write("".join(json.dumps(r) + "\n" for r in h1))
    with open(os.path.join(rd, "emu", "steplag.csv"), "w") as f:
        f.write("k,max1_us,max2_us,max3_us,lag_p3_us,lag_end_us,catchup,n\n")
        for k in range(steps):
            f.write("%d,100.0,100.0,100.0,50.0,50.0,0,700\n" % k)
    with open(os.path.join(rd, "clock.csv"), "w") as f:
        f.write("host_ns,host,offset_ns,rtt_ns\n")
        for k in range(0, steps, 10):
            for h, off in (("gs", 20_000), ("v1", 150_000), ("v2", -80_000), ("v3", 0), ("v4", 60_000)):
                f.write("%d,%s,%d,50000\n" % (int(k * TS * 1e9) + 1000, h, off))
    out = paper_post.post(rd, warm=0)
    with open(os.path.join(rd, "metrics.json"), "w") as f:
        json.dump(out, f)
    return out


def build_campaign(root):
    runs = []
    k = [0]

    def add(exp, cell, block, method, factors, **kw):
        rid = "%s-%s-b%d-%s" % (exp, cell, block, method)
        k[0] += 1
        make_run(os.path.join(root, rid), seed=1000 + k[0], **kw)
        runs.append({"id": rid, "exp": exp, "cell": cell, "block": {"seed": 100 + block, "slot": 0},
                     "method": method, "factors": factors, "status": "done"})
    for s in ("S1", "S3"):                                                     # calibration DTA
        for b in range(3):
            add("Cal", "cal-%s" % s, b, "fleetdt", {"M": 4, "scenario": s, "load": "saturated"})
    for load in ("idle", "saturated"):                                         # E1, 800 steps: n >= 2995 pairs
        for b in range(4):
            add("E1", "E1-%s" % load, b, "fleetdt", {"M": 4, "scenario": "S1", "load": load}, steps=800)
    for s in ("S1", "S3"):                                                     # E2
        for b in range(6):
            add("E2", "E2-%s" % s, b, "fleetdt", {"M": 4, "scenario": s})
    for U, fill in ((0.8, 0.88), (1.0, 0.5)):                                 # E3: the U = 1.0 cell fails the gate
        for b in range(3):
            add("E3", "E3-U%.1f" % U, b, "fleetdt", {"M": 4, "scenario": "S1", "rho": 1.5, "U": U}, U=U, fill=fill)
    for b in range(6):                                                         # E4
        f = {"M": 4, "scenario": "S3", "rho": 1.5}
        add("E4", "E4-r1.5", b, "fleetdt", f)
        add("E4", "E4-r1.5", b, "B1", f, bad_imu=0.4)
        add("E4", "E4-r1.5", b, "B4", f, bad_imu=0.1)
        add("E4", "E4-r1.5", b, "B5a", f, bad_state=0.3)
        g = {"M": 4, "scenario": "S5", "rho": 1.5, "fairness": True}
        add("E4", "E4-fair", b, "fleetdt", g)
        add("E4", "E4-fair", b, "B6", g, r_scale={1: 1.0, 2: 0.8, 3: 0.6})
        e = {"M": 4, "rho": 1.5, "regimes": ["S1", "S3", "S1"], "regime_changes": [100, 200]}
        add("E5", "E5", b, "O3", e)
        add("E5", "E5", b, "O1", e, u_scale=0.8)
        add("E5", "E5", b, "O2", e, u_scale=0.9)
        add("E5", "E5", b, "O4", e, u_scale=0.95)
    # a pending run stays out of the analysis
    runs.append({"id": "missing", "exp": "E4", "cell": "E4-r1.5", "block": {"seed": 100, "slot": 0}, "method": "B9",
                 "factors": {"M": 4, "rho": 1.5}, "status": "pending"})
    man = {"name": "synthetic", "cmd": "campaign", "runs": runs,
           "calibration": {"eps_s": {"4": 0.002}, "T2_s": {"4": 0.010}, "PICP_nom": 0.99, "N_w": 40}}
    with open(os.path.join(root, "manifest.json"), "w") as f:
        json.dump(man, f)
    return man


@pytest.fixture(scope="module")
def campaign(tmp_path_factory):
    root = str(tmp_path_factory.mktemp("campaign"))
    build_campaign(root)
    res = H.run(root, B=200, seed=1, arima_engine="css", burn=20)
    return root, res


def comps(res, cid):
    return [c for c in res["components"] if c["id"] == cid]


def test_outputs_and_table(campaign):
    root, res = campaign
    assert os.path.exists(os.path.join(root, "hypotheses.json"))
    md = open(os.path.join(root, "hypotheses.md")).read()
    assert md.startswith("| ID | Estimate | Bound or p | Threshold | Verdict |")
    ids = [r["id"] for r in res["rows"]]
    assert ids == ["H1a", "H1b", "H1c", "H2", "H3a", "H3b", "H3c", "H4a", "H4b", "H4c", "H4d", "H5", "H6a", "H6b"]
    assert all(("| %s |" % i) in md for i in ids)
    assert {"id": "missing", "why": "status pending"} in res["dropped"]


def test_h1_components(campaign):
    _, res = campaign
    a, = comps(res, "H1a")
    # 6400 steps of w2 < T2 - eps, i.i.d.: n_eff near n, CP upper 1 - 0.05^(1/n_eff) < 1e-3
    assert a["n"] == 6400 and a["events"] == 0 and a["reject"] and a["bound"] < 1e-3
    assert a["bound"] == pytest.approx(1 - 0.05 ** (1 / a["n_eff"]))
    b, = comps(res, "H1b")
    assert b["n"] == 4 * 6400 and b["events"] == 0 and b["reject"]
    c, = comps(res, "H1c")
    assert c["estimate"] == pytest.approx(130e-6) and c["reject"]      # |150 - 20| us < eps = 2 ms


def test_h2_and_calibration(campaign):
    _, res = campaign
    cs = comps(res, "H2")
    assert sorted(c["cell"] for c in cs) == ["E2-S1", "E2-S3"]
    for c in cs:   # C(t) = y(t) and c_y = 1: PICP = PICP_C in every step
        assert c["estimate"] == 0.0 and c["bound"] == 0.0 and c["reject"] and 0.3 <= c["U_star"] <= 1.0
    cal = res["calibration"]
    assert set(cal) == {"fit/M=4/without S1", "fit/M=4/without S3"}
    for v in cal.values():   # alpha* on the grid {0.01, ..., 1}
        assert v["alpha_star"] == pytest.approx(round(v["alpha_star"], 2))


def test_h3_components(campaign):
    _, res = campaign
    a = comps(res, "H3a")
    assert len(a) == 4
    gen = [c for c in a if c["status"] == "tested"]
    assert len(gen) == 2 and all(c["excluded_cells"] == ["E3-U1.0"] for c in gen)
    steps = [c for c in gen if c["cell"].startswith("steps")][0]
    assert steps["n"] == 3 * 300 and steps["events"] == 0
    assert not steps["reject"]                          # 900 steps: n_eff < 2995, the CP limit stays above 1e-3
    assert [c["status"] for c in a if "stratum" in c["cell"]] == ["descriptive", "descriptive"]
    b = comps(res, "H3b")
    assert [c["cell"] for c in b] == ["E3-U0.8"] and b[0]["estimate"] == 0.0 and b[0]["reject"]
    c, = comps(res, "H3c")
    assert c["status"] == "descriptive"


def test_h4_components(campaign):
    _, res = campaign
    a = {c["cell"]: c for c in comps(res, "H4a")}
    assert set(a) == {"E4-r1.5 vs B1"}                   # B9 pending; B4 and B5a enter H4b and H4c only
    assert a["E4-r1.5 vs B1"]["p"] == pytest.approx(2.0 ** -6) and a["E4-r1.5 vs B1"]["reject"]
    b, = comps(res, "H4b")
    assert b["n"] == 6 and b["estimate"] > 0.02
    c = comps(res, "H4c")
    assert sorted(x["cell"] for x in c) == ["E4-r1.5 vs B5a, vessel %d" % v for v in (2, 3, 4)]
    assert all(x["reject"] for x in c)
    d, = comps(res, "H4d")
    assert d["estimate"] > 0 and d["reject"] and d["p"] == pytest.approx(2.0 ** -6)


def test_h5_components(campaign):
    _, res = campaign
    cs = comps(res, "H5")
    names = {c["cell"].split(" vs ")[1] for c in cs}
    assert names == set(H.P.PREDICTORS)
    for c in cs:
        assert c["n"] == 6 and c["test"] == "SR" and c["threshold"] == 1.05
    # ES with alpha* is the optimal predictor of the local level model: it beats the last value and the moving average
    for c in cs:
        if c["cell"].endswith("last_value"):
            assert c["estimate"] < 1.0 and c["reject"]


def test_h6_components(campaign):
    _, res = campaign
    a, = comps(res, "H6a")
    assert a["test"] == "TOST" and a["n"] == 6 + 6               # E3 runs (both cells) and E4 Fleet-DT runs
    b = {c["cell"]: c for c in comps(res, "H6b")}
    assert b["E5, PICP of O3"]["n"] == 6
    for m, du in (("O1", 0.2), ("O2", 0.1), ("O4", 0.05)):
        c = b["E5, u of O3 vs %s" % m]
        assert c["reject"] and c["estimate"] > 0.01 and c["p"] == pytest.approx(2.0 ** -6)


def test_sequence_stops_at_the_first_failure(campaign):
    _, res = campaign
    seq = {s["id"]: s for s in res["sequence"]}
    assert seq["H1"]["rejected"] and seq["H2"]["rejected"]
    assert seq["H3"]["tested"] and not seq["H3"]["rejected"]   # H3a: too few steps for the CP limit
    assert all(not seq[h]["tested"] for h in ("H4", "H5", "H6"))
    rows = {r["id"]: r for r in res["rows"]}
    assert rows["H1a"]["verdict"] == "supported" and rows["H3a"]["verdict"] == "not supported"
    assert rows["H3c"]["verdict"] == "descriptive"
    assert rows["H4a"]["verdict"] == "not tested (sequence ended)"


def test_summarize_iut_and_no_data():
    c = lambda cid, rej, **kw: dict({"id": cid, "status": "tested", "reject": rej, "p": 0.01 if rej else 0.2,
                                     "cell": "x", "threshold": 0.0}, **kw)
    comps_ = [c("H1a", True), c("H1b", True), c("H1c", True), c("H2", True), c("H2", True, cell="y"),
              c("H3a", True), c("H3b", True), c("H4a", True), c("H4a", False, cell="z"), c("H4b", True),
              c("H4c", True), c("H4d", True)]
    s = H.summarize(comps_, 0.05)
    assert s["hypotheses"]["H1"] and s["hypotheses"]["H2"] and s["hypotheses"]["H3"]
    assert s["hypotheses"]["H4"] is False                       # IUT: one cell of H4a fails
    rows = {r["id"]: r for r in s["rows"]}
    assert rows["H4a"]["worst"]["cell"] == "z" and rows["H4a"]["verdict"] == "not supported"
    assert rows["H5"]["verdict"] == "no data" and s["hypotheses"]["H5"] is None
    assert [x["tested"] for x in s["sequence"]] == [True, True, True, True, False, False]


def test_descriptive_rule():
    mk = lambda **f: H.Run("/x", {"id": "r", "exp": f.pop("exp", "E2"), "cell": "c", "factors": f}, 0)
    assert H.is_descriptive(mk(scenario="S4")) and H.is_descriptive(mk(queue="codel"))
    assert H.is_descriptive(mk(ablation="R8")) and H.is_descriptive(mk(exp="E4", rho=1.0))
    assert not H.is_descriptive(mk(exp="E4", rho=1.5)) and not H.is_descriptive(mk(scenario="S3"))
    r = H.Run("/x", {"id": "r", "exp": "E2", "cell": "c", "descriptive": True, "factors": {"scenario": "S1"}}, 0)
    assert H.is_descriptive(r)


def test_invalid_runs_and_blocks(tmp_path):
    root = str(tmp_path)
    for rid in ("a", "b", "c", "d"):
        make_run(os.path.join(root, rid), seed=7, steps=60)
    m = json.load(open(os.path.join(root, "b", "metrics.json")))
    m["rt_valid"] = False
    json.dump(m, open(os.path.join(root, "b", "metrics.json"), "w"))
    runs = [{"id": "a", "exp": "E4", "cell": "c1", "block": "s1", "method": "fleetdt", "factors": {"M": 4}},
            {"id": "b", "exp": "E4", "cell": "c1", "block": "s2", "method": "fleetdt", "factors": {"M": 4}},
            {"id": "c", "exp": "E4", "cell": "c1", "block": "s3", "method": "B1", "factors": {"M": 4},
             "block_invalid": True}]
    json.dump({"runs": runs}, open(os.path.join(root, "manifest.json"), "w"))
    c = H.load_campaign(root)
    assert [r.id for r in c["runs"]] == ["a"]
    assert {d["id"]: d["why"] for d in c["dropped"]} == {"b": "not real-time valid", "c": "invalid block"}
    assert [r.id for r in H.load_campaign(root, keep_invalid=True)["runs"]] == ["a", "b"]
    # a rerun of the same (exp, cell, block, method) replaces the earlier run
    runs.append({"id": "d", "exp": "E4", "cell": "c1", "block": "s1", "method": "fleetdt", "factors": {"M": 4}})
    json.dump({"runs": runs}, open(os.path.join(root, "manifest.json"), "w"))
    assert [r.id for r in H.load_campaign(root)["runs"]] == ["d"]
