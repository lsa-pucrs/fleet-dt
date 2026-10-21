#!/usr/bin/env python3
"""Platform qualification Q of the paper (Section VI, Table of experiments; design Section 6):
real-time lag of the emulator with the traffic of the paper model, M in {2, 4} x 5 channel profiles
{S1, S2, S3, S6, S7} x K in {1, 3} concurrent sessions x 3 runs (60 runs), and the equivalence of K = 1
and K = 3 (TOST) on the main outcomes.

  python3 emu/experiments/qualify.py manifest --out DIR [--steps 1200] [--seed0 7000] [--dtp dtp.json]
          [--source-scale 1.0] [--U 0.8] [--bare-session]
      DIR/manifest.json and DIR/specs/*.json for emu/experiments/runner.py (paper stack, fleet.py):
      the K = 3 runs first, then the K = 1 runs with "max_K": 1 (they start only alone); the same seed
      for the K = 1 and the K = 3 run of a (M, profile, repetition), so the comparison is paired; every
      spec holds "session": the session of the paper campaign (emu/fleet.py PAPER_SESSION: CPU quota
      0.25, clock injector, one probe read per step), or with --bare-session the session of the pilot
      runs of Q (no quota, no injector, probe every 0.1 s)
  python3 emu/experiments/qualify.py analyze RESULTS [--manifest DIR/manifest.json] [--warm-steps 240]
          [--hard-limit-us 1000] [--delta-us 1000] [--criterion bytes] [--json OUT]

Inputs of a run directory (RESULTS/<id>/): emu/steplag.csv (fdt-emu --phaseLag: largest lag of each
phase of each step, catch-up flag), emu/summary.json (phase_lag.aborted), result.json (status), and for
the outcomes either outcomes.json ({"gamma", "phi", "Pi"}) or vms/gs/dta.jsonl with dtp.json
(fleetdt.analysis.run_metrics; Pi = mean share of the complete prefix of sigma_t); optional
vms/v<i>/outq.jsonl (steal time of the guests per step).

Rules (drafted Section VI, "Validity of the runs"; design Section 6):
  run validity  the emulator completed the run (status ok, summary present, not aborted) and the lag of
                every step from scenario step 0 stayed at or below the HardLimit (1 ms)
  step validity a step after the warm-up is valid when its lag in phases 2 and 3 stays below delta and
                it has no catch-up; the analysis reports the exclusion rate per method and per cell; a run
                with invalid steps stays valid for the run rule (worst-case analyses count such steps
                as failures)
  invalid runs  rate per method and per K, Fisher exact test (pairwise against the first method, and
                K = 1 against K = 3); the runner repeats an invalid run at most twice
  TOST          per cell (M, profile) and pooled: paired differences d = outcome(K = 3) - outcome(K = 1)
                over the repetitions with both runs valid; equivalence when the 90 % t interval of the
                mean of d lies inside (-margin, margin): gamma 0.005 (H2), phi 0.005 (H4b), Pi 0.05 (H5a);
                a cell that fails leaves the design
"""
import argparse
import csv
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "emu", "scenarios"))
sys.path.insert(0, os.path.join(ROOT, "emu", "tools"))
sys.path.insert(0, os.path.join(ROOT, "emu"))

PROFILES = ("S1", "S2", "S3", "S6", "S7")
MS = (2, 4)
KS = (1, 3)
REPS = 3
MARGINS = {"gamma": 0.005, "phi": 0.005, "Pi": 0.05}
PHASES = ("max1_us", "max2_us", "max3_us")


# ------------------------------------------------------------------------------------------------
# manifest

def q_session(bare=False, Ts=0.125):
    """"session" of the Q specs: the paper campaign (fleet.PAPER_SESSION, probe period Ts) or the bare
    session of the pilot runs"""
    import fleet
    return dict(fleet.BARE_SESSION) if bare else fleet.paper_session({"Ts": Ts})


def q_spec(profile, M, K, rep, seed, steps, dtp, U, scale, Ts=0.125, T1=0.020, T2=0.010, session=None):
    import gen
    dur = steps * Ts + 30.0
    rid = "Q-%s-m%d-k%d-r%d" % (profile, M, K, rep)
    return {"kind": "fleet", "stack": "paper", "name": rid, "exp": "Q", "method": "fleetdt", "M": M, "Ts": Ts,
            "seed": seed, "steps": steps, "lead_s": 15, "dtp": dtp,
            "scenario": gen.make_paper(profile, M, seed, dur),
            "plugin_mode": "paper", "as": {"U": U},
            "dti": {"enabled": True, "offset_ms": 1.0, "stagger_ms": 0.0, "idle": 8},
            "jmcs": {"period_s": 10, "first": "survey", "video_prefix": "video"},
            "monitor": {"offset_us": 500, "jitter_us": 1000, "quant": "harvest", "harvest_ms": 0, "sigma": 0.0,
                        "backend": "auto"},
            "source": {"scale": scale, "size": "random", "min_frac": 0.5},
            "rt": "HardLimit",
            # the paper stack passes the per-step lag of each phase (--phaseLag from the DTP); record only
            # (--abortLagUs=0: the analysis applies the 1 ms rule), ns-3's own HardLimit of 50 ms stays as
            # a backstop
            "abort_lag_us": 0,
            "session": dict(session) if session is not None else q_session(False, Ts),
            "qualify": {"profile": profile, "K": K, "rep": rep}}


def write_manifest(out, steps=1200, seed0=7000, dtp="dtp.json", U=0.8, scale=1.0, order_seed=20261016,
                   bare_session=False):
    os.makedirs(os.path.join(out, "specs"), exist_ok=True)
    runs = []
    for pi, prof in enumerate(PROFILES):
        for M in MS:
            for rep in range(REPS):
                seed = seed0 + 100 * pi + 10 * M + rep
                for K in KS:
                    s = q_spec(prof, M, K, rep, seed, steps, dtp, U, scale, session=q_session(bare_session))
                    p = os.path.join(out, "specs", s["name"] + ".json")
                    with open(p, "w") as f:
                        json.dump(s, f)
                    r = {"id": s["name"], "spec": os.path.relpath(p, out), "M": M, "exp": "Q", "method": "fleetdt",
                         "scenario": prof, "K": K, "rep": rep, "seed": seed, "status": "pending",
                         "phy": s["scenario"]["phy"], "Ts": 0.125}
                    if K == 1:
                        r["max_K"] = 1
                    runs.append(r)
    rng = random.Random(order_seed)
    rng.shuffle(runs)
    runs.sort(key=lambda r: (r["K"] == 1, r["M"]))
    man = {"name": os.path.basename(os.path.abspath(out)), "cmd": "Q", "steps": steps, "order_seed": order_seed,
           "runs": runs, "note": "run with emu/experiments/runner.py DIR --K 3 (the K = 3 runs need three slots)"}
    with open(os.path.join(out, "manifest.json"), "w") as f:
        json.dump(man, f, indent=1)
    return man


# ------------------------------------------------------------------------------------------------
# per-run inputs

def read_steplag(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({"k": int(r["k"]), "max1_us": float(r["max1_us"]), "max2_us": float(r["max2_us"]),
                         "max3_us": float(r["max3_us"]), "lag_p3_us": float(r["lag_p3_us"]),
                         "lag_end_us": float(r["lag_end_us"]), "catchup": int(r["catchup"]), "n": int(r["n"])})
    return rows


def load_json(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def run_outcomes(rd, warm_steps, criterion):
    """{"gamma", "phi", "Pi"} of a run (None when not available)"""
    o = load_json(os.path.join(rd, "outcomes.json"))
    if o is not None:
        return {k: (float(o[k]) if o.get(k) is not None else None) for k in MARGINS}
    dta = os.path.join(rd, "vms", "gs", "dta.jsonl")
    dtpp = os.path.join(rd, "dtp.json")
    if not (os.path.exists(dta) and os.path.exists(dtpp)):
        return None
    try:
        sys.path.insert(0, ROOT)
        from fleetdt.analysis import run_metrics
        from fleetdt.as_model import DTP
        from fleetdt.outer import load_dta
    except ImportError:
        return None
    recs = load_dta(dta)
    res = load_json(os.path.join(rd, "result.json")) or {}
    S0 = int(res.get("S0") or (min(int(r["t"]) for r in recs) if recs else 0))
    recs = [r for r in recs if int(r["t"]) >= S0 + warm_steps]
    if not recs:
        return None
    m = run_metrics(recs, DTP.load(dtpp), criterion=criterion, ci=False)
    # keys of the pre-registered margins (gamma = PICP, phi = 1 - DMR, Pi = PCR with denominator |N(t)|, C21)
    return {"gamma": m["PICP"]["value"], "phi": m["phi"]["value"], "Pi": m["PCR"]["value"]}


def steal_summary(rd):
    import outq
    logs = outq.vessel_logs(rd)
    out = {}
    for i, p in sorted(logs.items()):
        _, steps = outq.load(p)
        v = [float(r.get("steal_ms", -1)) for r in steps.values() if float(r.get("steal_ms", -1)) >= 0]
        if v:
            out["v%d" % i] = {"mean_ms_per_step": sum(v) / len(v), "max_ms": max(v)}
    return out


def run_record(rd, warm_steps=240, hard_us=1000.0, delta_us=1000.0, criterion="bytes", expected_steps=None):
    """validity, lag statistics and outcomes of one run directory"""
    rec = {"dir": rd, "completed": False, "valid": False, "reasons": []}
    res = load_json(os.path.join(rd, "result.json")) or {}
    summ = load_json(os.path.join(rd, "emu", "summary.json"))
    sl = os.path.join(rd, "emu", "steplag.csv")
    rows = read_steplag(sl) if os.path.exists(sl) else []
    status = res.get("status")
    aborted = bool(((summ or {}).get("phase_lag") or {}).get("aborted"))
    if status != "ok":
        rec["reasons"].append("status %s" % status)
    if summ is None:
        rec["reasons"].append("no emulator summary")
    if aborted:
        rec["reasons"].append("emulator aborted on the lag")
    if not rows:
        rec["reasons"].append("no steplag.csv")
    if expected_steps and len(rows) < expected_steps:
        rec["reasons"].append("steplag has %d of %d steps" % (len(rows), expected_steps))
    rec["completed"] = status == "ok" and summ is not None and not aborted and bool(rows)
    lag_all = [max(r[p] for p in PHASES) for r in rows]
    rec["max_lag_us"] = max(lag_all) if lag_all else None
    if lag_all and rec["max_lag_us"] > hard_us:
        rec["reasons"].append("lag %.0f us above the HardLimit %.0f us" % (rec["max_lag_us"], hard_us))
    rec["valid"] = rec["completed"] and bool(lag_all) and rec["max_lag_us"] <= hard_us and \
        not (expected_steps and len(rows) < expected_steps)
    post = [r for r in rows if r["k"] >= warm_steps]
    rec["steps"] = len(rows)
    rec["steps_after_warmup"] = len(post)
    bad = [r for r in post if r["max2_us"] >= delta_us or r["max3_us"] >= delta_us or r["catchup"]]
    rec["invalid_steps"] = len(bad)
    rec["catchup_steps"] = sum(r["catchup"] for r in post)
    rec["phase_lag"] = {p: [r[p] for r in post] for p in PHASES}
    rec["outcomes"] = run_outcomes(rd, warm_steps, criterion) if rec["completed"] else None
    rec["steal"] = steal_summary(rd)
    return rec


# ------------------------------------------------------------------------------------------------
# statistics

def quantile(v, q):
    if not v:
        return None
    s = sorted(v)
    return s[min(len(s) - 1, int(q * len(s)))]


def lag_table(values, delta_us=1000.0):
    """per phase: p50, p99, p999, max of the per-step largest lag, and the share of steps >= delta"""
    out = {}
    for p in PHASES:
        v = values.get(p, [])
        out[p[:4]] = {"n": len(v), "p50": quantile(v, 0.5), "p99": quantile(v, 0.99), "p999": quantile(v, 0.999),
                      "max": max(v) if v else None,
                      "over_delta": (sum(1 for x in v if x >= delta_us) / len(v)) if v else None}
    return out


def tost_paired(d, margin, alpha=0.05):
    """TOST of mean(d) inside (-margin, margin) with the (1 - 2 alpha) t interval; d are the signed paired
    differences (K1 - K3), never their absolute values (C20)"""
    from scipy.stats import t as tdist
    x = [float(v) for v in d if v is not None and not math.isnan(v)]
    n = len(x)
    if n < 2:
        return {"n": n, "mean": x[0] if n else None, "ci90": None, "p": 1.0, "equivalent": False, "margin": margin}
    m = sum(x) / n
    sd = math.sqrt(sum((v - m) ** 2 for v in x) / (n - 1))
    se = sd / math.sqrt(n)
    if se == 0:
        eq = -margin < m < margin
        return {"n": n, "mean": m, "ci90": [m, m], "p": 0.0 if eq else 1.0, "equivalent": eq, "margin": margin}
    df = n - 1
    p_low = float(tdist.sf((m + margin) / se, df))
    p_high = float(tdist.cdf((m - margin) / se, df))
    q = float(tdist.ppf(1 - alpha, df))
    p = max(p_low, p_high)
    return {"n": n, "mean": m, "se": se, "ci90": [m - q * se, m + q * se], "p": p, "equivalent": p < alpha,
            "margin": margin}


def fisher(a_bad, a_n, b_bad, b_n):
    from scipy.stats import fisher_exact
    if a_n == 0 or b_n == 0:
        return None
    return float(fisher_exact([[a_bad, a_n - a_bad], [b_bad, b_n - b_bad]])[1])


def analyze(runs, warm_steps=240, hard_us=1000.0, delta_us=1000.0):
    """runs: list of {"id", "M", "scenario", "K", "rep", "method", "rec": run_record(...)}"""
    rep = {"rules": {"warm_steps": warm_steps, "hard_limit_us": hard_us, "delta_us": delta_us, "margins": MARGINS}}
    # lag per cell (M, profile, K) and per method, valid runs only
    cells, methods = {}, {}
    for r in runs:
        rec = r["rec"]
        if not rec["valid"]:
            continue
        for key, dst in (("%s-m%d-k%d" % (r["scenario"], r["M"], r["K"]), cells), (r["method"], methods)):
            acc = dst.setdefault(key, {"values": {p: [] for p in PHASES}, "runs": 0, "steps": 0, "invalid_steps": 0,
                                       "catchup_steps": 0})
            for p in PHASES:
                acc["values"][p] += rec["phase_lag"][p]
            acc["runs"] += 1
            acc["steps"] += rec["steps_after_warmup"]
            acc["invalid_steps"] += rec["invalid_steps"]
            acc["catchup_steps"] += rec["catchup_steps"]

    def table(dst):
        out = {}
        for k, acc in sorted(dst.items()):
            out[k] = {"runs": acc["runs"], "steps": acc["steps"],
                      "step_exclusion": acc["invalid_steps"] / acc["steps"] if acc["steps"] else None,
                      "catchup": acc["catchup_steps"] / acc["steps"] if acc["steps"] else None,
                      "lag_us": lag_table(acc["values"], delta_us)}
        return out
    rep["lag_by_cell"] = table(cells)
    rep["lag_by_method"] = table(methods)
    # invalid runs
    inv = {}
    for r in runs:
        for key in ("method:" + r["method"], "K:%d" % r["K"]):
            a = inv.setdefault(key, [0, 0])
            a[0] += 0 if r["rec"]["valid"] else 1
            a[1] += 1
    rep["invalid_runs"] = {k: {"invalid": v[0], "runs": v[1], "rate": v[0] / v[1]} for k, v in sorted(inv.items())}
    ms = sorted({r["method"] for r in runs})
    fz = {}
    if len(ms) > 1:
        a = inv["method:" + ms[0]]
        for m in ms[1:]:
            b = inv["method:" + m]
            fz["%s vs %s" % (ms[0], m)] = fisher(a[0], a[1], b[0], b[1])
    if "K:1" in inv and "K:3" in inv:
        fz["K1 vs K3"] = fisher(inv["K:1"][0], inv["K:1"][1], inv["K:3"][0], inv["K:3"][1])
    rep["fisher_invalid"] = fz
    # TOST K = 1 vs K = 3, paired by (M, profile, rep, method)
    pairs = {}
    for r in runs:
        key = (r["method"], r["scenario"], r["M"], r["rep"])
        pairs.setdefault(key, {})[r["K"]] = r["rec"]
    diffs = {}
    for (m, prof, M, rp), d in sorted(pairs.items()):
        if 1 not in d or 3 not in d or not (d[1]["valid"] and d[3]["valid"]):
            continue
        o1, o3 = d[1].get("outcomes"), d[3].get("outcomes")
        if not o1 or not o3:
            continue
        for k in MARGINS:
            if o1.get(k) is None or o3.get(k) is None:
                continue
            diffs.setdefault(("%s-m%d" % (prof, M), k), []).append(o3[k] - o1[k])
            diffs.setdefault(("pooled", k), []).append(o3[k] - o1[k])
    tost = {}
    for (cell, k), d in sorted(diffs.items()):
        tost.setdefault(cell, {})[k] = tost_paired(d, MARGINS[k])
    for cell, t in tost.items():
        t["all_equivalent"] = all(v["equivalent"] for k, v in t.items() if k in MARGINS) and \
            all(k in t for k in MARGINS)
    rep["tost_K1_vs_K3"] = tost
    rep["cells_kept"] = sorted(c for c, t in tost.items() if c != "pooled" and t["all_equivalent"])
    rep["cells_dropped"] = sorted({"%s-m%d" % (r["scenario"], r["M"]) for r in runs} - set(rep["cells_kept"]))
    rep["runs"] = [{"id": r["id"], "valid": r["rec"]["valid"], "reasons": r["rec"]["reasons"],
                    "max_lag_us": r["rec"]["max_lag_us"], "invalid_steps": r["rec"]["invalid_steps"],
                    "steps_after_warmup": r["rec"]["steps_after_warmup"], "outcomes": r["rec"]["outcomes"],
                    "steal": r["rec"]["steal"]} for r in runs]
    return rep


def collect(results, manifest=None, warm_steps=240, hard_us=1000.0, delta_us=1000.0, criterion="bytes"):
    if manifest:
        man = load_json(manifest)
        entries = man["runs"]
        steps = man.get("steps")
    else:
        entries, steps = [], None
        for d in sorted(os.listdir(results)):
            sp = load_json(os.path.join(results, d, "experiment.json")) or {}
            q = sp.get("qualify") or {}
            if not q:
                continue
            entries.append({"id": d, "M": sp["M"], "scenario": q["profile"], "K": q["K"], "rep": q["rep"],
                            "method": sp.get("method", "fleetdt")})
    runs = []
    for e in entries:
        rd = os.path.join(results, e["id"])
        if not os.path.isdir(rd):
            continue
        rec = run_record(rd, warm_steps, hard_us, delta_us, criterion, expected_steps=steps)
        runs.append({"id": e["id"], "M": e["M"], "scenario": e["scenario"], "K": e["K"], "rep": e["rep"],
                     "method": e.get("method", "fleetdt"), "rec": rec})
    return runs


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("manifest")
    m.add_argument("--out", required=True)
    m.add_argument("--steps", type=int, default=1200)
    m.add_argument("--seed0", type=int, default=7000)
    m.add_argument("--dtp", default="dtp.json")
    m.add_argument("--U", type=float, default=0.8)
    m.add_argument("--source-scale", type=float, default=1.0)
    m.add_argument("--bare-session", action="store_true",
                   help="the session of the pilot runs (no CPU quota, no clock injector, probe every 0.1 s)")
    a_ = sub.add_parser("analyze")
    a_.add_argument("results")
    a_.add_argument("--manifest", default=None)
    a_.add_argument("--warm-steps", type=int, default=240)
    a_.add_argument("--hard-limit-us", type=float, default=1000.0)
    a_.add_argument("--delta-us", type=float, default=1000.0)
    a_.add_argument("--criterion", default="bytes", choices=("bytes", "count"))
    a_.add_argument("--json", default=None)
    a = ap.parse_args()
    if a.cmd == "manifest":
        man = write_manifest(a.out, a.steps, a.seed0, a.dtp, a.U, a.source_scale, bare_session=a.bare_session)
        print("%d runs -> %s" % (len(man["runs"]), a.out))
        return 0
    runs = collect(a.results, a.manifest, a.warm_steps, a.hard_limit_us, a.delta_us, a.criterion)
    rep = analyze(runs, a.warm_steps, a.hard_limit_us, a.delta_us)
    if a.json:
        with open(a.json, "w") as f:
            json.dump(rep, f, indent=1)
    print("runs %d, valid %d" % (len(runs), sum(r["rec"]["valid"] for r in runs)))
    for k, v in rep["lag_by_method"].items():
        lt = v["lag_us"]
        print("method %s: steps %d, excluded %.4f, lag p99 (us) phase 1/2/3: %s / %s / %s" % (
            k, v["steps"], v["step_exclusion"] or 0, lt["max1"]["p99"], lt["max2"]["p99"], lt["max3"]["p99"]))
    for c, t in rep["tost_K1_vs_K3"].items():
        print("TOST %s: %s" % (c, ", ".join("%s %s (n %d)" % (k, "eq" if v["equivalent"] else "not eq", v["n"])
                                             for k, v in t.items() if k in MARGINS)))
    print("cells kept: %s" % rep["cells_kept"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
