#!/usr/bin/env python3
"""Hypothesis tests of Table hyp, paper-ready LaTeX tables and PDF figures from the metrics of the runs.

  python3 emu/analysis/report.py RESULTS [RESULTS ...] --out DIR [--dta-root RESULTS]

Reads RESULTS/*/metrics.json (emu/analysis/metrics.py) and writes DIR/hypotheses.json, DIR/runs.csv,
DIR/tab_*.tex (booktabs) and DIR/fig_*.pdf (matplotlib, one axis per figure).

| Hypothesis | Test (Section VI-E) |
|---|---|
| H1 | every run: no probe sample of a vessel with |offset| > eps (samples with a probe round trip <= 500 us); per scenario, the pooled fraction over its runs of the pairs (t, i) without the demand in phase 1 in the steps without interference <= 1e-3 |
| H2 | E1 runs: p99 of |kappa_AS - kappa| and of |b_AS - b| <= 0.01, and no step with X(t) <= kappa(t) T_W and f(t) = 1 without a lost table or report |
| H3 | E2: for each scenario, the runs of one seed paired; eq. (pred) against persistence, ES and the calibration mean; MAE (eq. smaller) and nu (eq. larger); Wilcoxon two-sided with Holm per metric (15 comparisons), Hodges-Lehmann estimate and 95 % interval |
| H4 | E2 and the ACI methods of E3 (Fleet-DT, slotted, B5): inequality (thm) at every prefix of every run, and f_bar <= beta + 0.02 in every run with F_R(T) = 0 |
| H5 | E3, rho in {1.5, 2}: Fleet-DT against B1, B2, B3, B5, TDMA in S1, S2, S4, S5: D_N (Fleet-DT larger) and the state miss ratio (Fleet-DT smaller); Wilcoxon with Holm over the 40 comparisons of each metric |
| H6 | E4: the what-if evaluation (fleetdt/as2/whatif.py) of the DTA of the E1 run of the same seed (S2, M = 4, Ts = 125 ms) for the measured E4 cells (M = 2; Ts = 62.5 and 250 ms) against the measured Fleet-DT runs: |delta f_bar| <= 0.02 and |delta nu| <= 0.05; M = 8 (--whatif DIR, the DTPs that matrix.py writes) is reported as a prediction without a measured counterpart |
| H7 | E4 Fleet-DT runs: no state delay above Ts + T3 + eps and no age above 2 Ts + T3 + eps in the steps that meet the hypothesis of Lemma 2 |
"""
import argparse
import csv
import glob
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
import stats as S  # noqa: E402

BASELINES_H5 = ("b1", "b2", "b3", "b5", "tdma")
ALT_PRED = ("persist", "es", "calmean")
NAMES = {"fleetdt": "Fleet-DT", "slotted": "Slotted", "b1": "B1", "b2": "B2", "b3": "B3", "b4": "B4", "b5": "B5",
         "tdma": "TDMA", "eq": "Eq. (pred)", "persist": "Persistence", "es": "ES of $\\kappa$",
         "calmean": "Calibration mean"}


def load(paths):
    out = []
    for root in paths:
        for p in sorted(glob.glob(os.path.join(root, "*", "metrics.json"))):
            with open(p) as f:
                m = json.load(f)
            m["_dir"] = os.path.dirname(p)
            out.append(m)
    return out


def g(m, *ks):
    v = m
    for k in ks:
        if v is None:
            return None
        v = v.get(k) if isinstance(v, dict) else None
    return v


def aoi_mean(m):
    v = [x["mean"] for x in (m.get("aoi") or {}).values() if x and x.get("mean") is not None]
    return float(np.mean(v)) if v else None


def fmt(x, d=3):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "--"
    return ("%%.%df" % d) % x


def fmt_p(p):
    if p is None:
        return "--"
    return "%.1e" % p if p < 1e-3 else "%.3f" % p


# ---------------------------------------------------------------------------------------- hypotheses
def h1(runs):
    bad_clock, by = [], {}
    for m in runs:
        c = g(m, "h1", "clock_over_eps")
        if (c or 0) > 0:
            bad_clock.append({"run": m["name"], "clock_over_eps": c})
        miss, pairs = g(m, "h1", "demand_missing"), g(m, "h1", "demand_pairs")
        if pairs is None and g(m, "h1", "demand_miss") is not None:      # metrics before demand_pairs
            pairs = (g(m, "h1", "quiet_steps") or 0) * (m.get("M") or 0)
            miss = int(round(g(m, "h1", "demand_miss") * pairs))
        if pairs:
            e = by.setdefault(m.get("scenario"), [0, 0, 0])
            e[0] += miss
            e[1] += pairs
            e[2] += 1
    pooled = {s: {"missing": e[0], "pairs": e[1], "runs": e[2], "fraction": e[0] / e[1]} for s, e in sorted(by.items())}
    bad_dem = [s for s, v in pooled.items() if v["fraction"] > 1e-3]
    return {"runs": len(runs), "clock_violations": bad_clock, "demand_pooled": pooled,
            "demand_violations": bad_dem, "holds": bool(runs) and not bad_clock and not bad_dem,
            "max_clock_abs_vessels_s": max([g(m, "h1", "clock_max_abs_vessels") or 0 for m in runs] or [None]),
            "max_demand_miss_run": max([g(m, "h1", "demand_miss") or 0 for m in runs] or [None])}


def h2(runs):
    rr = [m for m in runs if m.get("exp") in ("E1", "pilot") and m.get("method") == "fleetdt"]
    bad = []
    for m in rr:
        kp, bp = g(m, "h2", "kappa_absdiff", "p99"), g(m, "h2", "b_absdiff", "p99")
        cex = g(m, "h2", "lemma_counterexamples")
        if (kp is None or kp > 0.01) or (bp is None or bp > 0.01) or (cex or 0) > 0:
            bad.append({"run": m["name"], "kappa_p99": kp, "b_p99": bp, "lemma_cex": cex,
                        "cex_reasons": g(m, "h2", "lemma_cex_reasons")})
    return {"runs": len(rr), "violations": bad, "holds": not bad and bool(rr),
            "kappa_p99_max": max([g(m, "h2", "kappa_absdiff", "p99") or 0 for m in rr] or [None]),
            "b_p99_max": max([g(m, "h2", "b_absdiff", "p99") or 0 for m in rr] or [None]),
            "lemma_steps": sum(g(m, "h2", "lemma_steps") or 0 for m in rr),
            "lemma_counterexamples": sum(g(m, "h2", "lemma_counterexamples") or 0 for m in rr)}


def paired(runs, sel_a, sel_b, key, metric):
    """values of metric for the seeds that have a run in both selections, matched by key(m)"""
    A = {key(m): m for m in runs if sel_a(m)}
    B = {key(m): m for m in runs if sel_b(m)}
    ks = sorted(set(A) & set(B))
    xa = [metric(A[k]) for k in ks]
    xb = [metric(B[k]) for k in ks]
    ok = [i for i in range(len(ks)) if xa[i] is not None and xb[i] is not None]
    return [xa[i] for i in ok], [xb[i] for i in ok]


def family(comps, better):
    """comps: [(label, xa, xb)]; better: 'smaller' or 'larger' for xa (Fleet-DT / eq.)"""
    ps, rows = [], []
    for lab, xa, xb in comps:
        p, n = S.wilcoxon(xa, xb) if len(xa) >= 2 else (None, len(xa))
        est, ci = S.hodges_lehmann(xa, xb) if xa else (None, (None, None))
        ps.append(p if p is not None else 1.0)
        rows.append({"comparison": lab, "n": len(xa), "p": p, "hl": est, "ci95": ci,
                     "median_a": float(np.median(xa)) if xa else None, "median_b": float(np.median(xb)) if xb else None})
    adj = S.holm(ps) if ps else []
    for r, pa in zip(rows, adj):
        r["p_holm"] = float(pa)
        good = r["hl"] is not None and ((r["hl"] < 0) if better == "smaller" else (r["hl"] > 0))
        r["supports"] = bool(good and pa < 0.05)
    return rows


def h3(runs):
    rr = [m for m in runs if m.get("exp") == "E2"]
    out = {"mae": [], "nu": []}
    scen = sorted(set(m["scenario"] for m in rr))
    for metric, fn in (("mae", lambda m: m.get("mae_kappa")), ("nu", lambda m: m.get("nu"))):
        comps = []
        for s in scen:
            for p in ALT_PRED:
                xa, xb = paired(rr, lambda m: m["scenario"] == s and m["predictor"] == "eq",
                                lambda m: m["scenario"] == s and m["predictor"] == p, lambda m: m["seed"], fn)
                comps.append(("%s eq-%s" % (s, p), xa, xb))
        out[metric] = family(comps, "smaller" if metric == "mae" else "larger")
    out["holds"] = bool(rr) and all(r["supports"] for k in ("mae", "nu") for r in out[k])
    out["runs"] = len(rr)
    return out


def h4(runs):
    rr = [m for m in runs if m.get("exp") == "E2" or (m.get("exp") in ("E3", "pilot") and
                                                       m.get("method") in ("fleetdt", "slotted", "b5"))]
    bad_thm = [m["name"] for m in rr if g(m, "theorem1", "holds_all_prefixes") is False]
    bad_f = []
    for m in rr:
        fb, beta = g(m, "f_bar", "value"), m.get("beta", 0.05)
        if g(m, "theorem1", "F_R") == 0 and fb is not None and fb > beta + 0.02:
            bad_f.append({"run": m["name"], "f_bar": fb})
    return {"runs": len(rr), "theorem_violations": bad_thm, "fbar_violations": bad_f,
            "holds": bool(rr) and not bad_thm and not bad_f,
            "runs_with_FR0": sum(1 for m in rr if g(m, "theorem1", "F_R") == 0)}


def h5(runs):
    rr = [m for m in runs if m.get("exp") == "E3" and (m.get("rho") or 0) >= 1.5]
    out = {}
    for metric, fn, better in (("D_N", lambda m: m.get("D_N"), "larger"),
                               ("state_miss", lambda m: m.get("state_miss"), "smaller")):
        comps = []
        for s in ("S1", "S2", "S4", "S5"):
            for rho in (1.5, 2.0):
                for b in BASELINES_H5:
                    xa, xb = paired(rr, lambda m: m["scenario"] == s and m["rho"] == rho and m["method"] == "fleetdt",
                                    lambda m: m["scenario"] == s and m["rho"] == rho and m["method"] == b,
                                    lambda m: m["seed"], fn)
                    comps.append(("%s rho=%g fleetdt-%s" % (s, rho, b), xa, xb))
        out[metric] = family(comps, better)
    out["holds"] = bool(rr) and all(r["supports"] for k in ("D_N", "state_miss") for r in out[k])
    out["runs"] = len(rr)
    return out


def h6(runs, whatif_dir=None):
    from fleetdt.as2.analysis import load as load_dta
    from fleetdt.as2.dtp import DTP
    from fleetdt.as2.whatif import configure, replay, series
    base = {m["seed"]: m for m in runs if m.get("exp") == "E1" and m.get("scenario") == "S2"}
    rows = []
    for m in runs:
        if m.get("exp") != "E4" or m.get("method") != "fleetdt" or m["seed"] not in base:
            continue
        b = base[m["seed"]]
        try:
            with open(os.path.join(b["_dir"], "dtp.json")) as f:
                dtp = DTP.from_dict(json.load(f))
            with open(os.path.join(m["_dir"], "dtp.json")) as f:
                tgt = json.load(f)
            seq = series(load_dta(os.path.join(b["_dir"], "vms", "gs", "dta.jsonl")))
            cfg = configure(dtp, M=m["M"], Ts=m["Ts"])
            d = cfg.raw
            d["topics"] = tgt["topics"]          # the topics of the target run (state rate 1 / Ts)
            cfg = DTP.from_dict(d)
            w = replay(cfg, seq[240:])
        except (OSError, ValueError, KeyError) as e:
            rows.append({"run": m["name"], "error": str(e)})
            continue
        fm, nm = g(m, "f_bar", "value"), m.get("nu")
        rows.append({"run": m["name"], "M": m["M"], "Ts": m["Ts"], "f_bar": fm, "f_bar_whatif": w["f_bar"],
                     "nu": nm, "nu_whatif": w["nu"],
                     "df": None if fm is None else abs(fm - w["f_bar"]),
                     "dnu": None if nm is None or w["nu"] is None else abs(nm - w["nu"])})
    ok = [r for r in rows if "error" not in r]
    holds = bool(ok) and all(r["df"] is not None and r["df"] <= 0.02 and r["dnu"] is not None and r["dnu"] <= 0.05
                             for r in ok)
    # configurations without a measured counterpart (M = 8): the prediction alone, per E1 seed and pooled
    pred = []
    for p in sorted(glob.glob(os.path.join(whatif_dir, "*.json"))) if whatif_dir else []:
        with open(p) as f:
            wc = json.load(f)
        per = []
        for seed, b in sorted(base.items()):
            try:
                seq = series(load_dta(os.path.join(b["_dir"], "vms", "gs", "dta.jsonl")))
                w = replay(DTP.from_dict(wc["dtp"]), seq[240:])
                per.append({"seed": seed, "f_bar_whatif": w["f_bar"], "nu_whatif": w["nu"]})
            except (OSError, ValueError, KeyError) as e:
                per.append({"seed": seed, "error": str(e)})
        good = [x for x in per if "error" not in x]
        pred.append({"config": os.path.basename(p), "M": wc["M"], "Ts": wc["Ts"], "per_seed": per,
                     "f_bar_whatif_median": float(np.median([x["f_bar_whatif"] for x in good])) if good else None,
                     "nu_whatif_median": float(np.median([x["nu_whatif"] for x in good
                                                          if x["nu_whatif"] is not None])) if good else None})
    return {"rows": rows, "holds": holds, "predicted_only": pred}


def h7(runs):
    rr = [m for m in runs if m.get("exp") in ("E4", "pilot") and m.get("method") == "fleetdt"]
    bad = [{"run": m["name"], "delay_violations": g(m, "h7", "delay_violations"),
            "age_violations": g(m, "h7", "age_violations")} for m in rr
           if (g(m, "h7", "delay_violations") or 0) + (g(m, "h7", "age_violations") or 0) > 0]
    return {"runs": len(rr), "violations": bad, "holds": bool(rr) and not bad,
            "pairs": sum(g(m, "h7", "pairs") or 0 for m in rr)}


# ---------------------------------------------------------------------------------------- tables
def tex_table(path, caption, label, cols, rows, align=None):
    align = align or ("l" * 1 + "r" * (len(cols) - 1))
    with open(path, "w") as f:
        f.write("\\begin{table}[t]\n\\caption{%s}\\label{%s}\n\\centering\\footnotesize\n" % (caption, label))
        f.write("\\begin{tabular}{@{}%s@{}}\n\\toprule\n" % align)
        f.write(" & ".join(cols) + " \\\\\n\\midrule\n")
        for r in rows:
            f.write(" & ".join(str(x) for x in r) + " \\\\\n")
        f.write("\\bottomrule\n\\end{tabular}\n\\end{table}\n")


def med(vals):
    v = [x for x in vals if x is not None]
    return float(np.median(v)) if v else None


def run_table(runs, out, name, caption, label):
    """one row per run (pilot) or per cell (median over seeds)"""
    cells = defaultdict(list)
    for m in runs:
        cells[(m.get("exp"), m.get("method"), m.get("predictor"), m.get("scenario"), m.get("M"), m.get("Ts"),
               m.get("rho"))].append(m)
    cols = ["Exp.", "Method", "Sc.", "$M$", "$T_s$", "$\\rho$", "$n$", "$\\bar{f}$", "Cov.", "MAE", "$\\nu$",
            "$D_{\\mathcal{N}}$", "Miss", "$\\mu$", "AoI (ms)", "Lag"]
    rows = []
    for k in sorted(cells, key=lambda k: tuple(str(x) for x in k)):
        ms = cells[k]
        exp, meth, pred, sc, M, Ts, rho = k
        lab = NAMES.get(meth, meth) + ("" if pred in (None, "eq") else " / " + NAMES.get(pred, pred))
        rows.append([exp, lab, sc, M, fmt(Ts * 1e3, 1), fmt(rho, 1), len(ms), fmt(med([g(m, "f_bar", "value") for m in ms])),
                     fmt(med([m.get("coverage") for m in ms])), fmt(med([m.get("mae_kappa") for m in ms]), 4),
                     fmt(med([m.get("nu") for m in ms])), fmt(med([m.get("D_N") for m in ms])),
                     fmt(med([m.get("state_miss") for m in ms]), 4), fmt(med([m.get("mu") for m in ms]), 4),
                     fmt(None if med([aoi_mean(m) for m in ms]) is None else med([aoi_mean(m) for m in ms]) * 1e3, 1),
                     sum(m.get("lag_discarded") or 0 for m in ms)])
    tex_table(os.path.join(out, name), caption, label, cols, rows, "ll" + "r" * (len(cols) - 2))
    return cols, rows


def family_table(path, caption, label, rows):
    cols = ["Comparison", "$n$", "Median A", "Median B", "HL", "95\\% CI", "$p$", "$p_{\\mathrm{Holm}}$"]
    out = []
    for r in rows:
        ci = r["ci95"]
        out.append([r["comparison"].replace("_", "\\_"), r["n"], fmt(r["median_a"], 4), fmt(r["median_b"], 4),
                    fmt(r["hl"], 4), "[%s, %s]" % (fmt(ci[0], 4), fmt(ci[1], 4)), fmt_p(r["p"]), fmt_p(r["p_holm"])])
    tex_table(path, caption, label, cols, out, "lrrrrrrr")


# ---------------------------------------------------------------------------------------- figures
def figures(runs, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3, "figure.figsize": (3.4, 2.4),
                         "savefig.bbox": "tight"})
    e3 = [m for m in runs if m.get("exp") == "E3"]
    for metric, ylab in (("D_N", "$D_{\\mathcal{N}}$"), ("state_miss", "State miss ratio")):
        for sc in ("S1", "S2", "S4", "S5"):
            rr = [m for m in e3 if m["scenario"] == sc]
            if not rr:
                continue
            fig, ax = plt.subplots()
            for meth in ("fleetdt", "slotted", "b1", "b2", "b3", "b4", "b5", "tdma"):
                xs, ys, lo, hi = [], [], [], []
                for rho in (1.0, 1.5, 2.0):
                    v = [m.get(metric) for m in rr if m["method"] == meth and m["rho"] == rho and m.get(metric) is not None]
                    if v:
                        xs.append(rho)
                        ys.append(np.median(v))
                        lo.append(np.quantile(v, 0.25))
                        hi.append(np.quantile(v, 0.75))
                if xs:
                    ax.errorbar(xs, ys, yerr=[np.array(ys) - lo, np.array(hi) - ys], marker="o", ms=3, capsize=2,
                                label=NAMES[meth])
            ax.set_xlabel("Offered-load ratio $\\rho$")
            ax.set_ylabel(ylab)
            ax.set_title(sc)
            ax.legend(fontsize=6, ncol=2)
            fig.savefig(os.path.join(out, "fig_e3_%s_%s.pdf" % (metric.lower(), sc)))
            plt.close(fig)
    e2 = [m for m in runs if m.get("exp") == "E2"]
    if e2:
        fig, ax = plt.subplots()
        scen = sorted(set(m["scenario"] for m in e2))
        wd = 0.2
        for k, p in enumerate(("eq",) + ALT_PRED):
            ys = [med([m.get("mae_kappa") for m in e2 if m["scenario"] == s and m["predictor"] == p]) or 0 for s in scen]
            ax.bar(np.arange(len(scen)) + (k - 1.5) * wd, ys, wd, label=NAMES[p])
        ax.set_xticks(range(len(scen)))
        ax.set_xticklabels(scen)
        ax.set_ylabel("MAE of $\\hat{\\kappa}$")
        ax.legend(fontsize=6)
        fig.savefig(os.path.join(out, "fig_e2_mae.pdf"))
        plt.close(fig)
    fb = [m for m in runs if m.get("method") == "fleetdt" and m.get("exp") in ("E1", "pilot")]
    if fb:
        fig, ax = plt.subplots()
        for k, m in enumerate(sorted(fb, key=lambda m: (m["scenario"], m["seed"]))):
            v = g(m, "f_bar", "value")
            ci = g(m, "f_bar", "batch95") or [None, None]
            if v is None:
                continue
            yerr = None if ci[0] is None else [[v - ci[0]], [ci[1] - v]]
            ax.errorbar([k], [v], yerr=yerr, marker="o", ms=3, capsize=2, color="C0")
        ax.axhline(fb[0].get("beta", 0.05), color="k", lw=0.8, ls="--")
        ax.set_xlabel("Run (sorted by scenario and seed)")
        ax.set_ylabel("$\\bar{f}$ with batch-means 95% CI")
        fig.savefig(os.path.join(out, "fig_e1_fbar.pdf"))
        plt.close(fig)
    # kappa, Z kappa_hat and f of one run
    ex = sorted(fb, key=lambda m: m["name"])[:1]
    for m in ex:
        try:
            rows = [json.loads(l) for l in open(os.path.join(m["_dir"], "vms", "gs", "dta.jsonl")) if '"type":"as"' in l]
            G = {json.loads(l)["t"]: json.loads(l) for l in open(os.path.join(m["_dir"], "gt_steps.jsonl"))}
        except OSError:
            continue
        A = {r["t"]: r for r in rows}
        ts = sorted(t for t in A if t + 1 in A and t in G and G[t]["kappa"] is not None)
        if not ts:
            continue
        t0 = ts[0]
        fig, ax = plt.subplots(figsize=(6.8, 2.4))
        ax.plot([(t - t0) * m["Ts"] for t in ts], [G[t]["kappa"] for t in ts], lw=0.6, label="$\\kappa(t)$")
        ax.plot([(t - t0) * m["Ts"] for t in ts], [A[t]["Z"] * A[t]["kappa_hat"] for t in ts], lw=0.8,
                label="$Z_t(\\alpha_t)\\hat{\\kappa}(t)$")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Fraction of the window")
        ax.legend(fontsize=7)
        fig.savefig(os.path.join(out, "fig_kappa_%s.pdf" % m["name"]))
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("results", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--whatif", default=None, help="directory of the what-if configurations without runs "
                                                     "(the whatif/ directory of the eval manifest)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    runs = load(a.results)
    hyp = {"H1": h1(runs), "H2": h2(runs), "H3": h3(runs), "H4": h4(runs), "H5": h5(runs), "H6": h6(runs, a.whatif),
           "H7": h7(runs)}
    with open(os.path.join(a.out, "hypotheses.json"), "w") as f:
        json.dump(hyp, f, indent=1, default=float)
    keys = ["name", "exp", "method", "predictor", "scenario", "M", "Ts", "rho", "seed", "n_steps", "lag_discarded"]
    with open(os.path.join(a.out, "runs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(keys + ["f_bar", "coverage", "mae_kappa", "nu", "D_N", "state_miss", "mu", "aoi_mean",
                           "F_R", "H_R", "thm_ok", "kappa_p99", "b_p99", "lemma_cex"])
        for m in runs:
            w.writerow([m.get(k) for k in keys] + [g(m, "f_bar", "value"), m.get("coverage"), m.get("mae_kappa"),
                                                   m.get("nu"), m.get("D_N"), m.get("state_miss"), m.get("mu"),
                                                   aoi_mean(m), g(m, "theorem1", "F_R"), g(m, "theorem1", "H_R"),
                                                   g(m, "theorem1", "holds_all_prefixes"),
                                                   g(m, "h2", "kappa_absdiff", "p99"), g(m, "h2", "b_absdiff", "p99"),
                                                   g(m, "h2", "lemma_counterexamples")])
    run_table(runs, a.out, "tab_runs.tex", "Metrics per cell (median over the seeds of the cell).", "tab:runs")
    if hyp["H3"]["mae"]:
        family_table(os.path.join(a.out, "tab_h3_mae.tex"), "H3: MAE of $\\hat{\\kappa}$, eq.~(pred) against the other predictors.", "tab:h3mae", hyp["H3"]["mae"])
        family_table(os.path.join(a.out, "tab_h3_nu.tex"), "H3: utilization $\\nu$, eq.~(pred) against the other predictors.", "tab:h3nu", hyp["H3"]["nu"])
    if hyp["H5"].get("D_N"):
        family_table(os.path.join(a.out, "tab_h5_dn.tex"), "H5: $D_{\\mathcal{N}}$, Fleet-DT against the baselines.", "tab:h5dn", hyp["H5"]["D_N"])
        family_table(os.path.join(a.out, "tab_h5_miss.tex"), "H5: state miss ratio, Fleet-DT against the baselines.", "tab:h5miss", hyp["H5"]["state_miss"])
    figures(runs, a.out)
    print(json.dumps({k: v.get("holds") for k, v in hyp.items()}))


if __name__ == "__main__":
    main()
