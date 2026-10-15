#!/usr/bin/env python3
"""Calibration of the experiments (paper Section VI-C and Table exp, E0) from finished runs.

  python3 emu/experiments/calibrate.py sat RESULTS --calib emu/experiments/calibration.json
  python3 emu/experiments/calibrate.py e0  RESULTS --calib emu/experiments/calibration.json

sat: for each scenario and M, kappa_sat = the mean of kappa(t) (ground truth) over the steps of the
saturation runs (B1 with video far above the capacity) after a warm-up of 80 steps, and the mean PHY
rates of the data-bridge frames (uplink R_up, downlink R_down). emu/experiments/matrix.py derives the
video rate of each rho from these values.

e0: from the E0 runs (Fleet-DT, S1-S5, rho 1, 1.5, 2, M = 4, 4 calibration seeds), per step t >= S0 + 24
with the AS records (b(t) and eta(M + 1) of the record, kappa(t) and e(t) - w(t) of the closing record):
* lambda: the gain of eq. (pred) with the smallest mean |kappa_hat(t) - kappa(t)| over the E0 steps,
  on the grid LAMBDAS (replay of b_hat(t) = (1 - lambda) b_hat(t - 1) + lambda b(t));
* pred_gain: the same for the exponential smoothing of kappa (predictor "es" of E2);
* cal_kappa[S]: the mean kappa(t) of the E0 runs of scenario S (predictor "calmean" of E2);
* init_scores[S][p]: the scores z(t) = kappa(t) / kappa_hat_p(t) of the E0 steps of scenario S with
  e(t) - w(t) >= T_min, for each predictor p (eq with lambda, persist, es with pred_gain, calmean);
* c[S]: the ratio of the airtime that the admitted units need to their cost in scenario S (eq. cost bounds
  it from above): a step fails when kappa(t) T_W < (work), and with the budget filled the work is about
  c Z_t kappa_hat T_W, so f(t) = 1[z(t) < c Z_t]; c[S] is fitted on the E0 steps of S so that the rate of
  1[z(t) < c Z_t] (Z_t of the record) equals the observed failure fraction of these steps (the pooled c
  of all scenarios is reported too);
* gamma: on the grid GAMMAS, a replay of eq. (aci) from alpha_1 on the scores of each E0 run, with the
  scores of the other E0 runs of its scenario as initial scores and the failure event f(t) = 1[z(t) <
  c[S] Z_t(alpha_t)], f_bar over the steps after the warm-up of 240; gamma is the smallest value of the
  grid for which every replay has f_bar <= beta + 0.02 (the side that Theorem 1 bounds) and every replay
  in which beta is attainable has |f_bar - beta| <= 0.02 (the threshold of H4). beta is attainable in a
  replay when the level held at 1 (Z = the largest score) gives f_bar >= beta; otherwise no gamma brings
  f_bar to beta (the cost bound leaves a margin that the channel never uses up, S1). If no value meets
  the rule, the value with the smallest largest excess over the thresholds;
* z_b4[S]: the constant factor of B4, q_beta(z) / c[S] with q_beta the beta-quantile of the E0 scores (eq)
  of scenario S, so that the fixed bound fails in a fraction beta of the calibration steps under the same
  model;
* b_hat0[S]: the mean b(t) of the E0 runs of S (initial b_hat of the evaluation runs).
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "emu", "analysis"))
import gtstep  # noqa: E402

LAMBDAS = (0.02, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.75, 1.0)
GAMMAS = (0.001, 0.005, 0.01, 0.02, 0.05)     # the grid of Section VI-E


def runs_of(results, exp):
    out = []
    for d in sorted(glob.glob(os.path.join(results, "*"))):
        try:
            with open(os.path.join(d, "experiment.json")) as f:
                sp = json.load(f)
            with open(os.path.join(d, "result.json")) as f:
                rs = json.load(f)
        except (OSError, ValueError):
            continue
        if sp.get("exp") == exp and rs.get("status") == "ok":
            out.append((d, sp, rs))
    return out


def load_cal(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_cal(p, c):
    with open(p + ".tmp", "w") as f:
        json.dump(c, f, indent=1)
    os.replace(p + ".tmp", p)


def gt_rows(d):
    p = os.path.join(d, "gt_steps.jsonl")
    return gtstep.load_jsonl(p) if os.path.exists(p) else gtstep.analyze_run(d)


def cmd_sat(a):
    cal = load_cal(a.calib)
    acc = {}
    for d, sp, rs in runs_of(a.results, "SAT"):
        k = "%s-m%d" % (sp["scenario_id"], sp["M"])
        rows = [x for x in gt_rows(d) if x["t"] >= rs["S0"] + 80]
        ks = [x["kappa"] for x in rows if x["kappa"] is not None]
        ru = [x["R_up_kbps"] for x in rows if x.get("R_up_kbps")]
        rd = [x["R_down_kbps"] for x in rows if x.get("R_down_kbps")]
        e = acc.setdefault(k, {"kappa": [], "R_up": [], "R_down": [], "runs": [], "held": []})
        e["kappa"] += ks
        e["R_up"] += ru
        e["R_down"] += rd
        e["held"].append(float(np.mean([x["hold"] for x in rows])) if rows else None)
        e["runs"].append(os.path.basename(d))
    sat = {}
    for k, e in sorted(acc.items()):
        sat[k] = {"kappa": float(np.mean(e["kappa"])), "kappa_sd": float(np.std(e["kappa"])), "n_steps": len(e["kappa"]),
                  "R_up": float(np.mean(e["R_up"])) * 1e3, "R_down": float(np.mean(e["R_down"])) * 1e3,
                  "held_fraction": e["held"], "runs": e["runs"]}
        print("%s: kappa_sat %.4f (sd %.4f, %d steps), R_up %.2f Mb/s, R_down %.2f Mb/s, held %s" % (
            k, sat[k]["kappa"], sat[k]["kappa_sd"], sat[k]["n_steps"], sat[k]["R_up"] / 1e6, sat[k]["R_down"] / 1e6,
            e["held"]))
    cal["sat"] = sat
    save_cal(a.calib, cal)


def series(d, rs, dtp):
    """per step t >= S0 + 24: b(t), eta, kappa(t) (AS), score validity, kappa_gt(t)"""
    dta = [x for x in gtstep.load_jsonl(os.path.join(d, "vms", "gs", "dta.jsonl")) if x.get("type") == "as"]
    A = {int(x["t"]): x for x in dta}
    G = {x["t"]: x for x in gt_rows(d)}
    out = []
    for t in sorted(A):
        if t < rs["S0"] or (t + 1) not in A:
            continue
        nx = A[t + 1]
        k = nx.get("kappa_prev")
        ok = k is not None and nx.get("e_prev") is not None and (nx["e_prev"] - nx["w_prev"]) * 1e-6 >= dtp["T_min"] - 1e-9
        out.append({"t": t, "b": A[t].get("b"), "eta": A[t]["eta"], "kappa": k if ok else None,
                    "kappa_gt": G.get(t, {}).get("kappa"), "Z": A[t].get("Z"), "z": nx.get("z_prev"),
                    "f": nx.get("f_prev")})
    return out


def replay_eq(ser, lam, b0=0.0):
    bh, out = b0, []
    for x in ser:
        if x["b"] is not None:
            bh = (1 - lam) * bh + lam * min(1.0, max(0.0, x["b"]))
        out.append((1 - bh) * x["eta"])
    return out


def replay_es(ser, g):
    kh, out, prev = None, [], None
    for x in ser:
        if kh is None:
            kh = (1 - (x["b"] or 0.0)) * x["eta"]     # first step: eq. (pred) without history
        if prev is not None:
            kh = (1 - g) * kh + g * prev if g < 1 else prev
        out.append(kh)
        prev = x["kappa"] if x["kappa"] is not None else prev
    return out


def replay_persist(ser):
    return replay_es(ser, 1.0)


def mae(ser, kh, start):
    e = [abs(k - x["kappa"]) for x, k in zip(ser, kh) if x["kappa"] is not None and x["t"] >= start]
    return float(np.mean(e)) if e else float("nan"), len(e)


def aci_replay(z, init, beta, gamma, alpha1, skip, c=1.0, hold=False):
    """f(t) = 1[z(t) < c Z_t(alpha_t)] with Z_t from the sorted initial and past scores; f_bar after skip
    (hold: the level stays at alpha1)"""
    import bisect
    srt = sorted(init)
    alpha, fs = alpha1, []
    for k, zt in enumerate(z):
        if alpha <= 0:
            Z = 0.0
        elif not srt:
            Z = 1.0
        else:
            n = len(srt)
            Z = srt[min(n, max(1, int(math.ceil(min(alpha, 1.0) * n - 1e-12)))) - 1]
        f = 1 if (zt is not None and zt < c * Z) else 0
        fs.append(f)
        if not hold:
            alpha = min(1.0, max(-gamma * (1 - beta), alpha + gamma * (beta - f)))
        if zt is not None:
            bisect.insort(srt, zt)
    w = fs[skip:]
    return sum(w) / len(w) if w else float("nan")


def cmd_e0(a):
    cal = load_cal(a.calib)
    runs = runs_of(a.results, "E0")
    if not runs:
        raise SystemExit("no E0 runs in %s" % a.results)
    data = []
    for d, sp, rs in runs:
        dtp = sp["dtp"]
        data.append((sp, rs, dtp, series(d, rs, dtp)))
    start = lambda rs: rs["S0"] + 24    # noqa: E731
    # lambda of eq. (pred)
    res = {}
    for lam in LAMBDAS:
        tot, n = 0.0, 0
        for sp, rs, dtp, ser in data:
            m, k = mae(ser, replay_eq(ser, lam), start(rs))
            if k:
                tot += m * k
                n += k
        res[lam] = tot / n
    lam = min(res, key=res.get)
    print("lambda: MAE %s -> %g" % ({k: round(v, 5) for k, v in res.items()}, lam))
    res_g = {}
    for g in LAMBDAS:
        tot, n = 0.0, 0
        for sp, rs, dtp, ser in data:
            m, k = mae(ser, replay_es(ser, g), start(rs))
            if k:
                tot += m * k
                n += k
        res_g[g] = tot / n
    gain = min(res_g, key=res_g.get)
    print("ES gain: MAE %s -> %g" % ({k: round(v, 5) for k, v in res_g.items()}, gain))
    scen = sorted(set(sp["scenario_id"] for sp, _, _, _ in data))
    cal_kappa, b_hat0 = {}, {}
    for s in scen:
        ks = [x["kappa"] for sp, rs, dtp, ser in data if sp["scenario_id"] == s for x in ser
              if x["kappa"] is not None and x["t"] >= start(rs)]
        bs = [x["b"] for sp, rs, dtp, ser in data if sp["scenario_id"] == s for x in ser
              if x["b"] is not None and x["t"] >= start(rs)]
        cal_kappa[s] = float(np.mean(ks))
        b_hat0[s] = float(np.mean(bs))
    # scores per scenario, predictor and run
    per_run = {}
    for idx, (sp, rs, dtp, ser) in enumerate(data):
        s = sp["scenario_id"]
        preds = {"eq": replay_eq(ser, lam, b_hat0[s]), "persist": replay_persist(ser), "es": replay_es(ser, gain),
                 "calmean": [cal_kappa[s]] * len(ser)}
        for p, kh in preds.items():
            z = [(x["kappa"] / k if (x["kappa"] is not None and k > 0) else None) for x, k in zip(ser, kh)]
            per_run[(idx, p)] = (s, z, [x["t"] >= start(rs) for x in ser])
    init = {s: {} for s in scen}
    for (idx, p), (s, z, ok) in per_run.items():
        init[s].setdefault(p, [])
        init[s][p] += [round(v, 5) for v, o in zip(z, ok) if v is not None and o]
    beta = data[0][2]["beta"]
    alpha1 = data[0][2]["alpha1"]
    # c of the failure model, by matching the failure rate of the E0 steps (per scenario, and pooled)
    def fit_c(zz):
        fo = float(np.mean([f for _, _, f in zz])) if zz else 0.0
        ratio = np.array(sorted(z / Z for z, Z, _ in zz if Z > 0))
        return (float(np.quantile(ratio, fo)) if len(ratio) and fo > 0 else 1.0), fo, len(zz)
    zz_s = {s: [(x["z"], x["Z"], x["f"]) for sp, rs, dtp, ser in data if sp["scenario_id"] == s for x in ser
                if x["z"] is not None and x["Z"] is not None and x["f"] is not None and x["t"] >= start(rs)]
            for s in scen}
    c_pool, fobs, nz = fit_c([v for s in scen for v in zz_s[s]])
    print("failure model: observed f_bar %.4f over %d E0 steps -> pooled c = %.4f" % (fobs, nz, c_pool))
    c_s, f_s = {}, {}
    for s in scen:
        c_s[s], f_s[s], n_s = fit_c(zz_s[s])
        agree = float(np.mean([f == int(z < c_s[s] * Z) for z, Z, f in zz_s[s]])) if zz_s[s] else float("nan")
        print("  %s: observed f_bar %.4f over %d steps -> c = %.4f (per-step agreement of the model %.4f)" % (
            s, f_s[s], n_s, c_s[s], agree))
    z_b4 = {}
    for s in scen:
        v = sorted(init[s]["eq"])
        z_b4[s] = float(v[max(0, int(math.ceil(beta * len(v))) - 1)]) / c_s[s]
    # gamma
    gres, attain = {}, {}
    for idx, (sp, rs, dtp, ser) in enumerate(data):
        s = sp["scenario_id"]
        z = per_run[(idx, "eq")][1]
        others = [v for (j, p), (s2, zz, ok) in per_run.items() if p == "eq" and s2 == s and j != idx
                  for v, o in zip(zz, ok) if v is not None and o]
        attain[idx] = (z, others, aci_replay(z, others, beta, 0.0, 1.0, 240, c_s[s], hold=True) >= beta)
    for g in GAMMAS:
        fb_all, excess, dev_att = {}, [], []
        for idx, (sp, rs, dtp, ser) in enumerate(data):
            s = sp["scenario_id"]
            z, others, att = attain[idx]
            fb = aci_replay(z, others, beta, g, alpha1, 240, c_s[s])
            fb_all.setdefault(s, []).append(round(fb, 4))
            excess.append(max(fb - beta - 0.02, (abs(fb - beta) - 0.02) if att else -1.0))
            if att:
                dev_att.append(abs(fb - beta))
        gres[g] = {"max_excess": float(np.max(excess)), "f_bar": fb_all,
                   "max_f_bar": float(max(v for vs in fb_all.values() for v in vs)),
                   "max_dev_attainable": float(max(dev_att)) if dev_att else None,
                   "mean_dev_attainable": float(np.mean(dev_att)) if dev_att else None}
    meet = [g for g in GAMMAS if gres[g]["max_excess"] <= 0]
    gamma = min(meet) if meet else min(GAMMAS, key=lambda g: (gres[g]["max_excess"], g))
    n_att = {s: sum(1 for idx, d in enumerate(data) if d[0]["scenario_id"] == s and attain[idx][2]) for s in scen}
    n_run = {s: sum(1 for d in data if d[0]["scenario_id"] == s) for s in scen}
    print("beta attainable (runs): %s of %s" % (n_att, n_run))
    for g in GAMMAS:
        r = gres[g]
        print("gamma %-6g max f_bar %.4f, max |f_bar - beta| (attainable) %s, mean %s, max excess %.4f%s" % (
            g, r["max_f_bar"], None if r["max_dev_attainable"] is None else round(r["max_dev_attainable"], 4),
            None if r["mean_dev_attainable"] is None else round(r["mean_dev_attainable"], 4), r["max_excess"],
            "  <- meets the rule" if r["max_excess"] <= 0 else ""))
    print("gamma -> %g" % gamma)
    print("cal_kappa %s\nz_b4 %s\nb_hat0 %s" % (cal_kappa, z_b4, b_hat0))
    cal["e0"] = {"lambda": lam, "lambda_mae": res, "pred_gain": gain, "pred_gain_mae": res_g, "gamma": gamma,
                 "c_failure_model": c_s, "c_failure_model_pooled": c_pool, "f_bar_e0": fobs,
                 "f_bar_e0_scenario": f_s, "gamma_replay": gres, "gamma_attainable_runs": n_att, "cal_kappa": cal_kappa, "b_hat0": b_hat0, "z_b4": z_b4,
                 "init_scores": init, "n_scores": {s: {p: len(v) for p, v in init[s].items()} for s in scen},
                 "runs": [os.path.basename(r[0]) for r in runs]}
    save_cal(a.calib, cal)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", choices=("sat", "e0"))
    ap.add_argument("results")
    ap.add_argument("--calib", required=True)
    a = ap.parse_args()
    {"sat": cmd_sat, "e0": cmd_e0}[a.cmd](a)


if __name__ == "__main__":
    main()
