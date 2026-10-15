#!/usr/bin/env python3
"""Metrics of one Fleet-DT run (paper Section VI-D, Def. fidelity, Table hyp) from the DTA of the AS, the
ground truth (gtstep.py), the source logs, the vessel logs and the clock probe.

  python3 emu/analysis/metrics.py RUN [--skip 240] [--out RUN/metrics.json]

Steps: the run has the AS records of steps S0 .. S0 + N (the record of t + 1 closes step t); the
metrics use the steps S0 + skip .. S0 + N - 1 (skip = 240, Section VI-E) without the steps in which the
real-time lag of the emulator exceeds 1 ms ("lag_discarded"); Theorem 1 is checked at every prefix of the
whole run (from S0, with alpha_1). kappa(t), X(t) and the interference come from the ground truth.

| Key | Metric |
|---|---|
| f_bar | failure fraction (Def. fidelity (i)), with Wilson and batch-means 95 % intervals (40 batches) |
| coverage | fraction of the steps with kappa(t) >= Z_t(alpha_t) kappa_hat(t) (ii) |
| mae_kappa | mean |kappa_hat(t) - kappa(t)| (iii) |
| edge_ratio | accepted bytes d_il(t) over released bytes, per edge with p_il(t) >= 1, and pooled (iv) |
| nu | sum of the admitted cost over sum kappa(t) T_W (v) |
| aoi | age of information of the state per vessel (vi): time average, mean peak, max (s) |
| D_N | bytes of the units of N_i(t) that the GS accepts within their step over the bytes of these units that the sources release (outputs count only when complete) |
| state_miss | fraction of the pairs (t, i) in which the GS does not accept the state of vessel i within step t |
| mu | fraction of the released messages that the GS does not accept within their step |
| theorem1 | inequality (thm) at every prefix, F_R(T), H_R(T) |
| h1 | clock offsets of the vessels (probe) against eps; fraction of the pairs (t, i) without the demand in phase 1 in the steps without interference |
| h2 | p99 of |kappa_AS - kappa_gt| and |b_AS - b_gt|; steps with X(t) <= kappa(t) T_W and f(t) = 1 without a lost table or report |
| h7 | state delay and age against the bounds of Corollary 2 in the steps that meet the hypothesis of Lemma 2 |
"""
import argparse
import json
import math
import os
import struct
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))     # repository root (fleetdt)
sys.path.insert(0, HERE)
from fleetdt.as2.aci import theorem_check  # noqa: E402
import gtstep  # noqa: E402

DEFAULT_EPOCH = 1767225600.0


def wilson(k, n, z=1.96):
    if n == 0:
        return (None, None)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def batch_means(x, nb=40):
    """95 % CI of the mean of a correlated series by nb non-overlapping batch means (Student t)"""
    from scipy import stats
    x = np.asarray(x, dtype=float)
    b = len(x) // nb
    if b < 1:
        return (None, None)
    m = x[:nb * b].reshape(nb, b).mean(axis=1)
    s = m.std(ddof=1) / math.sqrt(nb)
    q = stats.t.ppf(0.975, nb - 1)
    return (max(0.0, float(m.mean() - q * s)), min(1.0, float(m.mean() + q * s)))   # f(t) in {0, 1}


def q(x, p):
    return float(np.quantile(x, p)) if len(x) else None


def analyze(rd, skip=240, gt_rows=None):
    load = gtstep.load_jsonl
    with open(os.path.join(rd, "result.json")) as f:
        res = json.load(f)
    with open(os.path.join(rd, "dtp.json")) as f:
        dtp = json.load(f)
    with open(os.path.join(rd, "experiment.json")) as f:
        spec = json.load(f)
    names = [tp["name"] for tp in dtp["topics"]]
    L = len(names)
    R = set(names.index(n) for n in dtp.get("required", []))
    l_state = names.index(dtp["required"][0]) if dtp.get("required") else 0
    outputs = set(k for k, tp in enumerate(dtp["topics"]) if tp["kind"] == "output")
    epoch = DEFAULT_EPOCH if dtp.get("epoch") is None else float(dtp["epoch"])
    Ts, T1, T2 = float(dtp["Ts"]), float(dtp["T1"]), float(dtp["T2"])
    eps, drx = float(dtp["eps"]), float(dtp["d_rx"])
    T3 = Ts - T1 - T2
    T_W = T3 - 2 * eps - drx
    beta, gamma, alpha1 = float(dtp["beta"]), float(dtp["gamma"]), float(dtp["alpha1"])
    M = int(res["M"])
    vessels = list(range(1, M + 1))
    S0, N = int(res["S0"]), int(res["steps"])

    def start(t):
        return epoch + (t - 1) * Ts

    def end(t):
        return epoch + t * Ts

    dta = load(os.path.join(rd, "vms", "gs", "dta.jsonl"))
    A = {int(x["t"]): x for x in dta if x.get("type") == "as"}
    acc = [x for x in dta if x.get("type") == "acc"]
    outs = [x for x in dta if x.get("type") == "out"]
    vlog = {(int(x["i"]), int(x["t"])): x for x in dta if x.get("type") == "vlog"}
    if gt_rows is None:
        p = os.path.join(rd, "gt_steps.jsonl")
        gt_rows = load(p) if os.path.exists(p) else gtstep.analyze_run(rd)
    G = {int(x["t"]): x for x in gt_rows}
    out = {"name": spec.get("name"), "exp": spec.get("exp"), "method": spec.get("method"),
           "predictor": spec.get("predictor", "eq"), "scenario": spec.get("scenario_id"), "M": M, "Ts": Ts,
           "rho": spec.get("rho"), "seed": spec.get("seed"), "S0": S0, "steps": N, "skip": skip,
           "status": res.get("status"), "revision": res.get("revision"), "beta": beta}

    # ---------------------------------------------------------------- per-step table
    steps_all = [t for t in range(S0, S0 + N) if t in A and (t + 1) in A]
    rows = {}
    for t in steps_all:
        r, nx = A[t], A[t + 1]
        g = G.get(t, {})
        rows[t] = {"f": nx.get("f_prev"), "reasons": nx.get("f_reasons") or [], "kappa_as": nx.get("kappa_prev"),
                   "b_as": r.get("b"), "kh": r.get("kappa_hat"), "Z": r.get("Z"), "alpha": r.get("alpha"),
                   "cost_A": r.get("cost_A") or 0.0, "cB": sum(r.get("cB") or []), "A": r.get("A") or [],
                   "Mt": r.get("M") or [], "p": r.get("p") or {}, "needed": r.get("needed") or {},
                   "acc": nx.get("acc_prev") or {}, "d": nx.get("d_prev") or {}, "rep": nx.get("rep_prev") or {},
                   "kappa": g.get("kappa"), "b_gt": g.get("b"), "X": g.get("X"), "empty": g.get("empty"),
                   "interf": g.get("interf"), "lag": g.get("lag_max_us")}
    win = [t for t in steps_all if t >= S0 + skip]
    lagbad = [t for t in win if rows[t]["lag"] is not None and rows[t]["lag"] > 1000.0]
    use = [t for t in win if t not in set(lagbad) and rows[t]["f"] is not None]
    out["n_steps"] = len(use)
    out["lag_discarded"] = len(lagbad)
    out["missing_steps"] = S0 + N - S0 - len(steps_all)
    try:
        lags = np.loadtxt(os.path.join(rd, "emu", "lag.txt"))
        lv = lags[:, 1] if lags.ndim == 2 else np.zeros(0)
        out["lag_us"] = {"p50": q(lv, .5), "p99": q(lv, .99), "max": float(lv.max()) if len(lv) else None,
                         "n": int(len(lv))}
    except (OSError, ValueError):
        out["lag_us"] = None
    if not use:
        return out
    f = np.array([rows[t]["f"] for t in use], dtype=float)
    k = int(f.sum())
    out["f_bar"] = {"value": k / len(f), "k": k, "n": len(f), "wilson95": wilson(k, len(f)),
                    "batch95": batch_means(f, 40), "abs_err": abs(k / len(f) - beta)}
    kr = [t for t in use if rows[t]["kappa"] is not None]
    if kr:
        kap = np.array([rows[t]["kappa"] for t in kr])
        kh = np.array([rows[t]["kh"] for t in kr])
        Zk = np.array([(rows[t]["Z"] or 0.0) * rows[t]["kh"] for t in kr])
        out["coverage"] = float(np.mean(kap >= Zk))
        out["mae_kappa"] = float(np.mean(np.abs(kh - kap)))
        out["mean_kappa"] = float(kap.mean())
        out["mean_kappa_hat"] = float(kh.mean())
        out["nu"] = float(sum(rows[t]["cost_A"] for t in kr) / (kap.sum() * T_W))
        out["n_kappa"] = len(kr)
    # ---------------------------------------------------------------- edges (iv)
    num, den = defaultdict(float), defaultdict(float)
    for t in use:
        for si, pl in rows[t]["p"].items():
            i = int(si)
            v = vlog.get((i, t))
            if v is None or "xrel" not in v:
                continue
            for l, p in enumerate(pl):
                if p >= 1:
                    num[(i, l)] += rows[t]["d"].get(si, [0] * L)[l]
                    den[(i, l)] += v["xrel"][l]
    out["edge_ratio"] = {"%d/%s" % (i, names[l]): (num[(i, l)] / den[(i, l)] if den[(i, l)] else None)
                         for (i, l) in sorted(den)}
    td = sum(den.values())
    out["edge_ratio_pooled"] = sum(num.values()) / td if td else None
    # ---------------------------------------------------------------- mu (released vs accepted in step)
    rel = accn = 0
    for t in use:
        for i in vessels:
            v = vlog.get((i, t))
            if v is None or "rel" not in v:
                continue
            rel += sum(v["rel"])
            accn += sum(rows[t]["acc"].get(str(i), [0] * L))
    out["mu"] = 1.0 - accn / rel if rel else None
    out["released_msgs"] = rel
    # ---------------------------------------------------------------- D_N from the source logs
    T12 = T1 + T2

    def coll_step(t_rel):
        return int(math.floor((t_rel - epoch - T12) / Ts)) + 2

    src_out = {}      # (i, l, j mod 2^32) -> (step, bytes)
    src_stream = defaultdict(float)      # (i, l, step) -> bytes
    for i in vessels:
        for x in load(os.path.join(rd, "vms", "v%d" % i, "src.jsonl")):
            if x.get("topic") not in names:
                continue
            l = names.index(x["topic"])
            s = coll_step(x["t_us"] * 1e-6)
            if l in outputs:
                src_out[(i, l, int(x.get("out", x.get("j", 0))) & 0xFFFFFFFF)] = (s, float(x["bytes"]))
            else:
                src_stream[(i, l, s)] += float(x["bytes"])
    out_ok = {}
    for x in outs:
        if x.get("complete"):
            out_ok[(int(x["i"]), int(x["l"]), int(x["o"]))] = float(x["at"])
    useset = set(use)
    dn_num = dn_den = 0.0
    dn_out_num = dn_out_den = 0.0
    for (i, l, j), (s, b) in src_out.items():
        if s not in useset:
            continue
        need = rows[s]["needed"].get(str(i))
        if need is None or l not in need:
            continue
        dn_den += b
        dn_out_den += b
        at = out_ok.get((i, l, j))
        if at is not None and at <= end(s):
            dn_num += b
            dn_out_num += b
    for (i, l, s), b in src_stream.items():
        if s not in useset:
            continue
        need = rows[s]["needed"].get(str(i))
        if need is None or l not in need:
            continue
        dn_den += b
        dn_num += min(b, float(rows[s]["d"].get(str(i), [0] * L)[l]))
    out["D_N"] = dn_num / dn_den if dn_den else None
    out["D_N_outputs"] = dn_out_num / dn_out_den if dn_out_den else None
    # ---------------------------------------------------------------- state: miss, delay, AoI
    st_acc = defaultdict(list)       # (i, t) -> [(at, gen)]
    for x in acc:
        if int(x["l"]) != l_state:
            continue
        b = bytes.fromhex(x.get("c", ""))
        gen = int.from_bytes(b[4:12], "big") * 1e-6 if len(b) >= 12 else None
        st_acc[(int(x["i"]), int(x["t"]))].append((float(x["at"]), gen))
    miss = 0
    for t in use:
        for i in vessels:
            ok = any(at <= end(t) for at, _ in st_acc.get((i, t), []))
            miss += 0 if ok else 1
    out["state_miss"] = miss / (len(use) * M)
    # AoI over the window (acceptance of the newest state)
    aoi = {}
    for i in vessels:
        ev = sorted((at, gen) for (ii, t), lst in st_acc.items() if ii == i and t in useset
                    for at, gen in lst if gen is not None)
        if len(ev) < 2:
            aoi[str(i)] = None
            continue
        area, peaks, g = 0.0, [], ev[0][1]
        tp = ev[0][0]
        for at, gen in ev[1:]:
            area += 0.5 * ((tp - g) + (at - g)) * (at - tp)
            if gen > g:
                peaks.append(at - g)
                g = gen
            tp = at
        span = ev[-1][0] - ev[0][0]
        aoi[str(i)] = {"mean": area / span if span > 0 else None, "peak_mean": float(np.mean(peaks)) if peaks else None,
                       "max": float(max(peaks)) if peaks else None}
    out["aoi"] = aoi
    # ---------------------------------------------------------------- H1
    h1 = {}
    try:
        # the probe reads the VM clock over vsock; a sample whose round trip exceeds 500 us (the VM answered
        # late) has an error of up to RTT/2 and is reported apart ("clock_all")
        off, offall = defaultdict(list), defaultdict(list)
        t0, t1 = res.get("t0", 0), res.get("t_end", 1e30)
        with open(os.path.join(rd, "clock.csv")) as fcl:
            fcl.readline()
            for line in fcl:
                a, host, o, rtt = line.strip().split(",")
                ts = int(a) * 1e-9
                if t0 <= ts <= t1:
                    offall[host].append(int(o) * 1e-9)
                    if int(rtt) <= 500000:
                        off[host].append(int(o) * 1e-9)

        def cs(v):
            return {"n": len(v), "max_abs": float(np.max(np.abs(v))), "p99_abs": q(np.abs(v), .99),
                    "over_eps": int(np.sum(np.abs(v) > eps))}
        h1["clock"] = {h: cs(v) for h, v in off.items() if v}
        h1["clock_all"] = {h: cs(v) for h, v in offall.items() if v}
        h1["clock_over_eps"] = sum(x["over_eps"] for h, x in h1["clock"].items() if h != "gs")
        h1["clock_max_abs_vessels"] = max([x["max_abs"] for h, x in h1["clock"].items() if h != "gs"] or [None])
    except OSError:
        pass
    quiet = [t for t in use if not rows[t]["interf"]]
    pairs = len(quiet) * M
    missing = sum(sum(1 for i in vessels if i not in rows[t]["Mt"]) for t in quiet)
    h1["demand_miss"] = missing / pairs if pairs else None
    h1["demand_missing"] = missing
    h1["demand_pairs"] = pairs
    h1["quiet_steps"] = len(quiet)
    allpairs = sum(sum(1 for i in vessels if i not in rows[t]["Mt"]) for t in use)
    h1["demand_miss_all"] = allpairs / (len(use) * M)
    out["h1"] = h1
    # ---------------------------------------------------------------- H2
    h2 = {}
    dk = [abs(rows[t]["kappa_as"] - rows[t]["kappa"]) for t in use
          if rows[t]["kappa_as"] is not None and rows[t]["kappa"] is not None]
    db = [abs(rows[t]["b_as"] - rows[t]["b_gt"]) for t in use if rows[t]["b_as"] is not None and rows[t]["b_gt"] is not None]
    h2["kappa_absdiff"] = {"n": len(dk), "p99": q(dk, .99), "max": max(dk) if dk else None}
    h2["b_absdiff"] = {"n": len(db), "p99": q(db, .99), "max": max(db) if db else None}
    h2["kappa_as_missing"] = sum(1 for t in use if rows[t]["kappa"] is not None and rows[t]["kappa_as"] is None)

    def lost(t):
        return any(len(x) > 1 and x[1] in ("noreport", "notab") for x in rows[t]["reasons"])
    # X(t) is the work of Lemma 2 only when the data bridges hold no byte of step t before v(t+1) - eps
    # ("empty"); otherwise the ground truth gives a lower bound and the step stays undetermined
    lemma = [t for t in use if rows[t]["kappa"] is not None and rows[t]["X"] is not None and rows[t]["empty"]
             and rows[t]["X"] <= rows[t]["kappa"] * T_W]
    undet = [t for t in use if rows[t]["kappa"] is not None and rows[t]["X"] is not None and not rows[t]["empty"]
             and rows[t]["X"] <= rows[t]["kappa"] * T_W]
    cex = [t for t in lemma if rows[t]["f"] == 1 and not lost(t)]
    h2["lemma_steps"] = len(lemma)
    h2["lemma_undetermined"] = len(undet)
    h2["lemma_undetermined_f1"] = sum(1 for t in undet if rows[t]["f"] == 1 and not lost(t))
    h2["lemma_counterexamples"] = len(cex)
    h2["lemma_cex_reasons"] = [rows[t]["reasons"][:4] for t in cex[:10]]
    out["h2"] = h2
    # ---------------------------------------------------------------- Theorem 1 and Corollary 1 (whole run)
    allf = [t for t in steps_all if rows[t]["f"] is not None]
    fs = [int(rows[t]["f"]) for t in allf]
    al = [rows[t]["alpha"] if rows[t]["alpha"] is not None else alpha1 for t in allf]
    chk = theorem_check(fs, al, beta, gamma, alpha1) if fs else []
    viol = [T for T, (lft, rgt, _) in enumerate(chk, start=1) if lft > rgt + 1e-12]
    HR = 0
    for t in allf:
        r = rows[t]
        R_alone = all(u[1] in R for u in r["A"])
        over = r["kappa"] is not None and r["X"] is not None and (r["X"] > r["kappa"] * T_W or not r["empty"])
        if (R_alone and over) or lost(t):
            HR += 1
    out["theorem1"] = {"T": len(fs), "holds_all_prefixes": not viol, "violations": viol[:10],
                       "left": chk[-1][0] if chk else None, "right": chk[-1][1] if chk else None,
                       "F_R": chk[-1][2] if chk else None, "H_R": HR,
                       "corollary1": (chk[-1][2] <= HR) if chk else None}
    # ---------------------------------------------------------------- H7
    bound_delay = Ts + T3 + eps
    bound_age = 2 * Ts + T3 + eps
    dl, ag, vd, va, nh = [], [], 0, 0, 0
    lemma_set = set(lemma)
    for t in lemma:
        for i in vessels:
            cur = st_acc.get((i, t), [])
            if not cur or cur[0][1] is None:
                continue
            at, gen = min(cur)
            d = at - gen
            dl.append(d)
            vd += d > bound_delay
            nxt = st_acc.get((i, t + 1), [])
            if nxt and (t + 1) in lemma_set:            # a sequence of steps that meet the hypothesis
                age = min(nxt)[0] - gen          # age just before the acceptance of the state of t + 1
                ag.append(age)
                va += age >= bound_age
            nh += 1
    out["h7"] = {"pairs": nh, "delay_p99": q(dl, .99), "delay_max": max(dl) if dl else None,
                 "delay_bound": bound_delay, "delay_violations": int(vd), "age_p99": q(ag, .99),
                 "age_max": max(ag) if ag else None, "age_bound": bound_age, "age_violations": int(va)}
    # failure reasons
    rc = defaultdict(int)
    for t in use:
        for x in rows[t]["reasons"]:
            if len(x) > 1:
                rc[x[1]] += 1
    out["reasons"] = dict(rc)
    out["admitted_outputs_per_step"] = float(np.mean([sum(1 for u in rows[t]["A"] if u[1] in outputs) for t in use]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run")
    ap.add_argument("--skip", type=int, default=240)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    m = analyze(a.run, a.skip)
    p = a.out or os.path.join(a.run, "metrics.json")
    with open(p, "w") as f:
        json.dump(m, f, indent=1, default=float)
    show = {k: m.get(k) for k in ("method", "scenario", "n_steps", "lag_discarded", "f_bar", "coverage", "mae_kappa",
                                  "nu", "D_N", "state_miss", "mu", "edge_ratio_pooled")}
    print(json.dumps(show, default=float))


if __name__ == "__main__":
    main()
