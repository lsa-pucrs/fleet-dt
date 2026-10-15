"""Analysis of a DTA of the AS v2: fidelity of the DT (Def. fidelity), Theorem 1 check, F_R and H_R,
delivery and deadline metrics, and the age of information of the state.

  python -m fleetdt.as2.analysis --dtp dtp_v2.json --dta dta.jsonl [--gt truth.jsonl] [--src src.jsonl]
         [--ts-offset N] [--skip 8]

Inputs: the DTA lines of service.py ("as", "acc", "vlog"); optional ground truth (JSON lines
{"t", "kappa", "X"}) from the emulator; optional source logs (JSON lines {"i", "topic" or "l", "rel_us" or
"rel_at", "c"}: the release time of each message of the topics of R with the hex prefix of its content).
f(t), kappa(t), e(t), z(t) of step t are the "_prev" fields of the record of step t+1.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .aci import theorem_check
from .dtp import DTP


def load(path: str) -> List[Dict]:
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    return out


def wilson(k: int, n: int, z: float = 1.96) -> Tuple[float, float]:
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def batch_means_ci(x: Sequence[float], batch: int = 50, z: float = 1.96) -> Tuple[float, float]:
    """CI of the mean of a correlated series by non-overlapping batch means."""
    x = np.asarray(x, dtype=float)
    nb = len(x) // batch
    if nb < 2:
        return (float("nan"), float("nan"))
    m = x[:nb * batch].reshape(nb, batch).mean(axis=1)
    s = m.std(ddof=1) / math.sqrt(nb)
    return (float(x.mean() - z * s), float(x.mean() + z * s))


def steps(recs: Iterable[Dict], skip: int = 0) -> Dict[int, Dict]:
    """{t: record of step t} of the AS records (type "as"), after `skip` warm-up steps."""
    a = {int(r["t"]): r for r in recs if r.get("type", "as") == "as" and "t" in r}
    if skip and a:
        t0 = min(a) + skip
        a = {t: r for t, r in a.items() if t >= t0}
    return a


def per_step(a: Dict[int, Dict], gt: Optional[Dict[int, Dict]] = None) -> List[Dict]:
    """One row per step t with the record of t+1: f, kappa, e, z of step t and the decision of step t."""
    rows = []
    for t in sorted(a):
        nx = a.get(t + 1)
        if nx is None or nx.get("f_prev") is None:
            continue
        r = a[t]
        kappa = nx.get("kappa_prev")
        X = None
        if gt and t in gt:
            kappa = gt[t].get("kappa", kappa)
            X = gt[t].get("X")
        rows.append({"t": t, "f": int(nx["f_prev"]), "reasons": nx.get("f_reasons") or [], "kappa": kappa,
                     "z": nx.get("z_prev"), "e": nx.get("e_prev"), "w": nx.get("w_prev"),
                     "kappa_hat": r["kappa_hat"], "Z": r.get("Z"), "alpha": r.get("alpha"), "K": r.get("K"),
                     "cost_A": r.get("cost_A", 0.0), "cost_R": r.get("cost_R"), "cB": sum(r.get("cB") or []),
                     "T_W": r.get("T_W"), "A": r.get("A") or [], "M": r.get("M") or [],
                     "p": r.get("p") or {}, "acc": nx.get("acc_prev") or {}, "d": nx.get("d_prev") or {},
                     "late": nx.get("late_prev") or {}, "rep": nx.get("rep_prev") or {},
                     "needed": r.get("needed") or {}, "X": X})
    return rows


def aoi(events: Sequence[Tuple[float, float]]) -> Dict:
    """Age of information from (acceptance time, generation time) pairs of one vessel: time average
    over [first, last acceptance], mean peak and maximum (seconds)."""
    ev = sorted(events)
    if len(ev) < 2:
        return {"n": len(ev)}
    area, peaks = 0.0, []
    g = ev[0][1]
    tprev = ev[0][0]
    for at, gen in ev[1:]:
        # age grows linearly from (tprev, tprev - g) to (at, at - g)
        a0, a1 = tprev - g, at - g
        area += 0.5 * (a0 + a1) * (at - tprev)
        if gen > g:
            peaks.append(a1)
            g = gen
        tprev = at
    span = ev[-1][0] - ev[0][0]
    return {"n": len(ev), "mean": area / span if span > 0 else float("nan"),
            "peak_mean": float(np.mean(peaks)) if peaks else float("nan"),
            "max": float(max(peaks)) if peaks else float("nan")}


def _gen_times(recs: List[Dict], dtp: DTP, src: Optional[List[Dict]], ts_offset: Optional[int]):
    """{i: [(acceptance time, generation time)]} of the accepted messages of the topics of R."""
    acc = [r for r in recs if r.get("type") == "acc"]
    out: Dict[int, List[Tuple[float, float]]] = defaultdict(list)
    if src:
        rel = {}
        for s in src:
            l = s.get("l")
            if l is None and "topic" in s:
                l = dtp.names.get(s["topic"])
            at = s.get("rel_at")
            if at is None and "rel_us" in s:
                at = s["rel_us"] * 1e-6
            if l is None or at is None or "c" not in s:
                continue
            rel[(int(s["i"]), int(l), s["c"][:32])] = float(at)
        for r in acc:
            g = rel.get((r["i"], r["l"], r["c"][:32]))
            if g is not None:
                out[r["i"]].append((r["at"], g))
    elif ts_offset is not None:
        for r in acc:
            b = bytes.fromhex(r["c"])
            if len(b) >= ts_offset + 8:
                us = int.from_bytes(b[ts_offset:ts_offset + 8], "big")
                out[r["i"]].append((r["at"], us * 1e-6))
    return out


def analyze(recs: List[Dict], dtp: DTP, gt: Optional[Dict[int, Dict]] = None, src: Optional[List[Dict]] = None,
            ts_offset: Optional[int] = None, skip: int = 0, batch: int = 50) -> Dict:
    a = steps(recs, skip)
    rows = per_step(a, gt)
    res: Dict = {"steps": len(rows)}
    if not rows:
        return res
    f = [r["f"] for r in rows]
    n, k = len(f), sum(f)
    res["f_bar"] = {"value": k / n, "beta": dtp.beta, "wilson95": wilson(k, n), "batch95": batch_means_ci(f, batch)}
    # (ii) coverage and (iii) MAE on the steps with kappa(t)
    kr = [r for r in rows if r["kappa"] is not None]
    if kr:
        cov = [1 if r["kappa"] >= (r["Z"] or 0.0) * r["kappa_hat"] else 0 for r in kr]
        res["coverage"] = {"value": float(np.mean(cov)), "n": len(kr), "target": 1 - dtp.beta}
        res["mae_kappa_hat"] = float(np.mean([abs(r["kappa_hat"] - r["kappa"]) for r in kr]))
        res["mean_kappa"] = float(np.mean([r["kappa"] for r in kr]))
        res["mean_kappa_hat"] = float(np.mean([r["kappa_hat"] for r in kr]))
        den = sum(r["kappa"] * r["T_W"] for r in kr)
        res["nu"] = sum(r["cost_A"] for r in kr) / den if den > 0 else None
    zs = [r["z"] for r in rows if r["z"] is not None]
    res["scores"] = {"n": len(zs), "q05": float(np.quantile(zs, 0.05)) if zs else None,
                     "median": float(np.median(zs)) if zs else None}
    # (iv) per-edge delivery ratio d / released bytes, edges with p >= 1
    vl = {}
    for r in recs:
        if r.get("type") == "vlog" and "t" in r:
            vl[(int(r["i"]), int(r["t"]))] = r
    num, den = defaultdict(float), defaultdict(float)
    numm, denm = defaultdict(float), defaultdict(float)
    for r in rows:
        for si, pl in r["p"].items():
            i = int(si)
            v = vl.get((i, r["t"]))
            rep = r["rep"].get(si)
            for l, p in enumerate(pl):
                if p < 1:
                    continue
                d = r["d"].get(si, [0] * dtp.L)[l]
                if v is not None and "xrel" in v:
                    num[(i, l)] += d
                    den[(i, l)] += v["xrel"][l]
                if rep is not None and rep.get("t") == r["t"] and l < len(rep.get("rel", [])):
                    numm[(i, l)] += r["acc"].get(si, [0] * dtp.L)[l]
                    denm[(i, l)] += sum(rep["rel"][l])
    res["edge_bytes"] = {"%d/%s" % (i, dtp.topics[l].name): (num[(i, l)] / den[(i, l)] if den[(i, l)] else None)
                         for (i, l) in sorted(den)}
    res["edge_msgs"] = {"%d/%s" % (i, dtp.topics[l].name): (numm[(i, l)] / denm[(i, l)] if denm[(i, l)] else None)
                        for (i, l) in sorted(denm)}
    # delivery ratio of the needed data and deadline miss ratio (vessel logs)
    nd_num = nd_den = dl_acc = dl_rel = 0.0
    for r in rows:
        for i in dtp.vessels:
            v = vl.get((i, r["t"]))
            if v is None:
                continue
            accl = r["acc"].get(str(i), [0] * dtp.L)
            N = set(r["needed"].get(str(i), dtp.needed))
            for l in range(dtp.L):
                if l in N and "rx" in v:
                    nd_num += accl[l]
                    nd_den += v["rx"][l]
                if "rel" in v:
                    dl_acc += accl[l]
                    dl_rel += v["rel"][l]
    res["needed_delivery"] = nd_num / nd_den if nd_den else None
    res["deadline_miss"] = 1 - dl_acc / dl_rel if dl_rel else None
    # (vi) age of information of the state
    gens = _gen_times(recs, dtp, src, ts_offset)
    res["aoi"] = {str(i): aoi(ev) for i, ev in sorted(gens.items())}
    # Theorem 1 over every prefix; F_R and H_R
    alphas = [r["alpha"] if r["alpha"] is not None else dtp.alpha1 for r in rows]
    chk = theorem_check(f, alphas, dtp.beta, dtp.gamma, alphas[0])
    viol = [T for T, (lft, rgt, _) in enumerate(chk, start=1) if lft > rgt + 1e-12]
    FR = chk[-1][2]
    R = dtp.R
    HR = 0
    for r in rows:
        lost = any(x[1] in ("noreport", "notab") for x in r["reasons"] if len(x) > 1)
        R_alone = all(u[1] in R for u in r["A"])
        X = r["X"] if r["X"] is not None else (r["cost_A"] + r["cB"])
        over = r["kappa"] is not None and X > r["kappa"] * r["T_W"]
        if (R_alone and over) or lost:
            HR += 1
    res["theorem1"] = {"T": n, "left": chk[-1][0], "right": chk[-1][1], "holds_all_prefixes": not viol,
                       "violations": viol[:10], "alpha1": alphas[0]}
    res["F_R"] = FR
    res["H_R"] = HR
    res["H_R_X"] = "ground truth" if gt and any(r["X"] is not None for r in rows) else "cost bound"
    res["reasons"] = dict(_count_reasons(rows))
    tm = [x.get("pub_ms") for x in a.values() if x.get("pub_ms") is not None]
    if tm:
        res["phase2_pub_ms"] = {"p50": float(np.median(tm)), "p99": float(np.quantile(tm, 0.99)), "max": max(tm)}
    return res


def _count_reasons(rows):
    c = defaultdict(int)
    for r in rows:
        for x in r["reasons"]:
            c[x[1]] += 1
    return c


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.as2.analysis", description=__doc__.split("\n")[0])
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--dta", required=True, nargs="+")
    ap.add_argument("--gt", default=None, help="ground truth JSON lines {t, kappa, X}")
    ap.add_argument("--src", default=None, help="source log JSON lines {i, topic, rel_us, c}")
    ap.add_argument("--ts-offset", type=int, default=None, help="offset of a u64 timestamp (us of the Unix time) in the content")
    ap.add_argument("--skip", type=int, default=0, help="warm-up steps to drop")
    a = ap.parse_args(argv)
    dtp = DTP.load(a.dtp)
    gt = None
    if a.gt:
        gt = {int(o["t"]): o for o in load(a.gt) if "t" in o}
    src = load(a.src) if a.src else None
    out = {}
    for p in a.dta:
        out[p] = analyze(load(p), dtp, gt, src, a.ts_offset, a.skip)
    json.dump(out, sys.stdout, indent=1, default=float)
    print()


if __name__ == "__main__":
    main()
