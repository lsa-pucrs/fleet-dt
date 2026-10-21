#!/usr/bin/env python3
"""Post-processing of one paper-stack run (emu/fleet.py --stack paper): the metrics of the paper (Section VI,
fleetdt/analysis.py) from the DTA of the VE, with the per-step flags of the emulator and the inputs of the
pooled statistics of the hypothesis driver (emu/analysis/hypotheses.py).

  python3 emu/analysis/paper_post.py RUN [--warm 240] [--criterion count|bytes] [--ci] [--lag-us 1000]
                                         [--catchup-us 100] [--out RUN/metrics.json]

Inputs: RUN/dtp.json, RUN/result.json (S0, steps, M), RUN/vms/gs/dta.jsonl (G(t) of every step),
RUN/emu/steplag.csv (fdt-emu --phaseLag: largest lag of each phase of each step, lag at the start of phase 3
and at the end of the step, catch-up flag of the emulator), and when present RUN/vms/v<i>/regsteps.jsonl
(regulator step log: rx, fwd, late_fwd, x per topic), RUN/vms/v<i>/regulator.jsonl (vessel log: bytes of each
message not forwarded), RUN/csub/c_t.jsonl (C(t) of the subset replays), RUN/clock.csv (clock probe) and
RUN/h1.jsonl (per step {"t", "w1", "w2"} in seconds on the GS clock; no program writes it yet). The older
model of the as2 stack has its own post-processing (emu/analysis/gtstep.py and metrics.py).

Steps: S0 + warm .. S0 + steps - 1 (30 s at Ts = 125 ms: warm = 240), all kept in the metrics. Each record gets
the flags of its step: "lag" (largest lag over the phases above --lag-us, or no row in steplag.csv) and
"catchup" (the lag changes by more than --catchup-us between the start of phase 3 and the end of the step,
|lag_end_us - lag_p3_us|, the 0.1 ms of Section VI-B, or the emulator flags the step); H3a counts both as
failures. The run is real-time valid ("rt_valid") when the emulator did not abort it and no step from
scenario step 0 has a lag above --lag-us (the 1 ms rule of Section VI-B, applied offline because the
paper stack runs with --abortLagUs=0).

Sources of the byte delivery ratio: for each edge (i, l) of M(t), src_n = rx of the regulator step log (the
messages of step t that the sources produced) and src_x = their content bytes, the forwarded content
x - 6 (fwd + late_fwd) plus the bytes of the vessel log of (t, l); "src" = src_n for the completeness of PCR.

Output (JSON, RUN/metrics.json): "stack": "paper", "S0", "steps", "warm_steps", "M", "method", "criterion",
"n_steps" (steps in the metrics), "lag_flagged" and "lag_steps", "catchup_flagged" and "catchup_steps",
"steps_over_1ms" (lag_flagged / n_steps, read by the lag rule of emu/experiments/runner.py), "rt_valid",
"aborted", "status", "sources" (which optional inputs were found), "clock" (largest |theta_i(t) - theta_0(t)|
after the warm-up, H1c), "metrics" (fleetdt.analysis.run_metrics with series=True), "use" (per topic: received
channel bytes r_il over the declared d_il of the admitted edges).
"""
import argparse
import bisect
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)

from fleetdt.analysis import PAPER_CRITERION, run_metrics  # noqa: E402
from fleetdt.as_model import DTP  # noqa: E402
from fleetdt.outer import load_dta  # noqa: E402

PHASES = ("max1_us", "max2_us", "max3_us")
STEP_HEADER = 6


def load_json(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_jsonl(p):
    if not os.path.exists(p):
        return None
    out = []
    with open(p) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    continue
    return out


def read_steplag(path, catchup_us=100.0):
    """{scenario step k: (largest lag over the phases in us, catch-up flag)}; the catch-up flag is the flag of
    the emulator or |lag_end_us - lag_p3_us| > catchup_us (columns of fdt-emu --phaseLag)."""
    out = {}
    try:
        with open(path) as f:
            for r in csv.DictReader(f):
                cu = int(r.get("catchup") or 0) != 0
                if r.get("lag_p3_us") not in (None, "") and r.get("lag_end_us") not in (None, ""):
                    cu = cu or abs(float(r["lag_end_us"]) - float(r["lag_p3_us"])) > catchup_us
                out[int(r["k"])] = (max(float(r[p]) for p in PHASES), cu)
    except (OSError, KeyError, ValueError):
        return None
    return out


def topic_use(records, dtp):
    """per topic: sum of r_il over sum of d_il on the edges of M(t) with p_l(t) >= 1 and d_il > 0"""
    num, den = {}, {}
    for g in records:
        vs = set(g.get("vessels") or [])
        for e in g.get("edges") or []:
            if e.get("i") in vs and (e.get("p") or 0) >= 1 and (e.get("d") or 0) > 0:
                name = dtp.topics[int(e["l"]) - 1]
                num[name] = num.get(name, 0.0) + float(e.get("r") or 0)
                den[name] = den.get(name, 0.0) + float(e["d"])
    return {n: {"r_over_d": num[n] / den[n], "d": den[n]} for n in sorted(den, key=dtp.topics.index)}


def _clean(x):
    """JSON-safe copy (numpy scalars to floats, NaN to None)"""
    if isinstance(x, dict):
        return {str(k): _clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if hasattr(x, "item") and not isinstance(x, (str, bytes)):
        x = x.item()
    if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
        return None
    return x


def source_counts(rd, vessels):
    """{(t, i, l): (messages produced, content bytes)} from the regulator step logs and the vessel logs; None
    when no step log exists."""
    out, found = {}, False
    for i in vessels:
        vd = os.path.join(rd, "vms", "v%d" % i)
        st = load_jsonl(os.path.join(vd, "regsteps.jsonl"))
        if st is None:
            continue
        found = True
        held = {}
        for r in load_jsonl(os.path.join(vd, "regulator.jsonl")) or []:
            if r.get("t") is None or r.get("l") is None:
                continue
            k = (int(r["t"]), int(r["l"]))
            held[k] = held.get(k, 0) + int(r.get("bytes") or 0)
        for r in st:
            if r.get("t") is None or not r.get("rx"):
                continue
            t = int(r["t"])
            late = r.get("late_fwd") or [0] * len(r["rx"])
            for l, rx in enumerate(r["rx"], start=1):
                fwd = int((r.get("fwd") or [0] * l)[l - 1])
                x = int((r.get("x") or [0] * l)[l - 1])
                lf = int(late[l - 1]) if l - 1 < len(late) else 0
                xc = max(0, x - STEP_HEADER * (fwd + lf)) + held.get((t, l), 0)
                out[(t, i, l)] = (int(rx), xc)
    return out if found else None


def clock_theta(path, t_from):
    """largest |theta_i - theta_0| over the samples of the vessels at or after t_from (s, host clock), with
    theta_0 the offset of the GS at its nearest sample in time; None without clock.csv or GS samples."""
    try:
        with open(path) as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return None
    gs = sorted((int(r["host_ns"]), int(r["offset_ns"])) for r in rows if r.get("host") == "gs")
    if not gs:
        return None
    gt = [g[0] for g in gs]
    best, n = 0.0, 0
    for r in rows:
        if r.get("host") in (None, "gs"):
            continue
        h = int(r["host_ns"])
        if h < t_from * 1e9:
            continue
        k = bisect.bisect_left(gt, h)
        cand = [j for j in (k - 1, k) if 0 <= j < len(gs)]
        j = min(cand, key=lambda j: abs(gt[j] - h))
        best = max(best, abs(int(r["offset_ns"]) - gs[j][1]) / 1e9)
        n += 1
    return {"theta_max_abs_s": best if n else None, "n": n, "from_s": t_from}


def post(rd, warm=240, criterion=PAPER_CRITERION, ci=False, lag_us=1000.0, catchup_us=100.0, B=2000):
    res = load_json(os.path.join(rd, "result.json")) or {}
    dtp = DTP.load(os.path.join(rd, "dtp.json"))
    dta = os.path.join(rd, "vms", "gs", "dta.jsonl")
    recs = load_dta(dta) if os.path.exists(dta) else []
    S0 = int(res.get("S0") or (min(int(r["t"]) for r in recs) if recs else 0))
    steps = int(res.get("steps") or 0) or (max(int(r["t"]) for r in recs) - S0 + 1 if recs else 0)
    M = int(res.get("M") or len(dtp.vessels) or max([len(r.get("vessels") or []) for r in recs] or [0]))
    summ = load_json(os.path.join(rd, "emu", "summary.json")) or {}
    lag = read_steplag(os.path.join(rd, "emu", "steplag.csv"), catchup_us)
    window = sorted((r for r in recs if S0 + warm <= int(r["t"]) < S0 + steps), key=lambda r: int(r["t"]))
    lag_steps, cu_steps = [], []
    if lag is not None:
        for r in window:
            v = lag.get(int(r["t"]) - S0)
            r["lag"] = v is None or v[0] > lag_us
            r["catchup"] = bool(v is None or v[1])
            if r["lag"]:
                lag_steps.append(int(r["t"]))
            if r["catchup"]:
                cu_steps.append(int(r["t"]))
    aborted = bool((summ.get("phase_lag") or {}).get("aborted"))
    rt_valid = (not aborted) and lag is not None and all(v[0] <= lag_us for k, v in lag.items() if k >= 0) and \
        all(lag.get(k) is not None for k in range(steps))
    src = source_counts(rd, range(1, M + 1))
    if src is not None:
        for r in window:
            t = int(r["t"])
            for e in r.get("edges") or []:
                v = src.get((t, int(e["i"]), int(e["l"])))
                if v is not None:
                    e["src_n"], e["src_x"], e["src"] = v[0], v[1], v[0]
    h1 = {int(r["t"]): r for r in load_jsonl(os.path.join(rd, "h1.jsonl")) or [] if r.get("t") is not None}
    for r in window:
        x = h1.get(int(r["t"]))
        if x:
            for k in ("w1", "w2"):
                if x.get(k) is not None:
                    r[k] = float(x[k])
    C_gt = {int(r["t"]): float(r["C"]) for r in load_jsonl(os.path.join(rd, "csub", "c_t.jsonl")) or []
            if r.get("t") is not None and r.get("C") is not None}
    clock = clock_theta(os.path.join(rd, "clock.csv"), dtp.t0 + (S0 + warm) * dtp.Ts)
    n = len(window)
    out = {"stack": "paper", "dir": rd, "status": res.get("status"), "S0": S0, "steps": steps, "warm_steps": warm,
           "M": M, "criterion": criterion, "n_steps": n, "lag_flagged": len(lag_steps), "lag_steps": lag_steps,
           "catchup_flagged": len(cu_steps), "catchup_steps": cu_steps,
           "steps_over_1ms": (len(lag_steps) / n) if n else None, "rt_valid": bool(rt_valid),
           "lag_source": "steplag.csv" if lag is not None else None, "lag_us_limit": lag_us, "catchup_us": catchup_us,
           "aborted": aborted, "method": res.get("method"), "clock": clock,
           "sources": {"regsteps": src is not None, "csub": bool(C_gt), "h1": bool(h1), "clock": clock is not None}}
    if window:
        out["metrics"] = run_metrics(window, dtp, criterion=criterion, ci=ci, B=B, C_gt=C_gt or None, series=True)
        out["use"] = topic_use(window, dtp)
    else:
        out["metrics"] = None
        out["use"] = {}
    return _clean(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir")
    ap.add_argument("--warm", type=int, default=240)
    ap.add_argument("--criterion", choices=("bytes", "count"), default=PAPER_CRITERION,
                    help="reception criterion (paper: count, a forwarded message arrives late or never)")
    ap.add_argument("--ci", action="store_true", help="bootstrap intervals of the metrics (slower)")
    ap.add_argument("--lag-us", type=float, default=1000.0, help="lag limit of a valid run (us)")
    ap.add_argument("--catchup-us", type=float, default=100.0,
                    help="catch-up: change of the lag between the start of phase 3 and the end of the step (us)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    out = post(a.run_dir, a.warm, a.criterion, a.ci, a.lag_us, a.catchup_us)
    path = a.out or os.path.join(a.run_dir, "metrics.json")
    with open(path, "w") as f:
        json.dump(out, f)
    m = out.get("metrics") or {}
    print("paper_post %s: %d steps (%d with lag, %d catch-up), PICP %s, DMR %s, fill %s" % (
        a.run_dir, out["n_steps"], out["lag_flagged"], out["catchup_flagged"], (m.get("PICP") or {}).get("value"),
        (m.get("DMR") or {}).get("value"), (m.get("fill") or {}).get("mean")))
    return 0 if out["metrics"] is not None else 1


if __name__ == "__main__":
    sys.exit(main())
