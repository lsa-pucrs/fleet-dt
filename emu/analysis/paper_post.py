#!/usr/bin/env python3
"""Post-processing of one paper-stack run (emu/fleet.py --stack paper): the metrics of the paper (Section VI,
fleetdt/analysis.py) from the DTA of the VE, with the real-time validity of the emulator.

  python3 emu/analysis/paper_post.py RUN [--warm 240] [--criterion bytes|count] [--ci] [--out RUN/metrics.json]

Inputs: RUN/dtp.json, RUN/result.json (S0, steps), RUN/vms/gs/dta.jsonl (G(t) of every step) and
RUN/emu/steplag.csv (fdt-emu --phaseLag: largest lag of each phase of each step). The older model of the
as2 stack has its own post-processing (emu/analysis/gtstep.py and metrics.py), which does not apply here.

Steps: S0 + warm .. S0 + steps - 1. A step whose largest lag over the phases exceeds --lag-us (1000 us) or
that the emulator flags as a catch-up counts in "lag_discarded" and stays out of the metrics; the H4a
statistics of the analysis count such steps as failures (worst case), which the analysis scripts do
from the pooled runs.

Output (JSON): "stack": "paper", "n_steps" (steps in the metrics), "lag_discarded", "lag_steps" (their
indices), "warm_steps", "criterion", "metrics" (fleetdt.analysis.run_metrics: MASE, gamma, phi, delta, eta,
mu, nu, prefix completeness, fleet consistency, unused share, fill ratio), "use" (per topic: received
channel bytes r_il over the declared d_il of the admitted edges, the ratio of forwarded bytes to demand),
"aborted" (emulator lag abort) and "status" of the run. emu/experiments/runner.py reads "n_steps" and
"lag_discarded" for its lag rule.
"""
import argparse
import csv
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)

from fleetdt.analysis import run_metrics  # noqa: E402
from fleetdt.as_model import DTP  # noqa: E402
from fleetdt.outer import load_dta  # noqa: E402

PHASES = ("max1_us", "max2_us", "max3_us")


def load_json(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def read_steplag(path):
    """{scenario step k: (largest lag over the phases in us, catch-up flag)}"""
    out = {}
    try:
        with open(path) as f:
            for r in csv.DictReader(f):
                out[int(r["k"])] = (max(float(r[p]) for p in PHASES), int(r.get("catchup") or 0))
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


def post(rd, warm=240, criterion="bytes", ci=False, lag_us=1000.0):
    res = load_json(os.path.join(rd, "result.json")) or {}
    dtp = DTP.load(os.path.join(rd, "dtp.json"))
    dta = os.path.join(rd, "vms", "gs", "dta.jsonl")
    recs = load_dta(dta) if os.path.exists(dta) else []
    S0 = int(res.get("S0") or (min(int(r["t"]) for r in recs) if recs else 0))
    steps = int(res.get("steps") or 0) or (max(int(r["t"]) for r in recs) - S0 + 1 if recs else 0)
    summ = load_json(os.path.join(rd, "emu", "summary.json")) or {}
    lag = read_steplag(os.path.join(rd, "emu", "steplag.csv"))
    window = [r for r in recs if S0 + warm <= int(r["t"]) < S0 + steps]
    bad = []
    if lag is not None:
        for r in window:
            k = int(r["t"]) - S0
            v = lag.get(k)
            if v is None or v[0] > lag_us or v[1]:
                bad.append(int(r["t"]))
    keep = [r for r in window if int(r["t"]) not in set(bad)]
    out = {"stack": "paper", "dir": rd, "status": res.get("status"), "S0": S0, "steps": steps, "warm_steps": warm,
           "criterion": criterion, "n_steps": len(keep), "lag_discarded": len(bad), "lag_steps": bad,
           "lag_source": "steplag.csv" if lag is not None else None, "lag_us_limit": lag_us,
           "aborted": bool((summ.get("phase_lag") or {}).get("aborted")), "method": res.get("method")}
    if keep:
        out["metrics"] = run_metrics(keep, dtp, criterion=criterion, ci=ci)
        out["use"] = topic_use(keep, dtp)
    else:
        out["metrics"] = None
        out["use"] = {}
    return _clean(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir")
    ap.add_argument("--warm", type=int, default=240)
    ap.add_argument("--criterion", choices=("bytes", "count"), default="bytes")
    ap.add_argument("--ci", action="store_true", help="bootstrap intervals of the metrics (slower)")
    ap.add_argument("--lag-us", type=float, default=1000.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    out = post(a.run_dir, a.warm, a.criterion, a.ci, a.lag_us)
    path = a.out or os.path.join(a.run_dir, "metrics.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
    m = out.get("metrics") or {}
    print("paper_post %s: %d steps (%d with lag), gamma %s, phi %s, fill %s" % (
        a.run_dir, out["n_steps"], out["lag_discarded"], (m.get("gamma") or {}).get("value"),
        (m.get("phi") or {}).get("value"), (m.get("fill") or {}).get("mean")))
    return 0 if out["metrics"] is not None else 1


if __name__ == "__main__":
    sys.exit(main())
