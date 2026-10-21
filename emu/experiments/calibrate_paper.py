#!/usr/bin/env python3
"""Calibration Cal of the paper (Section VI-E): the constants T1, T2, eps, delta_GS, T_m and B_ctl_bar per number
of vessels M, the global c_y, C_ref per (M, scenario), and alpha* and U* per (M, left-out scenario), from the run
directories of the calibration runs (emu/fleet.py --stack paper; the computation is fleetdt/calibration.py).

  python3 emu/experiments/calibrate_paper.py compute RUN_OR_DIR [...] --out calibration.json [--warm 240]
          [--lag-ms 1] [--target 0.99] [--conf 0.95] [--B 1000] [--seed 0] [--data-ports 1884,1885,1886]
          [--no-gt] [--include-failed]
      every directory with a result.json of the paper stack under the arguments is a run; runs whose status is
      not "ok" are left out (unless --include-failed); the Fleet-DT runs (--fit-methods) give the constants,
      the runs of B4, B6 and B7 (the E4 pilot) give the U of these baselines by the rule of U* ("pilot";
      fleetdt.calibration.pilot), and the runs of the other methods stay out ("ignored")
  python3 emu/experiments/calibrate_paper.py dtp --calibration calibration.json --dtp BASE.json --M 4
          --scenario S3 --out dtp.json
      the DTP of an evaluation run: T1, T2, T_m, B_ctl_bar of M, c_y, and the alpha* and U* of M without the
      scenario of the run (the fit on every calibration scenario when the calibration has no run of it)

Files of a run (RUN/):
  result.json            status, M, S0, steps, method
  dtp.json               epoch, Ts, T1, T2, h_W of the run
  scenario.json          scenario_id (else experiment.json "scenario_id", else "qualify"."profile")
  vms/gs/as.jsonl        decisions of the AS ("d_at", "M") and "demand_late" records: w1(t), M(t)
  vms/v<i>/regsteps.jsonl  step log of the vessel regulator ("p_rx_us", "p_late_us", "late_us", "late_p1_us")
  vms/<host>/chrony.jsonl  e_i(t) of emu/tools/chrony_sampler.py (host gs and v<i>)
  vms/gs/gsdelivery.jsonl  delivery times of the GS broker (fdt_gsstamp.so)
  clock.csv              probe offsets theta_i(t)
  emu/gt.bin (or gt.bin.zst, decompressed with zstd)  ground truth of the emulator: B_ctl(t)
  csub/c_t.jsonl         C(t) of emu/replay_subsets.py; throughput.jsonl (else the DTA) y(t)
  vms/gs/dta.jsonl       DTA of the run: alpha*, U*
"""
import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "emu"))

from fleetdt import calibration as CAL  # noqa: E402
from fleetdt.as_model import DTP  # noqa: E402


def load_json(p):
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def jsonl(p):
    out = []
    try:
        with open(p, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        continue
    except OSError:
        pass
    return out


def find_runs(paths):
    """run directories (with result.json) under the given paths, sorted"""
    out = []
    for p in paths:
        if os.path.isfile(os.path.join(p, "result.json")):
            out.append(os.path.abspath(p))
            continue
        for dp, dn, fn in os.walk(p):
            dn[:] = sorted(d for d in dn if d not in ("vms", "emu", "csub"))
            if "result.json" in fn:
                out.append(os.path.abspath(dp))
                dn[:] = []
    return sorted(set(out))


def scenario_of(rd, res):
    sc = load_json(os.path.join(rd, "scenario.json")) or {}
    if sc.get("scenario_id"):
        return str(sc["scenario_id"])
    ex = load_json(os.path.join(rd, "experiment.json")) or {}
    for v in (ex.get("scenario_id"), (ex.get("qualify") or {}).get("profile"), res.get("scenario_id")):
        if v:
            return str(v)
    return "unknown"


def load_gt(rd):
    """(origin_ns, records) of RUN/emu/gt.bin or gt.bin.zst; None without the file"""
    import gt as GT
    p = os.path.join(rd, "emu", "gt.bin")
    if os.path.isfile(p):
        hdr, recs = GT.load(p)
        return int(hdr.get("origin_ns", 0)), recs
    pz = p + ".zst"
    if not os.path.isfile(pz):
        return None
    r = subprocess.run(["zstd", "-dc", pz], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    if r.returncode != 0:
        return None
    return parse_gt(r.stdout)


def parse_gt(data):
    import struct
    import numpy as np
    import gt as GT
    if data[:6] != b"FDTGT1":
        return None
    (n,) = struct.unpack("<I", data[8:12])
    hdr = json.loads(data[12:12 + n])
    body = data[12 + n:]
    k = len(body) // GT.DT.itemsize
    return int(hdr.get("origin_ns", 0)), np.frombuffer(body[:k * GT.DT.itemsize], dtype=GT.DT)


def load_run(rd, with_gt=True):
    """input of fleetdt.calibration.measure_run from a run directory"""
    res = load_json(os.path.join(rd, "result.json")) or {}
    dtp = load_json(os.path.join(rd, "dtp.json")) or {}
    M = int(res.get("M") or len(dtp.get("vessels") or []))
    vms = os.path.join(rd, "vms")
    host = lambda h: os.path.join(vms, "gs" if h == 0 else "v%d" % h)   # noqa: E731
    off = None
    if os.path.isfile(os.path.join(rd, "clock.csv")):
        with open(os.path.join(rd, "clock.csv")) as f:
            off = CAL.Offsets.from_csv_lines(f)
    C = {int(r["t"]): float(r["C"]) for r in jsonl(os.path.join(rd, "csub", "c_t.jsonl"))
         if r.get("t") is not None and r.get("C") is not None}
    dta = jsonl(os.path.join(host(0), "dta.jsonl"))
    ys = jsonl(os.path.join(rd, "throughput.jsonl")) or dta
    y = {int(r["t"]): float(r["y"]) for r in ys if r.get("t") is not None and r.get("y") is not None}
    return {"name": os.path.basename(rd), "dir": rd, "M": M, "scenario": scenario_of(rd, res),
            "method": res.get("method") or "fleetdt",
            "status": res.get("status"), "S0": res.get("S0"), "steps": res.get("steps"), "dtp": dtp,
            "as": jsonl(os.path.join(host(0), "as.jsonl")),
            "regsteps": {i: jsonl(os.path.join(host(i), "regsteps.jsonl")) for i in range(1, M + 1)},
            "chrony": {h: jsonl(os.path.join(host(h), "chrony.jsonl")) for h in range(M + 1)},
            "offsets": off, "gsdelivery": jsonl(os.path.join(host(0), "gsdelivery.jsonl")),
            "gt": load_gt(rd) if with_gt else None, "C": C, "y": y, "dta": dta}


def _clean(o):
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, float) and o != o:
        return None
    if hasattr(o, "item"):
        return _clean(o.item())
    return o


def cmd_compute(a):
    dirs = find_runs(a.runs)
    runs, skipped = [], []
    for rd in dirs:
        res = load_json(os.path.join(rd, "result.json")) or {}
        if res.get("stack", "paper") != "paper" or (res.get("status") != "ok" and not a.include_failed):
            skipped.append({"run": rd, "status": res.get("status"), "stack": res.get("stack")})
            continue
        runs.append(load_run(rd, with_gt=not a.no_gt))
    if not runs:
        raise SystemExit("no calibration run under %s" % a.runs)
    ports = tuple(int(p) for p in a.data_ports.split(",") if p)
    fit_methods = tuple(m for m in a.fit_methods.split(",") if m)
    try:
        cal = CAL.calibrate(runs, warm=a.warm, lag_s=a.lag_ms / 1e3, target=a.target, conf=a.conf, B=a.B,
                            seed=a.seed, data_ports=ports, fit_methods=fit_methods)
    except ValueError as e:
        raise SystemExit(str(e))
    cal["created"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    cal["skipped"] = skipped
    cal = _clean(cal)
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(cal, f, indent=1)
    for M, mg in sorted(cal["M"].items()):
        print("M = %s: %d runs, T1 %s, T2 %s, eps %s, delta_GS %s, T_m %s s, B_ctl_bar %s bytes" % (
            M, mg["runs"], mg["T1"], mg["T2"], mg["eps"], mg["delta_GS"], mg["T_m"], mg["B_ctl_bar"]))
        for s, o in sorted(cal["outer"][M]["left_out"].items()):
            print("  %s: alpha* %s, U* %s (%d runs)" % ("every scenario" if s == CAL.ALL else "without " + s,
                                                         o["alpha"], o["U"], o["runs"]))
    for M, by_s in sorted(cal["pilot"].items()):
        for s, by_m in sorted(by_s.items()):
            for m, f in sorted(by_m.items()):
                print("  pilot M = %s, %s, %s: U %s (%d runs)" % (M, s, m, f.get("U"), f.get("runs", 0)))
    if cal["ignored"]:
        print("runs of other methods, not used: %d" % len(cal["ignored"]))
    print("c_y = %s (%d steps); %s" % (cal["c_y"], cal["c_y_n"], a.out))
    missing = sorted({x for m in cal["runs"] for x in m["missing"]})
    for x in missing:
        print("missing in some runs: %s" % x)


def cmd_dtp(a):
    cal = load_json(a.calibration)
    if cal is None:
        raise SystemExit("cannot read %s" % a.calibration)
    base = load_json(a.dtp)
    if base is None:
        raise SystemExit("cannot read %s" % a.dtp)
    if str(a.M) not in cal.get("M", {}):
        raise SystemExit("the calibration has no M = %d (it has %s)" % (a.M, sorted(cal.get("M", {}))))
    out, kept = CAL.apply_to_dtp(base, cal, a.M, a.scenario)
    out["vessels"] = base.get("vessels") or list(range(1, a.M + 1))
    DTP.from_dict(out)   # the result must be a valid DTP
    with open(a.out, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1)
    print("%s: M = %d, scenario %s (left out: %s); T1 %s, T2 %s, T_m %s, B_ctl_bar %s, c_y %s, alpha %s, U %s" % (
        a.out, a.M, a.scenario, out["calibration"]["left_out"], out.get("T1"), out.get("T2"), out.get("T_m"),
        out.get("B_ctl_bar"), out.get("c_y"), out.get("alpha"), out.get("U")))
    if kept:
        print("not calibrated, kept from %s: %s" % (a.dtp, ", ".join(kept)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("compute", help="calibration.json from the calibration runs")
    c.add_argument("runs", nargs="+")
    c.add_argument("--out", required=True)
    c.add_argument("--warm", type=int, default=CAL.WARM_STEPS, help="warm-up steps after S0 (240 = 30 s)")
    c.add_argument("--lag-ms", type=float, default=CAL.LAG_S * 1e3, help="largest lag of a valid run in eps")
    c.add_argument("--target", type=float, default=CAL.PICP_NOM, help="PICP_nom of U*")
    c.add_argument("--conf", type=float, default=CAL.CONF, help="confidence of the lower bound of PICP(U)")
    c.add_argument("--B", type=int, default=1000, help="bootstrap resamples")
    c.add_argument("--seed", type=int, default=0)
    c.add_argument("--data-ports", default=",".join(str(p) for p in CAL.DATA_PORTS),
                   help="TCP ports of the data bridges (frames outside them count in B_ctl)")
    c.add_argument("--no-gt", action="store_true", help="skip the ground truth (no B_ctl_bar)")
    c.add_argument("--include-failed", action="store_true", help="also runs whose status is not ok")
    c.add_argument("--fit-methods", default=",".join(CAL.FIT_METHODS),
                   help="methods whose runs give the constants (the runs of B4, B6 and B7 give the pilot U)")
    d = sub.add_parser("dtp", help="DTP of an evaluation run from calibration.json")
    d.add_argument("--calibration", required=True)
    d.add_argument("--dtp", required=True, help="base DTP (dtp.json)")
    d.add_argument("--M", type=int, required=True)
    d.add_argument("--scenario", default=None, help="scenario of the evaluation run (S1..S9)")
    d.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    if a.cmd == "compute":
        cmd_compute(a)
    else:
        cmd_dtp(a)


if __name__ == "__main__":
    main()
