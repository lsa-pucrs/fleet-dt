#!/usr/bin/env python3
"""One run of the simulation (fdt-sim, not real time) with the AS of the repository (scale study,
emu/experiments/addendum/ADDENDUM.md, section A2).

  python3 emu/sim/simrun.py SPEC.json --out RUN_DIR [--bin FDT_SIM] [--campaign DIR] [--no-metrics]

SPEC is a run file of emu/experiments/matrix.py (the format of the emulation runs). The driver
* fixes S0 and t0 = start(S0) (a fixed host time, so the step indices look like those of the emulation);
* computes the release instants and the sizes of the sources with the code of
  vessel/publishers/fdt_source.py (mode --t0, seed of the run), and the goals and the orders of the
  operation control stub with fleetdt/as2/opctl.py (seed of the run), as the emulation does;
* starts fdt-sim, which simulates the channel and the hosts, and runs ASCore (fleetdt/as2/core.py,
  unchanged) at each phase 2 with the events of the GS and the monitor batches that fdt-sim sends;
* writes RUN_DIR in the layout of an emulation run (experiment.json, dtp.json, scenario.json,
  result.json, emu/gt.bin, emu/summary.json, vms/gs/dta.jsonl, vms/gs/order.jsonl, vms/v<i>/src.jsonl), so
  emu/analysis/gtstep.py and metrics.py analyze it unchanged; then runs them (unless --no-metrics).

Differences from the emulation (declared in ADDENDUM.md A2): the host clocks have no offset; the hosts
run the TCP of ns-3 (CUBIC, SACK, timestamps, delayed ACK 40 ms, minimum RTO 200 ms) in place of Linux;
the MQTT bridges are the codec of fdt-sim (Mosquitto semantics for QoS 0, 1, 2); the loopback hops
inside the GS and inside a vessel take fixed times; the clock synchronization traffic is absent.
"""
import argparse
import json
import math
import os
import random
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "vessel", "publishers"))

from fleetdt.as2.core import ASCore  # noqa: E402
from fleetdt.as2.dtp import DTP  # noqa: E402
from fleetdt.as2 import opctl  # noqa: E402
import fdt_source  # noqa: E402

DEFAULT_EPOCH = 1767225600.0
MONITOR_LAG = 0.0025       # emu/fleet.py: the monitor batch MONITOR_LAG after the end of Q(t) and of W(t)
AS_START = 0.0044          # phase 2 starts start(t) + T1 + AS_START (E1: publication - compute = 4.6 ms)
HOST_T = 1792454400.0      # 2026-10-20T00:00:00Z: host time near which the simulated runs start
METHODS = {"fleetdt": ("fleetdt", "fleetdt"), "slotted": ("slotted", "slotted"), "b1": ("b1", "b1"),
           "b3": ("b3", "b3"), "b4": ("fleetdt", "b4"), "b5": ("b5", "b5"), "tdma": ("tdma", "tdma"),
           "b6": ("slotted", "ipact")}
DEFAULT_BIN = os.path.expanduser("~/fleetdt-deploy/emu/sim-ns3/ns-3.48/build/scratch/fdt-sim/ns3.48-fdt-sim-optimized")


def epoch_of(d):
    return DEFAULT_EPOCH if d.get("epoch") is None else float(d["epoch"])


class Args(object):
    pass


def source_schedule(dtp_json, dtp_path, vessel, seed, t0, t_start, t_end, trace):
    """(lines of the schedule, lines of src.jsonl) of the sources of one vessel: the release instants and
    sizes of fdt_source.Source in mode --t0 (the same draws as the emulation runs of this seed)."""
    a = Args()
    a.seed, a.vessel, a.t0 = int(seed), int(vessel), float(t0)
    a.size_trace = os.path.join(ROOT, trace) if trace else None
    a.median, a.sigma, a.load, a.load_mode = None, 0.5, 1.0, "size"
    a.phase, a.stream_size, a.topics, a.log = "random", "max", None, None
    d = fdt_source.load_dtp(dtp_path)
    src = fdt_source.Source(a, d, random.Random(a.seed * 1000 + a.vessel))
    names = [tp.name for tp in d["topics"]]
    heap = []
    import heapq
    for tp in d["topics"]:
        r = src.rate(tp)
        if r <= 0:
            continue
        period = 1.0 / r
        ph = a.t0 + src.phase(tp, period)
        j0 = int(math.ceil((t_start + 0.05 - ph) / period))
        base = ph + j0 * period
        heapq.heappush(heap, (base + src.draw(tp, j0, "jit").uniform(0, tp.jitter), base, tp.name, tp, j0))
    lines, logs = [], []
    while heap:
        at, nominal, name, tp, j = heapq.heappop(heap)
        if at >= t_end:
            continue
        t_us = int(at * 1e6)
        l = names.index(name)
        if tp.kind == "stream":
            sizes = [tp.s] * tp.n
            total, trunc, oid = sum(sizes), False, j
            kind = 0
        else:
            size, trunc = src.output_size(tp, j)
            chunk = tp.s - fdt_source.OUT_HEADER.size
            m = max(1, int(math.ceil(size / float(chunk))))
            sizes, left = [], size
            for _ in range(m):
                part = min(chunk, left)
                left -= part
                sizes.append(fdt_source.OUT_HEADER.size + part)
            total, oid = size, j & 0xFFFFFFFF
            kind = 1
        lines.append("S %d %d %d %d %d %s" % (t_us, vessel, l, j & 0xFFFFFFFF, kind, " ".join(str(s) for s in sizes)))
        logs.append({"topic": name, "out": oid, "j": j, "n": len(sizes), "bytes": total, "t_us": t_us, "trunc": bool(trunc)})
        nxt = nominal + 1.0 / src.rate(tp)
        heapq.heappush(heap, (nxt + src.draw(tp, j + 1, "jit").uniform(0, tp.jitter), nxt, name, tp, j + 1))
    return lines, logs


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("spec")
    ap.add_argument("--out", required=True)
    ap.add_argument("--bin", default=DEFAULT_BIN)
    ap.add_argument("--campaign", default=None, help="directory against which init_scores_file resolves")
    ap.add_argument("--lead", type=float, default=6.0, help="simulated seconds before start(S0)")
    ap.add_argument("--no-metrics", action="store_true")
    ap.add_argument("--keep-raw", action="store_true", help="keep gt.bin uncompressed")
    ap.add_argument("--gt-states", default="ap")
    ap.add_argument("--sim-args", default="", help="extra arguments of fdt-sim, one string (e.g. '--delAckCount=1')")
    a = ap.parse_args()
    with open(a.spec) as f:
        spec = json.load(f)
    rd = a.out
    os.makedirs(os.path.join(rd, "emu"), exist_ok=True)
    os.makedirs(os.path.join(rd, "vms", "gs"), exist_ok=True)
    dtpj = spec["dtp"]
    M = int(spec["M"])
    Ts = float(dtpj["Ts"])
    steps = int(spec.get("steps", 1200))
    method = spec["method"]
    if method not in METHODS:
        raise SystemExit("method %s is not simulated" % method)
    pmode, amode = METHODS[method]
    with open(os.path.join(rd, "experiment.json"), "w") as f:
        json.dump(spec, f, indent=1)
    dtp_path = os.path.join(rd, "dtp.json")
    with open(dtp_path, "w") as f:
        json.dump(dtpj, f, indent=1)
    scen_path = os.path.join(rd, "scenario.json")
    with open(scen_path, "w") as f:
        json.dump(spec["scenario"], f, indent=1)
    ep = epoch_of(dtpj)
    S0 = int(math.floor((HOST_T - ep) / Ts)) + 1
    t0 = ep + (S0 - 1) * Ts
    T1 = float(dtpj["T1"])
    t_end = ep + (S0 + steps) * Ts + T1 + 1.5
    seed = int(spec.get("seed", 1))
    res = {"name": spec.get("name"), "slot": None, "M": M, "method": method, "seed": seed, "simulated": True,
           "t_start": time.time(), "status": "started", "S0": S0, "t0": t0, "steps": steps, "t_end_plan": t_end}
    try:
        res["revision"] = json.load(open(os.path.join(ROOT, ".fdt-revision.json")))
    except (OSError, ValueError):
        res["revision"] = {"commit": None}

    def save():
        with open(os.path.join(rd, "result.json"), "w") as f:
            json.dump(res, f, indent=1)
    save()
    dtp = DTP.load(dtp_path)

    # ---- sources (release instants of the emulation for this seed) and goals, orders of the stub
    t_src = t0 - a.lead + 2.0
    sched = []
    for i in range(1, M + 1):
        lines, logs = source_schedule(dtpj, dtp_path, i, seed, t0, t_src, t_end + 3.0,
                                      (spec.get("source") or {}).get("trace"))
        sched += lines
        os.makedirs(os.path.join(rd, "vms", "v%d" % i), exist_ok=True)
        with open(os.path.join(rd, "vms", "v%d" % i, "src.jsonl"), "w") as f:
            for x in logs:
                f.write(json.dumps(x, separators=(",", ":")) + "\n")
    op = spec.get("opctl") or {}
    period = int(op.get("period", round(10.0 / Ts)))
    schedule = opctl.Schedule(dtp, period=period, t0=S0, seed=seed)
    rng = random.Random(seed)
    goal_every = max(1, int(round(1.0 / (dtp.goals_rate * dtp.Ts)))) if dtp.goals_rate > 0 else 0
    orders = []
    with open(os.path.join(rd, "vms", "gs", "order.jsonl"), "w") as fo:
        for t in range(S0 - 16, S0 + steps + 40):
            at = dtp.start(t) - 0.005
            order, needed, mode = schedule.plan(t)
            msg = {"t": t, "order": order, "needed": needed, "mode": mode}
            orders.append((at, msg))
            fo.write(json.dumps(dict(msg, at=at), separators=(",", ":")) + "\n")
            if goal_every and t % goal_every == 0:
                for i in dtp.vessels:
                    b = opctl.goals_payload(dtp, i, t, mode, rng)
                    sched.append("G %d %d %d" % (int(at * 1e6), i, len(b)))
    sched_path = os.path.join(rd, "emu", "schedule.txt")
    with open(sched_path, "w") as f:
        f.write("\n".join(sched) + "\n")

    # ---- AS (the arguments of emu/fleet.py as_cmd and fleetdt/as2/service.py main)
    asa = spec.get("as") or {}
    zs, bh = [], None
    if asa.get("init_scores_file"):
        p = asa["init_scores_file"]
        if not os.path.isabs(p):
            p = os.path.join(a.campaign or os.path.dirname(os.path.dirname(os.path.abspath(a.spec))), p)
        with open(p) as f:
            o = json.load(f)
        zs = [float(z) for z in (o["scores"] if isinstance(o, dict) else o)]
        if isinstance(o, dict) and o.get("b_hat") is not None:
            bh = float(o["b_hat"])
    dta = open(os.path.join(rd, "vms", "gs", "dta.jsonl"), "w")
    pend = []

    def line(rec):
        pend.append(rec)

    def acc_line(r):
        i, l, t, sq, at, c = r
        pend.append({"type": "acc", "i": i, "l": l, "t": t, "seq": sq, "at": at, "c": c})

    core = ASCore(dtp, mode=amode, z0=float(asa.get("z0", 1.0)), initial_scores=zs, b_hat0=bh or 0.0,
                  predictor=spec.get("predictor", "eq"), pred_gain=float(asa.get("pred_gain", 0.25)),
                  cal_kappa=asa.get("cal_kappa"), acc_log=acc_line, out_log=line)

    # ---- simulator
    flush = "--flush=%d:%d,%d" % (round(Ts * 1e6), round((T1 + MONITOR_LAG) * 1e6),
                                  round((Ts - dtpj["eps"] - dtpj["d_rx"] + MONITOR_LAG) * 1e6))
    cmd = [a.bin, "--scenario=%s" % scen_path, "--outdir=%s" % os.path.join(rd, "emu"), "--dtp=%s" % dtp_path,
           "--schedule=%s" % sched_path, "--regMode=%s" % pmode, "--S0=%d" % S0, "--steps=%d" % steps,
           "--t0Us=%d" % round(t0 * 1e6), "--leadS=%g" % a.lead, "--endUs=%d" % round(t_end * 1e6),
           "--p2OffUs=%d" % round((T1 + AS_START) * 1e6), flush, "--gtStates=%s" % a.gt_states] + a.sim_args.split()
    res["cmd"] = cmd
    save()
    log = open(os.path.join(rd, "emu", "sim.log"), "w")
    w0 = time.time()
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, text=True, bufsize=1)
    oi = 0
    n_p2 = 0
    dmarg = []    # start(t) + T1 - arrival of the demand of step t at the AS (s)
    ok = False
    try:
        for raw in proc.stdout:
            msg = json.loads(raw)
            if msg.get("k") == "end":
                res["sim"] = msg
                ok = True
                break
            if msg.get("k") != "p2":
                continue
            t, now = int(msg["t"]), msg["now_us"] * 1e-6
            evs = msg["ev"]
            # the orders of the stub reach the AS at start(t) - 5 ms; the events of the GS in time order
            items = []
            while oi < len(orders) and orders[oi][0] <= now:
                items.append((orders[oi][0], 0, orders[oi][1]))
                oi += 1
            items += [(e[1] * 1e-6, 1, e) for e in evs]
            items.sort(key=lambda x: (x[0], x[1]))
            for at, kind, e in items:
                if kind == 0:
                    core.on_order(e, at)
                elif e[0] == "a":
                    payload = bytes.fromhex(e[4])
                    payload = payload + bytes(max(0, int(e[3]) - len(payload)))
                    core.acceptor.on_message(e[2], payload, at)
                elif e[0] == "d":
                    try:
                        dj = json.loads(e[3])
                        dmarg.append(dtp.start(int(dj["t"])) + dtp.T1 - at)
                        core.on_demand(int(e[2]), dj, at)
                    except (ValueError, KeyError, TypeError, IndexError):
                        core.bad_inputs += 1
                elif e[0] == "v":
                    try:
                        o = json.loads(e[3])
                        pend.append(dict({"type": "vlog", "i": int(e[2]), "at": at}, **o))
                    except ValueError:
                        core.bad_inputs += 1
            for b in msg["mon"]:
                core.monitor.add(b)
            tabs, rec = core.phase2(t, publish=True)
            dl = dtp.phase2(t)
            rec.update({"published": True, "wake_ms": (now - dl) * 1e3,
                        "pub_ms": (now - dl) * 1e3 + rec["t_compute_ms"], "ack_ms_prev": None})
            out = [json.dumps(x, separators=(",", ":")) for x in pend]
            out.append(json.dumps(rec, separators=(",", ":")))
            pend.clear()
            dta.write("\n".join(out) + "\n")
            rep = {"c_us": int(round(rec["t_compute_ms"] * 1e3)),
                   "tabs": [[i, json.dumps(tb, separators=(",", ":"))] for i, tb in sorted(tabs.items())]}
            proc.stdin.write(json.dumps(rep, separators=(",", ":")) + "\n")
            proc.stdin.flush()
            n_p2 += 1
    finally:
        if pend:
            dta.write("\n".join(json.dumps(x, separators=(",", ":")) for x in pend) + "\n")
        dta.close()
        try:
            proc.stdin.close()
        except OSError:
            pass
        rc = proc.wait()
        log.close()
    if dmarg:
        dm = sorted(dmarg)
        q = lambda f: dm[min(len(dm) - 1, int(f * (len(dm) - 1)))]
        res["demand_margin_ms"] = {"n": len(dm), "late": sum(1 for x in dm if x < 0), "min": dm[0] * 1e3,
                                   "p001": q(0.001) * 1e3, "p01": q(0.01) * 1e3, "p50": q(0.5) * 1e3}
    res.update({"t_end": time.time(), "wall_s": time.time() - w0, "phase2": n_p2, "rc": rc,
                "status": "ok" if ok and rc == 0 and n_p2 == steps + 1 else "failed",
                "bad_inputs": core.bad_inputs, "late_demands": core.late_demands})
    save()
    if res["status"] != "ok":
        print("simrun: %s failed (rc %s, %d phases 2)" % (spec.get("name"), rc, n_p2), file=sys.stderr)
        sys.exit(1)
    if not a.no_metrics:
        for s in ("gtstep.py", "metrics.py"):
            r = subprocess.run([sys.executable, os.path.join(ROOT, "emu", "analysis", s), rd],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            if r.returncode != 0:
                res["status"] = "analysis_failed"
                res["analysis_error"] = r.stdout[-2000:]
                save()
                sys.exit(1)
    if not a.keep_raw and os.path.exists(os.path.join(rd, "emu", "gt.bin")):
        subprocess.run(["zstd", "-q", "-9", "-T2", "--rm", "-f", os.path.join(rd, "emu", "gt.bin")], check=False)
    print("simrun: %s ok in %.0f s" % (spec.get("name"), res["wall_s"]))


if __name__ == "__main__":
    main()
