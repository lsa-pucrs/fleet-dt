#!/usr/bin/env python3
"""Full-software run of Fleet-DT on the simulated network of topology.sh (run as root in the VM).

The GS programs run in namespace gs: GS broker with the bridges to 10.0.0.99 and 10.0.0.98, AS,
channel monitor on the GS radio (nl80211 backend, survey of the patched mac80211_hwsim), and the
VE collector of this script. Each vessel namespace runs its broker with the vessel regulator, the
DTI program and the synthetic load. The injector occupies the channel during the bursts of the
plan, so C(t) drops as with the TX99 injector of Section VI-B.

  ip netns exec gs python3 tests/hwsim/scenario.py --out runs/hwsim-1 [--seconds 90]
         [--plan "30 30 0.7"] [--U 0.8] [--alpha 0.125]
Writes OUT/dta.jsonl, OUT/as.jsonl, OUT/metrics.json, OUT/summary.json and the logs of every program.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from fleetdt.analysis import analyze  # noqa: E402
from fleetdt.as_model import DTP, admission  # noqa: E402
from fleetdt.gsconf import gs_conf, vessel_conf  # noqa: E402
from fleetdt.ve.collector import Assembler, MqttCollector, StepClock  # noqa: E402

MOSQ = Path(os.environ.get("MOSQ", "/opt/mosquitto-2.1"))
BUILD = Path(os.environ.get("FDT_BUILD", ROOT / "build"))
VESSELS = {1: ("b1", "10.0.0.99"), 2: ("b2", "10.0.0.98")}


def iface(ns: str) -> str:
    out = subprocess.run(["ip", "netns", "exec", ns, "iw", "dev"], capture_output=True, text=True).stdout
    return next(l.split()[1] for l in out.splitlines() if l.strip().startswith("Interface"))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=90)
    ap.add_argument("--plan", default="30 30 0.7", help='bursts "OFFSET DURATION DUTY; ..." from the start of the load')
    ap.add_argument("--U", type=float, default=None)
    ap.add_argument("--alpha", type=float, default=None)
    ap.add_argument("--dtp", default=str(ROOT / "dtp.json"))
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    raw = json.loads(Path(a.dtp).read_text())
    raw["vessels"] = sorted(VESSELS)
    if a.U is not None:
        raw["U"] = a.U
    if a.alpha is not None:
        raw["alpha"] = a.alpha
    dtp_path = out / "dtp.json"
    dtp_path.write_text(json.dumps(raw, indent=1))
    dtp = DTP.from_dict(raw)

    procs = []

    def spawn(name, args, ns=None):
        log = open(out / ("%s.log" % name), "w")
        cmd = (["ip", "netns", "exec", ns] if ns else []) + [str(x) for x in args]
        p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=str(ROOT))
        procs.append((name, p))
        return p

    try:
        for i, (ns, addr) in VESSELS.items():
            conf = out / ("vessel%d.conf" % i)
            conf.write_text(vessel_conf(i, str(dtp_path), plugin=str(BUILD / "fdt_regulator.so"), stats_s=10,
                                        log_dest="stdout") + "user root\n")   # the plugin lives in a home directory
            spawn("broker-b%d" % i, [MOSQ / "sbin" / "mosquitto", "-c", conf], ns)
        time.sleep(1)
        gconf = out / "gs.conf"
        gconf.write_text(gs_conf({i: addr for i, (ns, addr) in VESSELS.items()}, listen="127.0.0.1:1883")
                         + "log_dest stdout\nuser root\n")
        spawn("broker-gs", [MOSQ / "sbin" / "mosquitto", "-c", gconf])
        time.sleep(2)
        spawn("chmon", [BUILD / "fdt_chmon", "--dtp", dtp_path, "--iface", iface("gs"), "--broker", "127.0.0.1:1883",
                        "--record", out / "samples.jsonl"])
        spawn("as", [sys.executable, "-m", "fleetdt.as_service", "--dtp", dtp_path, "--broker", "127.0.0.1:1883",
                     "--log", out / "as.jsonl", "-v"])
        graphs = []
        asm = Assembler(dtp, StepClock(dtp.Ts, dtp.t0))
        col = MqttCollector(asm, graphs.append, "127.0.0.1", 1883)
        col.start()
        time.sleep(2)
        for i, (ns, addr) in VESSELS.items():
            spawn("dti-b%d" % i, [BUILD / "fdt_dti", "--dtp", dtp_path, "--vessel", i, "--broker", "127.0.0.1:1883"], ns)
            spawn("load-b%d" % i, [sys.executable, ROOT / "vessel" / "publishers" / "raw_load.py", "--dtp", dtp_path,
                                   "--vessel", i, "--port", 1883, "--seed", 7], ns)
        t_load = time.time()
        plan = out / "plan.txt"
        plan.write_text("\n".join(b.strip() for b in a.plan.split(";") if b.strip()) + "\n")
        spawn("injector", [sys.executable, ROOT / "tests" / "hwsim" / "injector.py", iface("inj"), "--plan", plan,
                           "--start", t_load], "inj")
        time.sleep(a.seconds)
        col.stop()
    finally:
        for name, p in reversed(procs):
            if p.poll() is None:
                p.terminate()
        for name, p in reversed(procs):
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()

    with open(out / "dta.jsonl", "w") as f:
        for g in graphs:
            f.write(json.dumps(g.to_dict()) + "\n")
    t0 = math.floor((t_load - dtp.t0) / dtp.Ts)
    bursts = [tuple(map(float, l.split()[:3])) for l in plan.read_text().splitlines() if l.strip()]

    def phase(t):
        s = (t - t0) * dtp.Ts
        for off, dur, duty in bursts:
            if off <= s < off + dur:
                return "burst"
        return "clean"

    steady = [g for g in graphs if g.t >= t0 + 24]
    video = dtp.index("video")
    rows = {"clean": [], "burst": []}
    alg1 = policing = checked = 0
    for g in steady:
        rows[phase(g.t)].append(g)
        if g.vessels and g.b is not None and all(g.edge(g.vessels[0], l).p is not None for l in range(1, dtp.L + 1)):
            D = {i: [g.edge(i, l).d for l in range(1, dtp.L + 1)] for i in g.vessels}
            p = [g.edge(g.vessels[0], l).p for l in range(1, dtp.L + 1)]
            checked += 1
            alg1 += int(admission(D, g.order, g.needed, g.b) == p)
            policing += int(all(e.r <= e.d + 1 for e in g.edges))
    summary = {"steps": len(steady), "alg1_ok": alg1, "policing_ok": policing, "checked": checked}
    for k, gs in rows.items():
        ys = [g.y for g in gs if g.y is not None]
        rec = [g.to_dict() for g in gs]
        summary[k] = {
            "steps": len(gs),
            "y_mean": sum(ys) / len(ys) if ys else None,
            "C_hat_mean": sum(g.C_hat for g in gs if g.C_hat) / max(1, len(gs)),
            "m_full": sum(1 for g in gs if len(g.vessels) == 2),
            "video_admitted": sum(1 for g in gs if g.vessels and (g.edge(g.vessels[0], video).p or 0) > 0),
            "metrics": {m: v.get("value") if isinstance(v, dict) else v
                        for m, v in analyze(rec, dtp, ci=False).items()
                        if m in ("MAPE", "gamma", "delta", "phi", "eta", "mu", "nu")} if rec else {},
        }
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    (out / "metrics.json").write_text(json.dumps(analyze([g.to_dict() for g in steady], dtp, B=500), indent=1))
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
