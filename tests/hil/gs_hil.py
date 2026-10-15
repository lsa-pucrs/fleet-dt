#!/usr/bin/env python3
"""GS side of a hardware-in-the-loop run: GS broker with the bridges to the vessels, AS, optional
channel monitor, and the VE collector; writes the DTA, the decisions of the AS and a summary.

  python3 tests/hil/gs_hil.py --out runs/hil-1 --dtp dtp.json --vessel 1=192.168.80.102,11883,11884
         [--seconds 120] [--port 21883] [--iface wlan0] [--y CONST]
--iface runs fdt_chmon on that interface; --y publishes a constant y(t) per step instead (no monitor).
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from fleetdt.analysis import analyze  # noqa: E402
from fleetdt.as_model import DTP, admission  # noqa: E402
from fleetdt.gsconf import gs_conf  # noqa: E402
from fleetdt.model import tcp_nodelay  # noqa: E402
from fleetdt.ve.collector import Assembler, MqttCollector, StepClock  # noqa: E402

MOSQ = Path(os.environ.get("MOSQ", "/opt/mosquitto-2.1"))
BUILD = Path(os.environ.get("FDT_BUILD", ROOT / "build"))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--dtp", required=True)
    ap.add_argument("--vessel", action="append", required=True, metavar="I=ADDR,CTL,DATA")
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--port", type=int, default=21883)
    ap.add_argument("--iface", default=None)
    ap.add_argument("--y", type=float, default=None)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    dtp_path = Path(a.dtp).resolve()
    dtp = DTP.load(dtp_path)
    vessels = {int(s.split("=")[0]): s.split("=")[1] for s in a.vessel}
    procs = []

    def spawn(name, args):
        p = subprocess.Popen([str(x) for x in args], stdout=open(out / ("%s.log" % name), "w"),
                             stderr=subprocess.STDOUT, cwd=str(ROOT))
        procs.append(p)
        return p

    stop = threading.Event()
    try:
        (out / "gs.conf").write_text(gs_conf(vessels, listen="127.0.0.1:%d" % a.port) + "log_dest stdout\n")
        spawn("broker-gs", [MOSQ / "sbin" / "mosquitto", "-c", out / "gs.conf"])
        time.sleep(2)
        if a.iface:
            spawn("chmon", [BUILD / "fdt_chmon", "--dtp", dtp_path, "--iface", a.iface,
                            "--broker", "127.0.0.1:%d" % a.port, "--prio", "0"])
        elif a.y is not None:
            import paho.mqtt.client as mqtt
            c = tcp_nodelay(mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fdt-hil-y"))
            c.connect("127.0.0.1", a.port)
            c.loop_start()

            def ypub():
                t = math.floor((time.time() - dtp.t0) / dtp.Ts) + 1
                while not stop.is_set():
                    at = dtp.t0 + t * dtp.Ts + 0.0005
                    time.sleep(max(0.0, at - time.time()))
                    c.publish("fleet/throughput", json.dumps({"t": t - 1, "y": a.y}))
                    t += 1
            threading.Thread(target=ypub, daemon=True).start()
        spawn("as", [sys.executable, "-m", "fleetdt.as_service", "--dtp", dtp_path, "--broker", "127.0.0.1:%d" % a.port,
                     "--log", out / "as.jsonl", "-v"])
        graphs = []
        col = MqttCollector(Assembler(dtp, StepClock(dtp.Ts, dtp.t0)), graphs.append, "127.0.0.1", a.port)
        col.start()
        t_start = time.time()
        time.sleep(a.seconds)
        col.stop()
    finally:
        stop.set()
        for p in reversed(procs):
            p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
    with open(out / "dta.jsonl", "w") as f:
        for g in graphs:
            f.write(json.dumps(g.to_dict()) + "\n")
    t0 = math.floor((t_start - dtp.t0) / dtp.Ts) + 40          # 5 s of start-up
    steady = [g for g in graphs if g.t >= t0]
    rec = [g.to_dict() for g in steady]
    checked = alg1 = pol = 0
    for g in steady:
        if g.vessels and g.b is not None and all(g.edge(g.vessels[0], l).p is not None for l in range(1, dtp.L + 1)):
            D = {i: [g.edge(i, l).d for l in range(1, dtp.L + 1)] for i in g.vessels}
            checked += 1
            alg1 += int(admission(D, g.order, g.needed, g.b) == [g.edge(g.vessels[0], l).p for l in range(1, dtp.L + 1)])
            pol += int(all(e.r <= e.d + 1 for e in g.edges))
    dec = [json.loads(l) for l in (out / "as.jsonl").read_text().splitlines()] if (out / "as.jsonl").exists() else []
    pub = sorted(d["pub_delay"] for d in dec if d.get("published") and "pub_delay" in d)
    per_topic = {}
    for g in steady:
        for e in g.edges:
            s = per_topic.setdefault(dtp.topics[e.l - 1], {"n": 0, "fwd": 0, "held": 0, "late": 0, "dup": 0})
            s["n"] += e.n
            s["late"] += e.late
            s["dup"] += e.dup
            s["fwd"] += e.fwd or 0
            s["held"] += e.held or 0
    summary = {
        "steps": len(steady), "with_all_vessels": sum(1 for g in steady if len(g.vessels) == len(vessels)),
        "checked": checked, "alg1_ok": alg1, "policing_ok": pol,
        "as_pub_delay_ms": {"p50": 1e3 * pub[len(pub) // 2], "p99": 1e3 * pub[int(0.99 * (len(pub) - 1))],
                            "max": 1e3 * pub[-1]} if pub else None,
        "per_topic": per_topic,
        "metrics": {k: (v.get("value") if isinstance(v, dict) else v) for k, v in analyze(rec, dtp, ci=False).items()
                    if k in ("MAPE", "gamma", "delta", "phi", "eta", "mu", "nu", "J")} if rec else {},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
