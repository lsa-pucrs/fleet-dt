"""Command line of the VE.

  python -m fleetdt.ve live    --broker localhost:1883 --dtp dtp.json --dta run.jsonl [--epoch T0]
  python -m fleetdt.ve replay  run.jsonl
  python -m fleetdt.ve mock    [--M 3] [--dta mock.jsonl]
  python -m fleetdt.ve publish-mock --broker localhost:1883 [--M 3] [--steps 400]
"""
from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path

from ..as_model import DTP


def _broker(s: str):
    host, _, port = s.partition(":")
    return host or "localhost", int(port or 1883)


def main() -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.ve", description="Virtual environment (VE) of Fleet-DT")
    sub = ap.add_subparsers(dest="cmd", required=True)

    lv = sub.add_parser("live", help="draw G(t) from the GS MQTT broker")
    lv.add_argument("--broker", default="localhost:1883")
    lv.add_argument("--dtp", type=Path, required=True, help="JSON with topics, Ts, T1, T2, alpha, U, C0, c_b, c_m")
    lv.add_argument("--dta", type=Path, help="JSON-lines file that receives each G(t)")
    lv.add_argument("--epoch", type=str, default=None, help="Unix time of the start of step 0 on the GS clock (default: the DTP epoch, or 2026-01-01T00:00:00Z when it is null; 'auto' takes it from the first demand table)")
    lv.add_argument("--grace", type=float, default=0.0625, help="seconds after the end of step t before G(t) closes")
    lv.add_argument("--port", type=int, default=8080)

    rp = sub.add_parser("replay", help="replay the sequence of G(t) of a DTA file")
    rp.add_argument("file", type=Path)
    rp.add_argument("--port", type=int, default=8080)

    mk = sub.add_parser("mock", help="draw G(t) of a synthetic fleet")
    mk.add_argument("--M", type=int, default=3)
    mk.add_argument("--seed", type=int, default=1)
    mk.add_argument("--dta", type=Path)
    mk.add_argument("--port", type=int, default=8080)

    pm = sub.add_parser("publish-mock", help="publish the topics of a synthetic fleet on a broker")
    pm.add_argument("--broker", default="localhost:1883")
    pm.add_argument("--M", type=int, default=3)
    pm.add_argument("--steps", type=int, default=400)
    pm.add_argument("--seed", type=int, default=1)
    pm.add_argument("--epoch", type=float, default=None, help="Unix time of the start of step 0 (default: the DTP epoch)")
    pm.add_argument("--publish-as", action="store_true", help="also publish fleet/as")

    a = ap.parse_args()
    from . import server
    if a.cmd == "live":
        dtp = DTP.from_dict(json.loads(a.dtp.read_text()))
        host, port = _broker(a.broker)
        epoch = None if a.epoch == "auto" else (float(a.epoch) if a.epoch is not None else dtp.t0)
        asyncio.run(server.run_live(dtp, host, port, a.port, a.dta, epoch, a.grace))
    elif a.cmd == "replay":
        asyncio.run(server.run_replay(a.file, a.port))
    elif a.cmd == "mock":
        asyncio.run(server.run_mock(a.M, a.port, a.seed, a.dta))
    else:
        from ..mock import publish
        host, port = _broker(a.broker)
        publish(host, port, M=a.M, steps=a.steps, seed=a.seed, epoch=a.epoch, publish_as=a.publish_as)


if __name__ == "__main__":
    main()
