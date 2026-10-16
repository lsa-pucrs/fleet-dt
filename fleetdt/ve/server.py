"""Web server of the VE: serves the page and streams G(t) over a WebSocket.

Modes:
  live   : assembles G(t) from the GS MQTT broker (MqttCollector) and appends each G(t) to the DTA file
  replay : serves the sequence of G(t) of a DTA file (JSON lines)
  mock   : generates G(t) with the synthetic fleet, one graph per step
"""
from __future__ import annotations

import asyncio
import json
import time
from collections import deque
from pathlib import Path
from typing import Deque, Optional, Set

from aiohttp import WSMsgType, web

from ..model import Graph

STATIC = Path(__file__).parent / "static"


class Hub:
    def __init__(self, history: int = 4000, dta_path: Optional[Path] = None, meta: Optional[dict] = None):
        self.history: Deque[dict] = deque(maxlen=history)
        self.clients: Set[web.WebSocketResponse] = set()
        self.dta = open(dta_path, "a", encoding="utf-8") if dta_path else None
        self.meta = meta or {}

    async def publish(self, g: Graph) -> None:
        d = g.to_dict()
        d.setdefault("emitted", time.time())   # GS time at which G(t) left the VE (lag of the DTI)
        self.history.append(d)
        if self.dta:
            self.dta.write(json.dumps(d, separators=(",", ":")) + "\n")
            self.dta.flush()
        msg = json.dumps({"type": "graph", "g": d}, separators=(",", ":"))
        for ws in list(self.clients):
            try:
                await ws.send_str(msg)
            except (ConnectionResetError, RuntimeError):
                self.clients.discard(ws)


def make_app(hub: Hub, replay: Optional[list] = None) -> web.Application:
    app = web.Application()

    async def index(_):
        return web.FileResponse(STATIC / "index.html")

    async def meta(_):
        return web.json_response({**hub.meta, "mode": "replay" if replay is not None else hub.meta.get("mode")})

    async def dta(request):
        data = replay if replay is not None else list(hub.history)
        lo = int(request.query.get("from", 0))
        hi = int(request.query.get("to", len(data)))
        return web.json_response(data[lo:hi])

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        hub.clients.add(ws)
        try:
            async for m in ws:
                if m.type == WSMsgType.ERROR:
                    break
        finally:
            hub.clients.discard(ws)
        return ws

    app.router.add_get("/", index)
    app.router.add_get("/api/meta", meta)
    app.router.add_get("/api/dta", dta)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/static", STATIC)
    return app


def load_dta(path: Path) -> list:
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


async def run_live(dtp, host, port, http_port, dta_path, epoch, grace):
    from .collector import Assembler, MqttCollector, StepClock
    hub = Hub(dta_path=dta_path, meta={"mode": "live", "topics": dtp.topics, "Ts": dtp.Ts, "U": dtp.U})
    loop = asyncio.get_running_loop()
    asm = Assembler(dtp, StepClock(dtp.Ts, epoch))
    col = MqttCollector(asm, lambda g: asyncio.run_coroutine_threadsafe(hub.publish(g), loop),
                        host=host, port=port, grace=grace)
    col.start()
    await _serve(make_app(hub), http_port)


async def run_replay(path: Path, http_port: int):
    data = load_dta(path)
    topics = data[0]["topics"] if data else []
    hub = Hub(meta={"mode": "replay", "topics": topics, "file": str(path)})
    await _serve(make_app(hub, replay=data), http_port)


async def run_mock(M: int, http_port: int, seed: int, dta_path: Optional[Path]):
    from ..mock import FleetSim
    sim = FleetSim(M=M, seed=seed)
    hub = Hub(dta_path=dta_path, meta={"mode": "mock", "topics": sim.dtp.topics, "Ts": sim.dtp.Ts, "U": sim.dtp.U})

    async def gen():
        t, nxt = 0, time.monotonic()
        while True:
            await hub.publish(sim.step(t).graph)
            t += 1
            nxt += sim.dtp.Ts
            await asyncio.sleep(max(0.0, nxt - time.monotonic()))

    asyncio.create_task(gen())
    await _serve(make_app(hub), http_port)


async def _serve(app: web.Application, http_port: int):
    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, "0.0.0.0", http_port).start()
    print("VE at http://localhost:%d" % http_port, flush=True)
    await asyncio.Event().wait()
