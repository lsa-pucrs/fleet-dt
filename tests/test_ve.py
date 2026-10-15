import json
import shutil
import socket
import subprocess
import threading
import time

import pytest

from fleetdt.as_model import DTP, admission, grant_limit, smooth
from fleetdt.ve.collector import Assembler, StepClock
from fleetdt.mock import FleetSim
from fleetdt.model import (Graph, decode_demand, decode_levels, decode_step_header, encode_demand,
                              encode_levels, encode_step_header)


def test_wire_formats_roundtrip():
    assert decode_demand(encode_demand(7, [1, 2, 300000])) == (7, [1, 2, 300000])
    assert decode_levels(encode_levels(9, [2, 1, 0])) == (9, [2, 1, 0])
    assert decode_step_header(encode_step_header(123456, 65535) + b"xyz") == (123456, 65535)
    with pytest.raises(ValueError):
        decode_demand(encode_demand(1, [1, 2])[:-1])
    with pytest.raises(ValueError):
        decode_levels(encode_levels(1, [3]))


def test_algorithm1_example_of_the_paper():
    D = {1: [100, 600, 400, 200], 2: [100, 500, 200, 250]}
    assert admission(D, [1, 2, 3, 4], [1, 2], 1000) == [2, 2, 0, 1]


def test_smoothing_and_grant_limit():
    assert smooth(100.0, None, 0.5) == 100.0
    assert smooth(100.0, 200.0, 0.25) == pytest.approx(125.0)
    assert grant_limit(1000.0, 0.8, 0.095, 0.125, 0) is None
    assert grant_limit(1000.0, 0.8, 0.095, 0.125, 2) == pytest.approx(0.8 * 1000 * 0.095 / 0.125 / 2)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def test_assembler_builds_graph():
    dtp = DTP(topics=["state", "video"], Ts=0.125, T1=0.02, T2=0.01, alpha=0.5, U=0.8, C0=1000.0)
    fc = FakeClock()
    asm = Assembler(dtp, StepClock(dtp.Ts, 0.0, now=fc))
    t = 10
    start = t * dtp.Ts
    fc.now = start + 0.005
    asm.on_message("boat/b1/demand", encode_demand(t, [64, 5000]), at=fc.now)
    asm.on_message("boat/b2/demand", encode_demand(t, [64, 5000]), at=start + 0.03)  # after phase 1
    asm.on_message("fleet/order", json.dumps({"t": t, "order": ["state", "video"], "needed": ["state"]}).encode())
    asm.on_message("boat/b1/levels", encode_levels(t, [2, 1]), at=start + 0.025)
    asm.on_message("boat/b1/tx/state", encode_step_header(t, 0) + bytes(58), at=start + 0.04)
    asm.on_message("boat/b1/tx/state", encode_step_header(t, 0) + bytes(58), at=start + 0.05)  # repeated
    asm.on_message("boat/b1/tx/video", encode_step_header(t, 0) + bytes(994), at=start + 0.06)
    asm.on_message("boat/b1/tx/video", encode_step_header(t, 1) + bytes(994), at=start + 0.13)  # late
    asm.on_message("fleet/throughput", json.dumps({"t": t, "y": 900.0}).encode())
    fc.now = start + 0.2
    gs = asm.close_ready(grace=0.05)
    assert [g.t for g in gs] == [t]
    g = gs[0]
    assert g.vessels == [1]
    e_state, e_video = g.edge(1, 1), g.edge(1, 2)
    assert (e_state.p, e_state.n, e_state.dup, e_state.x) == (2, 1, 1, 58)
    assert (e_video.p, e_video.n, e_video.late, e_video.x) == (1, 1, 1, 994)
    assert g.y == 900.0 and g.C_hat == 1000.0
    assert g.b == pytest.approx(0.8 * 1000 * (0.125 - 0.03) / 0.125 / 1)
    assert g.needed == [1] and g.order == [1, 2]
    # next step uses y(t) in eq. (2)
    t2 = t + 1
    asm.on_message("boat/b1/demand", encode_demand(t2, [64, 5000]), at=t2 * dtp.Ts + 0.001)
    fc.now = (t2 + 1) * dtp.Ts + 0.1
    g2 = asm.close_ready(grace=0.05)[0]
    assert g2.C_hat == pytest.approx(0.5 * 1000 + 0.5 * 900)


def test_graph_dict_roundtrip():
    g = FleetSim(M=2, seed=3).step(0).graph
    h = Graph.from_dict(json.loads(json.dumps(g.to_dict())))
    assert h.to_dict() == g.to_dict()


def test_simulator_meets_proposition_2():
    sim = FleetSim(M=3, seed=5)
    checked = 0
    for t in range(600):
        g = sim.step(t).graph
        if t > 0 and sim.C_last >= g.U * g.C_hat:
            checked += 1
            for e in g.edges:
                if e.i in g.vessels and e.p:
                    assert e.r >= e.d - 1, (t, e)
    assert checked > 100


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


@pytest.mark.skipif(shutil.which("mosquitto") is None, reason="mosquitto not installed")
def test_end_to_end_over_mqtt(tmp_path):
    from fleetdt.ve.collector import MqttCollector
    from fleetdt.mock import publish
    port = _free_port()
    conf = tmp_path / "m.conf"
    conf.write_text("listener %d 127.0.0.1\nallow_anonymous true\nset_tcp_nodelay true\n" % port)
    broker = subprocess.Popen(["mosquitto", "-c", str(conf)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(0.5)
        sim = FleetSim(M=3, seed=11)
        asm = Assembler(sim.dtp, StepClock(sim.dtp.Ts))
        got = []
        col = MqttCollector(asm, got.append, host="127.0.0.1", port=port, grace=0.06)
        col.start()
        time.sleep(0.5)
        steps = 40
        th = threading.Thread(target=publish, kwargs=dict(host="127.0.0.1", port=port, M=3, steps=steps, seed=11))
        th.start()
        th.join()
        time.sleep(0.5)
        col.stop()
        sim_ref = FleetSim(M=3, seed=11)
        ref = [sim_ref.step(k).graph for k in range(steps)]
        got = [g for g in got if g.vessels]
        assert len(got) >= steps - 2
        def dvec(g):
            return [(e.i, e.l, e.d) for e in sorted(g.edges, key=lambda e: (e.i, e.l)) if e.i in g.vessels]
        k0 = next(k for k in range(5) if dvec(ref[k]) == dvec(got[0]))  # first step that the collector closed
        t0 = got[0].t - k0
        match = 0
        for g in got:
            k = g.t - t0
            if not 0 <= k < steps:
                continue
            r = ref[k]
            assert g.vessels == r.vessels
            for e in r.edges:
                if e.i not in r.vessels:
                    continue
                ge = g.edge(e.i, e.l)
                assert (ge.d, ge.p) == (e.d, e.p)
                assert ge.n == e.n and ge.x == e.x
            match += 1
        assert match >= steps - 2
    finally:
        broker.terminate()
