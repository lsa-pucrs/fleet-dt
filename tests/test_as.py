import json
import math
import random
import shutil
import socket
import subprocess
import threading
import time
from pathlib import Path

import pytest

from fleetdt.as_model import DTP, admission, grant_limit, smooth
from fleetdt.as_service import AS, Service, decode_throughput
from fleetdt.model import decode_levels, encode_demand, tcp_nodelay

ROOT = Path(__file__).resolve().parents[1]
MOSQUITTO = "/usr/sbin/mosquitto" if Path("/usr/sbin/mosquitto").exists() else shutil.which("mosquitto")


def small_dtp(**kw):
    # T3 / Ts = 0.5, U = 1, C0 = 4000: b = 1000 for m = 2
    base = dict(topics=["a", "b", "c", "d"], Ts=1.0, T1=0.25, T2=0.25, alpha=0.5, U=1.0, C0=4000.0,
                order=["a", "b", "c", "d"], needed=["a", "b"])
    base.update(kw)
    return DTP(**base)


D1, D2 = [100, 600, 400, 200], [100, 500, 200, 250]


def test_algorithm1_example_through_the_as():
    a = AS(small_dtp(), epoch=0.0)
    assert a.on_demand(1, 0, D1, at=0.01)
    assert a.on_demand(2, 0, D2, at=0.02)
    dec = a.close_phase1(0)
    assert dec["M"] == [1, 2] and dec["m"] == 2
    assert dec["C_hat"] == 4000.0 and dec["K_hat"] == 2000.0 and dec["b"] == 1000.0
    assert dec["p"] == [2, 2, 0, 1]


def test_membership_by_deadline():
    a = AS(small_dtp(), epoch=10.0)
    t = 3
    start = 10.0 + 3 * 1.0
    assert a.on_demand(1, t, D1, at=start + 0.2499)
    assert not a.on_demand(2, t, D2, at=start + 0.25)      # at start(t) + T1: too late
    assert not a.on_demand(3, t, D2, at=start + 0.30)
    dec = a.close_phase1(t)
    assert dec["M"] == [1]
    assert not a.on_demand(1, t, D1, at=start + 0.1)       # step already closed


def test_foreign_and_malformed_tables_are_ignored():
    a = AS(small_dtp(), epoch=0.0)
    assert not a.on_demand(1, 0, [1, 2, 3], at=0.01)        # L differs from the DTP
    assert a.on_demand(2, 1, D2, at=0.05)                  # table of step 1 does not enter M(0)
    assert a.close_phase1(0) is None                       # m(0) = 0
    dec = a.close_phase1(1)
    assert dec["M"] == [2]


def test_m_zero_publishes_nothing_but_advances_eq2():
    a = AS(small_dtp(), epoch=0.0)
    a.on_throughput(0, 2000.0, at=0.9)
    assert a.close_phase1(0) is None
    assert a.last["m"] == 0 and a.last["C_hat"] == 4000.0
    a.on_demand(1, 1, D1, at=1.01)
    dec = a.close_phase1(1)
    assert dec["C_hat"] == pytest.approx(smooth(4000.0, 2000.0, 0.5))


def test_missing_or_late_y_keeps_C_hat():
    a = AS(small_dtp(), epoch=0.0)
    for t in range(4):
        a.on_demand(1, t, D1, at=t + 0.01)
    a.close_phase1(0)
    a.on_throughput(0, 1000.0, at=1.0)
    c1 = a.close_phase1(1)["C_hat"]
    assert c1 == pytest.approx(0.5 * 4000 + 0.5 * 1000)
    # y(1) missing
    c2 = a.close_phase1(2)["C_hat"]
    assert c2 == c1
    # y(2) after start(3) + T1
    a.on_throughput(2, 9000.0, at=3.3)
    assert a.close_phase1(3)["C_hat"] == c1


def test_eq2_over_skipped_steps():
    a = AS(small_dtp(alpha=0.25), epoch=0.0)
    a.close_phase1(0)
    for k, y in enumerate([1000.0, 2000.0, 3000.0]):
        a.on_throughput(k, y, at=k + 1.0)
    a.on_demand(1, 3, D1, at=3.01)
    C = 4000.0
    for y in [1000.0, 2000.0, 3000.0]:
        C = smooth(C, y, 0.25)
    assert a.close_phase1(3)["C_hat"] == pytest.approx(C)


def test_order_fallback_and_update():
    a = AS(small_dtp(), epoch=0.0)
    a.on_demand(1, 0, D1, at=0.01)
    a.on_demand(2, 0, D2, at=0.01)
    dec = a.close_phase1(0)
    assert dec["order"] == [1, 2, 3, 4] and dec["needed"] == [1, 2]       # DTP order before fleet/order
    assert a.on_order(["d", "a", "b", "c"], ["d"], at=0.9)
    assert a.on_order(["c", "d", "a", "b"], ["c"], at=1.3)                 # after start(1) + T1
    assert not a.on_order(["a", "b"], [], at=0.95)                         # not a permutation
    assert not a.on_order(["a", "b", "c", "d"], ["x"], at=0.95)
    a.on_demand(1, 1, D1, at=1.01)
    a.on_demand(2, 1, D2, at=1.01)
    dec = a.close_phase1(1)
    assert dec["order"] == [4, 1, 2, 3] and dec["needed"] == [4]
    C = smooth(4000.0, None, 0.5)
    assert dec["p"] == admission({1: D1, 2: D2}, [4, 1, 2, 3], [4], grant_limit(C, 1.0, 0.5, 1.0, 2))
    a.on_demand(1, 2, D1, at=2.01)
    assert a.close_phase1(2)["order"] == [3, 4, 1, 2]


def test_decode_throughput_formats():
    import struct
    assert decode_throughput(b'{"t": 5, "y": 1.5e5, "Tb": 0.01}') == (5, 150000.0)
    assert decode_throughput(struct.pack("!Id", 6, 2.5)) == (6, 2.5)


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _wait(ev: threading.Event, what: str, timeout: float = 10.0) -> None:
    if not ev.wait(timeout):
        raise AssertionError("timeout waiting for %s" % what)


def _client(mqtt, cid, port, subs=None):
    """paho client connected to the test broker. Returns (client, connected, subscribed): the events are
    set by CONNACK and by the SUBACK of `subs`, so the caller synchronizes on the broker itself."""
    connected, subscribed = threading.Event(), threading.Event()
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)
    tcp_nodelay(c)

    def on_connect(cl, u, f, rc, props=None):
        if subs:
            cl.subscribe(subs)
        connected.set()

    c.on_connect = on_connect
    c.on_subscribe = lambda cl, u, mid, rcs, props=None: subscribed.set()
    c.connect_async("127.0.0.1", port)
    c.loop_start()
    return c, connected, subscribed


def _start_broker(conf, port, timeout=10.0):
    """Starts mosquitto and returns when it accepts TCP connections on `port` (no fixed sleep)."""
    broker = subprocess.Popen([MOSQUITTO, "-c", str(conf)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    end = time.monotonic() + timeout
    while True:
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
            return broker
        except OSError:
            if broker.poll() is not None or time.monotonic() > end:
                broker.terminate()
                raise AssertionError("mosquitto did not open port %d" % port)
            time.sleep(0.01)


@pytest.mark.skipif(MOSQUITTO is None, reason="mosquitto not installed")
def test_service_over_mqtt(tmp_path):
    """The AS over a real broker: M(t), eq. (2), Algorithm 1, sigma_t and the phase-2 deadline.

    Synchronization (the cause of the earlier flakiness): the test used fixed sleeps for the broker
    start and never waited for the SUBACK of its own subscriber nor of the AS, and it fixed the epoch
    before the setup ended, so a slow start (full suite, loaded host) shifted the first steps; it also
    asserted the deadline T2 on the arrival time at the test subscriber, a thread of the same Python
    process as the AS, so the scheduling delay of the test client (GIL and CPU contention with other
    processes) counted as AS delay. Now the broker, the subscriber, the publisher and the AS are
    synchronized on CONNACK and SUBACK events, the epoch is set after that, the AS runs exactly `steps`
    steps (Service.run(steps)), the deadline T2 is checked on the publication instant that the AS
    measures (pub_delay of the decision log, Section V-C), and the arrival at the subscriber is only
    required to fall inside step t (after start(t) + T1, before start(t + 1))."""
    import paho.mqtt.client as mqtt
    port = _free_port()
    conf = tmp_path / "m.conf"
    conf.write_text("listener %d 127.0.0.1\nallow_anonymous true\nset_tcp_nodelay true\n" % port)
    broker = _start_broker(conf, port)
    svc = sub = pub = None
    run = None
    try:
        dtp = DTP.load(ROOT / "dtp.json")
        got = []
        lock = threading.Lock()

        def on_msg(c, u, msg):
            with lock:
                got.append((msg.topic, bytes(msg.payload), time.time()))

        sub, sub_conn, sub_ack = _client(mqtt, "test-sub-%d" % port, port, [("boat/+/levels", 1), ("fleet/as", 0)])
        sub.on_message = on_msg
        pub, pub_conn, _ = _client(mqtt, "test-pub-%d" % port, port)
        _wait(sub_conn, "CONNACK of the subscriber")
        _wait(sub_ack, "SUBACK of the subscriber")
        _wait(pub_conn, "CONNACK of the publisher")

        # the AS: Service.connect() returns after the SUBACK of its subscriptions (demand, throughput, order)
        svc = Service(dtp, "127.0.0.1", port, epoch=0.0, log_path=str(tmp_path / "dec.jsonl"))
        svc.connect()
        assert svc.subscribed.is_set()
        epoch = time.time() + 0.3            # every subscription is active: step 0 starts 0.3 s from now
        svc.core.epoch = epoch               # read by run() and the deadlines; no step is open yet
        steps = 20
        run = threading.Thread(target=svc.run, kwargs={"steps": steps}, daemon=True)
        run.start()

        rng = random.Random(7)
        L = dtp.L
        video = {dtp.index(n) for n in dtp.topics if n.startswith("video")}
        valid = {}        # t -> {i: D_i(t)} expected in M(t)
        ys = {}
        order_from = 10
        # sigma with the needed topics first (state, gps), then the rest of the DTP order reversed
        new_needed = ["state", "gps"]
        new_order = (new_needed + [n for n in reversed(dtp.order) if n not in new_needed], new_needed)

        def wait_until(at):
            dt = at - time.time()
            if dt > 0:
                time.sleep(dt)

        for t in range(steps):
            start = epoch + t * dtp.Ts
            wait_until(start + 0.001)
            if t >= 1 and t != 12:                  # y(11) missing: C_hat(12) = C_hat(11)
                ys[t - 1] = rng.uniform(80_000, 400_000)
                pub.publish("fleet/throughput", json.dumps({"t": t - 1, "y": ys[t - 1]}))
            if t == order_from:
                pub.publish("fleet/order", json.dumps({"t": t, "order": new_order[0], "needed": new_order[1]}), qos=1)
            valid[t] = {}
            for i in (1, 2, 3):
                d = [rng.randint(5_000, 20_000) if l in video else rng.randint(100, 3000) for l in range(1, L + 1)]
                if i == 2 and t == 7:
                    pub.publish("boat/b2/demand", encode_demand(t, d[:-1]), qos=1)   # wrong L
                    continue
                if i == 3 and t % 2 == 1:
                    continue                                               # published late below
                pub.publish("boat/b%d/demand" % i, encode_demand(t, d), qos=1)
                valid[t][i] = d
            if t % 2 == 1:
                wait_until(start + dtp.T1 + 0.004)
                pub.publish("boat/b3/demand", encode_demand(t, [1] * L), qos=1)
        run.join(timeout=steps * dtp.Ts + 5.0)
        assert not run.is_alive(), "the AS did not finish its %d steps" % steps
        wait_until(epoch + steps * dtp.Ts + 0.1)
        summary = svc.delay_summary()
        svc.stop()
        svc = None
        sub.loop_stop()
        pub.loop_stop()

        fas = {}
        levels = {}
        for topic, payload, at in got:
            if topic == "fleet/as":
                o = json.loads(payload)
                fas[o["t"]] = (o, at)
            else:
                t, p = decode_levels(payload)
                levels[(t, int(topic.split("/")[1][1:]))] = (p, at)

        lines = [json.loads(l) for l in (tmp_path / "dec.jsonl").read_text().splitlines()]
        assert [r["t"] for r in lines if r.get("published")] == list(range(steps))
        pub_delay = {r["t"]: r["pub_delay"] for r in lines if r.get("published")}

        C = dtp.C0
        sigma = [dtp.index(n) for n in dtp.order]
        N = [dtp.index(n) for n in dtp.needed]
        for t in range(steps):
            if t >= 1:
                C = smooth(C, ys.get(t - 1), dtp.alpha)
            if t == order_from:
                sigma = [dtp.index(n) for n in new_order[0]]
                N = [dtp.index(n) for n in new_order[1]]
            o, _ = fas[t]
            assert o["M"] == sorted(valid[t]), t
            assert o["m"] == len(valid[t])
            assert o["C_hat"] == pytest.approx(C, rel=1e-12), t
            b = grant_limit(C, dtp.U, dtp.T3, dtp.Ts, o["m"])
            assert o["b"] == pytest.approx(b, rel=1e-12)
            p = admission(valid[t], sigma, N, o["b"])
            assert o["p"] == p and len(p) == L, t
            assert o["order"] == sigma and o["needed"] == N, t
            deadline = epoch + t * dtp.Ts + dtp.T1
            assert pub_delay[t] < dtp.T2, (t, pub_delay[t])          # p(t) published within phase 2
            for i in valid[t]:
                pl, at = levels[(t, i)]
                assert pl == p
                assert deadline <= at < deadline - dtp.T1 + dtp.Ts, (t, i, at - deadline)   # inside step t
            assert not any(k == (t, 3) for k in levels) or 3 in valid[t]
        assert len({p_t for p_t in (tuple(fas[t][0]["p"]) for t in range(steps))}) > 1  # the decisions vary
        assert summary["n"] == steps and summary["max"] < dtp.T2
    finally:
        if svc is not None:
            svc.stop()
        for c in (sub, pub):
            if c is not None:
                c.loop_stop()
        broker.terminate()
        broker.wait(timeout=5)


def test_cli_help():
    out = subprocess.run(["python3", "-m", "fleetdt.as_service", "--help"], cwd=ROOT, capture_output=True, text=True)
    assert out.returncode == 0 and "--outer-every" in out.stdout


def test_decision_logs_the_admitted_bytes():
    """C5: the decision holds a_i(t) (demand of vessel i over its topics of level 1 or 2) and A(t) = sum_i a_i;
    the VE carries A from fleet/as into G(t)."""
    from fleetdt.model import Graph
    from fleetdt.ve.collector import Assembler, StepClock
    a = AS(small_dtp(), epoch=0.0)
    a.on_demand(1, 0, D1, at=0.01)
    a.on_demand(2, 0, D2, at=0.02)
    dec = a.close_phase1(0)
    assert dec["p"] == [2, 2, 0, 1]
    assert dec["a"] == {"1": 100 + 600 + 200, "2": 100 + 500 + 250} and dec["A"] == 1750
    assert dec["A"] <= dec["K_hat"]
    b7 = AS(small_dtp(), epoch=0.0, policy="b7")
    b7.on_demand(1, 0, D1, at=0.01)
    b7.on_demand(2, 0, D2, at=0.02)
    d7 = b7.close_phase1(0)
    assert d7["a"] == {str(i): sum(v for v, lev in zip(D, d7["p_by"][i]) if lev > 0)
                       for i, D in ((1, D1), (2, D2))}
    assert d7["A"] == sum(d7["a"].values())
    dtp = small_dtp()
    asm = Assembler(dtp, StepClock(dtp.Ts, 0.0, now=lambda: 0.0))
    asm.on_message("boat/b1/demand", encode_demand(0, D1), at=0.01)
    asm.on_message("fleet/as", json.dumps({k: dec[k] for k in ("t", "C_hat", "K_hat", "b", "U", "alpha", "A")}).encode(),
                   at=0.3)
    g = asm._finish(0)
    assert g.A == 1750 and Graph.from_dict(g.to_dict()).A == 1750
