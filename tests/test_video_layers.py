"""Video in 16 spatial stripe layer topics (L = 22): bit-reversal layering of the stripes, the share profile,
the DTP files, the per-vessel demands, the AS with L = 22, the publishers, the DTI program with a running
stripe source, and per cell (rho in {1.5, 2}, M in {2, 4}) the two validity ratios of the experiments:
the fill ratio f = max_i a_i / b of Algorithm 1 (mean f >= 0.85) and the ratio of the forwarded bytes of
each layer to its declared demand d_il (mean >= 0.8), the second from the stripes that fdt_source.py
replays from the trace and the policing of the regulator."""
import json
import math
import os
import random
import socket
import subprocess
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

import numpy as np
import pytest

from fleetdt.as_model import DTP, admission, grant_limit
from fleetdt.as_service import AS
from fleetdt.model import decode_demand, decode_levels, encode_demand, encode_levels

ROOT = Path(__file__).resolve().parents[1]
PUB = ROOT / "vessel" / "publishers"
sys.path.insert(0, str(PUB))

import video_layers as vl  # noqa: E402

LAYERS = ["video_%d" % j for j in range(1, 17)]
TELEMETRY = ["state", "imu", "gps", "baro", "power", "mag"]
FILL_MIN = 0.85
USE_MIN = 0.8
TRACE = ROOT / "emu" / "traces" / "water-sbs-q90-s1.txt"
C_STEP = 320_000.0          # bytes per step of the channel of mock.py
J_DTI = 0.002               # --jitter-ms of fdt_dti (plugin_opt_jitter_ms of the regulator)
STRIPES_OF_LAYER = [0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15]


def raw_dtp(name="dtp.json"):
    return json.loads((ROOT / name).read_text())


def trace_sizes():
    return [int(float(l.split()[0])) for l in TRACE.read_text().splitlines() if l.strip() and not l.startswith("#")]


# --------------------------------------------------------------------------- layering


def test_bit_reversal_of_the_stripes():
    assert [vl.stripe_of_layer(j) for j in range(1, 17)] == STRIPES_OF_LAYER
    for s in range(16):
        assert vl.stripe_of_layer(vl.layer_of_stripe(s)) == s
    with pytest.raises(ValueError):
        vl.stripe_of_layer(3, layers=12)
    with pytest.raises(ValueError):
        vl.stripe_of_layer(17)


@pytest.mark.parametrize("j", range(1, 17))
def test_prefix_of_layers_spreads_the_stripes_over_the_image(j):
    st = vl.admitted_stripes(j)
    assert len(st) == j                                     # j of the 16 stripes of every frame
    jp = 1 << int(math.floor(math.log2(j)))                 # largest power of two <= j
    gaps = np.diff(st + [16])                               # from each admitted stripe to the next (or the bottom)
    assert st[0] == 0 and gaps.max() <= 16 // jp
    if j == jp:
        assert set(gaps.tolist()) == {16 // j}              # one stripe out of every 16 / j


def test_stripe_rows():
    for h in (376, 188, 720):
        rows = vl.stripe_rows(h)
        assert rows[0][0] == 0 and rows[-1][1] == h
        assert all(a[1] == b[0] for a, b in zip(rows, rows[1:]))
        assert max(r1 - r0 for r0, r1 in rows) - min(r1 - r0 for r0, r1 in rows) <= 1
    img = np.arange(376)
    assert [len(x) for x in np.array_split(img, 16)] == [r1 - r0 for r0, r1 in vl.stripe_rows(376)]


def test_share_profile_of_the_traces():
    """emu/traces/stripe-shares.json: real stripe encodes of the water sequence; the default profile of
    video_layers.py is the q90-s1 profile of the file"""
    doc = json.loads((ROOT / "emu" / "traces" / "stripe-shares.json").read_text())
    assert len(doc["pairs"]) == 24 and doc["layers"] == 16
    for key, p in doc["profiles"].items():
        w = p["shares"]
        assert len(w) == 16 and all(x > 0 for x in w)
        assert abs(sum(w) - p["sum"]) < 1e-3 and p["sum"] > 1.0      # every stripe carries its own headers
    assert vl.DEFAULT_SHARES == doc["profiles"]["q90-s1"]["shares"]
    assert vl.load_profile("q50-s0p5") == doc["profiles"]["q50-s0p5"]["shares"]
    # the whole-frame sizes of the 24 pairs match the trace they come from
    sizes = trace_sizes()
    assert abs(doc["profiles"]["q90-s1"]["whole_mean"] / np.mean(sizes) - 1) < 0.02


def test_msg_split():
    for cap in (1, 500, 1392, 1393, 4000, 11000):
        m, b = vl.msg_split(cap)
        assert b <= 1400 and m * (b - 8) >= cap and (m - 1) * (1400 - 8) < cap
        sizes = vl.split_sizes(cap, b)
        assert len(sizes) == m and all(s <= b for s in sizes) and sum(sizes) == cap + 8 * m


# --------------------------------------------------------------------------- DTP files


def test_root_dtp_has_22_topics_in_sigma_order():
    d = raw_dtp()
    dtp = DTP.from_dict(d)
    assert dtp.L == 22
    assert dtp.topics == TELEMETRY + LAYERS
    assert dtp.order == TELEMETRY + LAYERS                 # state first, telemetry, then video_1..video_16
    assert dtp.order[0] == "state" and dtp.needed == TELEMETRY
    c = vl.config(d)
    assert c["layers"] == 16 and vl.layer_topics(d) == LAYERS and c["profile"] == "q90-s1"
    vl.check_layers(d)                                     # rates and bounds follow video_layers
    F = vl.frames_per_step(c["fps"], d["Ts"])
    assert F == c["fps"] * d["Ts"] == 2                    # an integer number of frames per step
    for j, name in enumerate(LAYERS, start=1):
        t = [x for x in d["topics"] if x["name"] == name][0]
        cap = vl.layer_cap(c, j)
        m = vl.msgs_per_stripe(d, name)
        assert t["jitter"] == 0 and t["bound"] <= 1400
        assert vl.msgs_per_step(t["rate"], d["Ts"]) == m * F
        assert m * (t["bound"] - 8) >= cap > (m - 1) * (1400 - 8)
        assert vl.stripe_cap(d, name) == m * (t["bound"] - 8)
    # the frame cap is the largest frame of the trace of the profile
    assert c["frame_cap"] == max(trace_sizes())


def test_publisher_dtp_reader_accepts_the_layers():
    from common import DTPFile
    d = DTPFile.load(str(ROOT / "dtp.json"))
    raw = raw_dtp()
    assert list(d.topics) == TELEMETRY + LAYERS
    assert d.per_step("video_7") == 2 * vl.msgs_per_stripe(raw, "video_7") and d.per_step("state") == 1


def test_dtp_v2_layers():
    d = raw_dtp("dtp_v2.json")
    names = [t["name"] for t in d["topics"]]
    assert names == TELEMETRY + LAYERS and d["order"] == TELEMETRY + LAYERS
    c = vl.config(d)
    for t in d["topics"][6:]:
        assert t["rate"] == c["fps"] and {k: t[k] for k in ("rate", "n", "s")} == vl.layer_topic_v2(c, t["layer"])
    import fdt_source
    tps = fdt_source.load_dtp(str(ROOT / "dtp_v2.json"))["topics"]
    lay = [t for t in tps if t.layer is not None]
    assert [t.layer for t in lay] == list(range(1, 17))
    assert [t.stripe for t in lay] == STRIPES_OF_LAYER
    assert all(abs(t.rate - 16.0) < 1e-9 for t in lay)      # one stripe per frame
    assert [t.share for t in lay] == [c["shares"][s] for s in STRIPES_OF_LAYER]


def test_resolved_vessel_dtp_differs_only_in_layer_topics():
    d = raw_dtp()
    per = {i: vl.resolve_vessel(d, i) for i in (1, 2, 3, 4)}
    c = vl.config(d)
    for i, r in per.items():
        assert [t["name"] for t in r["topics"]] == [t["name"] for t in d["topics"]]
        assert r["order"] == d["order"] and r["needed"] == d["needed"]
        s = vl.vessel_config(d, i)["size_scale"]
        for j, (t, t0) in enumerate(zip(r["topics"], d["topics"]), start=-5):
            if t["name"] in LAYERS:
                assert t == vl.layer_topic(c, j, d["Ts"], s, 1.0)
                assert vl.stripe_cap(r, t["name"]) >= math.ceil(s * c["frame_cap"] * c["shares"][STRIPES_OF_LAYER[j - 1]] - 1e-9)
            else:
                assert t == t0
    dem = {i: sum(vl.channel_demand(per[i], n) for n in LAYERS) for i in per}
    assert len(set(dem.values())) == 4                     # the demand tables differ between vessels
    assert vl.channel_demand(per[1], "state") == vl.channel_demand(per[4], "state")


# --------------------------------------------------------------------------- AS and wire formats with L = 22


def test_as_and_wire_formats_with_22_topics():
    dtp = DTP.load(ROOT / "dtp.json")
    d = [100] * 6 + [10_000] * 16
    assert decode_demand(encode_demand(5, d)) == (5, d)
    a = AS(dtp, epoch=0.0)
    assert a.sigma == list(range(1, 23))
    assert a.on_demand(1, 0, d, at=0.001)
    assert not a.on_demand(2, 0, d[:7], at=0.001)          # a 7-topic table no longer fits the DTP
    dec = a.close_phase1(0)
    assert len(dec["p"]) == 22
    assert dec["p"] == admission({1: d}, a.sigma, a.N, dec["b"])
    assert dec["D"] == {"1": d}                            # the AS log holds D_i(t) for the conformance replay
    t, p = decode_levels(encode_levels(0, dec["p"]))
    assert p == dec["p"]
    admitted = [l for l in range(7, 23) if p[l - 1] > 0]
    assert admitted == list(range(7, 7 + len(admitted)))   # a prefix of the layers
    assert all(p[l - 1] == 2 for l in range(1, 7))         # the six telemetry topics are needed


# --------------------------------------------------------------------------- cells: fill ratio and use of the demand


def base_dtp():
    """dtp.json as the base of the cells (vessels 1..4 of its "video_layers")"""
    return raw_dtp()


def demands(d, M):
    names = [t["name"] for t in d["topics"]]
    return {i: [vl.channel_demand(vl.resolve_vessel(d, i), n, jitter_s=J_DTI) for n in names] for i in range(1, M + 1)}


def fill_ratio(D, sigma, N, b):
    p = admission(D, sigma, N, b)
    a = [sum(D[i][l - 1] for l in range(1, len(p) + 1) if p[l - 1] > 0) for i in D]
    return max(a) / b, p


def forwarded_over_demand(d, vessel, scale, steps=600, seed=1001, delay_ms=3.0, tmpdir=None):
    """Mean over the steps of (channel bytes that the regulator forwards) / d_il(t) for each layer topic of
    vessel i: the stripes of fdt_source.Source (trace replay, seeded phase of mode --t0, sizes of the share
    profile times the size factor of the vessel and the scale of the cell), each stripe reaching the
    regulator U(0, delay_ms) after its frame instant, collected in [v(t-1), v(t)) and policed in arrival
    order to d_il(t) with the cost c_b (x + 6) + c_m of the regulator. Returns ({layer: ratio}, held)."""
    import fdt_source
    r = vl.resolve_vessel(d, vessel)
    r["epoch"] = 0.0
    fd, path = tempfile.mkstemp(suffix=".json", dir=tmpdir)
    with os.fdopen(fd, "w") as f:
        json.dump(r, f)
    try:
        dtp = fdt_source.load_dtp(path, J_DTI)
    finally:
        os.unlink(path)
    vc = vl.vessel_config(r, vessel)
    a = types.SimpleNamespace(seed=seed, vessel=vessel, t0=0.0, size_trace=str(TRACE), median=None, sigma=0.5,
                              load=scale, load_mode="size", phase="random", stream_size="max", topics=None, log=None,
                              video_scale=vc["size_scale"], fps_scale=vc["fps_scale"])
    src = fdt_source.Source(a, dtp, random.Random(1))
    Ts, v0 = r["Ts"], r["T1"] + r["T2"]
    lay = [tp for tp in dtp["topics"] if tp.layer is not None]
    dem = {tp.name: vl.channel_demand(r, tp.name, jitter_s=J_DTI) for tp in lay}
    acc = {tp.name: [0.0] * steps for tp in lay}
    rng = random.Random(seed)
    held = 0
    period = 1.0 / src.rate(lay[0])
    ph = src.phase(lay[0], period)
    assert all(src.phase(tp, period) == ph for tp in lay)  # one phase for the stripes of a frame
    k = 0
    while True:
        at0 = ph + k * period
        if at0 >= steps * Ts:
            break
        for tp in lay:
            at = at0 + rng.uniform(0, delay_ms * 1e-3)
            t = int(math.floor((at - v0) / Ts)) + 1
            msgs, size, trunc, oid = src.messages(tp, k, 0)
            assert oid == k and not trunc
            assert size == max(1, int(round(src.frame_size(tp, k, 0) * tp.share)))
            if not 0 <= t < steps:
                continue
            for m in msgs:
                c = vl.message_cost(r, len(m))
                if acc[tp.name][t] + c <= dem[tp.name] + 1e-9:
                    acc[tp.name][t] += c
                else:
                    held += 1
        k += 1
    return {n: sum(v[1:]) / ((steps - 1) * dem[n]) for n, v in acc.items()}, held


CELLS = [(2, 1.5), (2, 2.0), (4, 1.5), (4, 2.0)]


@pytest.mark.parametrize("M,rho", CELLS)
def test_cell_fill_and_use_of_the_demand(M, rho, capsys, tmp_path):
    """Per cell: video_layers.plan_for_rho picks the frame rate (8, 16, 24 or 32 frames per second: an integer
    number of frames per step) and the frame cap (scale of the trace) for which the demand tables of the M
    vessels sum to rho C; the fill ratio of Algorithm 1 (static tables of the DTI program, and tables with a
    per-step lognormal variation of the layer demand, sigma 0.2) stays >= 0.85 and the forwarded bytes of
    every layer of every vessel stay >= 0.8 of the declared demand."""
    d0 = base_dtp()
    plan = vl.plan_for_rho(d0, M, rho, C_STEP, max(trace_sizes()), jitter_s=J_DTI)
    d = plan["dtp"]
    assert abs(plan["rho"] - rho) < 0.01 and 0 < plan["scale"] <= 1.0
    assert vl.frames_per_step(plan["fps"], d["Ts"]) == plan["fps"] * d["Ts"]
    dtp = DTP.from_dict(d)
    sigma = [dtp.index(n) for n in dtp.order]
    N = [dtp.index(n) for n in dtp.needed]
    D = demands(d, M)
    b = grant_limit(C_STEP, dtp.U, dtp.T3, dtp.Ts, M)
    f, p = fill_ratio(D, sigma, N, b)
    n_layers = sum(1 for l in range(7, 23) if p[l - 1] > 0)
    rng = random.Random(11)
    fv = []
    for _ in range(300):
        Dv = {i: [v if l < 6 else int(v * rng.lognormvariate(0.0, 0.2)) for l, v in enumerate(row)] for i, row in D.items()}
        fv.append(fill_ratio(Dv, sigma, N, b)[0])
    use = {}
    for i in range(1, M + 1):
        ratios, held = forwarded_over_demand(d, i, plan["scale"], tmpdir=str(tmp_path))
        assert held == 0                                   # no stripe exceeds the demand of its layer
        use[i] = ratios
    lo = min(min(r.values()) for r in use.values())
    mean = float(np.mean([v for r in use.values() for v in r.values()]))
    with capsys.disabled():
        print("\ncell M=%d rho=%.1f: fps %g, frame cap %d (scale %.3f of the trace), rho %.3f, b %.0f, %d layers "
              "admitted; fill f = %.3f, per-step variation mean f = %.3f (min %.3f); forwarded / declared per "
              "layer: mean %.3f, min %.3f" % (M, rho, plan["fps"], plan["frame_cap"], plan["scale"], plan["rho"], b,
                                               n_layers, f, np.mean(fv), min(fv), mean, lo))
    assert FILL_MIN <= f <= 0.99
    assert np.mean(fv) >= FILL_MIN
    assert 1 <= n_layers < 16
    assert lo >= USE_MIN


def test_dtp_json_default_cell_use():
    """dtp.json itself (16 frames per second, frame cap of the q90-s1 trace): every layer forwards >= 0.8 of
    its demand with the default source (scale 1)"""
    ratios, held = forwarded_over_demand(base_dtp(), 1, 1.0, steps=300)
    assert held == 0 and min(ratios.values()) >= USE_MIN


def test_camera_rate_of_15_fps_needs_two_frames_per_step():
    """With the camera rate of 15 frames per second a step holds one or two frames (mean 1.875), so the
    declared two frames per step cap the use at 0.9375 of the byte ratio; plan_for_rho therefore keeps an
    integer number of frames per step."""
    assert vl.frames_per_step(15, 0.125) == 2 and 15 * 0.125 / 2 == 0.9375
    assert vl.frames_per_step(16, 0.125) == 2 and vl.frames_per_step(24, 0.125) == 3


# --------------------------------------------------------------------------- publishers


def test_zed_publisher_stripes():
    cv2 = pytest.importorskip("cv2")
    import zed_publisher as zp
    rng = np.random.default_rng(0)
    img = rng.integers(0, 255, size=(376, 1344, 3), dtype=np.uint8)
    st = zp.stripe_images(img, 16)
    assert [s.shape[0] for s in st] == [r1 - r0 for r0, r1 in vl.stripe_rows(376)]
    assert np.array_equal(np.vstack(st), img)
    d = vl.resolve_vessel(raw_dtp(), 2)
    caps = {n: vl.stripe_cap(d, n) for n in LAYERS}
    parts = zp.encode_stripes(img, LAYERS, caps, 50)
    assert [n for n, _ in parts] == LAYERS
    for name, (jpeg, q, h) in parts:
        assert len(jpeg) <= caps[name]
        dec = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        s = STRIPES_OF_LAYER[LAYERS.index(name)]
        r0, r1 = vl.stripe_rows(376)[s]
        hh, ww = r1 - r0, 1344
        for _ in range(h):
            hh, ww = hh // 2, ww // 2
        assert dec.shape[:2] == (hh, ww)
        bound = [t for t in d["topics"] if t["name"] == name][0]["bound"]
        msgs = zp.split_frame(jpeg, 7, bound)
        assert len(msgs) <= vl.msgs_per_stripe(d, name) and all(len(m) <= bound for m in msgs)
    s = zp.FrameRateScale(0.5)
    assert [s.keep() for _ in range(6)] == [False, True, False, True, False, True]
    flat = np.zeros((376, 1344, 3), np.uint8)
    assert all(q == 50 and h == 0 for _, (j, q, h) in zp.encode_stripes(flat, LAYERS, caps, 50))


class _FakeClient:
    def __init__(self):
        self.out = []

    def publish(self, topic, body, qos=0):
        self.out.append((topic, body))


def test_fdt_source_publishes_the_stripes_of_each_frame(tmp_path):
    import fdt_source
    d = vl.with_video(raw_dtp(), fps=32.0, frame_cap=20000)     # short test: 32 frames per second, small stripes
    d = vl.resolve_vessel(d, 2)
    p = tmp_path / "dtp_b2.json"
    p.write_text(json.dumps(d))
    log = tmp_path / "src.jsonl"
    a = types.SimpleNamespace(dtp=str(p), vessel=2, topics=set(LAYERS), stream_size="max", median=None, sigma=0.5,
                              size_trace=str(TRACE), load=0.2, load_mode="size", phase="zero", duration=None, seed=3,
                              t0=None, log=str(log), video_scale=None, fps_scale=None)
    dtp = fdt_source.load_dtp(a.dtp)
    a.video_scale, a.fps_scale = fdt_source.vessel_video(dtp, a.vessel)
    assert (a.video_scale, a.fps_scale) == (0.9, 1.0)
    src = fdt_source.Source(a, dtp, random.Random(1))
    c = _FakeClient()
    stop = threading.Event()
    src.run(c, stop, time.time() + 1.0)
    recs = [json.loads(l) for l in log.read_text().splitlines()]
    frames = {}
    for r in recs:
        frames.setdefault(r["out"], []).append(r)
    ks = sorted(frames)
    assert len(ks) > 20 and ks == list(range(ks[0], ks[0] + len(ks)))
    shares = vl.config(d)["shares"]
    tps = {t.name: t for t in dtp["topics"]}
    for k in ks[:-1]:
        fr = frames[k]
        assert sorted(r["topic"] for r in fr) == sorted(LAYERS)          # one stripe on every layer topic
        assert max(r["t_us"] for r in fr) - min(r["t_us"] for r in fr) < 20000
        F = src.frame_size(tps["video_1"], k, 0)
        for r in fr:
            s = STRIPES_OF_LAYER[LAYERS.index(r["topic"])]
            assert r["bytes"] == max(1, int(round(F * shares[s]))) and not r["trunc"]
    t = np.array([min(r["t_us"] for r in frames[k]) for k in ks]) / 1e6
    assert np.allclose(np.diff(t), 1 / 32.0, atol=0.008)
    # the message counts follow the bound of each layer
    for topic, body in c.out[:200]:
        name = topic.rsplit("/", 1)[1]
        assert len(body) <= tps[name].s


def test_group_phase_keeps_the_frames_off_the_collection_boundaries():
    import fdt_source
    d = raw_dtp()
    d["epoch"] = 0.0
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump(d, f)
    try:
        dtp = fdt_source.load_dtp(f.name)
    finally:
        os.unlink(f.name)
    tp = [t for t in dtp["topics"] if t.name == "video_1"][0]
    period = 1.0 / tp.rate
    for seed in range(50):
        a = types.SimpleNamespace(seed=seed, vessel=1, phase="random", t0=0.0, size_trace=None, log=None)
        ph = fdt_source.Source(a, dtp, random.Random(1)).phase(tp, period)
        off = (ph - (d["T1"] + d["T2"])) % period          # after the boundary v(t) = start(t) + T1 + T2
        assert 0.25 * period - 1e-9 <= off <= 0.75 * period + 1e-9


# --------------------------------------------------------------------------- DTI program with a running stripe source

MOSQ = Path(os.environ.get("MOSQ", "/opt/mosquitto-2.1.2-fleetdt"))
BUILD = Path(os.environ.get("FDT_BUILD", ROOT / "build"))


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


@pytest.mark.skipif(not ((MOSQ / "sbin" / "mosquitto").exists() and (BUILD / "fdt_dti").exists()),
                    reason="needs Mosquitto (MOSQ) and FDT_BUILD/fdt_dti (make)")
def test_dti_running_stripe_source_never_gets_zero_demand(tmp_path):
    """fdt_dti --idle 8 with the stripe source: every layer topic has messages in every step while the source
    runs, so no step after the first message of a topic gets d_l = 0; 8 steps after the last message the
    demand of the topic drops to 0 (the idle rule)."""
    import paho.mqtt.client as mqtt
    d = vl.with_video(raw_dtp(), fps=16.0, frame_cap=20000)
    d["epoch"] = float(math.floor(time.time()) - 30)
    d = vl.resolve_vessel(d, 1)
    p = tmp_path / "dtp_b1.json"
    p.write_text(json.dumps(d))
    port = _free_port()
    (tmp_path / "m.conf").write_text("listener %d 127.0.0.1\nallow_anonymous true\nset_tcp_nodelay true\n"
                                     "max_inflight_messages 0\nmax_queued_messages 0\n" % port)
    procs = [subprocess.Popen([str(MOSQ / "sbin" / "mosquitto"), "-c", str(tmp_path / "m.conf")],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)]
    tables = {}
    try:
        end = time.time() + 10
        while time.time() < end:
            try:
                socket.create_connection(("127.0.0.1", port), 0.2).close()
                break
            except OSError:
                time.sleep(0.05)
        procs.append(subprocess.Popen([str(BUILD / "fdt_dti"), "--dtp", str(p), "--vessel", "1", "--broker",
                                       "127.0.0.1:%d" % port, "--idle", "8", "--jitter-ms", "2", "--prio", "0"],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="dti-test", clean_session=True)
        c.on_connect = lambda cl, u, f, rc, pr=None: cl.subscribe("boat/b1/demand", 1)
        c.on_message = lambda cl, u, m: tables.__setitem__(*decode_demand(m.payload))
        c.connect("127.0.0.1", port)
        c.loop_start()
        time.sleep(0.6)
        log = tmp_path / "src.jsonl"
        r = subprocess.run([sys.executable, str(PUB / "fdt_source.py"), "--dtp", str(p), "--vessel", "1", "--port",
                            str(port), "--t0", "%.6f" % d["epoch"], "--seed", "5", "--duration", "3.0",
                            "--size-trace", str(TRACE), "--load", "0.2", "--log", str(log), "--jitter-ms", "2"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=30)
        assert r.returncode == 0, r.stdout
        time.sleep(0.125 * 12)
        c.loop_stop()
        c.disconnect()
    finally:
        for q in reversed(procs):
            q.terminate()
            try:
                q.wait(5)
            except subprocess.TimeoutExpired:
                q.kill()
    names = [t["name"] for t in d["topics"]]
    Ts, v0, ep = d["Ts"], d["T1"] + d["T2"], d["epoch"]
    steps = {}
    for rec in (json.loads(l) for l in log.read_text().splitlines()):
        t = int(math.floor((rec["t_us"] / 1e6 - ep - v0) / Ts)) + 1
        steps.setdefault(rec["topic"], set()).add(t)
    assert tables
    for name in LAYERS + ["imu", "mag"]:
        l = names.index(name)
        st = sorted(steps[name])
        first, last = st[0], st[-1]
        assert st == list(range(first, last + 1)), name     # stripes (and telemetry) in every step
        running = [t for t in range(first + 1, last + 1) if t in tables]
        assert len(running) >= 15
        zero = [t for t in running if tables[t][l] == 0]
        assert not zero, (name, zero)
        assert all(tables[t][l] > 0 for t in range(last + 1, last + 9) if t in tables)
        after = [t for t in tables if t >= last + 10]
        assert after and all(tables[t][l] == 0 for t in after), name
