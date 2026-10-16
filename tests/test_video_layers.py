"""Video in 16 temporal layer topics (L = 22): bit-reversal layering, the DTP files, the per-vessel
demands, the AS with L = 22, the publishers, and the fill ratio f = max_i a_i / b of Algorithm 1 at
rho in {1.5, 2} and M in {2, 4} (validity rule of the experiments: mean f >= 0.85)."""
import json
import math
import random
import sys
import threading
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


def raw_dtp(name="dtp.json"):
    return json.loads((ROOT / name).read_text())


# --------------------------------------------------------------------------- layering


def test_bit_reversal_layers():
    assert [vl.layer_of_frame(k) for k in range(16)] == [1, 9, 5, 13, 3, 11, 7, 15, 2, 10, 6, 14, 4, 12, 8, 16]
    assert [vl.layer_of_frame(k) for k in range(16, 32)] == [vl.layer_of_frame(k) for k in range(16)]
    for j in range(1, 17):
        assert vl.layer_of_frame(vl.residue_of_layer(j)) == j
    with pytest.raises(ValueError):
        vl.layer_of_frame(3, layers=12)


@pytest.mark.parametrize("j", range(1, 17))
def test_prefix_of_layers_spreads_frames_uniformly(j):
    res = vl.admitted_frames(j)
    assert len(res) == j                                   # j / 16 of the frame rate
    jp = 1 << int(math.floor(math.log2(j)))                # largest power of two <= j
    gaps = np.diff(res + [res[0] + 16])
    assert gaps.max() <= 16 // jp                          # no gap longer than with j' layers
    if j == jp:
        assert set(gaps.tolist()) == {16 // j}             # exactly one frame out of every 16 / j


# --------------------------------------------------------------------------- DTP files


def test_root_dtp_has_22_topics_in_sigma_order():
    d = raw_dtp()
    dtp = DTP.from_dict(d)
    assert dtp.L == 22
    assert dtp.topics == TELEMETRY + LAYERS
    assert dtp.order == TELEMETRY + LAYERS                 # state first, telemetry, then video_1..video_16
    assert dtp.order[0] == "state" and dtp.needed == ["state", "imu", "gps"]
    c = vl.config(d)
    assert c["layers"] == 16 and vl.layer_topics(d) == LAYERS
    for name in LAYERS:                                    # one frame of 8 messages per step at most
        t = [x for x in d["topics"] if x["name"] == name][0]
        assert vl.msgs_per_step(t["rate"], d["Ts"]) == c["msgs_per_frame"]
    assert c["fps"] / c["layers"] < 1.0 / d["Ts"]


def test_publisher_dtp_reader_accepts_the_layers():
    from common import DTPFile
    d = DTPFile.load(str(ROOT / "dtp.json"))
    assert list(d.topics) == TELEMETRY + LAYERS
    assert d.per_step("video_7") == 8 and d.per_step("state") == 1


def test_dtp_v2_layers():
    d = raw_dtp("dtp_v2.json")
    names = [t["name"] for t in d["topics"]]
    assert names == TELEMETRY + LAYERS and d["order"] == TELEMETRY + LAYERS
    import fdt_source
    tps = fdt_source.load_dtp(str(ROOT / "dtp_v2.json"))["topics"]
    lay = [t for t in tps if t.layer is not None]
    assert [t.layer for t in lay] == list(range(1, 17))
    assert sorted(t.residue for t in lay) == list(range(16))
    assert all(abs(t.rate * t.layers - 15.0) < 1e-9 for t in lay)


def test_resolved_vessel_dtp_differs_only_in_layer_bounds():
    d = raw_dtp()
    per = {i: vl.resolve_vessel(d, i) for i in (1, 2, 3, 4)}
    for i, r in per.items():
        assert [t["name"] for t in r["topics"]] == [t["name"] for t in d["topics"]]
        assert r["order"] == d["order"] and r["needed"] == d["needed"]
        s = vl.vessel_config(d, i)["size_scale"]
        for t, t0 in zip(r["topics"], d["topics"]):
            if t["name"] in LAYERS:
                assert t["bound"] == round(s * t0["bound"])
            else:
                assert t == t0
    dem = {i: vl.channel_demand(per[i], "video_1") for i in per}
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
    t, p = decode_levels(encode_levels(0, dec["p"]))
    assert p == dec["p"]
    admitted = [l for l in range(7, 23) if p[l - 1] > 0]
    assert admitted == list(range(7, 7 + len(admitted)))   # a prefix of the layers


# --------------------------------------------------------------------------- fill ratio of Algorithm 1


def _demands(d_raw, M, base_bound):
    """Demand tables of the M vessels as the DTI program computes them from the per-vessel DTP whose
    layer bound is round(size_scale * base_bound)."""
    d = json.loads(json.dumps(d_raw))
    for t in d["topics"]:
        if t["name"] in LAYERS:
            t["bound"] = base_bound
    names = [t["name"] for t in d["topics"]]
    D = {}
    for i in range(1, M + 1):
        r = vl.resolve_vessel(d, i)
        D[i] = [vl.channel_demand(r, n) for n in names]
    return D


def _bound_for_rho(d_raw, M, rho, C):
    """Base layer bound for which sum_i sum_l d_il = rho C (bisection over integer bounds)."""
    lo, hi = vl.CONTENT_HEADER + 1, 200_000
    while hi - lo > 1:
        mid = (lo + hi) // 2
        tot = sum(map(sum, _demands(d_raw, M, mid).values()))
        lo, hi = (mid, hi) if tot <= rho * C else (lo, mid)
    return lo


def fill_ratio(D, sigma, N, b):
    p = admission(D, sigma, N, b)
    a = [sum(D[i][l - 1] for l in range(1, len(p) + 1) if p[l - 1] > 0) for i in D]
    return max(a) / b, p


@pytest.mark.parametrize("M", [2, 4])
@pytest.mark.parametrize("rho", [1.5, 2.0])
def test_fill_ratio_of_algorithm1(M, rho, capsys):
    """Fill ratio with the per-vessel demands of dtp.json, C = 320 kB per step (channel of mock.py),
    U = 0.8: static tables (the DTI program) and tables with a per-step lognormal variation of the
    layer demand (sigma 0.2, sources whose demand follows the frame size)."""
    d_raw = raw_dtp()
    dtp = DTP.from_dict(d_raw)
    C = 320_000.0
    sigma = [dtp.index(n) for n in dtp.order]
    N = [dtp.index(n) for n in dtp.needed]
    B = _bound_for_rho(d_raw, M, rho, C)
    D = _demands(d_raw, M, B)
    rho_act = sum(map(sum, D.values())) / C
    b = grant_limit(C, dtp.U, dtp.T3, dtp.Ts, M)
    f, p = fill_ratio(D, sigma, N, b)
    n_layers = sum(1 for l in range(7, 23) if p[l - 1] > 0)
    rng = random.Random(11)
    fv = []
    for _ in range(400):
        Dv = {i: [v if l < 6 else int(v * rng.lognormvariate(0.0, 0.2)) for l, v in enumerate(row)] for i, row in D.items()}
        fv.append(fill_ratio(Dv, sigma, N, b)[0])
    with capsys.disabled():
        print("\nfill M=%d rho=%.1f (actual %.3f): layer bound %d, b %.0f, %d layers admitted, f = %.3f; "
              "per-step variation: mean f = %.3f (min %.3f)" % (M, rho, rho_act, B, b, n_layers, f, np.mean(fv), min(fv)))
    assert abs(rho_act - rho) < 0.01
    assert FILL_MIN <= f <= 0.99
    assert np.mean(fv) >= FILL_MIN
    assert 1 <= n_layers < 16


# --------------------------------------------------------------------------- publishers


def test_zed_publisher_layer_helpers():
    cv2 = pytest.importorskip("cv2")
    import zed_publisher as zp
    assert [zp.frame_topic(k, LAYERS) for k in range(4)] == ["video_1", "video_9", "video_5", "video_13"]
    s = zp.FrameRateScale(0.5)
    assert [s.keep() for _ in range(6)] == [False, True, False, True, False, True]
    s = zp.FrameRateScale(1.0)
    assert all(s.keep() for _ in range(5))
    rng = np.random.default_rng(0)
    img = rng.integers(0, 255, size=(376, 1344, 3), dtype=np.uint8)
    cap = 8 * (1400 - 8)
    jpeg, q, h = zp.encode_within(img, 50, cap)
    assert len(jpeg) <= cap and (q < 50 or h > 0)          # noise does not fit at quality 50
    small = np.zeros((376, 1344, 3), np.uint8)
    jpeg, q, h = zp.encode_within(small, 50, cap)
    assert (q, h) == (50, 0) and len(jpeg) <= cap
    msgs = zp.split_frame(jpeg, 3, 1400)
    assert len(msgs) <= 8 and all(len(m) <= 1400 for m in msgs)


class _FakeClient:
    def __init__(self):
        self.out = []

    def publish(self, topic, body, qos=0):
        self.out.append((topic, body))


def test_fdt_source_interleaves_the_layers(tmp_path):
    import fdt_source
    d = raw_dtp("dtp_v2.json")
    for t in d["topics"]:
        if t.get("layer"):
            t["rate"] = 64.0 / 16          # 64 frames per second for a short test
            t["jitter"] = 0.0
    p = tmp_path / "dtp_v2.json"
    p.write_text(json.dumps(d))
    log = tmp_path / "src.jsonl"
    import time
    import types
    a = types.SimpleNamespace(dtp=str(p), vessel=2, topics=set(LAYERS), stream_size="max", median=6000.0, sigma=0.3,
                              size_trace=None, load=1.0, load_mode="size", phase="zero", duration=None, seed=3,
                              t0=None, log=str(log), video_scale=None, fps_scale=None)
    dtp = fdt_source.load_dtp(a.dtp)
    a.video_scale, a.fps_scale = fdt_source.vessel_video(dtp, a.vessel)
    assert (a.video_scale, a.fps_scale) == (0.9, 1.0)
    src = fdt_source.Source(a, dtp, random.Random(1))
    c = _FakeClient()
    stop = threading.Event()
    src.run(c, stop, time.time() + 1.2)
    recs = [json.loads(l) for l in log.read_text().splitlines()]
    assert len(recs) > 40
    ks = [r["out"] for r in recs]
    assert ks == list(range(ks[0], ks[0] + len(ks)))       # consecutive frames, in time order
    for r in recs:
        assert r["topic"] == "video_%d" % vl.layer_of_frame(r["out"])
    t = np.array([r["t_us"] for r in recs]) / 1e6
    assert np.allclose(np.diff(t), 1 / 64.0, atol=0.006)
    # the same frame index gives the same size on every layer assignment (one draw per frame)
    again = fdt_source.Source(a, dtp, random.Random(9))
    tp = [x for x in dtp["topics"] if x.name == recs[0]["topic"]][0]
    j = (recs[0]["out"] - tp.residue) // 16
    assert again.output_size(tp, j)[0] == recs[0]["bytes"]
