"""fleetdt/monitor_adapter.py: the Python port of radio/channel_monitor/fdt_chmon.c prints the same
y(t) lines as the C program (built for the host) on recorded and synthetic counter streams, through the
offline replay (--replay --fast) and through the stream backend (--replay-stream); the noise option and
the output format that fleetdt.as_service parses."""
import json
import math
import os
import random
import shutil
import socket
import subprocess
import sys
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from fleetdt import monitor_adapter as ma  # noqa: E402
from fleetdt.as_service import decode_throughput  # noqa: E402

EPOCH = 1767225600.0
TS = 0.125
DTP = {"epoch": None, "Ts": TS, "T1": 0.020, "T2": 0.010, "R_PHY": 24e6, "h_W": 150,
       "topics": [{"name": "state", "rate": 8, "bound": 48}, {"name": "video", "rate": 30, "bound": 14000}]}
# the PHY fields of dtp.json (C7): T_fleet(t) charges h_W_busy = h_W - R_PHY (T_DIFS + T_bo) / 8 per frame
DTP_C7 = dict(DTP, h_W=570.25, T_DIFS=37e-6, T_bo=67.5e-6)


@pytest.fixture(scope="module")
def chmon(tmp_path_factory):
    if not shutil.which(os.environ.get("CC", "cc")):
        pytest.skip("no C compiler")
    out = tmp_path_factory.mktemp("chmon") / "fdt_chmon_host"
    r = subprocess.run(["sh", os.path.join(ROOT, "radio", "channel_monitor", "build_host.sh"), str(out)],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if r.returncode != 0:
        pytest.skip("fdt_chmon does not build here (cJSON?): " + r.stdout[-400:])
    return str(out)


@pytest.fixture(scope="module", params=["h_W", "C7"])
def dtp_path(tmp_path_factory, request):
    """the C and the Python monitor agree with h_W alone and with the busy part of h_W (C7)"""
    p = tmp_path_factory.mktemp("dtp") / "dtp.json"
    p.write_text(json.dumps(DTP if request.param == "h_W" else DTP_C7))
    return str(p)


def emu_like(n=400, seed=1, jitter_us=300, off_us=500, M=4, start_step=1000, gaps=True):
    """Counter reads like fdt-emu --cnt: at = start(t) + off + U[0, jitter], survey in whole ms
    (harvest quantization), 64-bit bytes and 32-bit packets that wrap, failures, a few lost reads."""
    rng = random.Random(seed)
    lines = []
    time_ns = busy_ns = 0
    tms = bms = 0
    rxb = txb = 0
    rxp = (1 << 32) - 2000 if seed % 2 else 0     # 32-bit wrap inside the stream
    txp = txf = 0
    prev_at = None
    for k in range(n):
        t = start_step + k
        at_us = int(round((EPOCH + t * TS) * 1e6)) + off_us + rng.randint(0, jitter_us)
        if prev_at is not None:
            dt = (at_us - prev_at) * 1000
            time_ns += dt
            busy_ns += int(dt * rng.uniform(0.05, 0.98))
            tms += time_ns // 1000000
            bms += busy_ns // 1000000
            time_ns %= 1000000
            busy_ns %= 1000000
            nrx, ntx = rng.randint(0, 400), rng.randint(0, 120)
            rxp = (rxp + nrx) & 0xFFFFFFFF
            txp = (txp + ntx) & 0xFFFFFFFF
            txf = (txf + rng.randint(0, max(0, ntx // 20))) & 0xFFFFFFFF
            rxb += nrx * rng.randint(60, 1534) + (1 << 40 if k == n // 2 else 0)
            txb += ntx * rng.randint(60, 1534)
        prev_at = at_us
        if gaps and rng.random() < 0.02:
            continue  # a read lost on the way (UDP)
        rec = {"at": "%d.%06d" % divmod(at_us, 1000000), "survey": 1, "time_ms": tms, "busy_ms": bms,
               "rx_bytes": rxb, "tx_bytes": txb, "rx_packets": rxp, "tx_packets": txp, "tx_failed": txf,
               "stations": M, "sta": []}
        line = json.dumps(rec).replace('"at": "%s"' % rec["at"], '"at": %s' % rec["at"])
        lines.append(line)
    return lines


def adversarial(seed=7):
    """Lines that exercise every branch of load_replay() and compute()."""
    rng = random.Random(seed)
    base = int(round((EPOCH + 5000 * TS) * 1e6))
    L = []

    def s(k, **kw):
        o = {"at": (base + k * 125000 + kw.pop("dus", 500)) / 1e6, "time_ms": 1000 + 125 * k,
             "busy_ms": 500 + 60 * k, "rx_bytes": 10 ** 6 + 9000 * k, "tx_bytes": 2 * 10 ** 6 + 7000 * k,
             "rx_packets": 100 + 7 * k, "tx_packets": 50 + 5 * k, "tx_failed": k // 3, "stations": 2}
        o.update(kw)
        return json.dumps(o)
    L += [s(0), s(1), s(2)]
    L.append(s(3, survey=0))                    # no survey in this read
    L += [s(4), s(5, survey=False), s(6, survey=True), s(7, survey=None), s(8, survey=2.5)]
    L.append(s(9, time_ms=10))                  # survey counter went back
    L += [s(10), s(11, busy_ms=0)]              # busy went back
    L += [s(12), s(13, rx_bytes=5)]             # station left: counters restart
    L += [s(14), s(15, tx_bytes=1)]
    L += [s(16), s(16), s(17)]                  # repeated read of one boundary
    L += [s(19), s(20)]                         # a missing boundary
    L += [s(21), s(22, time_ms=1000 + 125 * 21)]  # dt_ms = 0
    L += [s(23), s(24, busy_ms=10 ** 6)]        # Tb clipped to Ts
    L += [s(25), s(26, tx_packets=50 + 5 * 25, tx_failed=100)]   # no tx in the interval
    L += [s(27), s(28, tx_failed=10 ** 6)]      # txf > txp
    L += [s(29), s(30, dus=124000)]             # late read (jitter up to the next boundary)
    L += [s(31, dus=0), s(32, dus=1)]           # read exactly at the boundary
    L += ["", "not json", "{broken", "123", "[1, 2]", "null", s(40) + " trailing", s(41)]
    L += [s(42, rx_packets="7"), s(43, stations=True), s(44)]
    L += [s(45, rx_bytes=2 ** 60), s(46, rx_bytes=2 ** 60 + 2 ** 20)]
    L += [s(47, rx_packets=2 ** 33 + 5), s(48, rx_packets=2 ** 33 + 900)]
    L += [s(49, at=int(base / 1e6) + 49 * 0.125), s(50)]
    L += ['{"at": %r, "at": 0, "time_ms": 1, "busy_ms": 0}' % ((base + 51 * 125000 + 500) / 1e6),
          '{"at": %r, "time_ms": 126, "busy_ms": 3}' % ((base + 52 * 125000 + 500) / 1e6)]   # repeated key
    for k in range(60, 120):                    # random values
        L.append(s(k, busy_ms=500 + 60 * 59 + rng.randint(0, 125) * (k - 59), dus=rng.randint(0, 3000)))
    return L


def run_c(chmon, dtp_path, args, lines, tmp_path, name):
    p = tmp_path / name
    p.write_text("\n".join(lines) + "\n")
    r = subprocess.run([chmon, "--dtp", dtp_path] + args + [str(p)], stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    return [l for l in r.stdout.splitlines() if l.strip()]


def run_py_stream(dtp_path, lines):
    out = []
    b = ma.PyBackend(dtp_path, out.append)
    for l in lines:
        b.feed(l)
    return out


STREAMS = [("emu-s1", lambda: emu_like(seed=1)), ("emu-s2", lambda: emu_like(seed=2, jitter_us=5000)),
           ("emu-nogap", lambda: emu_like(seed=3, gaps=False, M=2)), ("adversarial", adversarial)]


@pytest.mark.parametrize("name,gen", STREAMS)
def test_fast_equals_c(chmon, dtp_path, tmp_path, name, gen):
    lines = gen()
    c = run_c(chmon, dtp_path, ["--fast", "--replay"], lines, tmp_path, name + ".jsonl")
    py = ma.fast_lines(ma.ChmonDTP.load(dtp_path), lines)
    assert len(c) > 10
    assert py == c


@pytest.mark.parametrize("name,gen", STREAMS)
def test_stream_equals_c(chmon, dtp_path, tmp_path, name, gen):
    lines = gen()
    c = run_c(chmon, dtp_path, ["--replay-stream"], lines, tmp_path, name + ".jsonl")
    py = run_py_stream(dtp_path, lines)
    assert len(c) > 10
    assert py == c


def test_c_backend_through_pipe(chmon, dtp_path):
    lines = emu_like(seed=5, n=200)
    got = []
    b = ma.CBackend(dtp_path, got.append, chmon)
    for l in lines:
        b.feed(l)
    b.close()
    assert got == run_py_stream(dtp_path, lines)
    assert any("stream published=" in e for e in b.stderr)


def test_recorded_counters(chmon, dtp_path, tmp_path):
    """Recorded emulator counter files (FDT_COUNTERS=path1:path2), when present."""
    paths = [p for p in os.environ.get("FDT_COUNTERS", "").split(":") if p and os.path.isfile(p)]
    if not paths:
        pytest.skip("no recorded counters (FDT_COUNTERS)")
    for p in paths:
        lines = open(p).read().splitlines()
        c = run_c(chmon, dtp_path, ["--replay-stream"], lines, tmp_path, os.path.basename(p))
        assert run_py_stream(dtp_path, lines) == c


def test_formula():
    """y = R_PHY (Ts - Tb + To) / 8 on one hand-computed pair."""
    d = ma.ChmonDTP.from_dict(DTP)
    a = ma.Sample(at_us=0, have_survey=1, time_ms=1000, busy_ms=100, rx_bytes=0, tx_bytes=0)
    b = ma.Sample(at_us=125000, have_survey=1, time_ms=1125, busy_ms=150, rx_bytes=30000, tx_bytes=10000,
                  rx_packets=20, tx_packets=10, tx_failed=2)
    r = ma.compute(d, a, b)
    Tb = 0.125 * 50 / 125
    To = (8 * (30000 + 10000 * 8 / 10) + (20 + 8) * 8 * 150) / 24e6
    assert r.Tb == pytest.approx(Tb)
    assert r.To == pytest.approx(To)
    assert r.y == pytest.approx(24e6 * (0.125 - Tb + To) / 8)


def test_formula_busy_part_of_h_W():
    """C7: T_fleet(t) counts only the busy part of each frame, h_W - R_PHY (T_DIFS + T_bo) / 8 (DIFS and mean
    backoff are idle time for the survey), and y stays in [0, R_PHY Ts / 8]."""
    d = ma.ChmonDTP.from_dict(DTP_C7)
    assert d.h_W_busy == pytest.approx(570.25 - 24e6 * (37e-6 + 67.5e-6) / 8) == pytest.approx(256.75)
    a = ma.Sample(at_us=0, have_survey=1, time_ms=1000, busy_ms=100, rx_bytes=0, tx_bytes=0)
    b = ma.Sample(at_us=125000, have_survey=1, time_ms=1125, busy_ms=150, rx_bytes=30000, tx_bytes=10000,
                  rx_packets=20, tx_packets=10, tx_failed=2)
    r = ma.compute(d, a, b)
    To = (8 * (30000 + 10000 * 8 / 10) + (20 + 8) * 8 * 256.75) / 24e6
    assert r.To == pytest.approx(To)
    assert r.y == pytest.approx(24e6 * (0.125 - 0.125 * 50 / 125 + To) / 8)
    # clamp: a busy-free interval with many frames stays at R_PHY Ts / 8
    b2 = ma.Sample(at_us=125000, have_survey=1, time_ms=1125, busy_ms=100, rx_bytes=10 ** 6, tx_bytes=0,
                   rx_packets=900)
    assert ma.compute(d, a, b2).y == 24e6 * 0.125 / 8
    # the busy part may not be negative
    with pytest.raises(ValueError):
        ma.ChmonDTP.from_dict(dict(DTP_C7, h_W=300.0))


def test_c_refuses_negative_busy_part(chmon, tmp_path):
    p = tmp_path / "bad.json"
    p.write_text(json.dumps(dict(DTP_C7, h_W=300.0)))
    r = subprocess.run([chmon, "--dtp", str(p), "--replay", "/dev/null", "--fast", "--stdout"],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    assert r.returncode != 0 and "T_DIFS" in r.stdout


def test_pairing_and_labels():
    """A read at start(t) + off closes step t - 1; a missing boundary gives no y."""
    d = ma.ChmonDTP.from_dict(DTP)
    p = ma.StreamPairer(d)
    mk = lambda t, ms: ma.Sample(at_us=d.epoch_us + t * d.Ts_us + 500, have_survey=1, time_ms=ms, busy_ms=0)
    assert p.feed(mk(10, 0)) is None
    t, r = p.feed(mk(11, 125))
    assert t == 10 and r.y == pytest.approx(24e6 * 0.125 / 8)
    assert p.feed(mk(13, 375)) is None and p.gaps == 1
    assert p.feed(mk(14, 500))[0] == 13


def test_noise():
    line = '{"t":7,"y":300000.0,"Tb":0.020000,"To":0.010000}'
    assert ma.Noise(0.0).apply(line) == line
    n1, n2 = ma.Noise(0.05, seed=3), ma.Noise(0.05, seed=3)
    ys = []
    for _ in range(4000):
        a, b = n1.apply(line), n2.apply(line)
        assert a == b
        o = json.loads(a)
        assert o["y0"] == 300000.0 and o["sigma"] == 0.05 and o["t"] == 7
        ys.append(o["y"] / 300000.0)
    m = sum(ys) / len(ys)
    sd = math.sqrt(sum((v - m) ** 2 for v in ys) / (len(ys) - 1))
    assert abs(m - 1) < 0.005 and abs(sd - 0.05) < 0.005


def test_output_parsed_by_as_service():
    out = []
    b = ma.PyBackend.__new__(ma.PyBackend)
    b.d, b.on_line, b.unparsed = ma.ChmonDTP.from_dict(DTP), out.append, 0
    b.pair = ma.StreamPairer(b.d)
    for l in emu_like(n=20, gaps=False):
        b.feed(l)
    assert out
    for line in out + [ma.Noise(0.02, 1).apply(out[0])]:
        t, y = decode_throughput(line.encode())
        o = json.loads(line)
        assert t == o["t"] and y == o["y"]


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_udp_to_mqtt(dtp_path, tmp_path):
    """Counter datagrams in, fleet/throughput out on a broker, as in the emulator."""
    mosq = shutil.which("mosquitto") or ("/usr/sbin/mosquitto" if os.path.exists("/usr/sbin/mosquitto") else None)
    if not mosq:
        pytest.skip("no mosquitto broker")
    pytest.importorskip("paho.mqtt.client")
    import paho.mqtt.client as mqtt
    bport, uport = _free_port(), _free_port()
    conf = tmp_path / "m.conf"
    conf.write_text("listener %d 127.0.0.1\nallow_anonymous true\n" % bport)
    broker = subprocess.Popen([mosq, "-c", str(conf)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ad = None
    try:
        time.sleep(0.5)
        got = []
        sub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t-sub")
        sub.on_message = lambda c, u, m: got.append(decode_throughput(m.payload))
        sub.connect("127.0.0.1", bport)
        sub.subscribe("fleet/throughput", 0)
        sub.loop_start()
        ylog = tmp_path / "y.jsonl"
        ad = subprocess.Popen([sys.executable, "-m", "fleetdt.monitor_adapter", "--dtp", dtp_path, "--udp",
                               "127.0.0.1:%d" % uport, "--broker", "127.0.0.1:%d" % bport, "--backend", "py",
                               "--log", str(ylog), "--record", str(tmp_path / "rec.jsonl")], cwd=ROOT)
        time.sleep(1.5)
        lines = emu_like(n=30, gaps=False)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for l in lines:
            s.sendto((l + "\n").encode(), ("127.0.0.1", uport))
            time.sleep(0.005)
        time.sleep(1.0)
        sub.loop_stop()
        want = [(json.loads(l)["t"], json.loads(l)["y"]) for l in run_py_stream(dtp_path, lines)]
        assert got == want
        assert len(open(tmp_path / "rec.jsonl").read().splitlines()) == len(lines)
        assert all("pub" in json.loads(l) for l in open(ylog))
    finally:
        if ad:
            ad.terminate()
            ad.wait(5)
        broker.terminate()
        broker.wait(5)
