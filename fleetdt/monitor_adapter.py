"""Channel monitor adapter of the emulator: y(t) of Section V-B (requirement R6) of the Fleet-DT paper,
computed from the nl80211 counters of the AP by the logic of radio/channel_monitor/fdt_chmon.c, and
published on fleet/throughput of the GS broker for fleetdt.as_service.

The emulator (emu/ns3/fdt-emu.cc --cnt) exports only what nl80211 gives on a deployed AP: the survey
of the channel in use (active and busy time in whole milliseconds) and, per associated station, the
RX/TX bytes and packets and tx_failed, read at the step boundaries with a read jitter. One read is one
JSON line in the sample format of `fdt_chmon --record`. The adapter pairs consecutive reads as the
live loop of fdt_chmon does and computes, for the step t closed by the read at start(t + 1),

    y(t) = R_PHY (Ts - Tb + To) / 8      bytes,

with two interchangeable backends:

  c    the unchanged fdt_chmon.c, built for the host (radio/channel_monitor/build_host.sh writes
       build/fdt_chmon_host), fed through its
       stream backend (`fdt_chmon --replay-stream -`); the adapter only moves lines
  py   a line-by-line port of load_replay(), compute() and emit() of fdt_chmon.c (this module);
       tests/test_monitor_adapter.py checks that both backends print the same lines

Measurement error (optional): the busy-time quantization comes from the whole-millisecond survey
counters; --sigma s adds multiplicative noise y' = max(0, y (1 + s e)), e ~ N(0, 1) from a seeded
generator, after the computation. The published line then keeps the noise-free value in "y0".

Output: one JSON object per step, {"t", "y", "Tb", "To"} (the format of fdt_chmon emit(), which
fleetdt.as_service.decode_throughput parses), QoS 0 on fleet/throughput; with --log, the same lines
with the host time of publication "pub" added.

  python -m fleetdt.monitor_adapter --dtp dtp.json --udp 127.0.0.1:9650 --broker 127.0.0.1:2883
         [--backend auto|c|py] [--chmon build/fdt_chmon_host] [--sigma 0.02 --seed 1]
         [--log y.jsonl] [--record counters.jsonl]
  python -m fleetdt.monitor_adapter --dtp dtp.json --replay counters.jsonl [--fast] [--backend ...]
         [--sigma ...] [--stdout] [--broker H:P]     (offline: every pair of the file, no timing)
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import os
import random
import shutil
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Tuple

log = logging.getLogger("fleetdt.monitor_adapter")

DEFAULT_EPOCH = 1767225600.0   # FDT_DEFAULT_EPOCH of common/fdt_common.h
U32 = 0xFFFFFFFF
U64 = 0xFFFFFFFFFFFFFFFF
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ------------------------------------------------------------------ DTP (fields of fdt_dtp_parse used here)
def _llround(x: float) -> int:
    """llround(): nearest integer, halfway cases away from zero."""
    r = math.floor(abs(x))
    if abs(x) - r >= 0.5:
        r += 1
    return int(r) if x >= 0 else -int(r)


@dataclass
class ChmonDTP:
    epoch: float
    Ts: float
    R_PHY: float
    h_W: float

    @property
    def epoch_us(self) -> int:
        return _llround(self.epoch * 1e6)

    @property
    def Ts_us(self) -> int:
        return _llround(self.Ts * 1e6)

    @staticmethod
    def from_dict(o: dict) -> "ChmonDTP":
        def num(k, default, required=False):
            v = o.get(k)
            if v is None:
                if required:
                    raise ValueError("DTP: field %s missing" % k)
                return float(default)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError("DTP: field %s is not a number" % k)
            return float(v)
        d = ChmonDTP(epoch=num("epoch", DEFAULT_EPOCH), Ts=num("Ts", 0, True),
                     R_PHY=num("R_PHY", 24e6), h_W=num("h_W", 0.0))
        if not d.Ts > 0:
            raise ValueError("DTP: Ts must be positive")
        if not d.R_PHY > 0:
            raise ValueError("fdt_chmon: R_PHY must be positive")
        return d

    @staticmethod
    def load(path: str) -> "ChmonDTP":
        with open(path, encoding="utf-8") as f:
            return ChmonDTP.from_dict(json.load(f))

    def step_at(self, us: int) -> int:
        """fdt_step_at(): floor((us - epoch_us) / Ts_us)."""
        return (us - self.epoch_us) // self.Ts_us


# ------------------------------------------------------------------ port of fdt_chmon.c
@dataclass
class Sample:
    at_us: int = 0
    have_survey: int = 0
    time_ms: int = 0
    busy_ms: int = 0
    rx_bytes: int = 0
    tx_bytes: int = 0
    rx_packets: int = 0
    tx_packets: int = 0
    tx_failed: int = 0
    stations: int = 0


@dataclass
class Result:
    y: float
    Tb: float
    To: float


def _first_key(pairs):
    """cJSON_GetObjectItemCaseSensitive returns the first of repeated keys."""
    out = {}
    for k, v in pairs:
        out.setdefault(k, v)
    return out


def _no_constant(name):
    raise ValueError("cJSON has no %s" % name)


_DECODER = json.JSONDecoder(object_pairs_hook=_first_key, parse_constant=_no_constant)


def _jnum(o, k: str) -> float:
    v = o.get(k) if isinstance(o, dict) else None
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return 0.0
    return float(v)


def _to_u64(x: float) -> int:
    """(uint64_t) of a double (truncation); the C conversion is undefined outside [0, 2^64), 0 here."""
    if not math.isfinite(x) or x < 0 or x >= 18446744073709551616.0:
        return 0
    return int(x)


def parse_sample(line: str) -> Optional[Sample]:
    """load_replay() of fdt_chmon.c for one line: None when the line holds no JSON value (cJSON_Parse
    reads the first value and ignores what follows it)."""
    s = line.lstrip(" \t\r\n\f\v")
    try:
        o, _ = _DECODER.raw_decode(s)
    except ValueError:
        return None
    x = Sample()
    x.at_us = int(_jnum(o, "at") * 1e6)
    sv = o.get("survey") if isinstance(o, dict) else None
    if not isinstance(o, dict) or "survey" not in o:
        x.have_survey = 1
    elif sv is True:
        x.have_survey = 1
    elif isinstance(sv, (int, float)) and not isinstance(sv, bool):
        x.have_survey = 1 if float(sv) != 0 else 0
    else:
        x.have_survey = 0
    x.time_ms = _to_u64(_jnum(o, "time_ms"))
    x.busy_ms = _to_u64(_jnum(o, "busy_ms"))
    x.rx_bytes = _to_u64(_jnum(o, "rx_bytes"))
    x.tx_bytes = _to_u64(_jnum(o, "tx_bytes"))
    x.rx_packets = _to_u64(_jnum(o, "rx_packets")) & U32
    x.tx_packets = _to_u64(_jnum(o, "tx_packets")) & U32
    x.tx_failed = _to_u64(_jnum(o, "tx_failed")) & U32
    x.stations = int(_jnum(o, "stations"))
    return x


def compute(d: ChmonDTP, a: Sample, b: Sample) -> Optional[Result]:
    """compute() of fdt_chmon.c, statement by statement (same operations in the same order, IEEE
    doubles): y of the interval between the readings a and b, or None."""
    if not a.have_survey or not b.have_survey or b.time_ms < a.time_ms or b.busy_ms < a.busy_ms:
        return None
    Ts = d.Ts
    dt_ms = (b.time_ms - a.time_ms) & U64
    db_ms = (b.busy_ms - a.busy_ms) & U64
    Tb = Ts * float(db_ms) / float(dt_ms) if dt_ms > 0 else float(db_ms) / 1000.0
    if Tb > Ts:
        Tb = Ts
    wall = float(b.at_us - a.at_us) / 1e6
    scale = Ts / wall if wall > 0 else 1.0
    rxb = float((b.rx_bytes - a.rx_bytes) & U64)
    txb = float((b.tx_bytes - a.tx_bytes) & U64)
    rxp = float((b.rx_packets - a.rx_packets) & U32)
    txp = float((b.tx_packets - a.tx_packets) & U32)
    txf = float((b.tx_failed - a.tx_failed) & U32)
    if b.rx_bytes < a.rx_bytes or b.tx_bytes < a.tx_bytes:   # a station left: counters restart
        return None
    txa = txp - txf if txp > txf else 0.0
    txba = txb * txa / txp if txp > 0 else 0.0
    To = (8.0 * (rxb + txba) + (rxp + txa) * 8.0 * d.h_W) / d.R_PHY * scale
    y = d.R_PHY * (Ts - Tb + To) / 8.0
    ymax = d.R_PHY * Ts / 8.0
    return Result(y=0.0 if y < 0 else (ymax if y > ymax else y), Tb=Tb, To=To)


def emit_line(t: int, r: Result) -> str:
    """emit() of fdt_chmon.c."""
    return '{"t":%d,"y":%.1f,"Tb":%.6f,"To":%.6f}' % (t, r.y, r.Tb, r.To)


class StreamPairer:
    """Pairing of the live loop of fdt_chmon (and of its stream backend): a sample read at the boundary
    start(t) closes step t - 1 when the previous sample belongs to step t - 1."""

    def __init__(self, d: ChmonDTP):
        self.d = d
        self.prev: Optional[Sample] = None
        self.prev_t = 0
        self.published = 0
        self.gaps = 0

    def feed(self, cur: Sample) -> Optional[Tuple[int, Result]]:
        t = self.d.step_at(cur.at_us)
        out = None
        if self.prev is not None and self.prev_t == t - 1:
            r = compute(self.d, self.prev, cur)
            if r is not None:
                out = (t - 1, r)
                self.published += 1
            else:
                self.gaps += 1
        elif self.prev is not None:
            self.gaps += 1
        self.prev, self.prev_t = cur, t
        return out


def fast_lines(d: ChmonDTP, lines: Iterable[str]) -> List[str]:
    """`fdt_chmon --replay FILE --fast`: y of each pair of consecutive samples, t = step of the first."""
    rs = [s for s in (parse_sample(l) for l in lines) if s is not None]
    out = []
    for k in range(1, len(rs)):
        r = compute(d, rs[k - 1], rs[k])
        if r is not None:
            out.append(emit_line(d.step_at(rs[k - 1].at_us + d.Ts_us // 2), r))
    return out


# ------------------------------------------------------------------ backends
class PyBackend:
    """Stream backend in Python: feed() takes one sample line and returns the y(t) line, if any."""
    name = "py"

    def __init__(self, dtp_path: str, on_line: Callable[[str], None]):
        self.d = ChmonDTP.load(dtp_path)
        self.pair = StreamPairer(self.d)
        self.on_line = on_line
        self.unparsed = 0

    def feed(self, line: str) -> None:
        s = parse_sample(line)
        if s is None:
            self.unparsed += 1
            return
        r = self.pair.feed(s)
        if r is not None:
            self.on_line(emit_line(*r))

    def close(self) -> None:
        pass


def has_stream(exe: str) -> bool:
    """True when the fdt_chmon executable has the stream backend (its usage text names it)."""
    try:
        r = subprocess.run([exe, "--help"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                           timeout=5)
    except (OSError, subprocess.SubprocessError):
        return False
    return "--replay-stream" in r.stdout


def find_chmon(explicit: Optional[str] = None) -> Optional[str]:
    """The given executable, else build/fdt_chmon_host, build/fdt_chmon or fdt_chmon on the PATH,
    the first one with the stream backend."""
    cands = [explicit] if explicit else [os.path.join(REPO, "build", "fdt_chmon_host"),
                                         os.path.join(REPO, "build", "fdt_chmon"), shutil.which("fdt_chmon")]
    for p in cands:
        if p and os.path.isfile(p) and os.access(p, os.X_OK) and has_stream(p):
            return p
    return None


class CBackend:
    """Stream backend in C: the unchanged fdt_chmon (stream backend) as a child process."""
    name = "c"

    def __init__(self, dtp_path: str, on_line: Callable[[str], None], chmon: Optional[str] = None):
        exe = find_chmon(chmon)
        if not exe:
            raise FileNotFoundError("fdt_chmon not found (radio/channel_monitor/build_host.sh)")
        self.exe = exe
        self.on_line = on_line
        self.p = subprocess.Popen([exe, "--dtp", dtp_path, "--replay-stream", "-"], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        self.stderr: List[str] = []
        self._rd = threading.Thread(target=self._read, daemon=True)
        self._rd.start()
        self._er = threading.Thread(target=lambda: self.stderr.extend(self.p.stderr), daemon=True)
        self._er.start()

    def _read(self) -> None:
        for line in self.p.stdout:
            line = line.strip()
            if line:
                self.on_line(line)

    def feed(self, line: str) -> None:
        if "\n" in line.rstrip("\n"):
            for part in line.splitlines():
                self.feed(part)
            return
        self.p.stdin.write(line.rstrip("\n") + "\n")
        self.p.stdin.flush()

    def close(self) -> None:
        try:
            self.p.stdin.close()
        except OSError:
            pass
        try:
            self.p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.p.kill()
        self._rd.join(timeout=2)
        self._er.join(timeout=2)


def make_backend(kind: str, dtp_path: str, on_line: Callable[[str], None], chmon: Optional[str] = None):
    if kind == "py":
        return PyBackend(dtp_path, on_line)
    if kind == "c":
        return CBackend(dtp_path, on_line, chmon)
    if find_chmon(chmon):
        return CBackend(dtp_path, on_line, chmon)
    log.warning("fdt_chmon not found: Python port of its logic")
    return PyBackend(dtp_path, on_line)


# ------------------------------------------------------------------ measurement error
class Noise:
    """Multiplicative noise y' = max(0, y (1 + sigma e)), e ~ N(0, 1); sigma = 0 passes the line as is."""

    def __init__(self, sigma: float = 0.0, seed: int = 1):
        self.sigma = float(sigma)
        self.rng = random.Random(seed)

    def apply(self, line: str) -> str:
        if self.sigma <= 0:
            return line
        o = json.loads(line)
        y0 = float(o["y"])
        y = max(0.0, y0 * (1.0 + self.sigma * self.rng.gauss(0.0, 1.0)))
        return '{"t":%d,"y":%.1f,"Tb":%.6f,"To":%.6f,"y0":%.1f,"sigma":%g}' % (
            int(o["t"]), y, float(o["Tb"]), float(o["To"]), y0, self.sigma)


# ------------------------------------------------------------------ service
class Adapter:
    def __init__(self, dtp_path: str, backend: str = "auto", chmon: Optional[str] = None,
                 broker: Optional[str] = None, topic: str = "fleet/throughput", sigma: float = 0.0,
                 seed: int = 1, log_path: Optional[str] = None, record_path: Optional[str] = None,
                 to_stdout: bool = False):
        self.noise = Noise(sigma, seed)
        self.topic = topic
        self.to_stdout = to_stdout
        self.lock = threading.Lock()
        self.log_file = open(log_path, "a", encoding="utf-8") if log_path else None
        self.rec_file = open(record_path, "a", encoding="utf-8") if record_path else None
        self.n_in = self.n_out = 0
        self.client = None
        if broker:
            import paho.mqtt.client as mqtt
            from .model import tcp_nodelay
            host, _, port = broker.rpartition(":") if ":" in broker else (broker, "", "1883")
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                      client_id="fdt-monitor-adapter-%d" % os.getpid(), clean_session=True)
            tcp_nodelay(self.client)
            self.client.reconnect_delay_set(0.2, 2)
            self.client.connect_async(host or "localhost", int(port or 1883), keepalive=10)
            self.client.loop_start()
        self.backend = make_backend(backend, dtp_path, self._out, chmon)
        log.info("backend %s", self.backend.name)

    def _out(self, line: str) -> None:
        line = self.noise.apply(line)
        with self.lock:
            self.n_out += 1
            if self.client is not None:
                self.client.publish(self.topic, line, qos=0)
            if self.to_stdout:
                print(line, flush=True)
            if self.log_file:
                self.log_file.write(line[:-1] + ',"pub":%.6f}\n' % time.time())
                self.log_file.flush()

    def feed(self, line: str) -> None:
        self.n_in += 1
        if self.rec_file:
            self.rec_file.write(line.rstrip("\n") + "\n")
            self.rec_file.flush()
        self.backend.feed(line)

    def close(self) -> None:
        self.backend.close()
        if self.client is not None:
            time.sleep(0.1)
            self.client.loop_stop()
            self.client.disconnect()
        for f in (self.log_file, self.rec_file):
            if f:
                f.close()


def serve_udp(ad: Adapter, listen: str, stop: threading.Event) -> None:
    host, _, port = listen.rpartition(":")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    s.bind((host or "127.0.0.1", int(port)))
    s.settimeout(0.5)
    while not stop.is_set():
        try:
            data, _ = s.recvfrom(1 << 16)
        except socket.timeout:
            continue
        for line in data.decode("utf-8", "replace").splitlines():
            if line.strip():
                ad.feed(line)
    s.close()


def main(argv: Optional[List[str]] = None) -> None:
    ap = argparse.ArgumentParser(prog="fleetdt.monitor_adapter", description=__doc__.split("\n")[0])
    ap.add_argument("--dtp", required=True)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--udp", help="host:port of the counter datagrams of fdt-emu --cntDst")
    src.add_argument("--replay", help="file of counter samples (fdt-emu counters.jsonl, fdt_chmon --record)")
    src.add_argument("--stdin", action="store_true", help="counter samples on stdin as they arrive")
    ap.add_argument("--fast", action="store_true",
                    help="with --replay: every consecutive pair, t = step of the first (fdt_chmon --fast)")
    ap.add_argument("--backend", choices=("auto", "c", "py"), default="auto")
    ap.add_argument("--chmon", default=None, help="fdt_chmon executable of the host (default: build/fdt_chmon_host)")
    ap.add_argument("--broker", default=None, help="GS broker host:port (publishes fleet/throughput)")
    ap.add_argument("--topic", default="fleet/throughput")
    ap.add_argument("--sigma", type=float, default=0.0, help="multiplicative noise of y (e.g. 0.02, 0.05)")
    ap.add_argument("--seed", type=int, default=1, help="seed of the noise")
    ap.add_argument("--log", default=None, help="JSON lines of the published y(t)")
    ap.add_argument("--record", default=None, help="copy of the received counter samples")
    ap.add_argument("--stdout", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO if a.verbose else logging.WARNING,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    if a.fast:
        if not a.replay:
            ap.error("--fast needs --replay")
        d = ChmonDTP.load(a.dtp)
        noise = Noise(a.sigma, a.seed)
        with open(a.replay, encoding="utf-8") as f:
            for line in fast_lines(d, f):
                print(noise.apply(line))
        return
    ad = Adapter(a.dtp, a.backend, a.chmon, a.broker, a.topic, a.sigma, a.seed, a.log, a.record,
                 to_stdout=a.stdout or not a.broker)
    print("monitor_adapter: backend %s" % ad.backend.name, file=sys.stderr, flush=True)
    stop = threading.Event()
    try:
        if a.udp:
            import signal
            signal.signal(signal.SIGTERM, lambda *_: stop.set())
            serve_udp(ad, a.udp, stop)
        else:
            f = sys.stdin if a.stdin else open(a.replay, encoding="utf-8")
            for line in f:
                if line.strip():
                    ad.feed(line)
    except KeyboardInterrupt:
        pass
    finally:
        ad.close()
        print("monitor_adapter: samples %d, y(t) %d" % (ad.n_in, ad.n_out), file=sys.stderr)


if __name__ == "__main__":
    main()
