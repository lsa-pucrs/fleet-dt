#!/usr/bin/env python3
"""Oscillator error and NTP loss of a vessel VM (paper R4 and E1; design item I9).

Reads the schedule of fdt_clock_schedule.py (default /etc/fleetdt/clock-schedule.json, or the path in
/etc/fleetdt/emu-host.json "clock": {"schedule": PATH}; reloaded when the file changes, so the harness
can install the schedule of each run) and, every --period (0.1 s):

  frequency  writes e(t) into the shared page of fdt-freqshim.so (preloaded into chronyd) and sets the
             kernel frequency to chronyd's last correction + e(t) (adjtimex ADJ_FREQUENCY), repeating the
             write when chronyd wrote in between (generation counter of the page); before chronyd's first
             write, the base is the kernel frequency found at the start
  NTP loss   inside an episode of the schedule, the nftables table `inet fdt_ntploss` drops UDP port 123
             in both directions (chronyd keeps polling and sees the loss); outside, the table is absent

Each change and one line per second go to --log (JSON lines: t_unix, sched_t, ppm, chrony_ppm,
kernel_ppm, ntp_blocked, writes). Without a schedule the injector keeps e = 0 and the table absent.

  fdt-clockerr.py [--schedule PATH] [--shm /run/fdt-clock/freq.shm] [--log /var/log/fdt-clockerr.jsonl]
                  [--period 0.1] [--dry-run]

--dry-run uses an in-memory kernel and no nftables (tests of the emulation host).
"""
import argparse
import ctypes
import ctypes.util
import json
import mmap
import os
import signal
import struct
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fdt_clock_schedule import Schedule  # noqa: E402

SCALE = 65536.0
MAX_FREQ = 500 * 65536
ADJ_FREQUENCY = 0x0002
FDT_MAGIC = 0x46544446
# layout of struct fdt_freq_shm (fdt-freqshim.c)
FREQ_SHM = {"magic": (0, "I"), "version": (4, "I"), "err": (8, "q"), "chrony_freq": (16, "q"),
            "chrony_tick": (24, "q"), "gen": (32, "Q"), "applied_err": (40, "q"), "chrony_valid": (48, "i"),
            "writes": (56, "Q")}
NFT_ON = """table inet fdt_ntploss {
  chain out { type filter hook output priority -5; policy accept; udp dport 123 drop; udp sport 123 drop; }
  chain in { type filter hook input priority -5; policy accept; udp sport 123 drop; udp dport 123 drop; }
}
"""


class Shm:
    def __init__(self, path):
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            if os.fstat(fd).st_size < 4096:
                os.ftruncate(fd, 4096)
            self.m = mmap.mmap(fd, 4096, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        finally:
            os.close(fd)
        if self.get("magic") != FDT_MAGIC:
            self.set("version", 1)
            self.set("magic", FDT_MAGIC)

    def get(self, k):
        off, fmt = FREQ_SHM[k]
        return struct.unpack_from("=" + fmt, self.m, off)[0]

    def set(self, k, v):
        off, fmt = FREQ_SHM[k]
        struct.pack_into("=" + fmt, self.m, off, int(v))


class Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class Timex(ctypes.Structure):
    _fields_ = [("modes", ctypes.c_uint), ("offset", ctypes.c_long), ("freq", ctypes.c_long),
                ("maxerror", ctypes.c_long), ("esterror", ctypes.c_long), ("status", ctypes.c_int),
                ("constant", ctypes.c_long), ("precision", ctypes.c_long), ("tolerance", ctypes.c_long),
                ("time", Timeval), ("tick", ctypes.c_long), ("ppsfreq", ctypes.c_long),
                ("jitter", ctypes.c_long), ("shift", ctypes.c_int), ("stabil", ctypes.c_long),
                ("jitcnt", ctypes.c_long), ("calcnt", ctypes.c_long), ("errcnt", ctypes.c_long),
                ("stbcnt", ctypes.c_long), ("tai", ctypes.c_int), ("pad", ctypes.c_int * 11)]


class KernelClock:
    """kernel frequency of CLOCK_REALTIME through the libc adjtimex (not the preloaded shim)"""

    def __init__(self):
        self.libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)

    def read_freq(self):
        tx = Timex()
        tx.modes = 0
        if self.libc.adjtimex(ctypes.byref(tx)) < 0:
            raise OSError(ctypes.get_errno(), "adjtimex")
        return int(tx.freq)

    def write_freq(self, f):
        tx = Timex()
        tx.modes = ADJ_FREQUENCY
        tx.freq = int(max(-MAX_FREQ, min(MAX_FREQ, f)))
        if self.libc.adjtimex(ctypes.byref(tx)) < 0:
            raise OSError(ctypes.get_errno(), "adjtimex")


class FakeClock:
    def __init__(self, f=0):
        self.f = int(f)
        self.writes = []

    def read_freq(self):
        return self.f

    def write_freq(self, f):
        self.f = int(max(-MAX_FREQ, min(MAX_FREQ, f)))
        self.writes.append(self.f)


class Nft:
    def __init__(self, dry=False):
        self.dry = dry
        self.on = False

    def set(self, on):
        if on == self.on:
            return
        if not self.dry:
            if on:
                subprocess.run(["nft", "-f", "-"], input=NFT_ON, text=True, check=False)
            else:
                subprocess.run(["nft", "delete", "table", "inet", "fdt_ntploss"], check=False,
                               stderr=subprocess.DEVNULL)
        self.on = on


class Injector:
    def __init__(self, shm, kernel, nft):
        self.shm, self.kernel, self.nft = shm, kernel, nft
        # base before chronyd's first write: the kernel frequency minus what was injected before
        self.base0 = kernel.read_freq() - shm.get("applied_err")
        self.err = shm.get("err")

    def apply(self, ppm, blocked):
        """inject ppm (and the NTP state); returns (chrony scaled freq, kernel scaled freq)"""
        err = int(round(ppm * SCALE))
        self.nft.set(blocked)
        for _ in range(10):
            g0 = self.shm.get("gen")
            if g0 % 2:                      # a write of chronyd is in progress
                time.sleep(1e-4)
                continue
            self.shm.set("err", err)
            base = self.shm.get("chrony_freq") if self.shm.get("chrony_valid") else self.base0
            if err != self.err or self.shm.get("applied_err") != err:
                self.kernel.write_freq(base + err)
                self.shm.set("applied_err", err)
            if self.shm.get("gen") == g0:
                break
            # chronyd wrote between the read of its frequency and our write: write again
            self.shm.set("applied_err", -10 ** 12)
        self.err = err
        base = self.shm.get("chrony_freq") if self.shm.get("chrony_valid") else self.base0
        return base, base + err


def schedule_path(arg):
    if arg:
        return arg
    try:
        with open("/etc/fleetdt/emu-host.json") as f:
            c = (json.load(f).get("clock") or {}).get("schedule")
        if c:
            return c
    except (OSError, ValueError):
        pass
    return "/etc/fleetdt/clock-schedule.json"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--schedule", default=None)
    ap.add_argument("--shm", default=os.environ.get("FDT_FREQ_SHM", "/run/fdt-clock/freq.shm"))
    ap.add_argument("--log", default="/var/log/fdt-clockerr.jsonl")
    ap.add_argument("--period", type=float, default=0.1)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--duration", type=float, default=0, help="stop after this many seconds (0: never)")
    a = ap.parse_args()
    stop = []
    signal.signal(signal.SIGTERM, lambda *x: stop.append(1))
    path = schedule_path(a.schedule)
    inj = Injector(Shm(a.shm), FakeClock() if a.dry_run else KernelClock(), Nft(a.dry_run))
    sched, mtime = None, None
    log = open(a.log, "a", buffering=1)
    t_start = time.time()
    last_line = 0.0
    prev = None
    while not stop and (a.duration <= 0 or time.time() - t_start < a.duration):
        try:
            mt = os.stat(path).st_mtime
        except OSError:
            mt = None
        if mt != mtime:
            mtime = mt
            sched = None
            if mt is not None:
                try:
                    with open(path) as f:
                        sched = Schedule(json.load(f))
                    log.write(json.dumps({"t_unix": time.time(), "schedule": path, "t0_unix": sched.t0}) + "\n")
                except (OSError, ValueError) as e:
                    log.write(json.dumps({"t_unix": time.time(), "schedule_error": str(e)}) + "\n")
        now = time.time()
        if sched is not None:
            ppm, blocked = sched.at_unix(now)
            st = now - sched.t0
        else:
            ppm, blocked, st = 0.0, False, None
        cf, kf = inj.apply(ppm, blocked)
        state = (round(ppm, 3), blocked)
        if state != prev or now - last_line >= 1.0:
            log.write(json.dumps({"t_unix": round(now, 3), "sched_t": None if st is None else round(st, 3),
                                  "ppm": round(ppm, 4), "chrony_ppm": round(cf / SCALE, 4),
                                  "kernel_ppm": round(kf / SCALE, 4), "ntp_blocked": blocked,
                                  "chrony_writes": inj.shm.get("writes")}) + "\n")
            last_line, prev = now, state
        time.sleep(max(0.0, a.period - (time.time() - now)))
    inj.apply(0.0, False)
    log.close()


if __name__ == "__main__":
    main()
