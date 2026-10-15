#!/usr/bin/env python3
"""Initial clock error of an emulated host (SPEC_v2 Section 7, Clock).

Before chronyd starts: sets CLOCK_REALTIME to the clock of the emulation host read through ptp_kvm
(/dev/ptp0; the RTC of the VM has a resolution of 1 s) plus the offset of /etc/fleetdt/emu-host.json
"clock": {"offset_ms": x, "ppm": y}, and sets the frequency error y with clock_adjtime(2)
ADJ_FREQUENCY. The vessels then follow the GS through chronyd over the emulated channel only, without
makestep, so chronyd slews the offset and estimates the frequency.
"""
import ctypes
import ctypes.util
import json
import os
import sys
import time

ADJ_FREQUENCY = 0x0002
CLOCK_REALTIME = 0


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


def phc_offset_ns(dev="/dev/ptp0", n=7):
    """host clock minus CLOCK_REALTIME of the VM, from the PHC of ptp_kvm (median of n readings)"""
    for _ in range(50):
        if os.path.exists(dev):
            break
        time.sleep(0.1)
    fd = os.open(dev, os.O_RDONLY)
    clk = ((~fd) << 3) | 3          # FD_TO_CLOCKID
    v = []
    for _ in range(n):
        a = time.clock_gettime_ns(CLOCK_REALTIME)
        p = time.clock_gettime_ns(clk)
        b = time.clock_gettime_ns(CLOCK_REALTIME)
        v.append(p - (a + b) // 2)
    os.close(fd)
    return sorted(v)[n // 2]


def main():
    with open("/etc/fleetdt/emu-host.json") as f:
        clk = json.load(f).get("clock") or {}
    off_ns = int(round(float(clk.get("offset_ms", 0)) * 1e6))
    ppm = float(clk.get("ppm", 0))
    try:
        d = phc_offset_ns()
    except OSError as e:
        sys.stderr.write("fdt-clockinit: no PHC (%s); the offset adds to the error of the RTC\n" % e)
        d = 0
    now = time.clock_gettime_ns(CLOCK_REALTIME)
    time.clock_settime_ns(CLOCK_REALTIME, now + d + off_ns)
    libc = ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)
    tx = Timex()
    tx.modes = ADJ_FREQUENCY
    tx.freq = int(round(ppm * 65536))
    if libc.clock_adjtime(CLOCK_REALTIME, ctypes.byref(tx)) < 0:
        sys.exit("clock_adjtime: errno %d" % ctypes.get_errno())
    print("fdt-clockinit: corrected %+.3f ms from the host PHC, then offset %+.3f ms, frequency %+.3f ppm"
          % (d / 1e6, off_ns / 1e6, ppm))


if __name__ == "__main__":
    main()
