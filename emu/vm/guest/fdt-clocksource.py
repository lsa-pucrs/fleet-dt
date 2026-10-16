#!/usr/bin/env python3
"""Clock source of an emulated host (paper R4; design item I9).

The guests count time with the TSC (kernel command line `clocksource=tsc tsc=reliable`, set by
emu/vm/provision-base.sh), not with kvm-clock, so that the clock of a guest is the TSC scaled by its
own timekeeping and the frequency of fdt-freqshim.so / fdt-clockerr.py acts on it alone. At boot this
program checks the current clock source, selects tsc when the kernel offers it and another one is in
use (e.g. an image booted without the command line), and writes the result to
/run/fdt-clock/clocksource.json ({"current", "available", "changed", "tsc_flags"}); it exits with
status 1 when tsc is not available, so `systemctl status fdt-clocksource` shows the failure.
"""
import json
import os
import sys

BASE = "/sys/devices/system/clocksource/clocksource0"


def read(p):
    try:
        with open(p) as f:
            return f.read().strip()
    except OSError:
        return ""


def main():
    cur = read(os.path.join(BASE, "current_clocksource"))
    avail = read(os.path.join(BASE, "available_clocksource")).split()
    flags = []
    for line in read("/proc/cpuinfo").splitlines():
        if line.startswith("flags"):
            flags = [x for x in line.split(":", 1)[1].split() if x in ("tsc", "constant_tsc", "nonstop_tsc",
                                                                         "tsc_known_freq", "tsc_reliable")]
            break
    changed = False
    if cur != "tsc" and "tsc" in avail:
        try:
            with open(os.path.join(BASE, "current_clocksource"), "w") as f:
                f.write("tsc")
            changed = True
        except OSError as e:
            sys.stderr.write("fdt-clocksource: cannot select tsc: %s\n" % e)
        cur = read(os.path.join(BASE, "current_clocksource"))
    os.makedirs("/run/fdt-clock", exist_ok=True)
    rec = {"current": cur, "available": avail, "changed": changed, "tsc_flags": flags,
           "cmdline": read("/proc/cmdline")}
    with open("/run/fdt-clock/clocksource.json", "w") as f:
        json.dump(rec, f)
    print("fdt-clocksource: %s (available %s)%s" % (cur, " ".join(avail), " (selected now)" if changed else ""))
    return 0 if cur == "tsc" else 1


if __name__ == "__main__":
    sys.exit(main())
