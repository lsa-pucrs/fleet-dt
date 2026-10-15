#!/usr/bin/env python3
"""Checks that every E5 run file equals the frozen E3 Fleet-DT run file of the same cell and seed in the
fields that fix the channel and the data (ADDENDUM.md, section A1).

  python3 emu/experiments/addendum/check_e5.py CAMPAIGN_DIR [--frozen emu/experiments/frozen/specs.tar.gz]
"""
import argparse
import glob
import json
import os
import sys
import tarfile

HERE = os.path.dirname(os.path.abspath(__file__))
FIELDS = ("dtp", "scenario", "source", "rho_setting", "opctl", "steps", "seed", "M", "Ts", "rho", "scenario_id")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("campaign")
    ap.add_argument("--frozen", default=os.path.join(os.path.dirname(HERE), "frozen", "specs.tar.gz"))
    a = ap.parse_args()
    e3 = {}
    with tarfile.open(a.frozen) as t:
        for m in t.getmembers():
            if "/E3-fleetdt-" in m.name:
                e3[os.path.basename(m.name)] = json.load(t.extractfile(m))
    bad, n = [], 0
    for p in sorted(glob.glob(os.path.join(a.campaign, "specs", "E5-b6-*.json"))):
        n += 1
        s = json.load(open(p))
        ref = e3.get(os.path.basename(p).replace("E5-b6-", "E3-fleetdt-"))
        if ref is None:
            bad.append((os.path.basename(p), "no E3 run"))
            continue
        for f in FIELDS:
            if s.get(f) != ref.get(f):
                bad.append((os.path.basename(p), f))
    print("%d E5 run files; %d differences" % (n, len(bad)))
    for b in bad:
        print("  %s: %s" % b)
    sys.exit(1 if bad or n != 144 else 0)


if __name__ == "__main__":
    main()
