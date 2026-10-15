#!/usr/bin/env python3
"""Copies the frozen inputs of the evaluation campaign into the repository (emu/experiments/frozen/).

  python3 emu/experiments/freeze.py --campaign ~/fleetdt-deploy/emu/campaigns/eval \
      --calib ~/fleetdt-deploy/emu/campaigns/calibration.json [--out emu/experiments/frozen]

Writes OUT/calibration.json, OUT/init/<S>-<predictor>.json (the initial scores; a file above 5 MB is
replaced by OUT/init/<name>.sha256 with its path on the server), OUT/manifest.json (the eval manifest),
OUT/specs.tar.gz (the run files of the manifest; deterministic archive: sorted, mtime 0), OUT/whatif/
(the what-if configurations without runs), OUT/seeds.json, OUT/dtp_base.json, OUT/scenarios/ (the reference
scenario files of the cells, seed 1; every run file holds the scenario of its own seed), OUT/traces.sha256
(the frame-size traces of the sources) and OUT/SHA256SUMS. FROZEN.md (hypotheses, thresholds, entry points,
commands) is written by hand and lists these files.
"""
import argparse
import glob
import gzip
import hashlib
import io
import json
import os
import shutil
import sys
import tarfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import matrix as MX  # noqa: E402

LIMIT = 5 * 1024 * 1024


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def det_targz(src_dir, out):
    """tar.gz of src_dir with sorted names, mtime 0, uid/gid 0 (same bytes for the same files)"""
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w", format=tarfile.PAX_FORMAT) as t:
        base = os.path.basename(src_dir.rstrip("/"))
        for p in sorted(glob.glob(os.path.join(src_dir, "*"))):
            ti = tarfile.TarInfo(os.path.join(base, os.path.basename(p)))
            data = open(p, "rb").read()
            ti.size, ti.mtime, ti.mode, ti.uid, ti.gid = len(data), 0, 0o644, 0, 0
            t.addfile(ti, io.BytesIO(data))
    with open(out, "wb") as f:
        with gzip.GzipFile(fileobj=f, mode="wb", mtime=0, filename="") as gz:
            gz.write(buf.getvalue())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--campaign", required=True)
    ap.add_argument("--calib", required=True)
    ap.add_argument("--out", default=os.path.join(HERE, "frozen"))
    a = ap.parse_args()
    out = a.out
    os.makedirs(out, exist_ok=True)
    for sub in ("init", "whatif", "scenarios"):
        shutil.rmtree(os.path.join(out, sub), ignore_errors=True)
        os.makedirs(os.path.join(out, sub))
    shutil.copy(a.calib, os.path.join(out, "calibration.json"))
    for p in sorted(glob.glob(os.path.join(a.campaign, "init", "*.json"))):
        if os.path.getsize(p) > LIMIT:
            with open(os.path.join(out, "init", os.path.basename(p) + ".sha256"), "w") as f:
                f.write("%s  %s\n" % (sha(p), os.path.abspath(p)))
        else:
            shutil.copy(p, os.path.join(out, "init"))
    for p in sorted(glob.glob(os.path.join(a.campaign, "whatif", "*.json"))):
        shutil.copy(p, os.path.join(out, "whatif"))
    shutil.copy(os.path.join(a.campaign, "manifest.json"), os.path.join(out, "manifest.json"))
    det_targz(os.path.join(a.campaign, "specs"), os.path.join(out, "specs.tar.gz"))
    man = json.load(open(os.path.join(a.campaign, "manifest.json")))
    seeds = {"eval": list(MX.EVAL_SEEDS), "calibration_E0": list(MX.CAL_SEEDS), "saturation": list(MX.SAT_SEEDS),
             "order_seed": man.get("order_seed"), "scenario_seed_rule": "each run: scengen.make(S, M, seed) "
             "(emu/scenarios/gen.py); the sources draw with --seed = the run seed (vessel/publishers/fdt_source.py)"}
    with open(os.path.join(out, "seeds.json"), "w") as f:
        json.dump(seeds, f, indent=1)
    shutil.copy(os.path.join(HERE, "dtp_base.json"), os.path.join(out, "dtp_base.json"))
    used = sorted({(r["scenario"], r["M"]) for r in man["runs"]})
    for sid, M in used:
        src = os.path.join(ROOT, "emu", "scenarios", "%s-m%d.json" % (sid, M))
        shutil.copy(src, os.path.join(out, "scenarios"))
    with open(os.path.join(out, "traces.sha256"), "w") as f:
        for p in sorted(glob.glob(os.path.join(ROOT, "emu", "traces", "*.txt"))):
            f.write("%s  emu/traces/%s\n" % (sha(p), os.path.basename(p)))
    with open(os.path.join(out, "SHA256SUMS"), "w") as f:
        for dp, dn, fn in sorted(os.walk(out)):
            for n in sorted(fn):
                if n in ("SHA256SUMS", "FROZEN.md"):
                    continue
                p = os.path.join(dp, n)
                f.write("%s  %s\n" % (sha(p), os.path.relpath(p, out)))
    by = {}
    for r in man["runs"]:
        by[r["exp"]] = by.get(r["exp"], 0) + 1
    print("frozen %d runs %s -> %s" % (len(man["runs"]), by, out))


if __name__ == "__main__":
    main()
