#!/usr/bin/env python3
"""Package the evaluation campaign for an archive deposit (e.g. Zenodo) in two parts.

Part A (derived data and software, about 16 MB per run):
  - git archive of the tagged commit (software, frozen inputs, paper Section VI text)
  - per run: experiment.json, dtp.json, scenario.json, result.json, metrics.json, gt_steps.jsonl,
    clock.csv, bridge-stats.jsonl, init_scores.json, emu/{summary.json,lag.txt}, vms/** (DTA, logs)
Part B (raw traces, about 27 MB per run):
  - per run: emu/gt.bin.zst (ground truth of the emulator), monitor.jsonl.zst (monitor stream)

Each part is split into tar files of at most --max-gb GB and gets a SHA256SUMS file and a README.

Usage (on the emulation host):
  python3 emu/tools/package_archive.py --repo ~/fleet-dt-dba --tag v2-freeze-20261016 \
      --results ~/fleetdt-deploy/emu/results/eval --out ~/fleetdt-deploy/emu/archive --max-gb 20
"""
import argparse, hashlib, os, subprocess, tarfile

RAW = ("emu/gt.bin.zst", "monitor.jsonl.zst")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def files_of(run_dir, raw):
    out = []
    for root, _, names in os.walk(run_dir):
        for n in sorted(names):
            p = os.path.join(root, n)
            rel = os.path.relpath(p, run_dir)
            if (rel in RAW) == raw:
                out.append((p, rel))
    return out


def write_part(name, runs, results, out, raw, max_bytes):
    os.makedirs(out, exist_ok=True)
    idx, cur, tf, written = 0, 0, None, []
    for run in runs:
        items = files_of(os.path.join(results, run), raw)
        size = sum(os.path.getsize(p) for p, _ in items)
        if tf is None or cur + size > max_bytes:
            if tf:
                tf.close()
            idx += 1
            tpath = os.path.join(out, f"{name}-{idx:03d}.tar")
            tf, cur = tarfile.open(tpath, "w"), 0
            written.append(tpath)
        for p, rel in items:
            tf.add(p, arcname=os.path.join(run, rel))
        cur += size
    if tf:
        tf.close()
    return written


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--repo", required=True)
    a.add_argument("--tag", required=True)
    a.add_argument("--results", required=True)
    a.add_argument("--out", required=True)
    a.add_argument("--max-gb", type=float, default=20)
    o = a.parse_args()
    os.makedirs(o.out, exist_ok=True)
    runs = sorted(d for d in os.listdir(o.results) if os.path.isfile(os.path.join(o.results, d, "result.json")))
    max_bytes = int(o.max_gb * (1 << 30))
    soft = os.path.join(o.out, f"software-{o.tag}.tar.gz")
    subprocess.run(["git", "-C", o.repo, "archive", "--format=tar.gz", "-o", soft, o.tag], check=True)
    part_a = [soft] + write_part("partA-derived", runs, o.results, o.out, False, max_bytes)
    part_b = write_part("partB-raw", runs, o.results, o.out, True, max_bytes)
    with open(os.path.join(o.out, "SHA256SUMS"), "w") as f:
        for p in part_a + part_b:
            f.write(f"{sha256(p)}  {os.path.basename(p)}\n")
    with open(os.path.join(o.out, "README.md"), "w") as f:
        f.write(f"# Fleet-DT evaluation campaign ({o.tag})\n\n"
                f"{len(runs)} runs. Part A: software of the tagged commit and the derived data of every run. "
                f"Part B: raw ground truth of the emulator and monitor streams. Formats: emu/README.md of the software.\n")
    print(f"{len(runs)} runs; part A {len(part_a)} files; part B {len(part_b)} files; out {o.out}")


if __name__ == "__main__":
    main()
