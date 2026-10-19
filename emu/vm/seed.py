#!/usr/bin/env python3
"""NoCloud seed image of cloud-init (volume label cidata) from a directory with user-data, meta-data and
optionally network-config.

  seed.py DIR OUT.iso

Uses xorriso (or genisoimage, mkisofs) when the host has one; otherwise writes the ISO 9660 image with
Joliet and Rock Ridge names in Python (pycdlib), so that a host without root and without these tools
(hydra: Rocky Linux 8) can build the seeds.
"""
import os
import shutil
import subprocess
import sys

FILES = ("user-data", "meta-data", "network-config", "vendor-data")


def tool():
    for t in ("xorriso", "genisoimage", "mkisofs"):
        p = shutil.which(t)
        if p:
            return t, p
    return None, None


def make_seed(d, out, prefer=None):
    """writes the seed image OUT from the files of directory d; returns the method used"""
    t, p = (None, None) if prefer == "python" else tool()
    if t == "xorriso":
        subprocess.run([p, "-as", "mkisofs", "-quiet", "-o", out, "-V", "cidata", "-J", "-r", d], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return t
    if t:
        subprocess.run([p, "-quiet", "-o", out, "-V", "cidata", "-J", "-r", d], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return t
    import pycdlib
    iso = pycdlib.PyCdlib()
    iso.new(interchange_level=3, joliet=3, rock_ridge="1.09", vol_ident="cidata")
    for k, name in enumerate(sorted(f for f in os.listdir(d) if os.path.isfile(os.path.join(d, f)))):
        with open(os.path.join(d, name), "rb") as f:
            data = f.read()
        import io
        iso.add_fp(io.BytesIO(data), len(data), "/F%d.;1" % k, rr_name=name, joliet_path="/" + name)
    tmp = out + ".tmp"
    iso.write(tmp)
    iso.close()
    os.replace(tmp, out)
    return "pycdlib"


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: seed.py DIR OUT.iso")
    print(make_seed(sys.argv[1], sys.argv[2]))


if __name__ == "__main__":
    main()
