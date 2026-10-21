#!/usr/bin/env python3
"""Data NIC of an emulated Fleet-DT host (SPEC_v2 Section 7).

Reads /etc/fleetdt/emu-host.json (written by cloud-init), finds the data NIC by its MAC, sets the
static address, one permanent neighbor entry per other host (the emulator relays IPv4 unicast only
and drops ARP), the subnet route with the TCP options of SPEC_v2 Section 5, and the sysctls.

  fdt-netsetup.py                              boot (fdt-netsetup.service): address, neighbors, route, sysctls
  fdt-netsetup.py --tcp-only [--quickack 0|1] [--rto-min-ms MS] [--slow-start-after-idle 0|1]
                                               only the subnet route and the TCP sysctl, for one run of the
                                               paper stack (emu/fleet.py "guest_tcp"; ablation R8 of E3:
                                               --quickack 0, delayed acknowledgments of Linux); without
                                               the options, the values of emu-host.json (quickack 1,
                                               rto_min 20ms, tcp_slow_start_after_idle 0)
"""
import argparse
import json
import os
import subprocess
import sys
import time

CONF = "/etc/fleetdt/emu-host.json"


def sh(*args, check=True):
    r = subprocess.run(list(args), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if check and r.returncode != 0:
        sys.stderr.write("%s: %s\n" % (" ".join(args), r.stdout))
        raise SystemExit(1)
    return r.stdout


def iface_by_mac(mac):
    for _ in range(100):
        for name in os.listdir("/sys/class/net"):
            try:
                with open("/sys/class/net/%s/address" % name) as f:
                    if f.read().strip().lower() == mac.lower():
                        return name
            except OSError:
                pass
        time.sleep(0.1)
    raise SystemExit("no interface with MAC %s" % mac)


def route_opts(c, quickack=None, rto_min_ms=None):
    """route attributes of the subnet route: those of emu-host.json "route_opts" (default quickack 1 rto_min
    20ms), with quickack (0/1: attribute present or absent) and rto_min (ms; 0: absent) replaced when given"""
    words = c.get("route_opts", "quickack 1 rto_min 20ms").split()
    if quickack is None and rto_min_ms is None:
        return words
    opts = dict(zip(words[0::2], words[1::2]))
    if quickack is not None:
        opts.pop("quickack", None)
        if int(quickack):
            opts["quickack"] = "1"
    if rto_min_ms is not None:
        opts.pop("rto_min", None)
        if float(rto_min_ms) > 0:
            opts["rto_min"] = "%gms" % float(rto_min_ms)
    return [x for kv in opts.items() for x in kv]


def sysctls(c, slow_start_after_idle=None):
    out = dict(c.get("sysctl", {"net.ipv4.tcp_slow_start_after_idle": "0"}))
    if slow_start_after_idle is not None:
        out["net.ipv4.tcp_slow_start_after_idle"] = str(int(slow_start_after_idle))
    return out


def tcp_only(c, quickack=None, rto_min_ms=None, slow_start_after_idle=None):
    me = c["hosts"][str(c["index"])]
    dev = iface_by_mac(me["mac"])
    net = c.get("subnet", "10.10.0.0/24")
    ro = route_opts(c, quickack, rto_min_ms)
    sh("ip", "route", "replace", net, "dev", dev, "proto", "kernel", "scope", "link", "src", me["ip"], *ro)
    for k, v in sysctls(c, slow_start_after_idle).items():
        sh("sysctl", "-q", "-w", "%s=%s" % (k, v))
    print("fdt-netsetup: route %s %s, sysctl %s" % (net, " ".join(ro), sysctls(c, slow_start_after_idle)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--tcp-only", action="store_true", help="set only the subnet route and the TCP sysctl")
    ap.add_argument("--quickack", type=int, choices=(0, 1), default=None)
    ap.add_argument("--rto-min-ms", type=float, default=None)
    ap.add_argument("--slow-start-after-idle", type=int, choices=(0, 1), default=None)
    ap.add_argument("--conf", default=CONF)
    a = ap.parse_args(argv)
    with open(a.conf) as f:
        c = json.load(f)
    if a.tcp_only:
        tcp_only(c, a.quickack, a.rto_min_ms, a.slow_start_after_idle)
        return
    h = str(c["index"])
    me = c["hosts"][h]
    dev = iface_by_mac(me["mac"])
    prefix = c.get("prefix", 24)
    sh("ip", "link", "set", "dev", dev, "mtu", str(c.get("mtu", 1500)), "up")
    sh("ip", "addr", "flush", "dev", dev)
    sh("ip", "addr", "add", "%s/%d" % (me["ip"], prefix), "dev", dev)
    for k, peer in c["hosts"].items():
        if k != h:
            sh("ip", "neigh", "replace", peer["ip"], "lladdr", peer["mac"], "dev", dev, "nud", "permanent")
    net = c.get("subnet", "10.10.0.0/24")
    sh("ip", "route", "replace", net, "dev", dev, "proto", "kernel", "scope", "link", "src", me["ip"],
       *route_opts(c))
    for k, v in sysctls(c).items():
        sh("sysctl", "-q", "-w", "%s=%s" % (k, v))
    # offloads that the dgram backend cannot carry are already off; keep GRO off so that tcpdump in
    # the VM shows the frames of the emulated channel
    sh("ethtool", "-K", dev, "gro", "off", check=False)
    print("fdt-netsetup: %s %s/%d, %d neighbors" % (dev, me["ip"], prefix, len(c["hosts"]) - 1))


if __name__ == "__main__":
    main()
