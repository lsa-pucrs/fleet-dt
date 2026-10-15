#!/usr/bin/env python3
"""Data NIC of an emulated Fleet-DT host (SPEC_v2 Section 7).

Reads /etc/fleetdt/emu-host.json (written by cloud-init), finds the data NIC by its MAC, sets the
static address, one permanent neighbor entry per other host (the emulator relays IPv4 unicast only
and drops ARP), the subnet route with the TCP options of SPEC_v2 Section 5, and the sysctls.
"""
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


def main():
    with open(CONF) as f:
        c = json.load(f)
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
       *c.get("route_opts", "quickack 1 rto_min 20ms").split())
    for k, v in c.get("sysctl", {"net.ipv4.tcp_slow_start_after_idle": "0"}).items():
        sh("sysctl", "-q", "-w", "%s=%s" % (k, v))
    # offloads that the dgram backend cannot carry are already off; keep GRO off so that tcpdump in
    # the VM shows the frames of the emulated channel
    sh("ethtool", "-K", dev, "gro", "off", check=False)
    print("fdt-netsetup: %s %s/%d, %d neighbors" % (dev, me["ip"], prefix, len(c["hosts"]) - 1))


if __name__ == "__main__":
    main()
