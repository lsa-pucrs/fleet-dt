#!/usr/bin/env python3
"""Orchestrator of the Fleet-DT v2 emulation testbed (SPEC_v2.md Section 7). Runs on the emulation host.

  emu/run.py build                         copy emu/ns3 into the ns-3 tree (scratch/fdt-emu) and build it
  emu/run.py image                         build the base image (emu/vm/build_image.sh)
  emu/run.py up --scenario S.json          VMs (GS + M vessels), emulator, monitor bridge, clock probe
  emu/run.py sync                          copy this repository into every VM (~/fleet-dt)
  emu/run.py ssh HOST [CMD...]             HOST: gs, v1..vM (interactive without CMD)
  emu/run.py run EXP.json                  one experiment; results in EMU/runs/<time>-<name>/ (or the
                                           "run_dir" of a fleet experiment, see emu/fleet.py)
  emu/run.py status
  emu/run.py down                          stop only the processes that `up` started (PID files)

  emu/run.py --slot K ...                  session K (0..9): its own VMs, ports, vsock CIDs, emulator and CPU
                                           set, so that up to 10 sessions run at once (emu/experiments/runner.py)

EMU (default ~/fleetdt-deploy/emu, env FDT_EMU) holds img/, vms/, session/, runs/ (session K >= 1:
vms-K/, session-K/).
"""
import argparse
import base64
import glob
import hashlib
import json
import math
import os
import random
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EMU = os.path.expanduser(os.environ.get("FDT_EMU", "~/fleetdt-deploy/emu"))
NS3 = os.path.expanduser(os.environ.get("FDT_NS3", "~/fleetdt-deploy/ns3/ns-3.48"))
QEMU = os.environ.get("QEMU", "/opt/qemu-upstream/bin/qemu-system-x86_64")
EMU_BIN = os.path.join(NS3, "build/scratch/fdt-emu/ns3.48-fdt-emu-optimized")
IMG = os.path.join(EMU, "img")
KEY = os.path.join(IMG, "id_ed25519")

# Session (slot) K: every resource below is offset by K, so that sessions run side by side.
SLOT = 0
SESS = os.path.join(EMU, "session")
VMS = os.path.join(EMU, "vms")
STATE = os.path.join(SESS, "state.json")
BASE_PORT = 41000          # data plane: QEMU BASE+2h, emulator BASE+2h+1 (slot K: 41000 + 100 K)
SSH_PORT = 2300            # management: ssh of host h on 127.0.0.1:SSH_PORT+h (slot K: 2300 + 20 K)
MQTT_FWD = 2883            # GS broker (1883) on 127.0.0.1:MQTT_FWD for the monitor bridge (slot K: 2883 + K)
MON_PORT = 9550            # emulator -> monitor bridge (UDP; slot K: 9550 + K)
CID_BASE = 100             # vsock CID of host h: CID_BASE + h (slot K: 100 + 20 K)
VM_PREFIX = "fdt-emu"
# CPU affinity (rootless taskset) on the i7-14700 (CPUs 0-15: 8 P-cores with 2 threads, 16-27: 12 E-cores).
# The user keeps CPUs 0-7 (4 P-cores) and 24-27 (4 E-cores). Session K runs its emulator on P-core 4+K
# (CPUs 8+2K, 9+2K: simulation and ground-truth writer); the VMs and the host-side helpers of every
# session share the E-cores 16-23. FDT_EMU_CPUS / FDT_VM_CPUS / FDT_AUX_CPUS override ("" disables pinning).
EMU_CPUS = "8,9"
VM_CPUS = "16-23"
AUX_CPUS = "16-23"         # monitor bridge, clock probe, monitor capture


def configure_slot(k):
    global SLOT, SESS, VMS, STATE, BASE_PORT, SSH_PORT, MQTT_FWD, MON_PORT, CID_BASE, VM_PREFIX
    global EMU_CPUS, VM_CPUS, AUX_CPUS
    if not 0 <= k <= 9:
        raise SystemExit("slot must be in 0..9")
    SLOT = k
    SESS = os.path.join(EMU, "session" if k == 0 else "session-%d" % k)
    VMS = os.path.join(EMU, "vms" if k == 0 else "vms-%d" % k)
    STATE = os.path.join(SESS, "state.json")
    BASE_PORT = 41000 + 100 * k
    SSH_PORT = 2300 + 20 * k
    MQTT_FWD = 2883 + k
    MON_PORT = 9550 + k
    CID_BASE = 100 + 20 * k
    VM_PREFIX = "fdt-emu" if k == 0 else "fdt-emu-s%d" % k
    EMU_CPUS = os.environ.get("FDT_EMU_CPUS", "%d,%d" % (8 + 2 * k, 9 + 2 * k) if k < 4 else "8-15")
    VM_CPUS = os.environ.get("FDT_VM_CPUS", "16-23")
    AUX_CPUS = os.environ.get("FDT_AUX_CPUS", "16-23")


configure_slot(int(os.environ.get("FDT_SLOT", "0")))


def pinned(cmd, cpus):
    return (["taskset", "-c", cpus] + cmd) if cpus else cmd


def log(*a):
    print("[run.py %s]" % time.strftime("%H:%M:%S"), *a, flush=True)


def host_name(h):
    return "gs" if h == 0 else "v%d" % h


def host_index(name):
    return 0 if name == "gs" else int(name.lstrip("v"))


def host_ip(h):
    return "10.10.0.1" if h == 0 else "10.10.0.%d" % (10 + h)


def vm_mac(h):
    return "52:54:00:fd:00:%02x" % h


def mgmt_mac(h):
    return "52:54:00:fe:00:%02x" % h


def load_state():
    try:
        with open(STATE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def save_state(st):
    os.makedirs(os.path.dirname(STATE), exist_ok=True)
    with open(STATE + ".tmp", "w") as f:
        json.dump(st, f, indent=1)
    os.replace(STATE + ".tmp", STATE)


def pid_alive(pid, marker):
    """True when pid runs and its command line holds marker (so only our own processes are signalled)."""
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            return marker.encode() in f.read()
    except OSError:
        return False


def kill_pid(pid, marker, sig=signal.SIGTERM, wait=10.0):
    if not pid or not pid_alive(pid, marker):
        return False
    os.kill(pid, sig)
    t = time.time()
    while time.time() - t < wait and pid_alive(pid, marker):
        time.sleep(0.1)
    if pid_alive(pid, marker):
        os.kill(pid, signal.SIGKILL)
    return True


# ------------------------------------------------------------------------------------------------
# ssh / rsync into the VMs

def ssh_cmd(h, tty=False):
    return ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
            "-o", "LogLevel=ERROR", "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5"] + \
           (["-t"] if tty else []) + ["-i", KEY, "-p", str(SSH_PORT + h), "fdt@127.0.0.1"]


def ssh(h, cmd, timeout=None, check=False, capture=True):
    r = subprocess.run(ssh_cmd(h) + [cmd], stdin=subprocess.DEVNULL, timeout=timeout, text=True,
                       stdout=subprocess.PIPE if capture else None, stderr=subprocess.STDOUT if capture else None)
    if check and r.returncode != 0:
        raise RuntimeError("%s: %s -> %d\n%s" % (host_name(h), cmd, r.returncode, r.stdout))
    return r


def rsync_to(h, src, dst, excludes=()):
    e = " ".join(shlex.quote(x) for x in ssh_cmd(h)[:-1])
    cmd = ["rsync", "-a", "--delete", "-e", e] + sum([["--exclude", x] for x in excludes], []) + \
          [src, "fdt@127.0.0.1:" + dst]
    subprocess.run(cmd, check=True, stdin=subprocess.DEVNULL)


def rsync_from(h, src, dst):
    e = " ".join(shlex.quote(x) for x in ssh_cmd(h)[:-1])
    os.makedirs(dst, exist_ok=True)
    return subprocess.run(["rsync", "-a", "-e", e, "fdt@127.0.0.1:" + src, dst], stdin=subprocess.DEVNULL,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def parallel(fn, items):
    out, th = {}, []

    def run(x):
        try:
            out[x] = fn(x)
        except Exception as e:      # noqa: BLE001 - reported to the caller
            out[x] = e
    for x in items:
        t = threading.Thread(target=run, args=(x,))
        t.start()
        th.append(t)
    for t in th:
        t.join()
    return out


# ------------------------------------------------------------------------------------------------
# VMs

def seed_iso(h, M, d, clock, chrony):
    os.makedirs(d, exist_ok=True)
    pub = open(KEY + ".pub").read().strip()
    hosts = {str(k): {"ip": host_ip(k), "mac": vm_mac(k)} for k in range(M + 1)}
    emu_host = {"index": h, "hosts": hosts, "prefix": 24, "subnet": "10.10.0.0/24", "mtu": 1500,
                "route_opts": "quickack 1 rto_min 20ms",
                "sysctl": {"net.ipv4.tcp_slow_start_after_idle": "0"}, "clock": clock}
    if h == 0:
        chrony_conf = ("# GS: follows the clock of the emulation host (ptp_kvm) and serves the vessels\n"
                       "refclock PHC /dev/ptp0 poll -2 dpoll -2\n"
                       "makestep 0.1 3\nallow 10.10.0.0/24\nlocal stratum 10\n")
    else:
        chrony_conf = ("# vessel: follows the GS; no makestep, so the initial offset is slewed (PR4)\n"
                       "server 10.10.0.1 iburst minpoll %d maxpoll %d\n" % (chrony["minpoll"], chrony["maxpoll"]))

    def block(s):
        return "\n".join("      " + line for line in s.rstrip("\n").split("\n"))
    ud = ("#cloud-config\nhostname: %s\nmanage_etc_hosts: true\n"
          "users:\n  - name: fdt\n    sudo: ALL=(ALL) NOPASSWD:ALL\n    shell: /bin/bash\n"
          "    ssh_authorized_keys:\n      - %s\n"
          "write_files:\n"
          "  - path: /etc/fleetdt/emu-host.json\n    content: |\n%s\n"
          "  - path: /etc/chrony/conf.d/fdt.conf\n    content: |\n%s\n") % (
        host_name(h), pub, block(json.dumps(emu_host, indent=1)), block(chrony_conf))
    nc = ("version: 2\nethernets:\n  mgmt0:\n    match:\n      macaddress: \"%s\"\n    dhcp4: true\n"
          "    dhcp6: false\n" % mgmt_mac(h))
    with open(os.path.join(d, "user-data"), "w") as f:
        f.write(ud)
    with open(os.path.join(d, "meta-data"), "w") as f:
        f.write("instance-id: fdt-emu-%s-%d\nlocal-hostname: %s\n" % (host_name(h), int(time.time()), host_name(h)))
    with open(os.path.join(d, "network-config"), "w") as f:
        f.write(nc)
    iso = os.path.join(os.path.dirname(d), "seed.iso")
    subprocess.run(["xorriso", "-as", "mkisofs", "-quiet", "-o", iso, "-V", "cidata", "-J", "-r", d], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return iso


def start_vm(h, M, clock, chrony, mem=768):
    d = os.path.join(VMS, host_name(h))
    if os.path.isdir(d):
        shutil.rmtree(d)
    os.makedirs(d)
    disk = os.path.join(d, "disk.qcow2")
    subprocess.run(["qemu-img", "create", "-q", "-f", "qcow2", "-F", "qcow2", "-b",
                    os.path.join(IMG, "base.qcow2"), disk], check=True)
    iso = seed_iso(h, M, os.path.join(d, "seed"), clock, chrony)
    fwd = "hostfwd=tcp:127.0.0.1:%d-:22" % (SSH_PORT + h)
    if h == 0:
        fwd += ",hostfwd=tcp:127.0.0.1:%d-:1883" % MQTT_FWD
    pidf = os.path.join(d, "qemu.pid")
    name = "%s-%s" % (VM_PREFIX, host_name(h))
    cmd = [QEMU, "-name", "%s,process=%s" % (name, name), "-enable-kvm", "-cpu", "host", "-smp", "1",
           "-m", str(mem), "-drive", "file=%s,if=virtio,cache=unsafe" % disk,
           "-drive", "file=%s,media=cdrom,readonly=on" % iso,
           "-netdev", "user,id=m0," + fwd, "-device", "virtio-net-pci,netdev=m0,mac=%s" % mgmt_mac(h),
           "-netdev", "dgram,id=d0,local.type=inet,local.host=127.0.0.1,local.port=%d,"
                      "remote.type=inet,remote.host=127.0.0.1,remote.port=%d" % (BASE_PORT + 2 * h,
                                                                                   BASE_PORT + 2 * h + 1),
           "-device", "virtio-net-pci,netdev=d0,mac=%s" % vm_mac(h),
           "-device", "vhost-vsock-pci,guest-cid=%d" % (CID_BASE + h),
           "-serial", "file:%s" % os.path.join(d, "serial.log"), "-monitor", "none", "-display", "none",
           "-pidfile", pidf, "-daemonize"]
    subprocess.run(pinned(cmd, VM_CPUS), check=True, stdin=subprocess.DEVNULL)
    return int(open(pidf).read().strip()), name


def wait_ssh(h, timeout=180):
    t = time.time()
    while time.time() - t < timeout:
        try:
            if ssh(h, "cloud-init status --wait >/dev/null 2>&1; systemctl is-system-running --wait >/dev/null 2>&1; "
                      "echo ok", timeout=60).stdout.strip().endswith("ok"):
                return time.time() - t
        except subprocess.TimeoutExpired:
            pass
        time.sleep(2)
    raise RuntimeError("%s: no ssh after %d s" % (host_name(h), timeout))


# ------------------------------------------------------------------------------------------------
# emulator, bridge, probe

def start_emulator(scenario, outdir, M, gt=True, rt="HardLimit", extra=(), run_start_us=None, gt_states=None):
    os.makedirs(outdir, exist_ok=True)
    out = open(os.path.join(outdir, "emu.out"), "w")
    cmd = [EMU_BIN, "--scenario=%s" % scenario, "--outdir=%s" % outdir, "--basePort=%d" % BASE_PORT,
           "--monitor=127.0.0.1:%d" % MON_PORT, "--rt=%s" % rt, "--gt=%d" % int(gt)] + list(extra)
    if run_start_us is not None:
        cmd.append("--runStartUs=%d" % run_start_us)
    if gt_states:
        cmd.append("--gtStates=%s" % gt_states)
    p = subprocess.Popen(pinned(cmd, EMU_CPUS), stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                         start_new_session=True)
    t = time.time()
    while time.time() - t < 15:
        txt = open(os.path.join(outdir, "emu.out")).read()
        if "READY" in txt:
            return p.pid
        if p.poll() is not None:
            raise RuntimeError("emulator exited:\n" + txt[-2000:])
        time.sleep(0.05)
    raise RuntimeError("emulator: stations not associated after 15 s")


def stop_emulator(pid):
    if kill_pid(pid, "fdt-emu-optimized", signal.SIGTERM, wait=30):
        log("emulator %d stopped" % pid)


def start_daemon(cmd, logpath):
    lf = open(logpath, "a")
    p = subprocess.Popen(pinned(cmd, AUX_CPUS), stdout=lf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    return p.pid


# ------------------------------------------------------------------------------------------------
# commands

def cpu_ticks(pid):
    """utime + stime of pid (clock ticks), or None"""
    try:
        with open("/proc/%d/stat" % pid) as f:
            v = f.read().rsplit(")", 1)[1].split()
        return int(v[11]) + int(v[12])
    except (OSError, IndexError, ValueError):
        return None


def cpu_snapshot(st):
    snap = {"t": time.time(), "emu": cpu_ticks(st.get("emu_pid") or 0)}
    for name, pid in st.get("vm_pids", {}).items():
        snap[name] = cpu_ticks(pid)
    with open("/proc/stat") as f:
        v = [int(x) for x in f.readline().split()[1:]]
    snap["host_busy"], snap["host_total"] = sum(v) - v[3] - v[4], sum(v)
    return snap


def cpu_report(s0, s1):
    """CPU use between two snapshots, in cores (1.0 = one core fully busy)"""
    hz, dt = os.sysconf("SC_CLK_TCK"), s1["t"] - s0["t"]
    out = {}
    for k in s0:
        if k in ("t", "host_busy", "host_total") or s0[k] is None or s1.get(k) is None:
            continue
        out[k] = round((s1[k] - s0[k]) / hz / dt, 3)
    out["host_cores_busy"] = round((s1["host_busy"] - s0["host_busy"]) / max(1, s1["host_total"] - s0["host_total"])
                                   * os.cpu_count(), 2)
    return out


def cmd_build(a):
    dst = os.path.join(NS3, "scratch", "fdt-emu")
    os.makedirs(dst, exist_ok=True)
    for f in glob.glob(os.path.join(ROOT, "emu", "ns3", "*")):
        shutil.copy(f, dst)
    subprocess.run(["./ns3", "build", "fdt-emu"], cwd=NS3, check=True)
    log("built", EMU_BIN)


def cmd_image(a):
    subprocess.run(["bash", os.path.join(ROOT, "emu", "vm", "build_image.sh"), IMG], check=True)


def cmd_up(a):
    st = load_state()
    if st and any(pid_alive(p, m) for p, m in st.get("procs", [])):
        raise SystemExit("already up (emu/run.py down first)")
    sc = json.load(open(a.scenario))
    M = int(sc["M"])
    rng = random.Random(a.clock_seed)
    clocks = {0: {"offset_ms": 0, "ppm": 0}}
    for h in range(1, M + 1):
        clocks[h] = {"offset_ms": round(rng.uniform(-50, 50), 3), "ppm": round(rng.uniform(-50, 50), 3)}
    if a.no_clock_error:
        clocks = {h: {"offset_ms": 0, "ppm": 0} for h in range(M + 1)}
    sess = SESS
    if os.path.isdir(sess):
        shutil.rmtree(sess)
    os.makedirs(sess)
    chrony = {"minpoll": a.chrony_minpoll, "maxpoll": a.chrony_maxpoll}
    st = {"M": M, "scenario": os.path.abspath(a.scenario), "clocks": clocks, "procs": [], "t_up": time.time(),
          "vm_pids": {}, "slot": SLOT, "clock_seed": a.clock_seed}
    log("starting %d VMs" % (M + 1))
    for h in range(M + 1):
        pid, name = start_vm(h, M, clocks[h], chrony, mem=a.mem)
        st["procs"].append([pid, name])
        st["vm_pids"][host_name(h)] = pid
    # the probe starts with the VMs, so clock.csv shows the convergence of chrony from the boot
    ppid = start_daemon([sys.executable, os.path.join(ROOT, "emu", "clockprobe", "probe.py"),
                         "--cids", ",".join(str(CID_BASE + h) for h in range(M + 1)),
                         "--names", ",".join(host_name(h) for h in range(M + 1)),
                         "--out", os.path.join(sess, "clock.csv")], os.path.join(sess, "probe.log"))
    st["probe_pid"] = ppid
    st["procs"].append([ppid, "probe.py"])
    save_state(st)
    epid = start_emulator(st["scenario"], os.path.join(sess, "emu-idle"), M, gt=False, rt=a.rt, gt_states="none")
    st["emu_pid"] = epid
    st["procs"].append([epid, "fdt-emu-optimized"])
    save_state(st)
    log("emulator %d up; waiting for the VMs" % epid)
    t = parallel(lambda h: wait_ssh(h), range(M + 1))
    bad = {k: v for k, v in t.items() if isinstance(v, Exception)}
    if bad:
        raise SystemExit("VMs not reachable: %s" % bad)
    log("VMs up (boot %s s)" % {host_name(h): round(v, 1) for h, v in t.items()})
    bpid = start_daemon([sys.executable, os.path.join(ROOT, "emu", "monitor_bridge.py"),
                         "--listen", "127.0.0.1:%d" % MON_PORT, "--broker", "127.0.0.1:%d" % MQTT_FWD,
                         "--stats", os.path.join(sess, "bridge-stats.jsonl")], os.path.join(sess, "bridge.log"))
    st["bridge_pid"] = bpid
    st["procs"] += [[bpid, "monitor_bridge.py"]]
    save_state(st)
    # data plane check: the GS pings every vessel through the emulated channel
    for h in range(1, M + 1):
        r = ssh(0, "ping -c 3 -i 0.2 -W 1 %s | tail -2" % host_ip(h), timeout=20)
        log("ping gs -> %s: %s" % (host_name(h), " ".join(r.stdout.split("\n")[-2:]).strip()))
    if a.sync:
        cmd_sync(a)
    log("up: M=%d, state %s" % (M, STATE))


def cmd_sync(a):
    st = load_state()
    M = st["M"]
    ex = [".git", "build", "build-armhf", "__pycache__", "*.pyc", "dta", ".pytest_cache"]
    r = parallel(lambda h: rsync_to(h, ROOT + "/", "fleet-dt/", ex), range(M + 1))
    bad = {k: v for k, v in r.items() if isinstance(v, Exception)}
    if bad:
        raise SystemExit("sync failed: %s" % bad)
    log("repository copied to ~/fleet-dt of %d VMs" % (M + 1))


def cmd_ssh(a):
    h = host_index(a.host)
    if a.remote:
        r = subprocess.run(ssh_cmd(h) + [" ".join(a.remote)])
        sys.exit(r.returncode)
    os.execvp("ssh", ssh_cmd(h, tty=True))


def hosts_of(spec, M):
    if spec == "all":
        return list(range(M + 1))
    if spec == "vessels":
        return list(range(1, M + 1))
    return [host_index(x) for x in spec.split(",")]


def fmt(cmd, h, M, extra):
    v = {"i": h, "name": host_name(h), "ip": host_ip(h), "gs_ip": host_ip(0), "M": M, "run": "/tmp/fdt-run",
         "repo": "~/fleet-dt", "vessels": " ".join(str(k) for k in range(1, M + 1)),
         "vessel_ips": " ".join(host_ip(k) for k in range(1, M + 1))}
    v.update(extra)
    return cmd.format(**v)


def cmd_run(a):
    st = load_state()
    if not st:
        raise SystemExit("not up")
    M = st["M"]
    exp = json.load(open(a.exp))
    if exp.get("kind") == "fleet":
        import fleet
        res = fleet.run_fleet(sys.modules[__name__], exp, a)
        if res["status"] != "ok":
            sys.exit(1)
        return
    name = exp.get("name", os.path.splitext(os.path.basename(a.exp))[0])
    rd = os.path.join(EMU, "runs", time.strftime("%Y%m%d-%H%M%S") + "-" + name)
    os.makedirs(rd)
    shutil.copy(a.exp, os.path.join(rd, "experiment.json"))
    scen = os.path.join(ROOT, exp["scenario"]) if exp.get("scenario") else st["scenario"]
    sc = json.load(open(scen))
    if int(sc["M"]) != M:
        raise SystemExit("scenario M=%d but the session has M=%d" % (sc["M"], M))
    shutil.copy(scen, os.path.join(rd, "scenario.json"))
    extra = exp.get("vars", {})
    res = {"name": name, "M": M, "t_start": time.time(), "steps": []}
    if exp.get("sync", True):
        cmd_sync(a)
    parallel(lambda h: ssh(h, "rm -rf /tmp/fdt-run && mkdir -p /tmp/fdt-run"), range(M + 1))

    def run_list(lst, tag):
        for k, s in enumerate(lst):
            hs = hosts_of(s.get("host", "all"), M)

            def one(h):
                t0 = time.time()
                r = ssh(h, fmt(s["cmd"], h, M, extra), timeout=s.get("timeout", 600))
                with open(os.path.join(rd, "%s-%02d-%s.log" % (tag, k, host_name(h))), "w") as f:
                    f.write(r.stdout)
                return {"host": host_name(h), "rc": r.returncode, "s": round(time.time() - t0, 2)}
            out = parallel(one, hs)
            res["steps"].append({"tag": tag, "k": k, "cmd": s["cmd"], "out": [str(v) if isinstance(v, Exception)
                                                                             else v for v in out.values()]})
            for v in out.values():
                if isinstance(v, Exception) or (v["rc"] != 0 and not s.get("may_fail")):
                    log("%s step %d failed: %s" % (tag, k, v))
                    if s.get("required", tag == "setup"):
                        raise SystemExit("%s step %d failed (%s)" % (tag, k, rd))

    try:
        run_list(exp.get("setup", []), "setup")
        # emulator of this run (ground truth on)
        stop_emulator(st.get("emu_pid"))
        epid = start_emulator(scen, os.path.join(rd, "emu"), M, gt=True, rt=exp.get("rt", "HardLimit"),
                              extra=exp.get("emu_args", []))
        st["emu_pid"] = epid
        st["procs"].append([epid, "fdt-emu-optimized"])
        save_state(st)
        res["t_emu"] = time.time()
        cpu0 = cpu_snapshot(st)
        # host-side copy of fleet/monitor as the GS broker delivers it (once the experiment starts it)
        cap = start_daemon([sys.executable, os.path.join(ROOT, "emu", "tools", "mon_capture.py"), "--broker",
                            "127.0.0.1:%d" % MQTT_FWD, "--out", os.path.join(rd, "monitor.jsonl")],
                           os.path.join(rd, "mon_capture.log"))
        for k, s in enumerate(exp.get("start", [])):
            for h in hosts_of(s.get("host", "all"), M):
                lg = s.get("log", "start%02d" % k)
                c = fmt(s["cmd"], h, M, extra)
                # the background job is the setsid command itself, so ssh returns at once
                ssh(h, "cd ~; setsid nohup bash -c %s > /tmp/fdt-run/%s.log 2>&1 < /dev/null & echo $! > "
                       "/tmp/fdt-run/%s.pid" % (shlex.quote(c), lg, lg), timeout=30)
            time.sleep(s.get("sleep", 0.5))
        t0 = time.time()
        steps = sorted(exp.get("steps", []), key=lambda s: s.get("at", 0))
        for k, s in enumerate(steps):
            dt = t0 + s.get("at", 0) - time.time()
            if dt > 0:
                time.sleep(dt)
            log("step %d: %s on %s" % (k, s["cmd"][:60], s.get("host", "all")))
            run_list([s], "step%02d" % k)
        dt = t0 + exp.get("wait_s", 0) - time.time()
        if dt > 0:
            log("waiting %.0f s" % dt)
            time.sleep(dt)
    finally:
        res["t_end"] = time.time()
        if "t_emu" in res:
            res["cpu_cores"] = cpu_report(cpu0, cpu_snapshot(st))
        run_list(exp.get("stop", []), "stop")
        parallel(lambda h: ssh(h, "for f in /tmp/fdt-run/*.pid; do [ -f $f ] && kill -TERM -- -$(cat $f) "
                                  "2>/dev/null; kill $(cat $f) 2>/dev/null; done; true", timeout=30), range(M + 1))
        time.sleep(1)
        if "t_emu" in res:
            kill_pid(cap, "mon_capture.py")
        stop_emulator(st.get("emu_pid"))
        for c in exp.get("collect", [{"host": "all", "path": "/tmp/fdt-run/"}]):
            for h in hosts_of(c.get("host", "all"), M):
                rsync_from(h, c["path"], os.path.join(rd, "vms", host_name(h)) + "/")
        # clock probe and bridge statistics of the run interval
        slice_csv(os.path.join(SESS, "clock.csv"), os.path.join(rd, "clock.csv"), res["t_start"],
                  res["t_end"])
        slice_jsonl(os.path.join(SESS, "bridge-stats.jsonl"), os.path.join(rd, "bridge-stats.jsonl"),
                    res["t_start"], res["t_end"])
        with open(os.path.join(rd, "result.json"), "w") as f:
            json.dump(res, f, indent=1)
        epid = start_emulator(st["scenario"], os.path.join(SESS, "emu-idle"), M, gt=False)
        st["emu_pid"] = epid
        st["procs"].append([epid, "fdt-emu-optimized"])
        save_state(st)
    log("run directory:", rd)
    print(rd)


def slice_csv(src, dst, t0, t1):
    try:
        with open(src) as f, open(dst, "w") as g:
            g.write(f.readline())
            for line in f:
                t = int(line.split(",", 1)[0]) / 1e9
                if t0 <= t <= t1:
                    g.write(line)
    except OSError:
        pass


def slice_jsonl(src, dst, t0, t1):
    try:
        with open(src) as f, open(dst, "w") as g:
            for line in f:
                try:
                    if t0 <= json.loads(line)["t"] <= t1:
                        g.write(line)
                except ValueError:
                    pass
    except OSError:
        pass


def cmd_status(a):
    st = load_state()
    if not st:
        print("not up")
        return
    for pid, marker in st["procs"]:
        if pid_alive(pid, marker):
            print("%-8d %s" % (pid, marker))
    print("M=%d scenario=%s clocks=%s" % (st["M"], st["scenario"], st["clocks"]))


def cmd_down(a):
    st = load_state()
    if not st:
        print("not up")
        return
    for pid, marker in reversed(st["procs"]):
        if kill_pid(pid, marker, wait=30 if "fdt-emu-optimized" in marker else 10):
            log("stopped %d (%s)" % (pid, marker))
    st["procs"] = []
    save_state(st)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--slot", type=int, default=None, help="session 0..9 (default: env FDT_SLOT or 0)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("build")
    sub.add_parser("image")
    u = sub.add_parser("up")
    u.add_argument("--scenario", required=True)
    u.add_argument("--clock-seed", type=int, default=1)
    u.add_argument("--no-clock-error", action="store_true")
    u.add_argument("--chrony-minpoll", type=int, default=0)
    u.add_argument("--chrony-maxpoll", type=int, default=2)
    u.add_argument("--mem", type=int, default=768)
    u.add_argument("--rt", default="HardLimit")
    u.add_argument("--sync", action="store_true")
    sub.add_parser("sync")
    s = sub.add_parser("ssh")
    s.add_argument("host")
    s.add_argument("remote", nargs=argparse.REMAINDER)
    r = sub.add_parser("run")
    r.add_argument("exp")
    sub.add_parser("status")
    sub.add_parser("down")
    a = ap.parse_args()
    if a.slot is not None:
        configure_slot(a.slot)
    {"build": cmd_build, "image": cmd_image, "up": cmd_up, "sync": cmd_sync, "ssh": cmd_ssh, "run": cmd_run,
     "status": cmd_status, "down": cmd_down}[a.cmd](a)


if __name__ == "__main__":
    main()
