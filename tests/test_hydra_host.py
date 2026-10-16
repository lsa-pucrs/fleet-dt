"""Rootless hosts without the tools of the i7-14700 host (hydra: Rocky Linux 8, QEMU 6.2 of RHEL, cgroup v1,
no xorriso, /dev/vhost-vsock root-only): the data NIC on -netdev socket (UDP), the clock probe over a
virtio-serial port, the NoCloud seed without xorriso, the emulator CPUs per slot, the user-space CPU cap
(emu/vm/cpucap.c, cpu_profile.py mechanism "user") and the VE watch of a paper-stack run."""
import io
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "emu"))
sys.path.insert(0, os.path.join(ROOT, "emu", "vm"))
sys.path.insert(0, os.path.join(ROOT, "emu", "clockprobe"))

import cpu_profile  # noqa: E402
import run as runpy_  # noqa: E402
import seed  # noqa: E402


def test_qemu_socket_netdev_and_serial_clock(monkeypatch):
    monkeypatch.setenv("FDT_QEMU_NETDEV", "socket")
    monkeypatch.setenv("FDT_CLOCK_CHANNEL", "serial")
    cmd = runpy_.qemu_cmd(2, "fdt-emu-v2", "/d.qcow2", "/s.iso", "hostfwd=x", "/q.pid")
    nd = [cmd[i + 1] for i, x in enumerate(cmd) if x == "-netdev"]
    local, remote = runpy_.BASE_PORT + 4, runpy_.BASE_PORT + 5
    assert nd[1] == "socket,id=d0,udp=127.0.0.1:%d,localaddr=127.0.0.1:%d" % (remote, local)
    assert not any("vhost-vsock" in x for x in cmd)
    j = " ".join(cmd)
    assert "virtserialport,bus=vser0.0,chardev=clk0,name=fdt.clock" in j
    assert "socket,id=clk0,path=%s,server=on,wait=off" % runpy_.clock_sock(2) in j
    monkeypatch.setenv("FDT_QEMU_NETDEV", "dgram")
    monkeypatch.setenv("FDT_CLOCK_CHANNEL", "vsock")
    cmd = runpy_.qemu_cmd(2, "fdt-emu-v2", "/d.qcow2", "/s.iso", "hostfwd=x", "/q.pid")
    assert any(x.startswith("dgram,id=d0,local.type=inet") for x in cmd)
    assert "vhost-vsock-pci,guest-cid=%d" % (runpy_.CID_BASE + 2) in cmd


def test_emulator_cpus_per_slot(monkeypatch):
    monkeypatch.delenv("FDT_EMU_CPUS", raising=False)
    monkeypatch.setenv("FDT_EMU_CPUS_SLOTS", "40,96; 41,97;42,98")
    assert [runpy_.slot_emu_cpus(k) for k in range(3)] == ["40,96", "41,97", "42,98"]
    with pytest.raises(SystemExit):
        runpy_.slot_emu_cpus(3)
    monkeypatch.setenv("FDT_EMU_CPUS", "7")
    assert runpy_.slot_emu_cpus(2) == "7"
    monkeypatch.delenv("FDT_EMU_CPUS")
    monkeypatch.delenv("FDT_EMU_CPUS_SLOTS")
    assert runpy_.slot_emu_cpus(1) == "10,11" and runpy_.slot_emu_cpus(5) == "8-15"


def test_seed_without_xorriso(tmp_path):
    pytest.importorskip("pycdlib")
    d = tmp_path / "seed"
    d.mkdir()
    (d / "user-data").write_text("#cloud-config\nhostname: v1\n")
    (d / "meta-data").write_text("instance-id: x\n")
    (d / "network-config").write_text("version: 2\n")
    out = str(tmp_path / "seed.iso")
    assert seed.make_seed(str(d), out, prefer="python") == "pycdlib"
    import pycdlib
    iso = pycdlib.PyCdlib()
    iso.open(out)
    assert iso.pvd.volume_identifier.decode().strip() == "cidata"
    got = {}
    for child in iso.list_children(rr_path="/"):
        if child.is_file():
            b = io.BytesIO()
            name = child.rock_ridge.name().decode()
            iso.get_file_from_iso_fp(b, rr_path="/" + name)
            got[name] = b.getvalue().decode()
    iso.close()
    assert got == {"user-data": "#cloud-config\nhostname: v1\n", "meta-data": "instance-id: x\n",
                   "network-config": "version: 2\n"}


def test_probe_over_unix_socket(tmp_path):
    """the probe reads the clock of a VM through the unix socket of its virtio-serial chardev"""
    path = str(tmp_path / "clock.sock")
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(path)
    srv.listen(1)

    def serve():
        c, _ = srv.accept()
        import struct
        while True:
            b = c.recv(1)
            if not b:
                return
            c.sendall(struct.pack("<q", time.clock_gettime_ns(time.CLOCK_REALTIME) + 5000000))
    threading.Thread(target=serve, daemon=True).start()
    out = str(tmp_path / "clock.csv")
    p = subprocess.Popen([sys.executable, os.path.join(ROOT, "emu", "clockprobe", "probe.py"), "--socks", path,
                          "--names", "v1", "--out", out, "--period", "0.05"])
    try:
        for _ in range(100):
            if os.path.exists(out) and len(open(out).read().splitlines()) >= 4:
                break
            time.sleep(0.05)
    finally:
        p.terminate()
        p.wait()
    rows = [line.split(",") for line in open(out).read().splitlines()[1:]]
    assert rows and all(r[1] == "v1" for r in rows)
    assert all(4.0e6 < int(r[2]) < 6.0e6 for r in rows)     # +5 ms within the round trip


def test_cpu_cap_profile_and_status(tmp_path, monkeypatch):
    monkeypatch.setenv("FDT_CPU_CAP", "user")
    assert cpu_profile.mechanism() == "user"
    c = cpu_profile.cap_cmd("/b/fdt_cpucap", [11, 12], 0.25, stats="/s.json")
    assert c == ["/b/fdt_cpucap", "--quota-us", "1000", "--period-us", "4000", "--poll-us", "100", "--stats", "/s.json",
                 "11", "12"]
    st = tmp_path / "cpucap.json"
    st.write_text(json.dumps({"quota_us": 1000, "period_us": 4000, "poll_us": 100, "source": "perf",
                              "procs": {"11": {"nr_throttled": 7, "throttled_usec": 21000, "ratio": 0.25}}}))
    s = cpu_profile.status_user(str(st), 11)
    assert s["nr_throttled"] == 7 and s["mechanism"] == "user" and s["period_us"] == 4000 and s["pid"] == 11
    assert cpu_profile.status_user(str(st), 13) is None
    assert cpu_profile.status_user(str(tmp_path / "none.json"), 11) is None
    # the QEMU of a vessel is not wrapped in a systemd scope with the user cap
    cmd = ["qemu", "-x"]
    assert runpy_.vm_launch_cmd(1, cmd, 0.25, "user") == runpy_.pinned(cmd, runpy_.VM_CPUS)


def test_fleet_cpu_status_reads_the_user_cap(tmp_path):
    import fleet
    st = tmp_path / "cpucap.json"
    st.write_text(json.dumps({"quota_us": 1000, "period_us": 4000, "poll_us": 100, "source": "perf",
                              "procs": {"21": {"nr_throttled": 3, "throttled_usec": 9000}}}))
    out = fleet.cpu_status({"vm_pids": {"v1": 21}, "cpu_cap": {"mechanism": "user", "stats": str(st)}})
    assert out["v1"]["nr_throttled"] == 3 and out["v1"]["mechanism"] == "user"


@pytest.mark.skipif(not shutil.which("cc"), reason="needs a C compiler")
def test_fdt_cpucap_caps_a_busy_process(tmp_path):
    """fdt_cpucap keeps a CPU-bound process near quota / period (perf task-clock; clock source fallback)"""
    b = str(tmp_path / "fdt_cpucap")
    subprocess.run(["cc", "-O2", "-o", b, os.path.join(ROOT, "emu", "vm", "cpucap.c")], check=True)
    busy = subprocess.Popen([sys.executable, "-c", "while True: pass"])
    stats = str(tmp_path / "s.json")
    try:
        cap = subprocess.Popen([b, "--quota-us", "2000", "--period-us", "8000", "--stats", stats, str(busy.pid)],
                               stderr=subprocess.PIPE)
        time.sleep(0.2)
        if cap.poll() is not None:     # no perf counter for this user: the CPU clock
            cap = subprocess.Popen([b, "--source", "clock", "--quota-us", "2000", "--period-us", "8000", "--stats",
                                    stats, str(busy.pid)])
        time.sleep(3.0)
        cap.terminate()
        cap.wait(timeout=5)
    finally:
        busy.kill()
        busy.wait()
    d = json.load(open(stats))
    p = d["procs"][str(busy.pid)]
    assert p["nr_throttled"] > 100
    assert 0.15 < p["ratio"] < (0.40 if d["source"] == "perf" else 0.55)


def test_paper_run_fails_when_the_ve_stops(tmp_path):
    """the run watches the VE collector as well as the AS (a VE without aiohttp died unnoticed)"""
    sys.path.insert(0, os.path.join(ROOT, "tests"))
    import fleet
    from test_paper_hooks import session, spec, run
    R, calls = session(tmp_path)
    ssh0 = R.ssh

    def ssh(h, cmd, timeout=None, **kw):
        r = ssh0(h, cmd, timeout, **kw)
        if "kill -0" in cmd and "/ve.pid" in cmd:
            r.stdout = "dead\n"
        return r
    R.ssh = ssh
    res = run(R, spec(tmp_path, lead_s=2.5, steps=40))
    assert res["status"] == "failed" and "VE stopped" in res["error"]
