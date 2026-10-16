#!/bin/bash
# Inside the builder VM (Debian 13, user fdt with sudo): packages of the Fleet-DT stack, patched
# Mosquitto 2.1.2 + cJSON 1.7.19 (mosquitto/build.sh), the guest services of the emulator, IPv6 off.
#   provision-base.sh <dir with mosquitto-2.1.2.tar.gz, cJSON-1.7.19.tar.gz> <dir with mosquitto/ and emu/vm/guest/>
set -euo pipefail
DIST=$(realpath "$1"); SRC=$(realpath "$2")
export DEBIAN_FRONTEND=noninteractive
sudo -E apt-get update -q > /dev/null
echo "iperf3 iperf3/start_daemon boolean false" | sudo debconf-set-selections
sudo -E apt-get install -y -q --no-install-recommends build-essential pkg-config python3 python3-paho-mqtt \
	python3-numpy chrony nftables iproute2 iperf3 rsync ethtool tcpdump iputils-ping procps jq \
	linux-perf ca-certificates > /dev/null
sudo -E apt-get purge -y -q unattended-upgrades systemd-timesyncd > /dev/null 2>&1 || true
sudo systemctl disable --now apt-daily.timer apt-daily-upgrade.timer man-db.timer e2scrub_all.timer fstrim.timer 2>/dev/null || true
[ -x /opt/mosquitto-2.1/sbin/mosquitto ] || bash "$SRC/mosquitto/build.sh" "$DIST" /opt/mosquitto-2.1
for b in /opt/mosquitto-2.1/bin/* /opt/mosquitto-2.1/sbin/*; do sudo ln -sf "$b" /usr/local/bin/; done
# record the patches of this build, so that a run can check that the guests run exactly the repository patches
( cd "$SRC/mosquitto/patches" && sha256sum *.patch ) | sudo tee /opt/mosquitto-2.1/patches.sha256 > /dev/null
sudo install -d /usr/local/lib/fdt /etc/fleetdt /etc/chrony/conf.d
sudo install -m 755 "$SRC"/emu/vm/guest/*.py /usr/local/lib/fdt/
sudo install -m 644 "$SRC"/emu/vm/guest/*.service /etc/systemd/system/
sudo install -m 644 "$SRC/emu/vm/guest/chrony.conf" /etc/chrony/chrony.conf
sudo rm -f /etc/chrony/sources.d/* /etc/chrony/conf.d/*
# oscillator error below chronyd (design item I9): preloaded shim + injector (idle without a schedule)
sudo cc -O2 -Wall -shared -fPIC -o /usr/local/lib/fdt/fdt-freqshim.so "$SRC/emu/vm/guest/fdt-freqshim.c" -ldl
# the AppArmor profile of chronyd (Debian) must allow the preloaded library and the shared page
if [ -d /etc/apparmor.d/local ]; then
	printf '%s\n' '/usr/local/lib/fdt/fdt-freqshim.so mr,' '/run/fdt-clock/ rw,' '/run/fdt-clock/freq.shm rw,' \
		| sudo tee /etc/apparmor.d/local/usr.sbin.chronyd > /dev/null
fi
sudo install -d /etc/systemd/system/chrony.service.d
sudo install -m 644 "$SRC/emu/vm/guest/chrony-freqshim.conf" /etc/systemd/system/chrony.service.d/fdt-freqshim.conf
sudo systemctl daemon-reload
sudo systemctl enable fdt-netsetup.service fdt-clocksource.service fdt-clockinit.service fdt-clockerr.service \
	fdt-clockd.service chrony.service
echo ptp_kvm | sudo tee /etc/modules-load.d/ptp_kvm.conf > /dev/null
# IPv6 off (SPEC_v2 Section 7), quiet console, no predictable-name surprises are needed (MAC lookup)
# clock source TSC instead of kvm-clock (design item I9; checked at boot by fdt-clocksource.service)
echo 'GRUB_CMDLINE_LINUX_DEFAULT="$GRUB_CMDLINE_LINUX_DEFAULT ipv6.disable=1 clocksource=tsc tsc=reliable"' | sudo tee /etc/default/grub.d/90-fdt.cfg > /dev/null
sudo update-grub > /dev/null 2>&1
# the mosquitto service of the distribution is not installed (source build); the stack starts its brokers
sudo sed -i 's/^#\?MaxRetentionSec=.*/MaxRetentionSec=1week/' /etc/systemd/journald.conf
{ /opt/mosquitto-2.1/sbin/mosquitto -h 2>&1 || true; } | head -1
python3 -c "import paho.mqtt; print('paho-mqtt', paho.mqtt.__version__)"
iperf3 --version | head -1
/usr/sbin/chronyd --version
ldd /usr/local/lib/fdt/fdt-freqshim.so > /dev/null && echo "fdt-freqshim.so: ok"
sudo cloud-init clean --logs --machine-id --seed > /dev/null
sudo rm -f /etc/ssh/ssh_host_*
echo provision-base: done
