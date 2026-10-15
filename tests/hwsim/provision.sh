#!/bin/bash
# Inside the VM (Debian 13, user with sudo): packages, patched Mosquitto, the C programs, the
# patched mac80211_hwsim of the running kernel and the patched wmediumd.
#   tests/hwsim/provision.sh <dir with mosquitto-2.1.2.tar.gz and cJSON-1.7.19.tar.gz>
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
HERE=$ROOT/tests/hwsim
DIST=$(realpath "$1")
W=${FDT_HWSIM_WORK:-$HOME/fdt-hwsim}
sudo apt-get update -q > /dev/null
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q build-essential "linux-headers-$(uname -r)" linux-source \
	hostapd wpasupplicant iw kmod git python3-paho-mqtt python3-numpy python3-scipy python3-aiohttp python3-pytest \
	libnl-3-dev libnl-genl-3-dev libconfig-dev > /dev/null
[ -x /opt/mosquitto-2.1/sbin/mosquitto ] || bash "$ROOT/mosquitto/build.sh" "$DIST" /opt/mosquitto-2.1
make -C "$ROOT" -s
mkdir -p "$W/hwsim"
src=$(ls /usr/src/linux-source-*.tar.xz | sort -V | tail -1)
tar xf "$src" -C "$W/hwsim" --wildcards '*/drivers/net/wireless/virtual/mac80211_hwsim.*' --strip-components=5
python3 "$HERE/patch_hwsim.py" "$W/hwsim/mac80211_hwsim.c"
printf 'obj-m += mac80211_hwsim.o\n' > "$W/hwsim/Kbuild"
make -s -C "/lib/modules/$(uname -r)/build" M="$W/hwsim" modules
[ -d "$W/wmediumd" ] || git clone -q https://github.com/bcopeland/wmediumd "$W/wmediumd"
git -C "$W/wmediumd" checkout -q 717e5d7 -- wmediumd/wmediumd.c
python3 "$HERE/patch_wmediumd.py" "$W/wmediumd/wmediumd/wmediumd.c"
make -s -C "$W/wmediumd" > /dev/null 2>&1
ls -l "$W/hwsim/mac80211_hwsim.ko" "$W/wmediumd/wmediumd/wmediumd"
