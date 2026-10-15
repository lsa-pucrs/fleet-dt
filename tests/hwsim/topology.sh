#!/bin/bash
# Simulated Wi-Fi network of the fleet (run as root in a VM whose kernel has mac80211):
# 4 radios of mac80211_hwsim (patched by patch_hwsim.py), the medium of wmediumd, and one network
# namespace per node.
#
#   namespace  radio  role                               address
#   gs         wlan0  GS radio, access point (hostapd)   10.0.0.10
#   b1         wlan1  radio of vessel 1, station         10.0.0.99
#   b2         wlan2  radio of vessel 2, station         10.0.0.98
#   inj        wlan3  interference injector, monitor     none
#
# Channel 6 (2437 MHz), no HT, no WMM, one legacy rate of 24 Mbit/s (R_PHY of dtp.json).
#   topology.sh up <patched mac80211_hwsim.ko> <wmediumd binary>
#   topology.sh down
set -euo pipefail
WORK=/run/fdt-hwsim
NS=(gs b1 b2 inj)
IP=(10.0.0.10 10.0.0.99 10.0.0.98 "")

down() {
	pkill -x hostapd 2>/dev/null || true
	pkill -x wpa_supplicant 2>/dev/null || true
	pkill -x wmediumd 2>/dev/null || true
	for n in "${NS[@]}"; do ip netns del "$n" 2>/dev/null || true; done
	sleep 0.5
	rmmod mac80211_hwsim 2>/dev/null || true
}

up() {
	local ko=$1 wmd=$2
	down
	mkdir -p $WORK
	modprobe mac80211
	insmod "$ko" radios=4
	sleep 0.5
	# radio k: phy and MAC address assigned by hwsim (02:00:00:00:0k:00)
	local phys=() macs=()
	for k in 0 1 2 3; do
		local dev
		dev=$(ls /sys/class/ieee80211/ | sort -V | sed -n "$((k + 1))p")
		phys+=("$dev")
		macs+=("$(printf '02:00:00:00:%02x:00' "$k")")
	done
	cat > $WORK/wmediumd.cfg <<CFG
ifaces : {
	ids = [ "${macs[0]}", "${macs[1]}", "${macs[2]}", "${macs[3]}" ];
};
model : {
	type = "snr";
	links = ( (0, 1, 40), (0, 2, 40), (0, 3, 40), (1, 2, 40), (1, 3, 40), (2, 3, 40) );
};
CFG
	setsid "$wmd" -c $WORK/wmediumd.cfg > $WORK/wmediumd.log 2>&1 &
	sleep 1
	for k in 0 1 2 3; do
		ip netns add "${NS[$k]}"
		iw phy "${phys[$k]}" set netns name "${NS[$k]}"
		ip -n "${NS[$k]}" link set lo up
	done
	# GS radio: access point
	local w0
	w0=$(ip netns exec gs iw dev | awk '$1 == "Interface" { print $2; exit }')
	cat > $WORK/hostapd.conf <<CFG
interface=$w0
driver=nl80211
ssid=fleet-dt
hw_mode=g
channel=6
ieee80211n=0
wmm_enabled=0
supported_rates=240
basic_rates=240
beacon_int=100
CFG
	ip netns exec gs hostapd -B -P $WORK/hostapd.pid $WORK/hostapd.conf > $WORK/hostapd.log 2>&1
	ip -n gs addr add 10.0.0.10/24 dev "$w0"
	ip -n gs link set "$w0" up
	# vessel radios: stations
	cat > $WORK/sta.conf <<CFG
ctrl_interface=$WORK/wpa
network={
	ssid="fleet-dt"
	key_mgmt=NONE
	scan_freq=2437
}
CFG
	for k in 1 2; do
		local n=${NS[$k]} w
		w=$(ip netns exec "$n" iw dev | awk '$1 == "Interface" { print $2; exit }')
		ip -n "$n" link set "$w" up
		ip netns exec "$n" wpa_supplicant -B -i "$w" -c $WORK/sta.conf -P $WORK/wpa-$n.pid > $WORK/wpa-$n.log 2>&1
		ip -n "$n" addr add "${IP[$k]}/24" dev "$w"
	done
	# injector: monitor interface on the channel of the fleet
	local w3
	w3=$(ip netns exec inj iw dev | awk '$1 == "Interface" { print $2; exit }')
	ip netns exec inj iw dev "$w3" set type monitor
	ip -n inj link set "$w3" up
	ip netns exec inj iw dev "$w3" set channel 6
	# association, then the fixed rate on every radio
	for t in $(seq 1 50); do
		ip netns exec b1 iw dev | grep -q ssid && ip netns exec b2 iw dev | grep -q ssid && break
		sleep 0.2
	done
	for k in 0 1 2; do
		local n=${NS[$k]} w
		w=$(ip netns exec "$n" iw dev | awk '$1 == "Interface" { print $2; exit }')
		ip netns exec "$n" iw dev "$w" set bitrates legacy-2.4 24
		echo "$n $w ${IP[$k]}"
	done
	echo "inj $w3"
	ip netns exec b1 ping -c 2 -W 1 10.0.0.10 > /dev/null && ip netns exec b2 ping -c 2 -W 1 10.0.0.10 > /dev/null
	echo "topology up"
}

case ${1:-} in
up) up "$2" "$3" ;;
down) down ;;
*) sed -n '2,15p' "$0"; exit 2 ;;
esac
