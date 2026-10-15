#!/bin/sh
# Fleet-DT interference injector (Section VI-B): a Bullet M2 with OpenWrt built with
# CONFIG_ATH9K_TX99 (ath9k TX99 mode) transmits a continuous carrier-modulated frame
# stream on the channel of the fleet, so the GS radio sees a busy channel and C(t) drops.
#
#   tx99.sh on    [-f FREQ_MHZ] [-p POWER_HALF_DBM]          start and keep transmitting
#   tx99.sh off                                              stop
#   tx99.sh burst [-f F] [-p P] -n ON_S -m OFF_S [-c CYCLES] [-a START_UNIX]
#                 periodic bursts: ON_S seconds on, OFF_S seconds off, CYCLES times
#                 (0: until killed), starting at the Unix time START_UNIX (default now)
#   tx99.sh plan  [-f F] [-p P] -s FILE -a START_UNIX
#                 bursts of a plan file: one line "OFFSET_S DURATION_S [POWER]" per burst,
#                 OFFSET_S counted from START_UNIX; the same file drives every run of a
#                 condition, so the runs see the same interference
#
# POWER_HALF_DBM is the tx99_power of ath9k, in 0.5 dBm (0..63). The clock of the radio
# follows the GS through sysntpd, so START_UNIX can be an instant of the operation
# (for example epoch + t*Ts of a step). Needs busybox sleep with fractional seconds.
set -eu

PHY=${PHY:-phy0}
MON=${MON:-mon0}
DBG=/sys/kernel/debug/ieee80211/$PHY/ath9k
FREQ=2437
POWER=40
ON=1; OFF=1; CYCLES=0; START=""; PLAN=""

log() { logger -t fdt-tx99 "$*"; echo "fdt-tx99: $*" >&2; }

now() { date +%s; }

setup() {
	[ -w "$DBG/tx99" ] || { log "no $DBG/tx99: build OpenWrt with CONFIG_ATH9K_TX99 and mount debugfs"; exit 1; }
	# TX99 needs a single monitor interface on the radio
	wifi down 2>/dev/null || true
	iw dev "$MON" del 2>/dev/null || true
	iw phy "$PHY" interface add "$MON" type monitor
	ip link set "$MON" up
	iw dev "$MON" set freq "$FREQ" HT20
}

tx_on() { echo "${1:-$POWER}" > "$DBG/tx99_power"; echo 1 > "$DBG/tx99"; }
tx_off() { echo 0 > "$DBG/tx99" 2>/dev/null || true; }

wait_until() {   # $1: Unix time; busybox date gives whole seconds, so the start is within 1 s
	d=$(awk -v a="$1" -v n="$(now)" 'BEGIN { x = a - n; if (x < 0) x = 0; printf "%.3f", x }')
	sleep "$d"
}

cmd=${1:-}; [ -n "$cmd" ] && shift || true
while getopts "f:p:n:m:c:a:s:" o; do
	case $o in
	f) FREQ=$OPTARG ;; p) POWER=$OPTARG ;; n) ON=$OPTARG ;; m) OFF=$OPTARG ;;
	c) CYCLES=$OPTARG ;; a) START=$OPTARG ;; s) PLAN=$OPTARG ;;
	*) exit 2 ;;
	esac
done
trap 'tx_off; log stopped; exit 0' INT TERM

case $cmd in
on)
	setup; tx_on; log "on: $FREQ MHz, power $POWER (0.5 dBm)" ;;
off)
	tx_off; iw dev "$MON" del 2>/dev/null || true; log off ;;
burst)
	setup
	[ -n "$START" ] && wait_until "$START"
	k=0
	while [ "$CYCLES" -eq 0 ] || [ "$k" -lt "$CYCLES" ]; do
		tx_on; log "burst $k on"; sleep "$ON"
		tx_off; sleep "$OFF"
		k=$((k + 1))
	done ;;
plan)
	[ -n "$PLAN" ] && [ -n "$START" ] || { log "plan needs -s FILE and -a START_UNIX"; exit 2; }
	setup
	grep -v '^[[:space:]]*#' "$PLAN" | while read -r off dur pw; do
		[ -n "$off" ] || continue
		wait_until "$(awk -v s="$START" -v o="$off" 'BEGIN { printf "%.3f", s + o }')"
		tx_on "${pw:-$POWER}"; log "plan burst at +$off s for $dur s"
		sleep "$dur"; tx_off
	done ;;
*)
	sed -n '2,25p' "$0"; exit 2 ;;
esac
