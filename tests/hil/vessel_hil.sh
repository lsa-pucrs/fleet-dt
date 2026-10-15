#!/bin/bash
# Vessel side of a hardware-in-the-loop run on the RP4 of a vessel, beside its running services:
# a second broker on PORT/PORT+1 with the vessel regulator, the DTI program under SCHED_FIFO, the
# telemetry publisher on a MAVProxy output, and the synthetic load for the topics without a source.
#   tests/hil/vessel_hil.sh start VESSEL DTP OUTDIR [PORT=11883] [MAVLINK=udpout:127.0.0.1:14551] [LOAD_TOPICS=video]
#   tests/hil/vessel_hil.sh stop OUTDIR
# MAV_ARGS in the environment adds options of mav_publisher.py (e.g. "--state-offset 0.0925").
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
case ${1:-} in
start)
	V=$2; DTP=$(realpath "$3"); OUT=$4; PORT=${5:-11883}; MAV=${6:-udpout:127.0.0.1:14551}; LOAD=${7:-video}
	mkdir -p "$OUT"
	cd "$ROOT"
	python3 -m fleetdt.gsconf vessel --vessel "$V" --dtp "$DTP" --plugin "$ROOT/build/fdt_regulator.so" \
		--port "$PORT" --data-port $((PORT + 1)) > "$OUT/vessel.conf"
	echo "log_dest stdout" >> "$OUT/vessel.conf"
	setsid /opt/mosquitto-2.1/sbin/mosquitto -c "$OUT/vessel.conf" > "$OUT/broker.log" 2>&1 & echo $! > "$OUT/broker.pid"
	sleep 1
	sudo -n setsid "$ROOT/build/fdt_dti" --dtp "$DTP" --vessel "$V" --broker 127.0.0.1:$PORT --prio 50 > "$OUT/dti.log" 2>&1 &
	setsid python3 vessel/publishers/mav_publisher.py --dtp "$DTP" --vessel "$V" --port "$PORT" --mavlink "$MAV" ${MAV_ARGS:-} \
		> "$OUT/mav.log" 2>&1 & echo $! > "$OUT/mav.pid"
	if [ -n "$LOAD" ]; then
		setsid python3 vessel/publishers/raw_load.py --dtp "$DTP" --vessel "$V" --port "$PORT" --topics "$LOAD" \
			> "$OUT/load.log" 2>&1 & echo $! > "$OUT/load.pid"
	fi
	sleep 1; echo started ;;
stop)
	OUT=$2
	for p in load mav broker; do
		[ -f "$OUT/$p.pid" ] && kill -TERM "$(cat "$OUT/$p.pid")" 2>/dev/null
	done
	sudo -n pkill -TERM -f "$ROOT/build/fdt_dti" 2>/dev/null
	sleep 2; echo stopped ;;
*) sed -n '2,7p' "$0"; exit 2 ;;
esac
