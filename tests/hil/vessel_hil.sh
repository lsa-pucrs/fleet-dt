#!/bin/bash
# Vessel side of a hardware-in-the-loop run on the RP4 of a vessel, beside its running services:
# a second broker on PORT/PORT+1 with the vessel regulator (mode paper, demand of fdt_dti), the DTI
# program under SCHED_FIFO, the telemetry publisher on a MAVProxy output, and the layered video source
# (fdt_source.py, frame sizes of TRACE) for the topics without a source.
#   tests/hil/vessel_hil.sh start VESSEL DTP OUTDIR [PORT=11883] [MAVLINK=udpout:127.0.0.1:14551] [LOAD_TOPICS=video layers]
#   tests/hil/vessel_hil.sh stop OUTDIR
# MAV_ARGS in the environment adds options of mav_publisher.py (e.g. "--state-offset 0.0925"); JITTER_MS
# (default 2) is the jitter allowance of the regulator and of fdt_dti; TRACE the frame-size trace.
# The vessel programs read OUTDIR/dtp_vessel.json, the DTP of this vessel (video_layers.py resolve).
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
case ${1:-} in
start)
	V=$2; DTP=$(realpath "$3"); OUT=$4; PORT=${5:-11883}; MAV=${6:-udpout:127.0.0.1:14551}
	J=${JITTER_MS:-2}; TRACE=${TRACE:-$ROOT/emu/traces/water-sbs-q50-s0p5.txt}
	mkdir -p "$OUT"
	cd "$ROOT"
	VDTP="$OUT/dtp_vessel.json"
	python3 vessel/publishers/video_layers.py resolve --dtp "$DTP" --vessel "$V" --out "$VDTP" || exit 1
	# default load: the video layer topics of the DTP
	LOAD=${7:-$(python3 -c 'import json, sys; print(",".join(t["name"] for t in json.load(open(sys.argv[1]))["topics"] if t["name"].startswith("video_")))' "$VDTP")}
	python3 -m fleetdt.gsconf vessel --vessel "$V" --dtp "$VDTP" --plugin "$ROOT/build/fdt_regulator.so" \
		--port "$PORT" --data-port $((PORT + 1)) --mode paper --demand external --jitter-ms "$J" \
		--reg-log "$OUT/regulator.jsonl" > "$OUT/vessel.conf"
	echo "log_dest stdout" >> "$OUT/vessel.conf"
	setsid /opt/mosquitto-2.1/sbin/mosquitto -c "$OUT/vessel.conf" > "$OUT/broker.log" 2>&1 & echo $! > "$OUT/broker.pid"
	sleep 1
	sudo -n setsid "$ROOT/build/fdt_dti" --dtp "$VDTP" --vessel "$V" --broker 127.0.0.1:$PORT --prio 50 \
		--jitter-ms "$J" > "$OUT/dti.log" 2>&1 &
	setsid python3 vessel/publishers/mav_publisher.py --dtp "$VDTP" --vessel "$V" --port "$PORT" --mavlink "$MAV" ${MAV_ARGS:-} \
		> "$OUT/mav.log" 2>&1 & echo $! > "$OUT/mav.pid"
	if [ -n "$LOAD" ]; then
		setsid python3 vessel/publishers/fdt_source.py --dtp "$VDTP" --vessel "$V" --port "$PORT" --topics "$LOAD" \
			--size-trace "$TRACE" --log "$OUT/src.jsonl" > "$OUT/load.log" 2>&1 & echo $! > "$OUT/load.pid"
	fi
	sleep 1; echo started ;;
stop)
	OUT=$2
	for p in load mav broker; do
		[ -f "$OUT/$p.pid" ] && kill -TERM "$(cat "$OUT/$p.pid")" 2>/dev/null
	done
	sudo -n pkill -TERM -f "$ROOT/build/fdt_dti" 2>/dev/null
	sleep 2; echo stopped ;;
*) sed -n '2,10p' "$0"; exit 2 ;;
esac
