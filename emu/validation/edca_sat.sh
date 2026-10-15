#!/bin/bash
# Saturated uplink of n = NMIN..NMAX vessels through fdt-emu itself (its EDCA AC_BE parameters, QoS data
# frames, the AP of the scenario), not in real time (--rt=none --vms=0), 1500-byte IP packets; writes
# n,Mbps (IP bytes of the acknowledged data frames, from the ground truth) to OUT. Compare with
#   python3 emu/validation/eta.py edca --meas OUT
#   emu/validation/edca_sat.sh [OUT=edca_sat.csv] [SCENARIO=emu/scenarios/base-m2.json] [NMIN=1] [NMAX=16] [DUR=12]
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT=$(realpath -m "${1:-edca_sat.csv}"); SC=$(realpath "${2:-$ROOT/emu/scenarios/base-m2.json}")
NMIN=${3:-1}; NMAX=${4:-16}; DUR=${5:-12}
NS3=${FDT_NS3:-$HOME/fleetdt-deploy/ns3/ns-3.48}
BIN="$NS3/build/scratch/fdt-emu/ns3.48-fdt-emu-optimized"
W=$(mktemp -d /tmp/fdt-edca.XXXX)
echo "n,Mbps" > "$OUT"
for n in $(seq "$NMIN" "$NMAX"); do
	python3 -c "import json,sys; s=json.load(open('$SC')); s['M']=$n; json.dump(s, open('$W/s.json','w'))"
	"$BIN" --scenario="$W/s.json" --outdir="$W/o$n" --vms=0 --rt=none --monitor=none --gtStates=none \
		--selftest=30e6,1500 --duration="$DUR" > "$W/o$n.log" 2>&1
	v=$(python3 "$ROOT/emu/gt.py" throughput "$W/o$n/gt.bin" --from 2 --to "$DUR" | awk '/^total/{print $2}')
	echo "$n,$v" | tee -a "$OUT"
done
rm -rf "$W"
