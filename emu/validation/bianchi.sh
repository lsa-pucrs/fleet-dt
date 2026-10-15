#!/bin/bash
# Bianchi validation of the ns-3 802.11g model at the PHY rate of the emulator (paper Section VI).
# Builds a copy of src/wifi/examples/wifi-bianchi.cc in which a number of stations without a value in the
# reference tables (n < 5 or n not a multiple of 5) is a warning instead of a fatal error, runs it for
# n = NMIN..NMAX saturated stations (infrastructure, DCF, 1500-byte packets every 400 us per station, so
# that one station also saturates) and writes n,Mbps to OUT.
#   emu/validation/bianchi.sh [OUT=bianchi.csv] [RATE=ErpOfdmRate24Mbps] [NMIN=1] [NMAX=16] [DURATION=20]
set -euo pipefail
OUT=$(realpath -m "${1:-bianchi.csv}"); RATE=${2:-ErpOfdmRate24Mbps}; NMIN=${3:-1}; NMAX=${4:-16}; DUR=${5:-20}
NS3=${FDT_NS3:-$HOME/fleetdt-deploy/ns3/ns-3.48}
mkdir -p "$NS3/scratch/fdt-bianchi"
sed -e 's/NS_FATAL_ERROR("No Bianchi results (DIFS) calculated for that number of stations!");/std::cout << "no DIFS reference for n" << std::endl;/' \
    -e 's/NS_FATAL_ERROR("No Bianchi results (EIFS) calculated for that number of stations!");/std::cout << "no EIFS reference for n" << std::endl;/' \
    "$NS3/src/wifi/examples/wifi-bianchi.cc" > "$NS3/scratch/fdt-bianchi/fdt-bianchi.cc"
( cd "$NS3" && ./ns3 build fdt-bianchi > /dev/null )
BIN="$NS3/build/scratch/fdt-bianchi/ns3.48-fdt-bianchi-optimized"
W=$(mktemp -d /tmp/fdt-bianchi.XXXX)
echo "n,Mbps" > "$OUT"
for n in $(seq "$NMIN" "$NMAX"); do
	( cd "$W" && "$BIN" --standard=11g --frequency=2.4 --phyMode="$RATE" --nMinStas="$n" --nMaxStas="$n" \
		--nStepSize=1 --duration="$DUR" --plotBianchiModel=0 --infra=1 --pktInterval=400 --workDir="$W/" > "$W/out-$n.txt" 2>&1 )
	v=$(grep "Total throughput" "$W/out-$n.txt" | tail -1 | awk '{print $3}')
	echo "$n,$v" | tee -a "$OUT"
done
rm -rf "$W"
