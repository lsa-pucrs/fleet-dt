#!/bin/sh
# Builds fdt-sim in the ns-3 tree NS3 (default ~/fleetdt-deploy/emu/sim-ns3/ns-3.48): copies emu/sim and the
# headers of the regulator into scratch/fdt-sim, builds libfdtreg.a (fdt_reg.c, fdt_dtp2.c, fdt_common.c,
# cJSON) and the scratch program.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
NS3=${NS3:-$HOME/fleetdt-deploy/emu/sim-ns3/ns-3.48}
DST=$NS3/scratch/fdt-sim
mkdir -p "$DST/cjson"
cp "$HERE/fdt-sim.cc" "$HERE/CMakeLists.txt" "$ROOT/emu/ns3/minijson.h" "$DST/"
cp "$ROOT/common/fdt_common.h" "$ROOT/common/fdt_dtp2.h" "$ROOT/vessel/regulator/fdt_reg.h" "$DST/"
cp "$HERE/third_party/cjson/cJSON.h" "$DST/cjson/"
OBJ=$(mktemp -d)
for f in "$ROOT/vessel/regulator/fdt_reg.c" "$ROOT/common/fdt_dtp2.c" "$ROOT/common/fdt_common.c" "$HERE/third_party/cjson/cJSON.c"; do
  gcc -O2 -fPIC -std=gnu11 -I"$DST" -I"$ROOT/common" -I"$ROOT/vessel/regulator" -c "$f" -o "$OBJ/$(basename "$f" .c).o"
done
rm -f "$DST/libfdtreg.a"
ar rcs "$DST/libfdtreg.a" "$OBJ"/*.o
rm -rf "$OBJ"
cd "$NS3"
./ns3 build -j 2 fdt-sim
ls -l "$NS3/build/scratch/fdt-sim/"
