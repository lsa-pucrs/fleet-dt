#!/bin/sh
# Builds the channel monitor for the emulation host, without MQTT (FDT_NO_MQTT): build/fdt_chmon_host.
# fleetdt/monitor_adapter.py (backend c) feeds it the counter samples of the emulator through the
# stream backend (fdt_chmon --replay-stream -) and publishes the y(t) lines it prints.
# Needs a C compiler and cJSON (libcjson-dev). -ffp-contract=off keeps the arithmetic of compute()
# free of fused multiply-adds, so the Python port gives the same doubles.
#
#   radio/channel_monitor/build_host.sh [OUT]      (default OUT: build/fdt_chmon_host)
set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT=${1:-$ROOT/build/fdt_chmon_host}
mkdir -p "$(dirname "$OUT")"
${CC:-cc} -O2 -g -Wall -Wextra -Wno-unused-parameter -std=gnu11 -ffp-contract=off -DFDT_NO_MQTT \
	-I"$ROOT/common" ${CJSON_CFLAGS:-} -o "$OUT" \
	"$ROOT/radio/channel_monitor/fdt_chmon.c" "$ROOT/common/fdt_common.c" ${CJSON_LIBS:--lcjson} -lm
echo "$OUT"
