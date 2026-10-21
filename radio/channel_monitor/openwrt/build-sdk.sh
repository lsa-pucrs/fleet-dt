#!/bin/bash
# Builds the fdt-chmon package with the OpenWrt SDK container of the Bullet M2 target (ath79,
# mips_24kc, musl) and copies the .ipk to OUT.
#   radio/channel_monitor/openwrt/build-sdk.sh [OUT] [SDK image]
# default image: docker.io/openwrt/sdk:ath79-tiny-22.03.7
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
OUT=$(realpath -m "${1:-$ROOT/build/openwrt}")
IMG=${2:-docker.io/openwrt/sdk:ath79-tiny-22.03.7}
SRC=$(mktemp -d /tmp/fdt-owrt-src.XXXX)
trap 'rm -rf "$SRC"' EXIT
mkdir -p "$OUT"
cp -r "$ROOT/common" "$ROOT/radio" "$ROOT/dtp.json" "$SRC/"
chmod -R a+rX "$SRC" && chmod a+rwx "$OUT"
podman run --rm -v "$SRC:/src:ro" -v "$OUT:/out" --security-opt label=disable "$IMG" bash -c '
set -e
./scripts/feeds update base packages >/dev/null
./scripts/feeds install mosquitto cJSON >/dev/null
mkdir -p package/fdt-chmon && cp /src/radio/channel_monitor/openwrt/Makefile package/fdt-chmon/
make defconfig >/dev/null
make package/fdt-chmon/compile FDT_SRC=/src -j"$(nproc)" V=s > /out/build.log 2>&1 || { tail -40 /out/build.log; exit 1; }
cp $(find bin -name "fdt-chmon_*.ipk") /out/
'
ls -l "$OUT"/*.ipk
