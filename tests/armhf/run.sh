#!/bin/bash
# Builds the vessel programs for an emulated Raspbian 10 (armhf) and runs the end-to-end test with
# the vessel brokers, DTI programs and synthetic load inside the container and the GS programs on
# the host. Needs podman, qemu-arm in binfmt_misc, and the tarballs of mosquitto/build.sh.
#   tests/armhf/run.sh <dir with mosquitto-2.1.2.tar.gz and cJSON-1.7.19.tar.gz>
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
DIST=$(realpath "$1")
IMG=localhost/fleetdt-rpi10:2
NAME=fdt-rpi10-$$
CTX=$(mktemp -d /tmp/fdt-armhf.XXXX)
trap 'podman rm -f "$NAME" >/dev/null 2>&1 || true; rm -rf "$CTX"' EXIT
if ! podman image exists "$IMG"; then
	cp -r "$ROOT/mosquitto" "$CTX/mosquitto"
	mkdir "$CTX/dist" && cp "$DIST/mosquitto-2.1.2.tar.gz" "$DIST/cJSON-1.7.19.tar.gz" "$CTX/dist/"
	podman build --platform linux/arm/v7 -t "$IMG" -f "$ROOT/tests/armhf/Containerfile" "$CTX"
fi
podman run -d --rm --name "$NAME" --platform linux/arm/v7 --network host --userns keep-id \
	--security-opt label=disable \
	-v "$ROOT:$ROOT" -v /tmp:/tmp -w "$ROOT" "$IMG" sleep infinity >/dev/null
podman exec "$NAME" bash -c "gcc --version | head -1; python3 --version; ldd --version | head -1"
podman exec "$NAME" bash -c 'make B=build-armhf MOSQ=/opt/mosquitto-2.1 CFLAGS="$CFLAGS -Wall -Wextra -Wno-unused-parameter -std=gnu11"' 
# no dependency on libatomic, which the armv6 baseline of Raspbian would need for 64-bit atomics
podman exec "$NAME" bash -c "! readelf -d build-armhf/fdt_dti build-armhf/fdt_chmon build-armhf/fdt_regulator.so | grep libatomic"
podman exec "$NAME" build-armhf/test_reg
FDT_VESSEL_EXEC="podman exec $NAME" FDT_VESSEL_BUILD="$ROOT/build-armhf" FDT_VESSEL_MOSQ=/opt/mosquitto-2.1 \
	FDT_VESSEL_PYTHON=python3 python3 -m pytest -q -p no:cacheprovider tests/test_integration.py "${@:2}"
