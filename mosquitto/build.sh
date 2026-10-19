#!/bin/bash
# Native build of cJSON 1.7.19 and Mosquitto 2.1.2 (+ every patch of patches/, 0001 to 0004) into PREFIX.
# Runs on the RP4 (Raspbian 10: gcc 8.3, glibc 2.28, CMake 3.16 < 3.18, so the plain
# Makefiles) and on the GS (RHEL 10). Usage:
#   build.sh <dir with mosquitto-2.1.2.tar.gz and cJSON-1.7.19.tar.gz> [PREFIX]
# cJSON: Buster ships 1.7.10, and Mosquitto 2.1 calls cJSON_ParseWithLength (>= 1.7.13).
set -euo pipefail
DIST=$(realpath "$1"); PFX=${2:-/opt/mosquitto-2.1}
HERE=$(dirname "$(realpath "$0")")
B=$(mktemp -d /tmp/fdt-mosq-build.XXXX)
tar xzf "$DIST/mosquitto-2.1.2.tar.gz" -C "$B"
tar xzf "$DIST/cJSON-1.7.19.tar.gz" -C "$B"
( cd "$B/mosquitto-2.1.2" && for p in "$HERE"/patches/*.patch; do patch -p1 --forward < "$p"; done )
make -C "$B/cJSON-1.7.19" PREFIX="$PFX" shared static >/dev/null
sudo make -C "$B/cJSON-1.7.19" PREFIX="$PFX" install-cjson >/dev/null
# a root umask of 077 (RHEL hardening) leaves PREFIX unreadable to the build user
sudo chmod -R a+rX "$PFX"
OPTS=(WITH_TLS=no WITH_TLS_PSK=no WITH_WEBSOCKETS=no WITH_DOCS=no WITH_SQLITE=no
      WITH_HTTP_API=no WITH_EDITLINE=no WITH_SYSTEMD=no WITH_SRV=no WITH_JEMALLOC=no
      WITH_LTO=no WITH_STRIP=no WITH_BUNDLED_DEPS=yes WITH_EPOLL=yes WITH_BRIDGE=yes
      WITH_PERSISTENCE=yes WITH_SHARED_LIBRARIES=yes WITH_STATIC_LIBRARIES=no
      WITH_THREADING=yes WITH_GMOCK=no DIRS="libcommon lib client src")
CPP="-I$PFX/include"; LDF="-L$PFX/lib -Wl,-rpath,$PFX/lib"
make -C "$B/mosquitto-2.1.2" -j"$(nproc)" "${OPTS[@]}" prefix="$PFX" CPPFLAGS="$CPP" LDFLAGS="$LDF" binary > "$B/build.log" 2>&1 || { tail -40 "$B/build.log"; exit 1; }
sudo make -C "$B/mosquitto-2.1.2" "${OPTS[@]}" prefix="$PFX" CPPFLAGS="$CPP" LDFLAGS="$LDF" install > "$B/install.log" 2>&1 || { tail -40 "$B/install.log"; exit 1; }
sudo chmod -R a+rX "$PFX"
"$PFX/sbin/mosquitto" -h 2>&1 | head -1
echo "build dir: $B"
