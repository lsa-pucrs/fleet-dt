#!/bin/bash
# On the emulation host: build the base image of the emulated hosts (SPEC_v2 Section 7, Hosts).
# Copies the Debian 13 cloud image (never modifies it), boots it once with a NoCloud seed, runs
# emu/vm/provision-base.sh inside, and keeps the result as IMG/base.qcow2 (standalone qcow2).
#   emu/vm/build_image.sh [IMG_DIR]       default ~/fleetdt-deploy/emu/img
# Env: CLOUD_IMG (default ~/.cache/fdt-vm/debian-13-generic-amd64.qcow2), DIST (dir with the upstream
#      tarballs of emu/vm/dist.sha256, default ~/fleetdt-deploy/emu/dist), QEMU (default /opt/qemu-upstream/bin/qemu-system-x86_64).
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
IMG=${1:-$HOME/fleetdt-deploy/emu/img}
CLOUD_IMG=${CLOUD_IMG:-$HOME/.cache/fdt-vm/debian-13-generic-amd64.qcow2}
DIST=${DIST:-$HOME/fleetdt-deploy/emu/dist}
QEMU=${QEMU:-/opt/qemu-upstream/bin/qemu-system-x86_64}
PORT=${BUILD_SSH_PORT:-2299}
mkdir -p "$IMG" && cd "$IMG"
for f in mosquitto-2.1.2.tar.gz cJSON-1.7.19.tar.gz; do
	[ -f "$DIST/$f" ] || { echo "missing $DIST/$f"; exit 1; }
	( cd "$DIST" && grep " $f\$" "$ROOT/emu/vm/dist.sha256" | sha256sum -c --quiet ) || { echo "checksum of $f differs"; exit 1; }
done
[ -f id_ed25519 ] || ssh-keygen -q -t ed25519 -N "" -f id_ed25519 -C fdt-emu
rm -f builder.qcow2 builder.pid
cp --reflink=auto "$CLOUD_IMG" builder.qcow2
qemu-img resize -q builder.qcow2 8G
mkdir -p seed-builder
printf 'instance-id: fdt-emu-builder-%s\nlocal-hostname: fdt-builder\n' "$(date +%s)" > seed-builder/meta-data
printf '#cloud-config\nusers:\n  - name: fdt\n    sudo: ALL=(ALL) NOPASSWD:ALL\n    shell: /bin/bash\n    ssh_authorized_keys:\n      - %s\n' \
	"$(cat id_ed25519.pub)" > seed-builder/user-data
xorriso -as mkisofs -quiet -o seed-builder.iso -V cidata -J -r seed-builder/
"$QEMU" -name fdt-emu-builder,process=fdt-emu-builder -enable-kvm -cpu host -smp 4 -m 4096 \
	-drive file=builder.qcow2,if=virtio -drive file=seed-builder.iso,media=cdrom,readonly=on \
	-netdev user,id=n0,hostfwd=tcp:127.0.0.1:$PORT-:22 -device virtio-net-pci,netdev=n0 \
	-serial file:builder-serial.log -monitor none -display none -pidfile builder.pid -daemonize
SSH=(ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -i id_ed25519 -p "$PORT" fdt@127.0.0.1)
for k in $(seq 1 100); do "${SSH[@]}" true < /dev/null 2>/dev/null && break; sleep 3; done
"${SSH[@]}" 'cloud-init status --wait > /dev/null; mkdir -p ~/dist ~/src/emu/vm' < /dev/null
scp -q -o BatchMode=yes -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -i id_ed25519 -P "$PORT" \
	"$DIST/mosquitto-2.1.2.tar.gz" "$DIST/cJSON-1.7.19.tar.gz" fdt@127.0.0.1:dist/
tar cf - -C "$ROOT" mosquitto emu/vm/guest emu/vm/provision-base.sh | "${SSH[@]}" 'tar xf - -C ~/src'
"${SSH[@]}" 'bash ~/src/emu/vm/provision-base.sh ~/dist ~/src && rm -rf ~/dist ~/src ~/.bash_history && sudo poweroff' < /dev/null || true
pid=$(cat builder.pid 2>/dev/null || true)
for k in $(seq 1 60); do [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null || break; sleep 1; done
[ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && { echo "builder VM did not power off"; exit 1; }
qemu-img convert -O qcow2 builder.qcow2 base.qcow2.new && mv base.qcow2.new base.qcow2 && rm -f builder.qcow2
qemu-img info base.qcow2 | grep -E 'virtual size|disk size'
echo "base image: $IMG/base.qcow2"
