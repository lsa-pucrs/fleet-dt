#!/bin/bash
# On a Linux host with KVM (the GS PC): a Debian 13 VM whose kernel carries mac80211, for the
# full-software test of tests/hwsim. RHEL kernels ship no mac80211_hwsim, so the test runs in a VM.
#   tests/hwsim/vm.sh up [DIR]        create (once) and boot the VM; ssh on 127.0.0.1:2222
#   tests/hwsim/vm.sh ssh [command]   run a command in the VM
#   tests/hwsim/vm.sh sync            copy this repository to ~/fleet-dt in the VM
#   tests/hwsim/vm.sh down
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
D=${FDT_VM_DIR:-$HOME/.cache/fdt-vm}
QEMU=${QEMU:-qemu-system-x86_64}
SSH=(ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o UserKnownHostsFile="$D/known_hosts" -i "$D/id_ed25519" -p 2222 fdt@127.0.0.1)
case ${1:-} in
up)
	mkdir -p "$D" && cd "$D"
	[ -f debian-13-generic-amd64.qcow2 ] || curl -sfLO https://cloud.debian.org/images/cloud/trixie/latest/debian-13-generic-amd64.qcow2
	[ -f id_ed25519 ] || ssh-keygen -q -t ed25519 -N "" -f id_ed25519 -C fdt-vm
	if [ ! -f disk.qcow2 ]; then
		qemu-img create -q -f qcow2 -F qcow2 -b debian-13-generic-amd64.qcow2 disk.qcow2 30G
		mkdir -p seed
		printf 'instance-id: fdt-hwsim-1\nlocal-hostname: fdt-hwsim\n' > seed/meta-data
		printf '#cloud-config\nusers:\n  - name: fdt\n    sudo: ALL=(ALL) NOPASSWD:ALL\n    shell: /bin/bash\n    ssh_authorized_keys:\n      - %s\n' \
			"$(cat id_ed25519.pub)" > seed/user-data
		xorriso -as mkisofs -quiet -o seed.iso -V cidata -J -r seed/
	fi
	setsid nohup "$QEMU" -enable-kvm -cpu host -smp 8 -m 8192 -nographic \
		-drive file=disk.qcow2,if=virtio -drive file=seed.iso,media=cdrom \
		-netdev user,id=n0,hostfwd=tcp:127.0.0.1:2222-:22 -device virtio-net-pci,netdev=n0 \
		-serial file:serial.log -monitor none -display none -pidfile qemu.pid > qemu.log 2>&1 &
	for k in $(seq 1 60); do "${SSH[@]}" true < /dev/null 2>/dev/null && break; sleep 3; done
	"${SSH[@]}" uname -r < /dev/null ;;
ssh) shift; "${SSH[@]}" "$@" ;;
sync) tar czf - --exclude=.git --exclude=./build --exclude=./build-armhf --exclude=__pycache__ -C "$ROOT" . | "${SSH[@]}" 'mkdir -p ~/fleet-dt && tar xzf - -C ~/fleet-dt' ;;
down) "${SSH[@]}" sudo poweroff || true ;;
*) sed -n '2,8p' "$0"; exit 2 ;;
esac
