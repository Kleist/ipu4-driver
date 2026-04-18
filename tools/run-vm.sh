#!/usr/bin/env bash
# Boot the test VM with the forked QEMU and the IPU4 device model attached.
# The guest kernel and initramfs live in tools/rootfs/out/.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
QEMU_DIR="$ROOT/tools/qemu"
ROOTFS="$ROOT/tools/rootfs/out"

QEMU_BIN="${IPU4_QEMU_BIN:-$QEMU_DIR/build/qemu-system-x86_64}"
KERNEL="${IPU4_KERNEL:-$ROOTFS/bzImage}"
INITRD="${IPU4_INITRD:-$ROOTFS/rootfs.cpio.gz}"
MODULE_DIR="${IPU4_MODULE_DIR:-$ROOT/tools/linux/drivers/media/pci/intel/ipu4}"
GUEST_SCRIPT="${IPU4_GUEST_SCRIPT:-$ROOT/tools/tests/guest-streamon.sh}"

for f in "$QEMU_BIN" "$KERNEL" "$INITRD"; do
	if [[ ! -e "$f" ]]; then
		echo "missing $f; run tools/bootstrap.sh and tools/rootfs/build.sh first" >&2
		exit 1
	fi
done

APPEND="console=ttyS0 earlyprintk=serial,ttyS0 panic=-1 nokaslr"
APPEND+=" init=/init"
APPEND+=" intel_ipu4.dyndbg=+p"
APPEND+=" trace_event=ipu6:* trace_buf_size=16M"
if [[ "${IPU4_MMIOTRACE:-0}" == 1 ]]; then
	APPEND+=" mmiotrace=1"
fi

exec "$QEMU_BIN" \
	-M q35,accel=kvm:tcg \
	-cpu host \
	-m 1G \
	-nographic \
	-no-reboot \
	-kernel "$KERNEL" \
	-initrd "$INITRD" \
	-append "$APPEND" \
	-virtfs local,path="$MODULE_DIR",mount_tag=modules,security_model=none,readonly=on \
	-virtfs local,path="$ROOT/tools/tests",mount_tag=tests,security_model=none,readonly=on \
	-device ipu4 \
	-serial mon:stdio
