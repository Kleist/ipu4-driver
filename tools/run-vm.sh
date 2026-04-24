#!/usr/bin/env bash
# Boot the test VM with the forked QEMU and the IPU4 device model attached.
# Reads bzImage / rootfs.cpio.gz produced by tools/build-kernel.sh and
# tools/rootfs/build.sh.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
QEMU_DIR="$ROOT/tools/qemu"
LINUX_DIR="$ROOT/tools/linux"

QEMU_BIN="${IPU4_QEMU_BIN:-$QEMU_DIR/build/qemu-system-x86_64}"
KERNEL="${IPU4_KERNEL:-$LINUX_DIR/arch/x86/boot/bzImage}"
INITRD="${IPU4_INITRD:-$ROOT/tools/rootfs/out/rootfs.cpio.gz}"
SHARE_DIR="${IPU4_TESTS_SHARE:-$ROOT/tools/tests/out}"
mkdir -p "$SHARE_DIR"

for f in "$QEMU_BIN" "$KERNEL" "$INITRD"; do
	if [[ ! -e "$f" ]]; then
		echo "missing $f" >&2
		echo "run: tools/bootstrap.sh + tools/build-qemu.sh +" >&2
		echo "     tools/build-kernel.sh + tools/rootfs/build.sh" >&2
		exit 1
	fi
done

APPEND="console=ttyS0 earlyprintk=serial,ttyS0 panic=-1 oops=panic nokaslr"
APPEND+=" loglevel=7 rdinit=/init"
APPEND+=" intel_ipu4.dyndbg=+p intel_ipu4_isys.dyndbg=+p"
if [[ "${IPU4_MMIOTRACE:-0}" == 1 ]]; then
	# Pre-size the ftrace ring buffer so a full probe-capture fits
	# without wrapping. The mmiotrace tracer itself is enabled by the
	# guest init (tools/rootfs/init.mmiotrace) because current_tracer
	# must be set before the driver's ioremap runs, not at boot.
	APPEND+=" trace_buf_size=16M"
fi

ACCEL="${IPU4_ACCEL:-}"
if [[ -z "$ACCEL" ]]; then
	if [[ -r /dev/kvm && -w /dev/kvm ]]; then
		ACCEL="kvm:tcg"
	else
		ACCEL="tcg"
	fi
fi

exec "$QEMU_BIN" \
	-M q35,accel="$ACCEL" \
	-cpu max \
	-m 1G \
	-nographic \
	-no-reboot \
	-kernel "$KERNEL" \
	-initrd "$INITRD" \
	-append "$APPEND" \
	-virtfs "local,path=$SHARE_DIR,mount_tag=tests,security_model=none" \
	-device ipu4 \
	-serial mon:stdio
