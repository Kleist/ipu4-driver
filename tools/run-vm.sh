#!/usr/bin/env bash
# Boot the test VM with the forked QEMU and the IPU4 device model attached.
# Reads bzImage / rootfs.cpio.gz produced by tools/build-kernel.sh and
# tools/rootfs/build.sh. IPU4_FLAVOR=ipu6 picks the ipu6-flavor kernel
# (tools/linux-ipu6/) and initramfs (tools/rootfs/out-ipu6/).
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
QEMU_DIR="$ROOT/tools/qemu"
case "${IPU4_FLAVOR:-fork}" in
fork)
	LINUX_DIR="$ROOT/tools/linux"
	ROOTFS_OUT="$ROOT/tools/rootfs/out"
	DYNDBG="intel_ipu4.dyndbg=+p intel_ipu4_isys.dyndbg=+p"
	;;
ipu6)
	LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu6}"
	ROOTFS_OUT="$ROOT/tools/rootfs/out-ipu6"
	DYNDBG="intel_ipu6.dyndbg=+p intel_ipu6_isys.dyndbg=+p"
	DYNDBG+=" virt_sensor.dyndbg=+p"
	;;
*) echo "unknown IPU4_FLAVOR '${IPU4_FLAVOR}' (expected fork or ipu6)" >&2; exit 1 ;;
esac

QEMU_BIN="${IPU4_QEMU_BIN:-$QEMU_DIR/build/qemu-system-x86_64}"
KERNEL="${IPU4_KERNEL:-$LINUX_DIR/arch/x86/boot/bzImage}"
INITRD="${IPU4_INITRD:-$ROOTFS_OUT/rootfs.cpio.gz}"
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
APPEND+=" $DYNDBG"
# Boot-time tuning. mitigations=off skips the Spectre/Meltdown/MDS/etc.
# init paths (~hundreds of ms on cold boot) — irrelevant in a single-
# tenant test VM. tsc=reliable skips the TSC stability watchdog and
# pins clocksource selection so we don't waste boot time calibrating
# against PIT/HPET. Neither flag affects debug ability: dmesg, ftrace,
# mmiotrace, dyndbg, and oops/KASAN backtraces are all unaffected.
APPEND+=" mitigations=off tsc=reliable"
# PCI device ID the model presents for IPU4_VARIANT (see below). The
# kernel hands the unknown parameter to /init, whose checks read it.
case "${IPU4_VARIANT:-ipu4}" in
ipu4) APPEND+=" ipu_pci_id=0x5a88" ;;
ipu4p) APPEND+=" ipu_pci_id=0x8a19" ;;
ipu6) APPEND+=" ipu_pci_id=0x9a19" ;;
*) echo "unknown IPU4_VARIANT '${IPU4_VARIANT}' (expected ipu4, ipu4p or ipu6)" >&2; exit 1 ;;
esac
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

# `-device ipu4` lives only in the patched qemu (tools/qemu-patches/).
# Stock qemu rejects it. Allow callers that don't exercise the IPU4
# device model (e.g. the kunit gcov harvest in tools/tests/kunit.sh)
# to opt out so they can run under the system qemu.
#
# IPU4_VARIANT (ipu4 | ipu4p | ipu6, default ipu4) selects which IPU the
# model presents (tools/tests/ipu6/ladder.sh, plan "Tracks and file ownership");
# the default leaves the command line exactly as before.
DEVICE_ARGS=(-device ipu4)
if [[ "${IPU4_VARIANT:-ipu4}" != ipu4 ]]; then
	DEVICE_ARGS=(-device "ipu4,variant=$IPU4_VARIANT")
fi
if [[ "${IPU4_NO_DEVICE:-0}" == "1" ]]; then
	DEVICE_ARGS=()
fi

# IPU4_MMIO_WRITE_LOG=<file>: log every MMIO write the guest makes, as
# QEMU's memory_region_ops_write trace event (log backend, one line per
# write with the absolute address, the value and the region name, e.g.
# 'ipu4-bar0'). Host-side, so it needs no guest kernel support and does
# not slow the guest down like mmiotrace. It covers every region
# (serial, VGA, APIC, ...), so callers filter it by name;
# tools/trace/check_writes.py reads it.
TRACE_ARGS=()
if [[ -n "${IPU4_MMIO_WRITE_LOG:-}" ]]; then
	TRACE_ARGS=(-D "$IPU4_MMIO_WRITE_LOG" -trace memory_region_ops_write)
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
	"${DEVICE_ARGS[@]}" \
	"${TRACE_ARGS[@]}" \
	-serial mon:stdio
