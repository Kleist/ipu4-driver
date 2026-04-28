#!/usr/bin/env bash
# Tier-1 tests: run KUnit suites under qemu-kvm via kunit.py.
# Target: <1 second wall clock for the IPU4 unit suites.
#
# Optional gcov harvest: if IPU4_KUNIT_GCOV=1 (default in CI), boot the
# already-built kunit kernel a second time under tools/run-vm.sh with
# init.kunit-cov to extract /sys/kernel/debug/gcov/ onto a 9p share.
# tools/coverage/collect.sh then merges the resulting kunit-gcov.tar
# with the streamon-smoke gcov tar before genhtml. The harvest is
# best-effort: a failure does not fail the suite.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
LINUX_DIR="$ROOT/tools/linux"

if [[ ! -d "$LINUX_DIR" ]]; then
	echo "tools/linux/ missing; run tools/bootstrap.sh first" >&2
	exit 1
fi

cd "$LINUX_DIR"

KUNITCFG="drivers/media/pci/intel/ipu4/tests/.kunitconfig"
if [[ ! -f "$KUNITCFG" ]]; then
	echo "missing $KUNITCFG (was bootstrap.sh run?)" >&2
	exit 2
fi

# kunit.py uses O=.kunit and refuses to run if the source tree is dirty
# from a prior in-tree make (e.g. tools/build.sh leaving .o files behind).
# Clean once before invoking; mrproper is a no-op on an already-clean tree.
# IPU4_KUNIT_NO_MRPROPER=1 skips the wipe — used by vm-smoke CI where
# kunit.sh runs against a freshly-bootstrapped (or cache-restored)
# tree and a mrproper would needlessly destroy a 10-min build cache.
if [[ "${IPU4_KUNIT_NO_MRPROPER:-0}" != "1" ]]; then
	make -s mrproper
fi

./tools/testing/kunit/kunit.py run \
	--arch=x86_64 \
	--kunitconfig=drivers/media/pci/intel/ipu4/tests \
	--jobs="$(nproc)" \
	'ipu4_*'

if [[ "${IPU4_KUNIT_GCOV:-1}" != "1" ]]; then
	exit 0
fi

# Second boot: same kunit-built kernel, but under our wrapper so the
# init.kunit-cov initramfs can tar /sys/kernel/debug/gcov/ onto the 9p
# share. The kunit suites re-run during this boot (they're registered
# as late_initcall) — the .gcda accumulated during that second run is
# what we harvest. kunit.py's first run is the source of truth for
# pass/fail; this run discards results.
KUNIT_BZ=""
for cand in \
	"$LINUX_DIR/.kunit/arch/x86_64/boot/bzImage" \
	"$LINUX_DIR/.kunit/arch/x86/boot/bzImage"; do
	if [[ -f "$cand" ]]; then
		KUNIT_BZ="$cand"
		break
	fi
done
if [[ -z "$KUNIT_BZ" ]]; then
	echo "kunit gcov harvest: no kunit bzImage under .kunit/ — skipping" >&2
	exit 0
fi

SHARE_DIR="$ROOT/tools/tests/out"
mkdir -p "$SHARE_DIR"
rm -f "$SHARE_DIR/kunit-gcov.tar"

# Build a minimal initramfs around init.kunit-cov. tools/rootfs/build.sh
# discovers init.<name> from $IPU4_INIT, so this is a one-env-var hop.
# The rootfs builder needs gen_init_cpio; we use the streamon-smoke
# build tree's copy when present, else fall back to the kunit tree.
GEN="$LINUX_DIR/usr/gen_init_cpio"
if [[ ! -x "$GEN" ]]; then
	GEN="$LINUX_DIR/.kunit/usr/gen_init_cpio"
fi
if [[ ! -x "$GEN" ]]; then
	echo "kunit gcov harvest: gen_init_cpio not built — skipping" >&2
	exit 0
fi

if ! IPU4_INIT=kunit-cov IPU4_GEN_INIT_CPIO="$GEN" \
     "$ROOT/tools/rootfs/build.sh" >&2; then
	echo "kunit gcov harvest: rootfs build failed — skipping" >&2
	exit 0
fi

# tools/run-vm.sh defaults to the streamon kernel and the patched
# qemu-system-x86_64 from tools/build-qemu.sh. The kunit kernel doesn't
# touch the IPU4 device model during boot, so the system qemu is fine
# when the patched build isn't around (build-and-kunit CI doesn't run
# build-qemu.sh).
QEMU_BIN="${IPU4_QEMU_BIN:-$ROOT/tools/qemu/build/qemu-system-x86_64}"
if [[ ! -x "$QEMU_BIN" ]]; then
	QEMU_BIN="$(command -v qemu-system-x86_64 || true)"
fi
if [[ -z "$QEMU_BIN" || ! -x "$QEMU_BIN" ]]; then
	echo "kunit gcov harvest: no qemu-system-x86_64 — skipping" >&2
	exit 0
fi

if ! IPU4_KERNEL="$KUNIT_BZ" IPU4_TESTS_SHARE="$SHARE_DIR" \
     IPU4_QEMU_BIN="$QEMU_BIN" IPU4_NO_DEVICE=1 \
     timeout 120 "$ROOT/tools/run-vm.sh" >&2; then
	echo "kunit gcov harvest: run-vm timed out or failed — skipping" >&2
	exit 0
fi

if [[ -s "$SHARE_DIR/kunit-gcov.tar" ]]; then
	echo ">>> kunit gcov harvest: $SHARE_DIR/kunit-gcov.tar" \
	     "($(stat -c%s "$SHARE_DIR/kunit-gcov.tar") bytes)"
else
	echo "kunit gcov harvest: no kunit-gcov.tar produced" >&2
fi
