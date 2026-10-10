#!/usr/bin/env bash
# Build the IPU4 driver in-tree in tools/linux/ and produce intel-ipu4.ko.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

# IPU4_FLAVOR=ipu6 selects the upstream-ipu6 + patch-series flow
# (tools/ipu6/, plan in tools/notes/ipu6-ipu4-plan.md). The default
# "fork" flow below is unchanged.
case "${IPU4_FLAVOR:-fork}" in
fork) ;;
ipu6) exec "$HERE/ipu6/build.sh" "$@" ;;
*) echo "unknown IPU4_FLAVOR '${IPU4_FLAVOR}' (expected fork or ipu6)" >&2; exit 1 ;;
esac

LINUX_DIR="$ROOT/tools/linux"

if [[ ! -d "$LINUX_DIR" ]]; then
	echo "tools/linux/ missing; run tools/bootstrap.sh first" >&2
	exit 1
fi

cd "$LINUX_DIR"

CONFIG="${IPU4_KCONFIG:-$ROOT/tools/rootfs/kernel.config}"
if [[ -f "$CONFIG" ]]; then
	cp "$CONFIG" .config
else
	# Only synthesise a fresh .config from defconfig when none exists.
	# On a cache-restored tree we already have one — running defconfig
	# would reset symbols the scripts/config block below depends on
	# being already-set, and would invalidate ~every cached .o. The
	# scripts/config + olddefconfig pass that follows is idempotent
	# and reapplies the canonical settings on top of either path.
	if [[ ! -f .config ]]; then
		make defconfig
	fi
	# MEDIA_SUPPORT must be =y (not =m) so MEDIA_CONTROLLER — a bool that
	# depends on it — can itself be =y. Without MEDIA_CONTROLLER the
	# driver cannot see struct v4l2_subdev::entity (ipu6-isys.c:114) or
	# struct video_device::entity, and the build dies with type errors.
	#
	# MEDIA_SUPPORT_FILTER must be =n. When it is =y (the defconfig
	# default on x86_64), the "Media core support" menu is `visible if
	# !MEDIA_SUPPORT_FILTER`, which strips the prompts from VIDEO_DEV
	# and MEDIA_CONTROLLER. olddefconfig then re-evaluates those symbols
	# via their default expressions (MEDIA_CAMERA_SUPPORT || ...), which
	# are themselves gated by the filter, so --enable MEDIA_CONTROLLER
	# silently reverts to =n and the driver no longer sees
	# struct v4l2_subdev::entity.
	# VIDEO_DEV / VIDEOBUF2_* are built =y (not =m) so the IPU4 module
	# resolves its kernel-side v4l2 symbols without a separate insmod
	# dance in the guest initramfs. MEDIA_SUPPORT_FILTER=n keeps
	# MEDIA_CAMERA_SUPPORT auto-enabled (see the MEDIA_CONTROLLER note).
	./scripts/config \
		--enable MEDIA_SUPPORT \
		--disable MEDIA_SUPPORT_FILTER \
		--enable MEDIA_PCI_SUPPORT \
		--enable MEDIA_CONTROLLER \
		--enable VIDEO_V4L2_SUBDEV_API \
		--enable VIDEO_DEV \
		--enable VIDEOBUF2_V4L2 \
		--enable VIDEOBUF2_DMA_SG \
		--enable VIDEOBUF2_DMA_CONTIG \
		--enable FTRACE \
		--enable MMIOTRACE \
		--enable DYNAMIC_DEBUG \
		--enable GCOV_KERNEL \
		--enable KUNIT \
		--disable VIDEO_INTEL_IPU6 \
		--module VIDEO_INTEL_IPU4 \
		--enable  VIDEO_IPU4_VIRT_SENSOR \
		--disable VIDEO_INTEL_IPU4_KUNIT_TESTS
	# KUnit tests are built only in the kunit kernel built by kunit.py
	# (.kunitconfig has CONFIG_VIDEO_INTEL_IPU4_KUNIT_TESTS=y, with the
	# driver also =y, so internal symbols resolve at link time). In the
	# streamon-smoke kernel built here the driver is =m, and the new
	# suites (ipu4_format_helpers_kunit, ipu4_video_accessors_kunit)
	# reference non-exported helpers — building them as separate
	# modules would fail modpost.

	# Trim the defconfig to what the QEMU test VM actually exercises
	# (shared with the ipu6 flavor; see the script for the rationale).
	"$HERE/vm-kconfig.sh"
fi
make olddefconfig

# Allow callers (e.g. tools/build-kernel.sh) to use this script purely
# for its canonical config block, skipping the driver M= build below.
# Without this, build-kernel.sh would skip build.sh whenever a cached
# .config existed and a config-affecting change to this script wouldn't
# take effect on cache-hit CI runs.
if [[ "${IPU4_BUILD_CONFIG_ONLY:-0}" == "1" ]]; then
	exit 0
fi

JOBS="$(nproc)"
make -j"$JOBS" modules_prepare

# Build the driver as an external module. KBUILD_MODPOST_WARN downgrades
# "undefined symbol" from an error to a warning: vmlinux is not built
# here, so Module.symvers is empty and every kernel-exported symbol the
# driver uses is technically unresolved. Those symbols are resolved at
# insmod time inside the test VM once M2's full kernel build lands.
make -j"$JOBS" KBUILD_MODPOST_WARN=1 M=drivers/media/pci/intel/ipu4

KO="$LINUX_DIR/drivers/media/pci/intel/ipu4/intel-ipu4.ko"
if [[ ! -f "$KO" ]]; then
	echo "build produced no $KO" >&2
	exit 2
fi
echo ">>> built $KO"
modinfo "$KO" | sed -n '1,10p'
