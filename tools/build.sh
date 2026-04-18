#!/usr/bin/env bash
# Build the IPU4 driver in-tree in tools/linux/ and produce intel-ipu4.ko.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
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
	make defconfig
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
	./scripts/config \
		--enable MEDIA_SUPPORT \
		--disable MEDIA_SUPPORT_FILTER \
		--enable MEDIA_PCI_SUPPORT \
		--enable MEDIA_CONTROLLER \
		--enable VIDEO_V4L2_SUBDEV_API \
		--module VIDEO_DEV \
		--module VIDEOBUF2_V4L2 \
		--module VIDEOBUF2_DMA_SG \
		--module VIDEOBUF2_DMA_CONTIG \
		--enable FTRACE \
		--enable MMIOTRACE \
		--enable DYNAMIC_DEBUG \
		--enable GCOV_KERNEL \
		--enable KUNIT \
		--disable VIDEO_INTEL_IPU6 \
		--module VIDEO_INTEL_IPU4 \
		--module VIDEO_INTEL_IPU4_KUNIT_TESTS
fi
make olddefconfig

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
