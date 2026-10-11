#!/usr/bin/env bash
# Build kernel/ipu4/ as an out-of-tree module (its own Makefile, as
# `make -C kernel/ipu4 KERNEL_SRC=...` does) against the stock kernel in
# tools/linux-ipu4-oot/ (IPU4_FLAVOR=out-of-tree; pin in ./pin, fetched
# by tools/ipu6/bootstrap.sh). The .ko files land in kernel/ipu4/.
#
# Same shape and knobs as tools/ipu6/build.sh: the kconfig pass is shared
# with the guest kernel (`IPU4_FLAVOR=out-of-tree tools/build-kernel.sh`
# runs it with IPU4_BUILD_CONFIG_ONLY=1), then modules_prepare and the
# module build, mirrored to $IPU4_IPU6_BUILD_LOG when set;
# IPU4_IPU6_CLEAN=1 recompiles every file. The build uses the module's
# own warning level (W=$IPU4_IPU6_W when set): it does not build at W=1
# yet.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu4-oot}"
DRV_DIR="$ROOT/kernel/ipu4"

if [[ ! -d "$LINUX_DIR/.git" ]]; then
	echo "${LINUX_DIR#"$ROOT"/} missing; run" \
		"IPU4_IPU6_PIN=tools/out-of-tree/pin IPU4_IPU6_PATCH_DIR=" \
		"IPU4_IPU6_LINUX_DIR=${LINUX_DIR#"$ROOT"/} tools/ipu6/bootstrap.sh" >&2
	exit 1
fi

cd "$LINUX_DIR"

# Keep an existing .config (cache-restored trees), as tools/ipu6/build.sh.
if [[ ! -f .config ]]; then
	make -s defconfig
fi

# Media core as for the other flavors (tools/build.sh explains
# MEDIA_SUPPORT / MEDIA_SUPPORT_FILTER). The module needs no Kconfig
# symbol of its own; vb2 and the v4l2 helpers it uses are built as
# modules so the guest loads them as on a distribution kernel. The
# in-tree ipu6 driver stays off so only the module is a candidate for
# the IPU.
./scripts/config \
	--enable MEDIA_SUPPORT \
	--disable MEDIA_SUPPORT_FILTER \
	--enable MEDIA_PCI_SUPPORT \
	--enable MEDIA_CONTROLLER \
	--enable VIDEO_V4L2_SUBDEV_API \
	--enable VIDEO_DEV \
	--module VIDEOBUF2_V4L2 \
	--module VIDEOBUF2_DMA_SG \
	--module VIDEOBUF2_DMA_CONTIG \
	--module V4L2_FWNODE \
	--enable ACPI \
	--enable PCI \
	--enable PCI_MSI \
	--enable DYNAMIC_DEBUG \
	--disable VIDEO_INTEL_IPU6
"$ROOT/tools/vm-kconfig.sh"
# As in tools/ipu6/build.sh: the `build` test gates on warnings in the
# module build log, so the guest kernel does not need -Werror.
./scripts/config --disable WERROR
make -s olddefconfig

if [[ "${IPU4_BUILD_CONFIG_ONLY:-0}" == "1" ]]; then
	exit 0
fi

JOBS="$(nproc)"
make -s -j"$JOBS" modules_prepare

if [[ "${IPU4_IPU6_CLEAN:-0}" == "1" ]]; then
	make -s M="$DRV_DIR" clean
fi

# The M= build the module Makefile's `all` target runs; called directly
# because `all` passes its own -j, which drops the jobserver ("make:
# warning: -j0 forced in submake") and so would trip the build test's
# warning scan. KBUILD_MODPOST_WARN: no vmlinux here, see
# tools/ipu6/build.sh.
build() {
	make -j"$JOBS" M="$DRV_DIR" KBUILD_MODPOST_WARN=1 \
		${IPU4_IPU6_W:+W="$IPU4_IPU6_W"}
}
if [[ -n "${IPU4_IPU6_BUILD_LOG:-}" ]]; then
	build 2>&1 | tee "$IPU4_IPU6_BUILD_LOG"
else
	build
fi

for ko in intel-ipu4.ko intel-ipu4-isys.ko ambu-ipu-bridge.ko; do
	if [[ ! -f "$DRV_DIR/$ko" ]]; then
		echo "build produced no kernel/ipu4/$ko" >&2
		exit 2
	fi
	echo ">>> built kernel/ipu4/$ko"
done
