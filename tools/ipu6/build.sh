#!/usr/bin/env bash
# Build intel-ipu6.ko + intel-ipu6-isys.ko in tools/linux-ipu6/ (the ipu6
# flavor, IPU4_FLAVOR=ipu6). Run tools/ipu6/bootstrap.sh first.
#
# Builds only the ipu6 directory as an external module on top of
# `modules_prepare` (~minutes) rather than `make modules` / bzImage: the
# static ladder rungs need just the .ko files. The guest kernel for the VM
# rungs is a separate step (plan PR 4).
#
# W=1 is on by default (IPU4_IPU6_W=1) because that is what the `build`
# rung checks. The compiler output is mirrored to $IPU4_IPU6_BUILD_LOG
# when set, so the ladder can scan it for warnings.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu6}"
W="${IPU4_IPU6_W:-1}"

if [[ ! -d "$LINUX_DIR/.git" ]]; then
	echo "${LINUX_DIR#"$ROOT"/} missing; run tools/ipu6/bootstrap.sh first" >&2
	exit 1
fi

cd "$LINUX_DIR"

# Keep an existing .config (cache-restored trees): re-running defconfig
# would invalidate every cached object. The scripts/config pass below is
# idempotent and reapplies the settings on top of either path.
if [[ ! -f .config ]]; then
	make -s defconfig
fi

# Media core settings: see tools/build.sh for why MEDIA_SUPPORT must be
# =y and MEDIA_SUPPORT_FILTER =n (otherwise MEDIA_CONTROLLER silently
# reverts to =n).
#
# The pinned tree also carries Ruslan's downstream driver
# (CONFIG_VIDEO_INTEL_IPU, drivers/media/pci/intel/ipu4/). It binds
# 8086:5a88 too, so it must stay off. The IPU7 staging driver is off so
# only the in-tree ipu6 driver is a candidate for the device.
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
	--enable V4L2_FWNODE \
	--enable ACPI \
	--enable PCI \
	--enable PCI_MSI \
	--enable DYNAMIC_DEBUG \
	--disable VIDEO_INTEL_IPU \
	--disable VIDEO_INTEL_IPU7 \
	--module VIDEO_INTEL_IPU6 \
	--enable VIDEO_INTEL_IPU6_IPU7
make -s olddefconfig

check_sym() {
	local want="$1" got
	got="$(grep -E "^(# )?CONFIG_$2[= ]" .config || true)"
	if [[ "$got" != "$want" ]]; then
		echo "kconfig: expected '$want', got '${got:-<unset>}'" >&2
		exit 1
	fi
}
check_sym "CONFIG_VIDEO_INTEL_IPU6=m" VIDEO_INTEL_IPU6
check_sym "# CONFIG_VIDEO_INTEL_IPU is not set" VIDEO_INTEL_IPU

if [[ "${IPU4_BUILD_CONFIG_ONLY:-0}" == "1" ]]; then
	exit 0
fi

JOBS="$(nproc)"
make -s -j"$JOBS" modules_prepare

# KBUILD_MODPOST_WARN: vmlinux is not built here, so Module.symvers is
# empty and every kernel export looks unresolved to modpost (same as
# tools/build.sh). Note that an M= build does not read the parent
# drivers/media/pci/intel/Makefile, so the downstream driver's
# subdir-ccflags warning suppressions there do not leak into ipu6/.
# IPU4_IPU6_CLEAN=1 drops the ipu6 objects first so every file is
# recompiled and its warnings show up (the `build` rung uses this; the
# directory is small, so it costs seconds).
if [[ "${IPU4_IPU6_CLEAN:-0}" == "1" ]]; then
	make -s M=drivers/media/pci/intel/ipu6 clean
fi

build() {
	make -j"$JOBS" KBUILD_MODPOST_WARN=1 W="$W" \
		M=drivers/media/pci/intel/ipu6
}
if [[ -n "${IPU4_IPU6_BUILD_LOG:-}" ]]; then
	build 2>&1 | tee "$IPU4_IPU6_BUILD_LOG"
else
	build
fi

for ko in intel-ipu6.ko intel-ipu6-isys.ko; do
	f="drivers/media/pci/intel/ipu6/$ko"
	if [[ ! -f "$f" ]]; then
		echo "build produced no $f" >&2
		exit 2
	fi
	echo ">>> built ${LINUX_DIR#"$ROOT"/}/$f"
done
