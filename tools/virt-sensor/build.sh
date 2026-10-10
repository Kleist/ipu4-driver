#!/usr/bin/env bash
# Build the harness-only virtual sensor (virt-sensor.ko) out of tree
# against the ipu6-flavor kernel. Needs a built tree (Module.symvers from
# `IPU4_FLAVOR=ipu6 tools/build-kernel.sh`). Output goes to
# tools/virt-sensor/out/, which tools/rootfs/build.sh packs for the ipu6
# flavor.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu6}"
OUT="$HERE/out"

if [[ ! -f "$LINUX_DIR/Module.symvers" ]]; then
	echo "missing $LINUX_DIR/Module.symvers;" \
		"run IPU4_FLAVOR=ipu6 tools/build-kernel.sh first" >&2
	exit 1
fi

mkdir -p "$OUT"
make -C "$LINUX_DIR" M="$HERE" MO="$OUT" W=1 -j"$(nproc)" modules
echo ">>> built $OUT/virt-sensor.ko"
