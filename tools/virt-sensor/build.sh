#!/usr/bin/env bash
# Build the harness-only virtual sensor (virt-sensor.ko) out of tree
# against a guest kernel ($IPU4_IPU6_LINUX_DIR, default the ipu6-flavor
# one). Needs a built tree (Module.symvers from tools/build-kernel.sh).
# Output goes to tools/virt-sensor/out/<tree>/, which
# tools/rootfs/build.sh packs for the flavor that boots that tree.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu6}"
OUT="$HERE/out/${LINUX_DIR##*/}"

if [[ ! -f "$LINUX_DIR/Module.symvers" ]]; then
	echo "missing $LINUX_DIR/Module.symvers;" \
		"run tools/build-kernel.sh first" >&2
	exit 1
fi

mkdir -p "$OUT"
# Built from a copy in $OUT: 6.12 has no MO= for a separate output dir.
cp "$HERE/Makefile" "$HERE/virt-sensor.c" "$OUT/"
make -C "$LINUX_DIR" M="$OUT" W=1 -j"$(nproc)" modules
echo ">>> built $OUT/virt-sensor.ko"
