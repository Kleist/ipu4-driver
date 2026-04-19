#!/usr/bin/env bash
# Configure and build qemu-system-x86_64 with the IPU4 device compiled in.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
QEMU_DIR="$ROOT/tools/qemu"

if [[ ! -d "$QEMU_DIR" ]]; then
	echo "tools/qemu/ missing; run tools/bootstrap.sh first" >&2
	exit 1
fi

cd "$QEMU_DIR"

# --disable-fdt avoids the dtc subproject fetch (gitlab.com not always
# reachable from CI); --disable-slirp drops the user-net subproject the
# same way. Both are irrelevant to the IPU4 smoke test.
if [[ ! -f build/build.ninja ]]; then
	./configure \
		--target-list=x86_64-softmmu \
		--disable-werror \
		--disable-plugins \
		--disable-docs \
		--disable-fdt \
		--disable-slirp
fi
ninja -C build qemu-system-x86_64

BIN="$QEMU_DIR/build/qemu-system-x86_64"
if [[ ! -x "$BIN" ]]; then
	echo "build produced no $BIN" >&2
	exit 2
fi
echo ">>> built $BIN"
"$BIN" -device help 2>&1 | grep -i ipu4 || {
	echo "ipu4 device not registered — check hw/misc wiring" >&2
	exit 3
}
