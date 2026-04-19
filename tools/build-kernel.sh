#!/usr/bin/env bash
# Build the guest kernel (arch/x86/boot/bzImage) + modules using the same
# config as tools/build.sh. Heavier than build.sh (minutes vs. seconds) —
# only needed for full-VM runs (vm-smoke, e2e).
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
LINUX_DIR="$ROOT/tools/linux"

if [[ ! -d "$LINUX_DIR" ]]; then
	echo "tools/linux/ missing; run tools/bootstrap.sh first" >&2
	exit 1
fi

# Re-run tools/build.sh's config step if no .config exists yet. Its
# config block is authoritative; we don't duplicate it.
if [[ ! -f "$LINUX_DIR/.config" ]]; then
	"$HERE/build.sh"
fi

cd "$LINUX_DIR"
JOBS="$(nproc)"
make -j"$JOBS" bzImage modules

BZ="$LINUX_DIR/arch/x86/boot/bzImage"
if [[ ! -f "$BZ" ]]; then
	echo "kernel build produced no $BZ" >&2
	exit 2
fi
echo ">>> built $BZ ($(stat -c%s "$BZ") bytes)"
