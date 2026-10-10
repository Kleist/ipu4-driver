#!/usr/bin/env bash
# Build the guest kernel (arch/x86/boot/bzImage) + modules using the same
# config as tools/build.sh. Heavier than build.sh (minutes vs. seconds) —
# only needed for full-VM runs (vm-smoke, e2e).
#
# IPU4_FLAVOR=ipu6 builds tools/linux-ipu6/ instead; build.sh then
# dispatches to tools/ipu6/build.sh for the config pass.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
case "${IPU4_FLAVOR:-fork}" in
fork) LINUX_DIR="$ROOT/tools/linux" ;;
ipu6) LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu6}" ;;
*) echo "unknown IPU4_FLAVOR '${IPU4_FLAVOR}' (expected fork or ipu6)" >&2; exit 1 ;;
esac

if [[ ! -d "$LINUX_DIR" ]]; then
	echo "${LINUX_DIR#"$ROOT"/}/ missing; run tools/bootstrap.sh first" >&2
	exit 1
fi

# Always run build.sh's config block — it's the canonical source for
# every kconfig option this kernel needs. Skipping on cached .config
# means a partial restore-keys cache hit (which has a *prior* .config)
# would mask config-affecting changes to build.sh. IPU4_BUILD_CONFIG_ONLY=1
# tells build.sh to do only the kconfig pass, not the driver M= build.
IPU4_BUILD_CONFIG_ONLY=1 "$HERE/build.sh"

cd "$LINUX_DIR"
JOBS="$(nproc)"
make -j"$JOBS" bzImage modules

BZ="$LINUX_DIR/arch/x86/boot/bzImage"
if [[ ! -f "$BZ" ]]; then
	echo "kernel build produced no $BZ" >&2
	exit 2
fi
echo ">>> built $BZ ($(stat -c%s "$BZ") bytes)"
