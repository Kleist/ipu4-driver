#!/usr/bin/env bash
# Build the guest kernel (arch/x86/boot/bzImage) + modules using the same
# config as tools/build.sh. Heavier than build.sh (minutes vs. seconds) —
# only needed for full-VM runs (vm-smoke, e2e).
#
# IPU4_FLAVOR=ipu6 builds tools/linux-ipu6/ instead, out-of-tree
# tools/linux-ipu4-oot/; build.sh then dispatches to tools/<flavor>/build.sh
# for the config pass.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
case "${IPU4_FLAVOR:-fork}" in
fork) LINUX_DIR="$ROOT/tools/linux" ;;
ipu6) LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu6}" ;;
out-of-tree) LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu4-oot}" ;;
*) echo "unknown IPU4_FLAVOR '${IPU4_FLAVOR}' (expected fork, ipu6 or out-of-tree)" >&2; exit 1 ;;
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

# `make modules` just rebuilt the ipu6 driver without instrumentation;
# redo the driver's own build so the guest gets the instrumented one.
if [[ "${IPU4_GCOV:-0}" == 1 && "${IPU4_FLAVOR:-fork}" != fork ]]; then
	"$HERE/build.sh"
fi

# The harness's virtual sensor, which the fork inits load before the
# driver; the other flavors' ladder tests build it themselves.
if [[ "${IPU4_FLAVOR:-fork}" == fork ]]; then
	IPU4_IPU6_LINUX_DIR="$LINUX_DIR" "$ROOT/tools/virt-sensor/build.sh"
fi
