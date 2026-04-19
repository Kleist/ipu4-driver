#!/usr/bin/env bash
# M3 smoke test. Boots the VM with an init that inserts intel-ipu4.ko
# and asserts the probe path made it past buttress init to firmware
# load. The exact stopping point is the kernel's firmware_class failing
# `ipu4_cpd_b0.bin` with -ENOENT; we don't ship a real CPD blob yet
# (that's M4 territory), so the passing criterion here is "the driver
# got far enough to _ask_ for firmware."
#
# Regression mode: if the probe was previously further along and now
# stops earlier, this script fails.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="$HERE/out"
mkdir -p "$OUT"

LOG="$OUT/probe-smoke.serial"
rm -f "$LOG"

IPU4_INIT=probe-ok "$ROOT/tools/rootfs/build.sh" >/dev/null

TIMEOUT="${IPU4_PROBE_TIMEOUT:-120}"

set +e
timeout --preserve-status "$TIMEOUT" "$ROOT/tools/run-vm.sh" 2>&1 | tee "$LOG"
set -e

# Progress markers, ordered from earliest to furthest-along. The last
# one that matches is the actual checkpoint reached. The marker patterns
# are taken from ipu6.c probe-time dev_{info,err,dbg}() calls.
markers=(
	"probe:entered|PCI bar\[0\] = "
	"probe:ipc_reset|IPC reset done"
	"probe:fw_load|FW version:"
	"probe:fw_valid|Found supported sensor"
	"probe:bridge|IPU6 bridge init"
	"probe:bound|driver bound"
)

reached=""
for m in "${markers[@]}"; do
	name="${m%%|*}"
	re="${m#*|}"
	if grep -qE "$re" "$LOG"; then
		reached="$name"
	fi
done

echo "probe-smoke: reached=$reached"

# The required checkpoint. Raised as each M3/M4 piece lands. Default is
# `probe:fw_valid` after M4 lands a CPD blob — probe now consistently
# loads+validates firmware and runs the driver far enough to call into
# ambu-ipu-bridge.
REQUIRED="${IPU4_PROBE_REQUIRED:-probe:fw_valid}"
case "$reached" in
"")
	echo "probe-smoke: FAIL (driver did not reach any progress marker)" >&2
	exit 1
	;;
esac

# Ordered check: fail if we reached less than the required marker.
idx_of() {
	local target="$1" i=0
	for m in "${markers[@]}"; do
		[[ "${m%%|*}" == "$target" ]] && { echo "$i"; return; }
		i=$((i + 1))
	done
	echo -1
}
req_idx=$(idx_of "$REQUIRED")
got_idx=$(idx_of "$reached")
if (( got_idx < req_idx )); then
	echo "probe-smoke: FAIL (reached=$reached, required=$REQUIRED)" >&2
	exit 1
fi
echo "probe-smoke: PASS (reached=$reached, required=$REQUIRED)"
