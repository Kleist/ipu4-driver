#!/usr/bin/env bash
# M2 smoke test: boot the VM, verify the IPU4 device probes.
#
# Runs tools/run-vm.sh with a hard timeout, captures serial output, and
# greps for the VM_SMOKE: PASS marker that the guest init prints. Exits
# non-zero on timeout, boot failure, or missing marker.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="$HERE/out"
mkdir -p "$OUT"

LOG="$OUT/vm-smoke.serial"
rm -f "$LOG"

# Rebuild initramfs with the M2 guest init embedded — probe-smoke runs
# with IPU4_INIT=probe-ok and can leave the shared rootfs/out state in
# the other mode otherwise.
IPU4_INIT=vm-smoke "$ROOT/tools/rootfs/build.sh" >/dev/null

TIMEOUT="${IPU4_SMOKE_TIMEOUT:-90}"

set +e
timeout --preserve-status "$TIMEOUT" "$ROOT/tools/run-vm.sh" 2>&1 | tee "$LOG"
set -e

if grep -q '^VM_SMOKE: PASS' "$LOG"; then
	echo "vm-smoke: PASS"
	exit 0
fi

echo "vm-smoke: FAIL" >&2
if grep -q '^VM_SMOKE: FAIL' "$LOG"; then
	grep '^VM_SMOKE:' "$LOG" >&2
else
	echo "(no VM_SMOKE marker found — see $LOG)" >&2
fi
exit 1
