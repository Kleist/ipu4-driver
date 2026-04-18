#!/usr/bin/env bash
# Tier-2 tests: boot the VM, stream 5 frames from /dev/video0, SHA-check
# the dequeued buffers against tools/tests/frames.sha256, harvest gcov.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="$ROOT/tools/tests/out"
mkdir -p "$OUT"

SERIAL_LOG="$OUT/serial.log"
GCOV_TAR="$OUT/gcov.tar"
EXPECTED="$HERE/frames.sha256"

rm -f "$SERIAL_LOG" "$GCOV_TAR"

# The guest script writes PASS/FAIL and frame hashes to the serial console
# and then powers off. Timeout bounds a stuck boot.
timeout 60 "$ROOT/tools/run-vm.sh" 2>&1 | tee "$SERIAL_LOG"

if ! grep -q '^E2E: PASS$' "$SERIAL_LOG"; then
	echo "e2e: FAIL (no PASS marker on serial)" >&2
	exit 1
fi

if [[ -f "$EXPECTED" ]]; then
	if ! grep -q -f "$EXPECTED" "$SERIAL_LOG"; then
		echo "e2e: frame hash mismatch" >&2
		exit 2
	fi
fi

# Expect the guest to tar up /sys/kernel/debug/gcov/ onto a virtio
# console or a shared 9p directory. Fetch whichever is set up.
GCOV_SRC="${IPU4_GCOV_SRC:-$OUT/gcov.tar}"
if [[ -s "$GCOV_SRC" ]]; then
	cp "$GCOV_SRC" "$GCOV_TAR"
	echo "e2e: gcov harvested -> $GCOV_TAR"
fi

echo "e2e: PASS"
