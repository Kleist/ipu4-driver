#!/usr/bin/env bash
# Regenerate tools/tests/compare-mmio-baseline.txt from the current
# silicon trace + a fresh QEMU capture. Run this when an MMIO
# divergence is intentional (model catching up to silicon, deliberate
# driver register change, new sub-block instrumented, etc.) and
# commit the result.
#
# Inputs / outputs are identical to compare-mmio.sh; this script just
# bypasses the baseline check (IPU4_NO_BASELINE_CHECK=1) and copies
# the freshly-produced report.txt into place.
#
# Prerequisites: a built QEMU + kernel + initramfs, and a recent
# tools/tests/mmiotrace.sh capture under tools/tests/out/mmiotrace/.
# If $IPU4_REFRESH_RUN_CAPTURE=1 (or the QEMU trace is missing) the
# script will run mmiotrace.sh first.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
BASELINE="$ROOT/tools/tests/compare-mmio-baseline.txt"
QEMU_TRACE="${QEMU_TRACE:-$ROOT/tools/tests/out/mmiotrace/qemu.trace}"

if [[ "${IPU4_REFRESH_RUN_CAPTURE:-0}" == "1" || ! -s "$QEMU_TRACE" ]]; then
	echo "refresh-mmio-baseline: capturing fresh QEMU trace"
	"$HERE/mmiotrace.sh"
fi

echo "refresh-mmio-baseline: running compare-mmio.sh in no-baseline mode"
IPU4_NO_BASELINE_CHECK=1 "$HERE/compare-mmio.sh"

REPORT="$ROOT/tools/tests/out/compare-mmio/report.txt"
if [[ ! -s "$REPORT" ]]; then
	echo "refresh-mmio-baseline: report missing or empty at $REPORT" >&2
	exit 1
fi

cp "$REPORT" "$BASELINE"
echo "refresh-mmio-baseline: wrote $BASELINE"
echo "  Review the change with: git diff -- $BASELINE"
echo "  Commit it together with the driver/QEMU change that motivated"
echo "  the refresh; an unexplained baseline change should fail review."
