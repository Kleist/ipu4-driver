#!/usr/bin/env bash
# Diff data/trace.txt (real-silicon mmiotrace) against an optional
# QEMU-side capture and emit a coverage report. Today the report is
# silicon-only because the QEMU capture isn't produced anywhere yet —
# that's still the useful worklist for upcoming ipu4.c handler PRs.
#
# Inputs:
#   data/trace.txt           — silicon side, committed.
#   $QEMU_TRACE (optional)   — guest-side ftrace mmiotrace, same format
#                              as data/trace.txt. Default location:
#                              tools/tests/out/compare-mmio/qemu.trace.
#                              When absent, the script reports the
#                              silicon-only baseline.
#
# Outputs (under tools/tests/out/compare-mmio/):
#   silicon.jsonl  — postprocess --json on data/trace.txt.
#   qemu.jsonl     — postprocess --json on $QEMU_TRACE, or empty.
#   report.txt     — human-readable diff (also tee'd to stdout).
#   report.json    — same diff in machine-readable form.
#
# Exit code: 0 if streams agree, 1 if there are unimplemented addresses
# or read-value mismatches. CI invokes this with `|| true` because the
# silicon-only baseline is expected to be non-empty until the model
# catches up; the report is published as an artifact regardless.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
cd "$ROOT"

SILICON_TRACE="$ROOT/data/trace.txt"
if [[ ! -f "$SILICON_TRACE" ]]; then
	echo "compare-mmio: silicon trace missing at $SILICON_TRACE" >&2
	exit 2
fi

OUT="$ROOT/tools/tests/out/compare-mmio"
mkdir -p "$OUT"

"$ROOT/postprocess_trace.py" "$SILICON_TRACE" --json > "$OUT/silicon.jsonl"

QEMU_TRACE="${QEMU_TRACE:-$OUT/qemu.trace}"
if [[ ! -s "$QEMU_TRACE" ]]; then
	echo "compare-mmio: no QEMU trace at $QEMU_TRACE; reporting silicon-only baseline"
	: > "$OUT/qemu.jsonl"
else
	"$ROOT/postprocess_trace.py" "$QEMU_TRACE" --json > "$OUT/qemu.jsonl"
fi

# Run twice so we get both formats without re-decoding. The text report
# is what humans read; the JSON one is a stable input for any future
# trend tracking.
python3 "$ROOT/tools/trace/compare.py" --json \
	"$OUT/silicon.jsonl" "$OUT/qemu.jsonl" > "$OUT/report.json" || true
python3 "$ROOT/tools/trace/compare.py" \
	"$OUT/silicon.jsonl" "$OUT/qemu.jsonl" | tee "$OUT/report.txt"
