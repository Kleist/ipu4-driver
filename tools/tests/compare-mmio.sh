#!/usr/bin/env bash
# Diff data/trace.txt (real-silicon mmiotrace) against a QEMU-side
# capture (produced by tools/tests/mmiotrace.sh) and emit the coverage
# report. Both sides are mandatory — a missing or empty QEMU trace is
# a hard failure, not a silicon-only fallback, because an empty
# qemu.jsonl renders compare.py's value_mismatch classification dead
# and produces a report indistinguishable from "no CI ran at all".
#
# Inputs:
#   data/trace.txt           — silicon side, committed.
#   $QEMU_TRACE (optional)   — guest-side mmiotrace in the same format
#                              as data/trace.txt. Default location:
#                              tools/tests/out/mmiotrace/qemu.trace,
#                              which is where tools/tests/mmiotrace.sh
#                              writes it.
#
# Outputs (under tools/tests/out/compare-mmio/):
#   silicon.jsonl  — postprocess --json on data/trace.txt.
#   qemu.jsonl     — postprocess --json on $QEMU_TRACE.
#   report.txt     — human-readable diff (also tee'd to stdout).
#   report.json    — same diff in machine-readable form.
#
# Baseline gate: when tools/tests/compare-mmio-baseline.txt exists, the
# fresh report.txt is normalized (per tools/tests/compare-mmio-volatile.txt,
# which redacts intrinsically-jittery rows like TSC and FW_COM ring
# cursors) and diffed against the committed baseline; any drift fails
# the script. That's the regression brake for refactor work — silicon
# and QEMU are both meant to evolve, but only via deliberate baseline
# refreshes (see tools/tests/refresh-mmio-baseline.sh). Set
# IPU4_NO_BASELINE_CHECK=1 to opt out (used by the refresh script
# itself and by anyone wanting the legacy report-only behaviour).
#
# Exit code: 0 if streams agree AND baseline matches; 1 on any
# divergence vs baseline OR on a harness fault (missing inputs, empty
# postprocess output, unexpected compare.py exit code).
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
cd "$ROOT"

fail() {
	echo "::error title=compare-mmio failed::$*"
	echo "compare-mmio: FAIL ($*)" >&2
	exit 1
}

SILICON_TRACE="$ROOT/data/trace.txt"
if [[ ! -f "$SILICON_TRACE" ]]; then
	fail "silicon trace missing at $SILICON_TRACE"
fi

OUT="$ROOT/tools/tests/out/compare-mmio"
mkdir -p "$OUT"

"$ROOT/postprocess_trace.py" "$SILICON_TRACE" --json > "$OUT/silicon.jsonl"
if [[ ! -s "$OUT/silicon.jsonl" ]]; then
	fail "silicon.jsonl empty after postprocess (data/trace.txt malformed?)"
fi

# Default points at tools/tests/mmiotrace.sh's output so the two
# scripts chain naturally: `mmiotrace.sh && compare-mmio.sh` is the
# full capture + diff loop.
QEMU_TRACE="${QEMU_TRACE:-$ROOT/tools/tests/out/mmiotrace/qemu.trace}"
if [[ ! -s "$QEMU_TRACE" ]]; then
	fail "QEMU trace missing or empty at $QEMU_TRACE — run tools/tests/mmiotrace.sh first"
fi

"$ROOT/postprocess_trace.py" "$QEMU_TRACE" --json > "$OUT/qemu.jsonl"
if [[ ! -s "$OUT/qemu.jsonl" ]]; then
	head_snippet=$(head -n 15 "$QEMU_TRACE" | tr '\n' '|')
	fail "qemu.jsonl empty after postprocess — format mismatch? (qemu.trace head: $head_snippet)"
fi

# Run twice so we get both formats without re-decoding. The text report
# is what humans read; the JSON one is a stable input for any future
# trend tracking.
python3 "$ROOT/tools/trace/compare.py" --json \
	"$OUT/silicon.jsonl" "$OUT/qemu.jsonl" > "$OUT/report.json" || true

set +e
python3 "$ROOT/tools/trace/compare.py" \
	"$OUT/silicon.jsonl" "$OUT/qemu.jsonl" | tee "$OUT/report.txt"
compare_rc=${PIPESTATUS[0]}
set -e

# compare.py returns 0 when streams agree and 1 when there are
# unimplemented addresses or value mismatches. Either is fine here —
# the gate below is "did the report change since the committed
# baseline?", not "is the report empty?". Only an unexpected exit code
# is a harness fault.
case "$compare_rc" in
	0|1) ;;
	*) fail "compare.py exited with unexpected code $compare_rc" ;;
esac

# Redact intrinsically-volatile rows (TSC, FW_COM ring cursors,
# PWR_STATE FSM transitionals, …) before diffing. The committed
# baseline is also stored in this normalized form, so the on-disk file
# declaratively shows which fields are accepted-as-volatile.
VOLATILE="$ROOT/tools/tests/compare-mmio-volatile.txt"
python3 "$ROOT/tools/trace/normalize_report.py" \
	--volatile "$VOLATILE" \
	--in "$OUT/report.txt" \
	--out "$OUT/report.normalized.txt"

BASELINE="$ROOT/tools/tests/compare-mmio-baseline.txt"
if [[ "${IPU4_NO_BASELINE_CHECK:-0}" == "1" ]]; then
	echo "compare-mmio: baseline check skipped (IPU4_NO_BASELINE_CHECK=1)"
	exit 0
fi
if [[ ! -f "$BASELINE" ]]; then
	echo "compare-mmio: no baseline at $BASELINE — skipping gate."
	echo "  Run tools/tests/refresh-mmio-baseline.sh on a clean main to"
	echo "  generate one. Until then, regressions are not detected."
	exit 0
fi

DIFF_OUT="$OUT/baseline.diff"
if diff -u "$BASELINE" "$OUT/report.normalized.txt" > "$DIFF_OUT"; then
	rm -f "$DIFF_OUT"
	echo "compare-mmio: baseline matches"
	exit 0
fi

echo "::error title=compare-mmio baseline drift::report.txt differs from $BASELINE"
echo "compare-mmio: BASELINE DRIFT — see $DIFF_OUT"
echo "  If this drift is intentional (model catching up, or a deliberate"
echo "  driver MMIO change), refresh via:"
echo "    tools/tests/refresh-mmio-baseline.sh"
echo "  and commit tools/tests/compare-mmio-baseline.txt."
echo "----- diff (first 80 lines) -----"
head -n 80 "$DIFF_OUT"
exit 1
