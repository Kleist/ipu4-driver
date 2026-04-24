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
# Exit code: 0 if streams agree, 1 if there are unimplemented addresses
# or read-value mismatches. The CI step invokes this with `|| true` so
# a non-empty divergence report doesn't fail the build — but missing
# inputs must still fail here because that's a harness regression.
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
# unimplemented addresses or value mismatches. Divergence is expected
# until the model catches up; we do NOT want that to fail CI because
# the report published as an artifact is the whole point. Harness
# faults (missing inputs, empty outputs) funnelled through fail()
# already exited 1 above; only reach here when the diff ran cleanly.
case "$compare_rc" in
	0) echo "compare-mmio: clean (streams agree)" ;;
	1) echo "compare-mmio: divergence reported (expected until model catches up)" ;;
	*) fail "compare.py exited with unexpected code $compare_rc" ;;
esac
