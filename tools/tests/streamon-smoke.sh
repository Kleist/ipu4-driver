#!/usr/bin/env bash
# M5b streaming smoke. Boots the VM with init.streamon, walks the
# v4l2 capture API one ioctl at a time, and reports the furthest
# STREAM:STEP marker reached:
#
#   STREAM:open       — /dev/video0 opened
#   STREAM:querycap   — VIDIOC_QUERYCAP succeeded
#   STREAM:s_fmt      — VIDIOC_S_FMT succeeded
#   STREAM:reqbufs    — VIDIOC_REQBUFS succeeded
#   STREAM:querybuf   — VIDIOC_QUERYBUF succeeded
#   STREAM:qbuf       — VIDIOC_QBUF succeeded
#   STREAM:streamon   — VIDIOC_STREAMON succeeded
#   STREAM:dqbuf      — first frame dequeued
#
# Default `IPU4_STREAM_REQUIRED` is `STREAM:reqbufs` for now — that's
# what passes today against the QEMU model with no MMU/syscom/frame-gen
# backing. It tightens as those land.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="$HERE/out"
mkdir -p "$OUT"

LOG="$OUT/streamon-smoke.serial"
rm -f "$LOG"

IPU4_INIT=streamon "$ROOT/tools/rootfs/build.sh" >/dev/null

TIMEOUT="${IPU4_STREAM_TIMEOUT:-180}"

set +e
timeout --preserve-status "$TIMEOUT" "$ROOT/tools/run-vm.sh" 2>&1 | tee "$LOG"
set -e

steps=(
	"MEDIA:link_enabled"
	"STREAM:open"
	"STREAM:querycap"
	"STREAM:s_fmt"
	"STREAM:reqbufs"
	"STREAM:querybuf"
	"STREAM:qbuf"
	"STREAM:streamon"
	"STREAM:dqbuf"
	"STREAM:pattern_ok"
)

reached=""
for s in "${steps[@]}"; do
	if grep -qF "$s" "$LOG"; then
		reached="$s"
	fi
done
echo "streamon-smoke: reached=$reached"

failline=$(grep -m1 '^STREAM:fail' "$LOG" || true)
[[ -n "$failline" ]] && echo "streamon-smoke: $failline"

# A kernel panic inside STREAMON leaves us without a STREAM:streamon
# or STREAM:fail marker — the ioctl never returns. Catch it so the
# caller knows the test moved past qbuf before the panic.
if grep -q 'Kernel panic' "$LOG" && ! grep -q '^STREAM:done' "$LOG"; then
	echo "streamon-smoke: kernel panic mid-STREAMON (see $LOG)"
fi

# Default tracks the firmware-responder rollout (see plan in
# /root/.claude/plans/data-trace-txt-is-an-mmiotrace-tranquil-babbage.md).
# Step 4 wires up PIN_DATA_READY frame delivery, so DQBUF returns the
# first qbuf'd buffer with the same deterministic byte[k] = (k + seq)
# pattern that pre-Step-2 buf_queue_virt() used to fill — written this
# time by the QEMU device model via DMA. STREAM:pattern_ok is the
# strictest gate: it asserts that the firmware path opened the
# stream, kicked CSI2 set_stream(), DMA-wrote a frame, signalled
# PIN_DATA_READY, the driver matched it to the qbuf'd buffer, and
# the userspace pattern probes verified.
REQUIRED="${IPU4_STREAM_REQUIRED:-STREAM:pattern_ok}"
case "$reached" in
"")
	echo "streamon-smoke: FAIL (no STREAM marker)" >&2
	exit 1
	;;
esac

idx_of() {
	local target="$1" i=0
	for s in "${steps[@]}"; do
		[[ "$s" == "$target" ]] && { echo "$i"; return; }
		i=$((i + 1))
	done
	echo -1
}
req_idx=$(idx_of "$REQUIRED")
got_idx=$(idx_of "$reached")
if (( got_idx < req_idx )); then
	echo "streamon-smoke: FAIL (reached=$reached, required=$REQUIRED)" >&2
	exit 1
fi
echo "streamon-smoke: PASS (reached=$reached, required=$REQUIRED)"
