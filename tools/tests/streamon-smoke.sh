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

REQUIRED="${IPU4_STREAM_REQUIRED:-STREAM:qbuf}"
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
