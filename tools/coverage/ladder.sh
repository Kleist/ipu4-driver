#!/usr/bin/env bash
# Line coverage of one ipu6-ladder leg: turn the gcov tars the ladder's
# probe and stream boots leave on the 9p share (tools/rootfs/gcov-harvest)
# into an lcov report of the driver.
#
# Usage: tools/coverage/ladder.sh DRV_DIR OUT_DIR TAR...
#
# DRV_DIR is the driver's build directory (its .gcno files; the guest's
# .gcda paths mirror it). Writes OUT_DIR/html/ and OUT_DIR/summary.json
# ({"lines": {"pct", "hit", "total"}, "functions": ...}). Run by
# tools/tests/ipu6/ladder.sh when IPU4_GCOV=1.
set -euo pipefail

DRV_DIR="$(cd "$1" && pwd)"
OUT="$2"
shift 2

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# Each boot's counters get their own copy of the build directory, so
# lcov sums the boots instead of one overwriting the other.
infos=()
i=0
for tar in "$@"; do
	[[ -s "$tar" ]] || continue
	i=$((i + 1))
	stage="$WORK/stage$i"
	mkdir -p "$stage"
	tar -xf "$tar" -C "$stage"
	if [[ ! -d "$stage$DRV_DIR" ]]; then
		echo "${tar##*/}: no counters for $DRV_DIR" >&2
		continue
	fi
	cp "$DRV_DIR"/*.gcno "$stage$DRV_DIR/"
	lcov --quiet --capture --directory "$stage$DRV_DIR" \
		--output-file "$WORK/$i.info"
	sed -i "s|$stage||g" "$WORK/$i.info"
	infos+=(--add-tracefile "$WORK/$i.info")
done
if (( ${#infos[@]} == 0 )); then
	echo "no gcov counters for $DRV_DIR" >&2
	exit 2
fi

INFO="$WORK/all.info"
lcov --quiet "${infos[@]}" --output-file "$INFO"
# Only the driver's own sources: not kernel headers, not the virt
# sensor built into the out-of-tree module for QEMU.
lcov --quiet --extract "$INFO" "$DRV_DIR/*" --output-file "$INFO"
lcov --quiet --ignore-errors unused --remove "$INFO" '*/virt-sensor.c' \
	--output-file "$INFO"

rm -rf "$OUT"
mkdir -p "$OUT"
cp "$INFO" "$OUT/lcov.info"
genhtml --quiet --output-directory "$OUT/html" "$INFO"
lcov --summary "$INFO" 2>&1 | tee "$WORK/summary.txt"
python3 - "$WORK/summary.txt" "$OUT/summary.json" <<'PY'
import json, re, sys
text = open(sys.argv[1]).read()
out = {}
for kind in ("lines", "functions"):
    m = re.search(rf"^\s*{kind}[.\s]*:\s*([\d.]+)%\s*\((\d+)\s+of\s+(\d+)", text, re.M)
    if m:
        out[kind] = {"pct": float(m.group(1)), "hit": int(m.group(2)),
                     "total": int(m.group(3))}
json.dump(out, open(sys.argv[2], "w"), indent=2)
PY
echo ">>> coverage report at $OUT/html/index.html"
