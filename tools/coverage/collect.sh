#!/usr/bin/env bash
# Turn the gcov tarball(s) harvested by init.streamon (and optionally
# init.kunit-cov) into an lcov HTML report.
#
# Usage: tools/coverage/collect.sh [gcov.tar] [output-dir]
#        tools/coverage/collect.sh --kunit-tar <kunit-gcov.tar> [gcov.tar] [output-dir]
#
# The guest tars /sys/kernel/debug/gcov/ whose directory layout mirrors
# the absolute path the kernel was compiled under. On the same host
# filesystem (which includes every CI run), that path also points at
# the build tree where the matching .gcno files already live. This
# script extracts the tar and overlays each .gcda onto its .gcno
# neighbour so `lcov --capture --directory $BUILD_TREE` can pair them.
#
# The kunit kernel is built under tools/linux/.kunit/ rather than
# tools/linux/, so its .gcno files live at a different absolute path.
# When --kunit-tar is supplied, both build trees are captured and the
# resulting .info files are merged before the IPU4-only --extract pass.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

KUNIT_TAR=""
if [[ "${1:-}" == "--kunit-tar" ]]; then
	KUNIT_TAR="$2"
	shift 2
fi

GCOV_TAR="${1:-$ROOT/tools/tests/out/gcov.tar}"
OUT="${2:-$ROOT/tools/coverage/html}"

IPU4_DRV="$ROOT/tools/linux/drivers/media/pci/intel/ipu4"
KUNIT_LINUX="$ROOT/tools/linux/.kunit"
KUNIT_DRV="$KUNIT_LINUX/drivers/media/pci/intel/ipu4"

if [[ ! -s "$GCOV_TAR" ]]; then
	echo "no gcov tar at $GCOV_TAR" >&2
	exit 1
fi
if [[ ! -d "$IPU4_DRV" ]]; then
	echo "build tree $IPU4_DRV missing; run tools/build-kernel.sh first" >&2
	exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# Overlay .gcda from $1 (a host-tree of /sys/kernel/debug/gcov/) onto
# the matching .gcno files already on disk under either build tree.
# Returns the count of overlaid files via stdout.
overlay_gcda() {
	local tar_path="$1" stage="$2" overlaid=0 skipped=0
	mkdir -p "$stage"
	tar -xf "$tar_path" -C "$stage"
	while IFS= read -r gcda; do
		local dst="${gcda#$stage}"
		local gcno="${dst%.gcda}.gcno"
		if [[ -f "$gcno" ]]; then
			install -m 0644 "$gcda" "$dst"
			overlaid=$((overlaid + 1))
		else
			skipped=$((skipped + 1))
		fi
	done < <(find "$stage" -name '*.gcda')
	echo ">>> overlaid $overlaid .gcda from $(basename "$tar_path"), skipped $skipped" >&2
	echo "$overlaid"
}

streamon_overlaid="$(overlay_gcda "$GCOV_TAR" "$WORK/streamon")"
kunit_overlaid=0
if [[ -n "$KUNIT_TAR" ]]; then
	if [[ -s "$KUNIT_TAR" ]]; then
		if [[ ! -d "$KUNIT_DRV" ]]; then
			echo "kunit build tree $KUNIT_DRV missing — kunit gcda will not match" >&2
		fi
		kunit_overlaid="$(overlay_gcda "$KUNIT_TAR" "$WORK/kunit")"
	else
		echo "kunit gcov tar empty/missing at $KUNIT_TAR — skipping kunit merge" >&2
	fi
fi

if (( streamon_overlaid == 0 && kunit_overlaid == 0 )); then
	echo "no .gcda matched any build tree; aborting" >&2
	exit 2
fi

INFO="$WORK/ipu4.info"
INFO_STREAMON="$WORK/streamon.info"
INFO_KUNIT="$WORK/kunit.info"

lcov --quiet --capture \
	--directory "$IPU4_DRV" \
	--output-file "$INFO_STREAMON"

if (( kunit_overlaid > 0 )) && [[ -d "$KUNIT_DRV" ]]; then
	lcov --quiet --capture \
		--directory "$KUNIT_DRV" \
		--output-file "$INFO_KUNIT"
	# Rewrite the kunit build-tree paths to match the streamon build
	# tree so lcov --add-tracefile sums the two by source file rather
	# than treating them as separate translation units.
	sed -i "s|$KUNIT_LINUX|$ROOT/tools/linux|g" "$INFO_KUNIT"
	lcov --quiet \
		--add-tracefile "$INFO_STREAMON" \
		--add-tracefile "$INFO_KUNIT" \
		--output-file "$INFO"
else
	cp "$INFO_STREAMON" "$INFO"
fi

# Narrow the report to the IPU4 driver sources — lcov --capture also
# pulls in the v4l2 core and other kernel files linked into the same
# build, which isn't what we're measuring.
lcov --quiet \
	--extract "$INFO" '*/drivers/media/pci/intel/ipu4/*' \
	--output-file "$INFO"

rm -rf "$OUT"
mkdir -p "$OUT"
genhtml --quiet --output-directory "$OUT" "$INFO"

echo ">>> coverage report at $OUT/index.html"

# Capture lcov --summary so the dashboard can pick up a stable
# JSON contract instead of scraping genhtml output. Lines look like:
#   lines.......: 33.1% (123 of 456 lines)
#   functions...: 40.7% (12 of 30 functions)
#   branches....: 18.7% (45 of 240 branches)
SUMMARY_TXT="$WORK/summary.txt"
lcov --summary "$INFO" 2>&1 | tee "$SUMMARY_TXT"
SUMMARY_JSON="$(dirname "$OUT")/summary.json"
python3 - "$SUMMARY_TXT" "$SUMMARY_JSON" <<'PY'
import json, re, sys
text = open(sys.argv[1]).read()
out = {}
for kind in ("lines", "functions", "branches"):
    m = re.search(rf"^\s*{kind}[.\s]*:\s*([\d.]+)%\s*\((\d+)\s+of\s+(\d+)", text, re.M)
    if m:
        out[kind] = {"pct": float(m.group(1)), "hit": int(m.group(2)), "total": int(m.group(3))}
with open(sys.argv[2], "w") as f:
    json.dump(out, f, indent=2)
PY
echo ">>> coverage summary at $SUMMARY_JSON"
