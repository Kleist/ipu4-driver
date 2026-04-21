#!/usr/bin/env bash
# Turn the gcov tarball harvested by init.streamon into an lcov HTML
# report.  Usage: tools/coverage/collect.sh [gcov.tar] [output-dir]
#
# The guest tars /sys/kernel/debug/gcov/ whose directory layout mirrors
# the absolute path the kernel was compiled under. On the same host
# filesystem (which includes every CI run), that path also points at
# the build tree where the matching .gcno files already live. This
# script extracts the tar and overlays each .gcda onto its .gcno
# neighbour so `lcov --capture --directory $BUILD_TREE` can pair them.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

GCOV_TAR="${1:-$ROOT/tools/tests/out/gcov.tar}"
OUT="${2:-$ROOT/tools/coverage/html}"

IPU4_DRV="$ROOT/tools/linux/drivers/media/pci/intel/ipu4"

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

tar -xf "$GCOV_TAR" -C "$WORK"

# Overlay .gcda files onto the build tree. Paths in the tar mirror
# the absolute path the kernel was compiled at; stripping the $WORK
# prefix yields that same absolute path. Skip entries that don't
# correspond to an existing .gcno (other kernel subsystems, stale
# gcov state).
overlaid=0
skipped=0
while IFS= read -r gcda; do
	dst="${gcda#$WORK}"
	gcno="${dst%.gcda}.gcno"
	if [[ -f "$gcno" ]]; then
		install -m 0644 "$gcda" "$dst"
		overlaid=$((overlaid + 1))
	else
		skipped=$((skipped + 1))
	fi
done < <(find "$WORK" -name '*.gcda')

echo ">>> overlaid $overlaid .gcda files, skipped $skipped"

if (( overlaid == 0 )); then
	echo "no .gcda matched the build tree; aborting" >&2
	exit 2
fi

INFO="$WORK/ipu4.info"
lcov --quiet --capture \
	--directory "$IPU4_DRV" \
	--output-file "$INFO"

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
lcov --summary "$INFO"
