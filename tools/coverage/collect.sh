#!/usr/bin/env bash
# Turn the gcov tarball harvested by e2e.sh into an lcov HTML report.
# Usage: tools/coverage/collect.sh [gcov.tar] [output-dir]
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

GCOV_TAR="${1:-$ROOT/tools/tests/out/gcov.tar}"
OUT="${2:-$ROOT/tools/coverage/html}"

if [[ ! -s "$GCOV_TAR" ]]; then
	echo "no gcov tar at $GCOV_TAR" >&2
	exit 1
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

tar -xf "$GCOV_TAR" -C "$WORK"

INFO="$WORK/ipu4.info"
lcov --quiet --capture \
	--directory "$WORK" \
	--base-directory "$ROOT/tools/linux" \
	--output-file "$INFO"

lcov --quiet \
	--extract "$INFO" '*/drivers/media/pci/intel/ipu4/*' \
	--output-file "$INFO"

rm -rf "$OUT"
mkdir -p "$OUT"
genhtml --quiet --output-directory "$OUT" "$INFO"

echo ">>> coverage report at $OUT/index.html"
lcov --summary "$INFO"
