#!/usr/bin/env bash
# Build the GitHub Pages dashboard at $OUT_DIR (default: ./site).
#
# Reads:
#   tools/coverage/summary.json                 ($COV_DIR/summary.json)
#   tools/coverage/html/                        ($COV_DIR/html/)  → site/coverage/
#   tools/tests/out/compare-mmio/report.json    ($MMIO_DIR/report.json)
#   tools/tests/out/compare-mmio/report.txt     ($MMIO_DIR/report.txt) → site/mmio/
#   tools/tests/out/ipu6-ladder-results/**/results.json  ($IPU6_LADDER_DIR,
#                                               vm-ipu6 artifacts) and
#   tools/tests/out/ipu6-ladder/results.json    (a local ladder.sh run)
#   tools/notes/registers.md
#   tools/notes/upstream-watch-state.json
#   tools/notes/upstream-diff/summary.md        (regenerated if missing)
#   .github/kernel-pins/*.json
#   GitHub Actions API (gh) — latest ci run, for the kernel tracks
#
# Writes:
#   $OUT_DIR/index.html            single-page dashboard
#   $OUT_DIR/style.css
#   $OUT_DIR/data/history.json     trend points (appended-to)
#   $OUT_DIR/coverage/             lcov HTML tree (copy)
#   $OUT_DIR/mmio/report.txt       compare-mmio plain-text report (copy)
#
# Inputs that don't exist degrade to "no data yet" placeholders rather
# than failing — that's the contract for first-time bring-up where
# vm-smoke hasn't published an artifact yet.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"

OUT_DIR="${OUT_DIR:-$ROOT/site}"
COV_DIR="${COV_DIR:-$ROOT/tools/coverage}"
MMIO_DIR="${MMIO_DIR:-$ROOT/tools/tests/out/compare-mmio}"
REPO_SLUG="${REPO_SLUG:-Kleist/ipu4-driver}"
VM_SMOKE_RUN_ID="${VM_SMOKE_RUN_ID:-}"
IPU6_LADDER_DIR="${IPU6_LADDER_DIR:-$ROOT/tools/tests/out/ipu6-ladder-results}"
VM_IPU6_RUN_ID="${VM_IPU6_RUN_ID:-}"

mkdir -p "$OUT_DIR" "$OUT_DIR/data"

# --- assemble the JSON payload for render.py ---
PAYLOAD="$(mktemp)"
trap 'rm -f "$PAYLOAD"' EXIT

python3 "$HERE/build_payload.py" \
  --root "$ROOT" \
  --out-dir "$OUT_DIR" \
  --cov-dir "$COV_DIR" \
  --mmio-dir "$MMIO_DIR" \
  --repo "$REPO_SLUG" \
  ${VM_SMOKE_RUN_ID:+--vm-smoke-run-id "$VM_SMOKE_RUN_ID"} \
  --ipu6-ladder-dir "$IPU6_LADDER_DIR" \
  --ipu6-ladder-dir "$ROOT/tools/tests/out/ipu6-ladder" \
  ${VM_IPU6_RUN_ID:+--vm-ipu6-run-id "$VM_IPU6_RUN_ID"} \
  > "$PAYLOAD"

# --- render ---
python3 "$HERE/render.py" --out "$OUT_DIR" --input "$PAYLOAD"

# --- copy auxiliary trees ---
if [[ -d "$COV_DIR/html" ]]; then
  rm -rf "$OUT_DIR/coverage"
  cp -r "$COV_DIR/html" "$OUT_DIR/coverage"
fi
if [[ -f "$MMIO_DIR/report.txt" ]]; then
  mkdir -p "$OUT_DIR/mmio"
  cp "$MMIO_DIR/report.txt" "$OUT_DIR/mmio/report.txt"
fi

# Disable Jekyll on the published Pages site — we serve the HTML
# verbatim and lcov puts files under directories like `gcov/` that
# Jekyll's _site filter treats specially.
touch "$OUT_DIR/.nojekyll"

echo ">>> dashboard at $OUT_DIR/index.html"
