#!/usr/bin/env bash
# Tier-2 helper: boot the QEMU harness, capture its MMIO stream, and diff
# against data/trace.txt (the real-silicon capture). Produces the
# "unimplemented address" worklist that drives ipu4.c model extension.
#
# Short-circuits when CONFIG_VIDEO_IPU4 isn't wired into the in-tree
# Kconfig yet, same way tools/tests/kunit.sh does. Probe cannot run
# against the stock IPU6 Kconfig even with our model, so running the
# harness produces only noise until the CONFIG lands.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
cd "$ROOT"

KCONFIG="tools/linux/drivers/media/pci/intel/Kconfig"
if ! grep -q 'VIDEO_IPU4' "$KCONFIG" 2>/dev/null; then
	echo "compare-mmio: skipping (CONFIG_VIDEO_IPU4 not wired into $KCONFIG)"
	exit 0
fi

SILICON_TRACE="$ROOT/data/trace.txt"
if [[ ! -f "$SILICON_TRACE" ]]; then
	echo "compare-mmio: silicon trace missing at $SILICON_TRACE" >&2
	exit 2
fi

OUT="$ROOT/tools/tests/out/compare-mmio"
mkdir -p "$OUT"

# Decode the silicon trace into the JSONL schema compare.py expects.
"$ROOT/postprocess_trace.py" "$SILICON_TRACE" --json > "$OUT/silicon.jsonl"

# Capture QEMU's trace. The expected source is a guest-side ftrace
# mmiotrace over the IPU4 BAR, shaped exactly like data/trace.txt so
# the same decoder works on both sides. Until tools/rootfs/build.sh
# emits one (see STATUS.md M5+), fall through with an empty capture —
# compare.py will report every silicon address as "unimplemented",
# which is still the useful worklist.
QEMU_TRACE="${QEMU_TRACE:-$OUT/qemu.trace}"
if [[ ! -s "$QEMU_TRACE" ]]; then
	echo "compare-mmio: no QEMU trace at $QEMU_TRACE; reporting silicon-only baseline"
	: > "$OUT/qemu.jsonl"
else
	"$ROOT/postprocess_trace.py" "$QEMU_TRACE" --json > "$OUT/qemu.jsonl"
fi

echo "compare-mmio: silicon=$OUT/silicon.jsonl qemu=$OUT/qemu.jsonl"
exec python3 "$ROOT/tools/trace/compare.py" "$OUT/silicon.jsonl" "$OUT/qemu.jsonl"
