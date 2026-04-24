#!/usr/bin/env bash
# Capture a QEMU-side mmiotrace for compare.py to diff against
# data/trace.txt. Mirrors the probe-smoke.sh pattern: rebuild the
# initramfs with the mmiotrace init, boot the VM with the mmiotrace
# buffer size bumped, wait for the guest to dump the trace to the 9p
# share, then validate the capture is non-empty and well-formed.
#
# Output: tools/tests/out/mmiotrace/qemu.trace — feeds directly into
# tools/tests/compare-mmio.sh.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="$HERE/out/mmiotrace"
mkdir -p "$OUT"

LOG="$OUT/serial"
TRACE="$OUT/qemu.trace"
rm -f "$LOG" "$TRACE"

fail() {
	# ::error:: promotes the reason to a GitHub Actions annotation so
	# it shows up on the check-runs annotations API — the only surface
	# that exposes the failure body without an artifact download.
	local reason="$*"
	echo "::error title=mmiotrace capture failed::$reason"
	echo "mmiotrace: FAIL ($reason)" >&2
	echo "--- last 40 lines of $LOG ---" >&2
	tail -n 40 "$LOG" >&2 2>/dev/null || true
	echo "--- ls -la $OUT ---" >&2
	ls -la "$OUT" >&2 2>/dev/null || true
	exit 1
}

IPU4_INIT=streamon-mmiotrace "$ROOT/tools/rootfs/build.sh" >/dev/null

# 300s because the init now runs the full streamon walk after the
# tracer is armed; the probe-only variant used to fit in 180s. If
# streamon hangs mid-ioctl we still want to time out while there's
# headroom for the guest to dump whatever the tracer captured up to
# that point.
TIMEOUT="${IPU4_MMIOTRACE_TIMEOUT:-300}"

# IPU4_TESTS_SHARE aims run-vm.sh's 9p share at $OUT so the guest's
# /mnt/tests/qemu.trace writes land exactly where we look for them.
set +e
IPU4_MMIOTRACE=1 IPU4_TESTS_SHARE="$OUT" \
	timeout --preserve-status "$TIMEOUT" "$ROOT/tools/run-vm.sh" 2>&1 | tee "$LOG"
rc=${PIPESTATUS[0]}
set -e

if ! grep -q '^MMIOTRACE: DONE' "$LOG"; then
	fail "guest did not reach MMIOTRACE: DONE marker (rc=$rc)"
fi

if grep -q '^MMIOTRACE: FAIL' "$LOG"; then
	# The guest init emits "MMIOTRACE: FAIL (reason)" with a specific
	# cause — surface it prominently.
	guest_fail=$(grep '^MMIOTRACE: FAIL' "$LOG" | head -n1)
	fail "guest reported $guest_fail"
fi

if [[ ! -s "$TRACE" ]]; then
	fail "no trace at $TRACE (9p share not syncing?)"
fi

# Silicon (data/trace.txt) was captured with the IPU4 BAR at
# 0x90000000. QEMU's PCI enumeration assigns whatever the topology
# leaves free — typically 0xfb000000 on ubuntu-24.04 q35. Detect the
# QEMU-side BAR from the first MAP line and rebase addresses to the
# silicon base so NAMED_REGIONS in postprocess_trace.py resolves and
# compare.py can actually diff the two streams.
python3 - "$TRACE" <<'PY'
import re
import sys

path = sys.argv[1]
TARGET_BAR = 0x90000000
BAR_SIZE = 0x1000000  # 16 MiB — matches IPU4_BAR_SIZE in ipu4.c.

with open(path) as f:
    lines = f.readlines()

# First MAP line's phys address column is our QEMU-side BAR.
# Format: "MAP <secs>.<usec> <map_id> 0x<phys> 0x<virt> 0x<size> ..."
src_bar = None
for line in lines:
    parts = line.split()
    if len(parts) >= 4 and parts[0] == "MAP":
        m = re.match(r"^0x([0-9a-fA-F]+)$", parts[3])
        if m:
            src_bar = int(m.group(1), 16)
            break

if src_bar is None:
    # Nothing to rebase; leave file alone.
    sys.exit(0)

if src_bar == TARGET_BAR:
    sys.exit(0)

delta = TARGET_BAR - src_bar
print(f"mmiotrace: rebasing 0x{src_bar:x} -> 0x{TARGET_BAR:x} (delta {delta:+d})",
      file=sys.stderr)

def rebase(m):
    v = int(m.group(0), 16)
    if src_bar <= v < src_bar + BAR_SIZE:
        return f"0x{v + delta:x}"
    return m.group(0)

with open(path, "w") as f:
    for line in lines:
        f.write(re.sub(r"0x[0-9a-fA-F]+", rebase, line))
PY

# Validator 1: at least one R/W line textually looks like what we
# expect (`^[RW] .*0x90xxxxxx`). postprocess_trace.py skips every
# non-R/W header line via _parse_rw so missing VERSION/PCIDEV headers
# are cosmetic, not semantic.
if ! grep -qE '^[RW] .*0x90[0-9a-fA-F]{6}' "$TRACE"; then
	head_snippet=$(head -n 15 "$TRACE" | tr '\n' '|')
	size=$(wc -c < "$TRACE")
	fail "no IPU4-BAR R/W lines in $TRACE (size=$size, head: $head_snippet)"
fi

# Validator 2: postprocess_trace.py actually parses at least one R/W
# line from the trace. The textual grep above and _parse_rw's field-
# position logic are distinct — if the mmiotrace output has an extra
# column or a timestamp format we don't handle, the grep passes but
# every record gets dropped and compare.py sees an empty QEMU stream.
# (Exactly what happened on the first passing run: qemu.jsonl ended
# up 0 bytes even though the trace had 1376 entries.) Parse-count
# failing here is strict — block merge until the format is fixed.
parsed=$("$ROOT/postprocess_trace.py" "$TRACE" --json 2>/dev/null | wc -l)
if [ "$parsed" -eq 0 ]; then
	matching=$(grep -cE '^[RW] .*0x90[0-9a-fA-F]{6}' "$TRACE")
	head_snippet=$(grep -E '^[RW]' "$TRACE" | head -n 3 | tr '\n' '|')
	fail "postprocess_trace.py parsed 0 records despite $matching R/W-looking lines in $TRACE (sample: $head_snippet)"
fi

lines=$(wc -l < "$TRACE")
echo "mmiotrace: PASS ($lines raw lines, $parsed parsed records, trace=$TRACE)"
exit "$rc"
