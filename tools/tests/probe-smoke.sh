#!/usr/bin/env bash
# Probe-progress smoke test. Boots the VM with an init that inserts
# intel-ipu4.ko and reports how far probe got. Each milestone raises
# the default required checkpoint:
#
#   M3: probe reached ipu6_cpd_copy_binary() (probe:fw_load).
#   M4: CPD blob accepted (probe:fw_valid).
#   M4.5: virt-sensor replaces ambu-bridge so probe moves past the
#         fwnode-graph check (probe:virt_sensor).
#   M5:   PWR_STATE constant in hw/misc/ipu4.c unblocks the buttress
#         power-FSM polls, probe completes, v4l2 core registers
#         /dev/video* (probe:video_node, the current default).
#
# A lower `IPU4_PROBE_REQUIRED` keeps the test green during regressions;
# the default is tightened as new pieces land.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
OUT="$HERE/out"
mkdir -p "$OUT"

LOG="$OUT/probe-smoke.serial"
rm -f "$LOG"

IPU4_INIT=probe-ok "$ROOT/tools/rootfs/build.sh" >/dev/null

TIMEOUT="${IPU4_PROBE_TIMEOUT:-120}"

set +e
timeout --preserve-status "$TIMEOUT" "$ROOT/tools/run-vm.sh" 2>&1 | tee "$LOG"
set -e

# Progress markers, ordered from earliest to furthest-along. The last
# one that matches is the actual checkpoint reached. The regexes match
# kernel log lines printed on the probe path plus the PROBE_OK: markers
# tools/rootfs/init.probe-ok prints after driver load.
markers=(
	"probe:entered|PCI bar\[0\] = "
	"probe:ipc_reset|IPC reset done"
	"probe:fw_load|FW version:"
	"probe:virt_sensor|virt-sensor installed"
	"probe:bound|PROBE_OK: driver bound"
	"probe:video_node|PROBE_OK: video=/dev/video"
)

reached=""
for m in "${markers[@]}"; do
	name="${m%%|*}"
	re="${m#*|}"
	if grep -qE "$re" "$LOG"; then
		reached="$name"
	fi
done

echo "probe-smoke: reached=$reached"

# Default is `probe:video_node` — with M5's PWR_STATE "all ready"
# constant the probe path completes through psys_init and video_device
# registration. The power-down poll in ipu6_buttress_power(on=false)
# logs a cosmetic "Change power status timeout" because our constant
# never reads 0; probe ignores it and continues.
REQUIRED="${IPU4_PROBE_REQUIRED:-probe:video_node}"
case "$reached" in
"")
	echo "probe-smoke: FAIL (driver did not reach any progress marker)" >&2
	exit 1
	;;
esac

idx_of() {
	local target="$1" i=0
	for m in "${markers[@]}"; do
		[[ "${m%%|*}" == "$target" ]] && { echo "$i"; return; }
		i=$((i + 1))
	done
	echo -1
}
req_idx=$(idx_of "$REQUIRED")
got_idx=$(idx_of "$reached")
if (( got_idx < req_idx )); then
	echo "probe-smoke: FAIL (reached=$reached, required=$REQUIRED)" >&2
	exit 1
fi
echo "probe-smoke: PASS (reached=$reached, required=$REQUIRED)"
