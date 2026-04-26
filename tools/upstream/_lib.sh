# shellcheck shell=bash
# Shared helpers for tools/upstream/{diff,watch}.sh.
#
# Defines the canonical mapping between upstream `drivers/media/pci/intel/ipu6/`
# and our local fork `kernel/ipu4/`, and the list of files that exist only on
# our side (no upstream peer). Both diff and watch source this so the two
# stay consistent.

set -euo pipefail

UPSTREAM_REL="drivers/media/pci/intel/ipu6"
LOCAL_REL="kernel/ipu4"

# Files in kernel/ipu4/ with no upstream IPU6 counterpart. The diff tool skips
# these; the watcher treats upstream commits whose only overlap is here as N/A.
IPU4_LOCAL_ONLY=(
	ambu-ipu-bridge.c
	ambu-ipu-bridge.h
	ipu4-compat.h
	ipu4-platform-buttress-regs.h
	ipu4-platform-isys-csi2-reg.h
	ipu4-platform-regs.h
	virt-sensor.c
	virt-sensor.h
)

is_local_only() {
	local f
	for f in "${IPU4_LOCAL_ONLY[@]}"; do
		[[ "$1" == "$f" ]] && return 0
	done
	return 1
}

# Translate an upstream pathname under drivers/media/pci/intel/ipu6/ to its
# local equivalent under kernel/ipu4/. Returns empty string if the path is
# outside the upstream IPU6 directory.
ipu6_to_ipu4_path() {
	local p="$1"
	[[ "$p" == "$UPSTREAM_REL"/* ]] || { echo ""; return; }
	echo "$LOCAL_REL/${p#"$UPSTREAM_REL/"}"
}

# Ensure tools/linux/ exists. Calls bootstrap.sh idempotently.
# Bootstrap does a --depth 1 clone, which is enough for diff.sh (just reads
# files at the pinned tag). Watch.sh additionally calls deepen_linux_clone
# below because it needs commit history.
ensure_linux_clone() {
	local root="$1"
	local linux_dir="$root/tools/linux"
	if [[ ! -d "$linux_dir/.git" ]]; then
		"$root/tools/bootstrap.sh"
	fi
}

# Deepen the linux clone to full history. Idempotent — no-op once the clone
# is already complete.
deepen_linux_clone() {
	local root="$1"
	local linux_dir="$root/tools/linux"
	if [[ -f "$linux_dir/.git/shallow" ]]; then
		echo ">>> deepening tools/linux/ to full history (first run)" >&2
		git -C "$linux_dir" fetch --unshallow origin
	fi
}
