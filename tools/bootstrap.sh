#!/usr/bin/env bash
# Bootstrap the dev/test harness: set up forked Linux and QEMU submodules,
# apply the patches carried in tools/linux-patches/ and tools/qemu-patches/.
#
# This script is idempotent and expects to be re-run after pulling.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

LINUX_URL="${IPU4_LINUX_URL:-https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git}"
LINUX_TAG="${IPU4_LINUX_TAG:-v6.12}"
LINUX_BRANCH="${IPU4_LINUX_BRANCH:-ipu4-6.12}"

QEMU_URL="${IPU4_QEMU_URL:-https://gitlab.com/qemu-project/qemu.git}"
QEMU_TAG="${IPU4_QEMU_TAG:-v9.1.0}"
QEMU_BRANCH="${IPU4_QEMU_BRANCH:-ipu4-qemu}"

LINUX_DIR="$ROOT/tools/linux"
QEMU_DIR="$ROOT/tools/qemu"

clone_if_missing() {
	local dir="$1" url="$2" tag="$3" branch="$4"
	if [[ -d "$dir/.git" ]]; then
		return
	fi
	echo ">>> cloning $url -> $dir @ $tag"
	git clone --depth 1 --branch "$tag" "$url" "$dir"
	git -C "$dir" checkout -B "$branch"
}

apply_patch_tree() {
	local src="$1" dst="$2"
	if [[ ! -d "$src" ]]; then
		return
	fi
	echo ">>> applying $src -> $dst"
	(cd "$src" && find . -type f -print0) | while IFS= read -r -d '' rel; do
		install -D -m 0644 "$src/$rel" "$dst/$rel"
	done
}

clone_if_missing "$LINUX_DIR" "$LINUX_URL"  "$LINUX_TAG"  "$LINUX_BRANCH"
clone_if_missing "$QEMU_DIR"  "$QEMU_URL"   "$QEMU_TAG"   "$QEMU_BRANCH"

apply_patch_tree "$ROOT/tools/linux-patches" "$LINUX_DIR"
apply_patch_tree "$ROOT/tools/qemu-patches"  "$QEMU_DIR"

# Copy the driver sources from kernel/ipu4/ into the in-tree path on first run.
# Source files are not stored under linux-patches/ to avoid confusing diffs
# against upstream IPU6; they are synced here with a rename pass.
#
# ipu4-compat.h is kept intact: on v6.12 its live branch is a single
# macro definition (v4l2_subdev_get_pad_format) that is not referenced
# by any .c file, so it is a no-op. Cleaning it up is tracked as part
# of a later upstreaming pass.
DRV_DST="$LINUX_DIR/drivers/media/pci/intel/ipu4"
if ! compgen -G "$DRV_DST/ipu6*.c" > /dev/null; then
	echo ">>> seeding driver sources into $DRV_DST"
	mkdir -p "$DRV_DST"
	cp "$ROOT"/kernel/ipu4/*.[ch] "$DRV_DST/"
fi

echo ">>> bootstrap complete"
echo "linux: $LINUX_DIR  branch=$LINUX_BRANCH"
echo "qemu:  $QEMU_DIR   branch=$QEMU_BRANCH"
