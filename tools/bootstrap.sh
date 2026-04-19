#!/usr/bin/env bash
# Bootstrap the dev/test harness: set up forked Linux and QEMU submodules,
# apply the patches carried in tools/linux-patches/ and tools/qemu-patches/.
#
# This script is idempotent and expects to be re-run after pulling.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

LINUX_URL="${IPU4_LINUX_URL:-https://github.com/torvalds/linux.git}"
LINUX_TAG="${IPU4_LINUX_TAG:-v6.12}"
LINUX_BRANCH="${IPU4_LINUX_BRANCH:-ipu4-6.12}"

QEMU_URL="${IPU4_QEMU_URL:-https://github.com/qemu/qemu.git}"
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
		local s="$src/$rel" d="$dst/$rel"
		# cmp-before-install so unchanged files keep their mtimes; the
		# CI cache would otherwise rebuild every dependent object.
		if ! cmp -s "$s" "$d" 2>/dev/null; then
			install -D -m 0644 "$s" "$d"
		fi
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
mkdir -p "$DRV_DST"
echo ">>> seeding driver sources into $DRV_DST"
# cmp-before-cp (see apply_patch_tree) so unchanged files keep their
# mtimes — otherwise the cached kernel build in CI rebuilds every
# ipu4 object on every run.
for src in "$ROOT"/kernel/ipu4/*.[ch]; do
	dst="$DRV_DST/$(basename "$src")"
	if ! cmp -s "$src" "$dst" 2>/dev/null; then
		cp "$src" "$dst"
	fi
done

# Wire CONFIG_VIDEO_INTEL_IPU4 into the parent Kconfig and Makefile so
# `make modules` / kunit.py descend into ipu4/. Both edits are appended
# only if not already present so re-runs are no-ops.
PARENT_KCONFIG="$LINUX_DIR/drivers/media/pci/intel/Kconfig"
PARENT_MAKEFILE="$LINUX_DIR/drivers/media/pci/intel/Makefile"

if ! grep -q '^source "drivers/media/pci/intel/ipu4/Kconfig"$' "$PARENT_KCONFIG"; then
	echo ">>> wiring ipu4/Kconfig into $PARENT_KCONFIG"
	printf '\nsource "drivers/media/pci/intel/ipu4/Kconfig"\n' >> "$PARENT_KCONFIG"
fi
if ! grep -q '^obj-\$(CONFIG_VIDEO_INTEL_IPU4)\s*+=\s*ipu4/$' "$PARENT_MAKEFILE"; then
	echo ">>> wiring ipu4/ into $PARENT_MAKEFILE"
	printf 'obj-$(CONFIG_VIDEO_INTEL_IPU4)\t+= ipu4/\n' >> "$PARENT_MAKEFILE"
fi

# Wire CONFIG_IPU4 into QEMU's hw/misc/Kconfig and hw/misc/meson.build so
# the forked QEMU picks up the ipu4.c device model that apply_patch_tree
# just copied over. Idempotent.
QEMU_HW_KCONFIG="$QEMU_DIR/hw/misc/Kconfig"
QEMU_HW_MESON="$QEMU_DIR/hw/misc/meson.build"

if [[ -f "$QEMU_HW_KCONFIG" ]] && ! grep -q '^config IPU4$' "$QEMU_HW_KCONFIG"; then
	echo ">>> wiring ipu4 into $QEMU_HW_KCONFIG"
	cat >> "$QEMU_HW_KCONFIG" <<'EOF'

config IPU4
    bool
    default y if PCI_DEVICES
    depends on PCI && MSI_NONBROKEN
EOF
fi
if [[ -f "$QEMU_HW_MESON" ]] && ! grep -q "files('ipu4.c')" "$QEMU_HW_MESON"; then
	echo ">>> wiring ipu4.c into $QEMU_HW_MESON"
	printf "\nsystem_ss.add(when: 'CONFIG_IPU4', if_true: files('ipu4.c'))\n" >> "$QEMU_HW_MESON"
fi

echo ">>> bootstrap complete"
echo "linux: $LINUX_DIR  branch=$LINUX_BRANCH"
echo "qemu:  $QEMU_DIR   branch=$QEMU_BRANCH"
