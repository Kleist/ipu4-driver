#!/usr/bin/env bash
# Bootstrap the ipu6 flavor: fetch the pinned kernel tree into
# tools/linux-ipu6/, reset it to the pin and `git am` every
# patches/ipu6-ipu4/*.patch in order.
#
# Idempotent: every run starts again from the pin, so the tree always
# equals pin + current series. Files the series does not touch keep their
# mtimes, so a cached build only recompiles what the series changes.
#
# IPU4_IPU6_PIN (default tools/ipu6/pin) and IPU4_IPU6_PATCH_DIR (default
# patches/ipu6-ipu4; empty for no series) select another base, together
# with IPU4_IPU6_LINUX_DIR; tools/tests/ipu6/ladder.sh sets all three per
# QEMU model variant (the ipu6 variant runs on unpatched mainline).
#
# Exit codes: 0 ok, 1 harness/fetch error, 3 the series does not apply.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
# shellcheck source=pin
. "${IPU4_IPU6_PIN:-$HERE/pin}"

LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu6}"
PATCH_DIR="${IPU4_IPU6_PATCH_DIR-$ROOT/patches/ipu6-ipu4}"
BRANCH="ipu6-ipu4"

# git am records a committer; CI runners have no identity configured.
export GIT_COMMITTER_NAME="${GIT_COMMITTER_NAME:-ipu4 harness}"
export GIT_COMMITTER_EMAIL="${GIT_COMMITTER_EMAIL:-ipu4-harness@invalid}"

if [[ ! -d "$LINUX_DIR/.git" ]]; then
	echo ">>> initialising $LINUX_DIR"
	git init -q "$LINUX_DIR"
fi

if ! git -C "$LINUX_DIR" cat-file -e "$IPU6_LINUX_SHA^{commit}" 2>/dev/null; then
	echo ">>> fetching $IPU6_LINUX_URL @ $IPU6_LINUX_SHA"
	git -C "$LINUX_DIR" fetch -q --depth 1 "$IPU6_LINUX_URL" "$IPU6_LINUX_SHA"
fi

# Drop any half-finished `git am` from an earlier failed run.
if [[ -d "$LINUX_DIR/.git/rebase-apply" ]]; then
	git -C "$LINUX_DIR" am --abort || rm -rf "$LINUX_DIR/.git/rebase-apply"
fi

# A cache-restored tree (actions/cache in vm-ipu6.yml) has the right
# contents and mtimes but new inodes and ctimes, so every index entry
# looks stat-dirty and `checkout -f` rewrites every file. That gives all
# sources a fresh mtime and turns the warm guest-kernel build into a
# full rebuild (~6 min in CI). Refreshing the index first re-hashes the
# files once and records their new stat data, so the checkout only
# touches files that really differ.
git -C "$LINUX_DIR" update-index -q --refresh > /dev/null || true

echo ">>> resetting $LINUX_DIR to $IPU6_LINUX_SHA"
git -C "$LINUX_DIR" checkout -q -f -B "$BRANCH" "$IPU6_LINUX_SHA"
# Files added by a previously applied series are untracked now; remove
# them so `git am` can recreate them. Ignored files (build output,
# .config) stay.
git -C "$LINUX_DIR" clean -q -fd

patches=()
if [[ -n "$PATCH_DIR" ]]; then
	shopt -s nullglob
	patches=("$PATCH_DIR"/*.patch)
	shopt -u nullglob
fi

if [[ -z "$PATCH_DIR" ]]; then
	echo ">>> no series for this base; tree is the bare pin"
elif (( ${#patches[@]} == 0 )); then
	echo ">>> no patches in ${PATCH_DIR#"$ROOT"/}; tree is the bare pin"
else
	echo ">>> applying ${#patches[@]} patch(es) from ${PATCH_DIR#"$ROOT"/}"
	if ! git -C "$LINUX_DIR" am -q "${patches[@]}"; then
		echo "!!! series does not apply on $IPU6_LINUX_SHA" >&2
		git -C "$LINUX_DIR" am --show-current-patch=diff >&2 || true
		git -C "$LINUX_DIR" am --abort || true
		exit 3
	fi
fi

echo ">>> bootstrap (ipu6 flavor) complete"
git -C "$LINUX_DIR" log --oneline -n $(( ${#patches[@]} + 1 ))
