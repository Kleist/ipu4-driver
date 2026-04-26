#!/usr/bin/env bash
# Produce a readable diff between kernel/ipu4/ (the fork) and upstream
# drivers/media/pci/intel/ipu6/ at the pinned upstream tag. Scope: only files
# that exist in both directories with the same name. IPU4-only files are
# skipped because they have no upstream peer to diff against.
#
# Output: tools/notes/upstream-diff/{summary.md, per-file/<name>.diff}.
# Both are gitignored — regenerated on demand, never committed.
#
# Override the pinned upstream tag for a one-off comparison:
#   IPU4_LINUX_TAG=v6.13 tools/upstream/diff.sh

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
# shellcheck source=tools/upstream/_lib.sh
source "$HERE/_lib.sh"

ensure_linux_clone "$ROOT"

UPSTREAM_DIR="$ROOT/tools/linux/$UPSTREAM_REL"
LOCAL_DIR="$ROOT/$LOCAL_REL"
OUT="$ROOT/tools/notes/upstream-diff"

if [[ ! -d "$UPSTREAM_DIR" ]]; then
	echo "error: $UPSTREAM_DIR does not exist; bootstrap may have failed" >&2
	exit 1
fi

rm -rf "$OUT"
mkdir -p "$OUT/per-file"

shared=()
local_only_skipped=()
no_upstream_peer=()
for path in "$LOCAL_DIR"/*.[ch]; do
	[[ -e "$path" ]] || continue
	base="$(basename "$path")"
	if is_local_only "$base"; then
		local_only_skipped+=("$base")
		continue
	fi
	if [[ ! -e "$UPSTREAM_DIR/$base" ]]; then
		no_upstream_peer+=("$base")
		continue
	fi
	shared+=("$base")
done

for f in "${shared[@]}"; do
	# git diff --no-index returns 1 on differences, 0 on identical, >1 on error.
	# We only want to fail on >1, so capture the exit status.
	rc=0
	git diff --no-index --no-color \
		--src-prefix="upstream/" --dst-prefix="local/" \
		"$UPSTREAM_DIR/$f" "$LOCAL_DIR/$f" \
		> "$OUT/per-file/$f.diff" || rc=$?
	if (( rc > 1 )); then
		echo "error: git diff failed on $f (rc=$rc)" >&2
		exit "$rc"
	fi
done

upstream_describe="$(git -C "$ROOT/tools/linux" describe --tags --always 2>/dev/null || echo unknown)"

{
	echo "# kernel/ipu4 vs drivers/media/pci/intel/ipu6"
	echo
	echo "Upstream pin: \`$upstream_describe\`"
	echo "Generated: $(date -u +'%Y-%m-%dT%H:%M:%SZ')"
	echo
	echo "## Per-file divergence"
	echo
	echo "| File | Hunks | + | − | Diff |"
	echo "|---|---:|---:|---:|---|"
	total_hunks=0
	total_plus=0
	total_minus=0
	for f in "${shared[@]}"; do
		d="$OUT/per-file/$f.diff"
		hunks=$(grep -c '^@@' "$d" || true)
		plus=$(grep -cE '^\+[^+]' "$d" || true)
		minus=$(grep -cE '^-[^-]' "$d" || true)
		total_hunks=$(( total_hunks + hunks ))
		total_plus=$(( total_plus + plus ))
		total_minus=$(( total_minus + minus ))
		if (( hunks == 0 )); then
			# No divergence — drop the dead per-file diff to keep the tree tidy.
			rm -f "$d"
			echo "| \`$f\` | 0 | 0 | 0 | _identical_ |"
		else
			echo "| \`$f\` | $hunks | $plus | $minus | [diff](per-file/$f.diff) |"
		fi
	done
	echo "| **total** | **$total_hunks** | **$total_plus** | **$total_minus** | |"
	echo

	if (( ${#local_only_skipped[@]} )); then
		echo "## Skipped (IPU4-only, no upstream peer)"
		echo
		for f in "${local_only_skipped[@]}"; do
			echo "- \`$f\`"
		done
		echo
	fi

	if (( ${#no_upstream_peer[@]} )); then
		echo "## Unexpected: present locally but not upstream"
		echo
		echo "These files are not in \`IPU4_LOCAL_ONLY\` but also have no upstream peer at the pinned tag. Either add them to \`IPU4_LOCAL_ONLY\` in \`tools/upstream/_lib.sh\` or check whether upstream renamed them."
		echo
		for f in "${no_upstream_peer[@]}"; do
			echo "- \`$f\`"
		done
		echo
	fi
} > "$OUT/summary.md"

echo ">>> wrote $OUT/summary.md (${#shared[@]} forked files)"
