#!/usr/bin/env bash
# Resolve the latest upstream refs for each kernel CI pin and rewrite
# every workflow file that contains a matching `# bump-pin:<key>`
# comment. Prints a summary of (file, key, old, new) so the calling
# workflow can paste it into the auto-PR's body.
#
# Usage:
#   tools/bump-kernel-pins.sh                # rewrite every key
#   tools/bump-kernel-pins.sh --key 6.12     # rewrite only the 6.12 lines
#
# Files touched:
#   - .github/workflows/ci.yml             (build+kunit matrix; 6.12 + 6.18 + 7.0)
#   - .github/workflows/vm-smoke.yml       (PR caller; 6.12 only)
#   - .github/workflows/vm-smoke-weekly.yml (Sunday cron; 6.18 + 7.0 matrix)
#
# Resolution rules:
#   - 6.12  — highest v6.12(.X) release tag on git.kernel.org stable mirror.
#   - 6.18  — same with v6.18(.X).
#   - 7.0   — same with v7.0(.X).
#
# Lines we touch look like one of:
#     ref:  <value>  # bump-pin:<key>          (matrix entry)
#     linux-ref:  <value>  # bump-pin:<key>    (workflow_call `with:`)
# The bumper preserves the prefix and the trailing comment exactly;
# only <value> is replaced. The optional `(linux-)?` group makes the
# regex match thin-caller workflows as well as matrix entries.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
WORKFLOWS="$ROOT/.github/workflows"

STABLE_URL="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git"

ALL_KEYS=(6.12 6.18 7.0)

# --key <key> filter restricts both resolution and rewriting to a
# single track, so the workflow can fan out a PR-per-key matrix.
ONLY_KEY=""
while [[ $# -gt 0 ]]; do
	case "$1" in
		--key)
			ONLY_KEY="${2:-}"
			shift 2
			;;
		*)
			echo "::error::unknown argument: $1" >&2
			exit 2
			;;
	esac
done

if [[ -n "$ONLY_KEY" ]]; then
	found=0
	for k in "${ALL_KEYS[@]}"; do
		[[ "$k" == "$ONLY_KEY" ]] && found=1
	done
	if [[ $found -eq 0 ]]; then
		echo "::error::--key must be one of: ${ALL_KEYS[*]}" >&2
		exit 2
	fi
	KEYS=("$ONLY_KEY")
else
	KEYS=("${ALL_KEYS[@]}")
fi

# Highest non-rc tag matching <pattern> on the stable mirror.
# Args: $1 = pattern (e.g. 'v6.12*'), $2 = anchor regex (e.g. '^v6\.12(\.[0-9]+)?$').
latest_stable_tag() {
	local pattern="$1" anchor="$2"
	git ls-remote --tags --refs --sort=-v:refname "$STABLE_URL" "$pattern" \
	  | sed -E 's@^.*refs/tags/@@' \
	  | grep -E "$anchor" \
	  | head -n1
}

# bump-pin:<key>  =>  resolved value
declare -A NEW
for key in "${KEYS[@]}"; do
	track_re="${key//./\\.}"
	NEW[$key]="$(latest_stable_tag "v${key}*" "^v${track_re}(\\.[0-9]+)?\$")"
done

for key in "${KEYS[@]}"; do
	if [[ -z "${NEW[$key]}" ]]; then
		echo "::error::could not resolve bump-pin:$key" >&2
		exit 1
	fi
done

# (file, key) targets to update. A file may carry only a subset of
# the keys; missing keys produce a warning, not a failure. Targets
# whose key isn't in $KEYS are filtered out below.
declare -a TARGETS=(
	"$WORKFLOWS/ci.yml 6.12"
	"$WORKFLOWS/ci.yml 6.18"
	"$WORKFLOWS/ci.yml 7.0"
	"$WORKFLOWS/vm-smoke.yml 6.12"
	"$WORKFLOWS/vm-smoke-weekly.yml 6.18"
	"$WORKFLOWS/vm-smoke-weekly.yml 7.0"
)

for tgt in "${TARGETS[@]}"; do
	file="${tgt% *}"
	key="${tgt##* }"
	# Skip targets whose key isn't in scope for this run.
	in_scope=0
	for k in "${KEYS[@]}"; do
		[[ "$k" == "$key" ]] && in_scope=1
	done
	[[ $in_scope -eq 1 ]] || continue
	new="${NEW[$key]}"
	key_re="${key//./\\.}"
	# `(linux-)?ref:` matches both the matrix-entry form and the
	# workflow_call `with: linux-ref:` form.
	line_re="(linux-)?ref:[[:space:]]+\S+[[:space:]]+# bump-pin:${key_re}\$"
	old="$(grep -E "$line_re" "$file" 2>/dev/null \
	       | sed -E "s@^.*(linux-)?ref:[[:space:]]+([^[:space:]]+)[[:space:]]+# bump-pin:${key_re}\$@\2@" \
	       | head -n1)"
	if [[ -z "$old" ]]; then
		echo "::warning::no bump-pin:$key in ${file##*/} (skipping)" >&2
		continue
	fi
	rel="${file#$ROOT/}"
	if [[ "$old" == "$new" ]]; then
		printf '%-40s %-7s %s (unchanged)\n' "$rel" "$key" "$old"
		continue
	fi
	sed -E -i \
	  "s@^([[:space:]]*(linux-)?ref:[[:space:]]+)[^[:space:]]+([[:space:]]+# bump-pin:${key_re})\$@\1${new}\3@" \
	  "$file"
	printf '%-40s %-7s %s -> %s\n' "$rel" "$key" "$old" "$new"
done
