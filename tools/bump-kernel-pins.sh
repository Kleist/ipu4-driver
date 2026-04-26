#!/usr/bin/env bash
# Resolve the latest upstream refs for each kernel CI pin and rewrite
# every workflow file that contains a matching `# bump-pin:<key>`
# comment. Prints a summary of (file, key, old, new) so the calling
# workflow can paste it into the auto-PR's body.
#
# Files touched:
#   - .github/workflows/ci.yml             (build+kunit matrix; 6.12 + 6.18)
#   - .github/workflows/vm-smoke.yml       (PR caller; 6.12 only)
#   - .github/workflows/vm-smoke-weekly.yml (Sunday cron; 6.18 only)
#
# Resolution rules:
#   - 6.12  — highest v6.12(.X) release tag on git.kernel.org stable mirror.
#   - 6.18  — same with v6.18(.X).
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
NEW[6.12]="$(latest_stable_tag 'v6.12*' '^v6\.12(\.[0-9]+)?$')"
NEW[6.18]="$(latest_stable_tag 'v6.18*' '^v6\.18(\.[0-9]+)?$')"

for key in 6.12 6.18; do
	if [[ -z "${NEW[$key]}" ]]; then
		echo "::error::could not resolve bump-pin:$key" >&2
		exit 1
	fi
done

# (file, key) targets to update. A file may carry only a subset of
# the keys; missing keys produce a warning, not a failure.
declare -a TARGETS=(
	"$WORKFLOWS/ci.yml 6.12"
	"$WORKFLOWS/ci.yml 6.18"
	"$WORKFLOWS/vm-smoke.yml 6.12"
	"$WORKFLOWS/vm-smoke-weekly.yml 6.18"
)

for tgt in "${TARGETS[@]}"; do
	file="${tgt% *}"
	key="${tgt##* }"
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
