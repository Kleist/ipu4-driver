#!/usr/bin/env bash
# Resolve the latest stable point release for each kernel CI track
# and rewrite the matching value in .github/kernel-pins.json.
# .github/workflows/{ci,vm-smoke,vm-smoke-weekly}.yml all read that
# file at workflow start, so this script no longer needs to touch
# any workflow YAML.
#
# Usage:
#   tools/bump-kernel-pins.sh                # update every track
#   tools/bump-kernel-pins.sh --key 6.12     # update only the 6.12 track
#
# Resolution rules:
#   - 6.12  — highest v6.12(.X) release tag on git.kernel.org stable mirror.
#   - 6.18  — same with v6.18(.X).
#   - 7.0   — same with v7.0(.X).
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
PINS="$ROOT/.github/kernel-pins.json"

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

if [[ ! -f "$PINS" ]]; then
	echo "::error::missing $PINS" >&2
	exit 1
fi

# Highest non-rc tag matching <pattern> on the stable mirror.
# Args: $1 = pattern (e.g. 'v6.12*'), $2 = anchor regex (e.g. '^v6\.12(\.[0-9]+)?$').
STABLE_URL="$(jq -r '."linux-url"' "$PINS")"
latest_stable_tag() {
	local pattern="$1" anchor="$2"
	git ls-remote --tags --refs --sort=-v:refname "$STABLE_URL" "$pattern" \
	  | sed -E 's@^.*refs/tags/@@' \
	  | grep -E "$anchor" \
	  | head -n1
}

rel="${PINS#$ROOT/}"
for key in "${KEYS[@]}"; do
	track_re="${key//./\\.}"
	new="$(latest_stable_tag "v${key}*" "^v${track_re}(\\.[0-9]+)?\$")"
	if [[ -z "$new" ]]; then
		echo "::error::could not resolve track $key" >&2
		exit 1
	fi
	old="$(jq -r --arg k "$key" '.tracks[$k] // ""' "$PINS")"
	if [[ -z "$old" ]]; then
		echo "::warning::no .tracks[\"$key\"] in $rel (skipping)" >&2
		continue
	fi
	if [[ "$old" == "$new" ]]; then
		printf '%-30s %-7s %s (unchanged)\n' "$rel" "$key" "$old"
		continue
	fi
	tmp="$(mktemp)"
	jq --arg k "$key" --arg v "$new" '.tracks[$k] = $v' "$PINS" > "$tmp"
	mv "$tmp" "$PINS"
	printf '%-30s %-7s %s -> %s\n' "$rel" "$key" "$old" "$new"
done
