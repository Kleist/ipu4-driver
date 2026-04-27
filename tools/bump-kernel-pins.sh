#!/usr/bin/env bash
# Resolve the latest stable point release for each kernel CI track
# and rewrite the matching .ref value in .github/kernel-pins/<key>.json.
# .github/workflows/{ci,vm-smoke,vm-smoke-weekly}.yml all read those
# files at workflow start, so this script no longer needs to touch
# any workflow YAML.
#
# Usage:
#   tools/bump-kernel-pins.sh                # update every track
#   tools/bump-kernel-pins.sh --key 6.12     # update only the 6.12 track
#
# Tracks are discovered from the directory: every <key>.json in
# .github/kernel-pins/ is a track. Adding a track is a one-file PR.
# Each file carries `{"url": ..., "ref": ...}`; the resolution rule
# is "highest v<key>(.X) tag at .url", so a track with a non-stable
# URL would just need a matching tag pattern.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
PIN_DIR="$ROOT/.github/kernel-pins"

if [[ ! -d "$PIN_DIR" ]]; then
	echo "::error::missing $PIN_DIR" >&2
	exit 1
fi

# Track keys come from the filenames so adding a track is purely
# a file add — no script edit, no workflow edit.
ALL_KEYS=()
for f in "$PIN_DIR"/*.json; do
	[[ -f "$f" ]] || continue
	ALL_KEYS+=("$(basename "$f" .json)")
done
if [[ ${#ALL_KEYS[@]} -eq 0 ]]; then
	echo "::error::no <key>.json files in $PIN_DIR" >&2
	exit 1
fi

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

# Highest non-rc tag matching <pattern> at <url>.
# Args: $1 = url, $2 = pattern (e.g. 'v6.12*'), $3 = anchor (e.g. '^v6\.12(\.[0-9]+)?$').
latest_stable_tag() {
	local url="$1" pattern="$2" anchor="$3"
	git ls-remote --tags --refs --sort=-v:refname "$url" "$pattern" \
	  | sed -E 's@^.*refs/tags/@@' \
	  | grep -E "$anchor" \
	  | head -n1
}

for key in "${KEYS[@]}"; do
	pin="$PIN_DIR/${key}.json"
	rel="${pin#$ROOT/}"
	url="$(jq -r '.url' "$pin")"
	track_re="${key//./\\.}"
	new="$(latest_stable_tag "$url" "v${key}*" "^v${track_re}(\\.[0-9]+)?\$")"
	if [[ -z "$new" ]]; then
		echo "::error::could not resolve track $key" >&2
		exit 1
	fi
	old="$(jq -r '.ref // ""' "$pin")"
	if [[ -z "$old" ]]; then
		echo "::warning::no .ref in $rel (skipping)" >&2
		continue
	fi
	if [[ "$old" == "$new" ]]; then
		printf '%-40s %-7s %s (unchanged)\n' "$rel" "$key" "$old"
		continue
	fi
	tmp="$(mktemp)"
	jq --arg v "$new" '.ref = $v' "$pin" > "$tmp"
	mv "$tmp" "$pin"
	printf '%-40s %-7s %s -> %s\n' "$rel" "$key" "$old" "$new"
done
