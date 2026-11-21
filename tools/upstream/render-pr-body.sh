#!/usr/bin/env bash
# Render the markdown body for an upstream-watch PR. Reads three NDJSON files
# (one record per line, jq-compatible) produced by watch.sh:
#
#   $1  applied.ndjson   {sha, subject, files: [...]}
#   $2  conflict.ndjson  {sha, subject, files: [...], reject_files: [...]}
#   $3  na.ndjson        {sha, subject, files: [...]}
#
# Plus environment variables describing the picked range:
#   MASTER_OLD, MASTER_NEW   (either may be empty)
#
# Output goes to stdout.

set -euo pipefail

applied="${1:?applied ndjson path required}"
conflict="${2:?conflict ndjson path required}"
na="${3:?na ndjson path required}"

count() { [[ -s "$1" ]] && wc -l < "$1" || echo 0; }

n_applied=$(count "$applied")
n_conflict=$(count "$conflict")
n_na=$(count "$na")

today=$(date -u +%Y-%m-%d)

echo "# Upstream sync — $today"
echo
echo "Picked up since previous run:"
if [[ -n "${MASTER_OLD:-}" || -n "${MASTER_NEW:-}" ]]; then
	echo "- \`master\`: \`${MASTER_OLD:-?}..${MASTER_NEW:-?}\`"
fi
echo

echo "## Applied clean ($n_applied)"
echo
if (( n_applied > 0 )); then
	echo "Each is its own commit on this branch with a \`(cherry picked from commit …)\` line. Makefile and Kconfig changes are left out: add new files to both Makefiles by hand."
	echo
	echo "| SHA | Subject | Files |"
	echo "|---|---|---|"
	jq -r '"| `\(.sha[0:12])` | \(.subject | gsub("\\|"; "\\|")) | \(.files | join(", ")) |"' "$applied"
else
	echo "_None._"
fi
echo

echo "## Conflicts — needs manual cherry-pick ($n_conflict)"
echo
if (( n_conflict > 0 )); then
	echo "These commits did not apply with \`git am --3way\`. Cherry-pick by hand."
	echo
	echo "| SHA | Subject | Files | Rejected |"
	echo "|---|---|---|---|"
	jq -r '"| `\(.sha[0:12])` | \(.subject | gsub("\\|"; "\\|")) | \(.files | join(", ")) | \(.reject_files // [] | join(", ")) |"' "$conflict"
else
	echo "_None._"
fi
echo

echo "## N/A — no overlap with forked files ($n_na)"
echo
if (( n_na > 0 )); then
	echo "<details><summary>expand</summary>"
	echo
	echo "| SHA | Subject | Files |"
	echo "|---|---|---|"
	jq -r '"| `\(.sha[0:12])` | \(.subject | gsub("\\|"; "\\|")) | \(.files | join(", ")) |"' "$na"
	echo
	echo "</details>"
else
	echo "_None._"
fi
echo

echo "---"
echo
echo "State updated in \`tools/notes/upstream-watch-state.json\` (last commit on this branch)."
