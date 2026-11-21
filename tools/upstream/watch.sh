#!/usr/bin/env bash
# Weekly upstream IPU6 commit watcher. Detects new commits on Linus master
# that touch drivers/media/pci/intel/ipu6/ (kernel/ipu4/ follows mainline,
# see tools/notes/ipu4-out-of-tree-plan.md), applies each to kernel/ipu4/
# with a real 3-way git am (the patch's mainline blobs are copied in), and
# (in normal mode) opens a single PR with the clean cherry-picks stacked.
# Conflicts and N/A commits are enumerated in the PR body for manual triage. Runs with nothing to apply,
# or whose commits an open PR already lists, stay silent; a new PR closes
# the older open ones it supersedes.
#
# State lives in tools/notes/upstream-watch-state.json and rolls forward in
# the last commit on the bot branch, so PR-tip and state advance atomically.
#
# First-run seeding: if either last_seen value is the sentinel
# "INIT_AT_FIRST_RUN", the watcher resolves it to the current ref tip without
# producing any cherry-picks, and opens a state-seed-only PR. From that point
# on the steady-state delta logic kicks in.
#
# Env knobs:
#   IPU4_UPSTREAM_WATCH_DRY_RUN=1    skip git push and gh pr create; print PR body
#   IPU4_UPSTREAM_WATCH_SINCE=<sha>  override last_seen.master for replay
#   IPU4_UPSTREAM_WATCH_BASE=<ref>   base branch (default: origin/main)
#   IPU4_UPSTREAM_WATCH_LINUX=<dir>  mainline clone (default: tools/linux-watch)
#   IPU4_UPSTREAM_WATCH_DAYS=<n>     history to fetch (default: 180 days)

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
# shellcheck source=tools/upstream/_lib.sh
source "$HERE/_lib.sh"

DRY_RUN="${IPU4_UPSTREAM_WATCH_DRY_RUN:-0}"
BASE_REF="${IPU4_UPSTREAM_WATCH_BASE:-origin/main}"
SENTINEL="INIT_AT_FIRST_RUN"

STATE_FILE="$ROOT/tools/notes/upstream-watch-state.json"
OUT="$ROOT/tools/upstream/out"
PATCH_DIR="$OUT/patches"

rm -rf "$OUT"
mkdir -p "$PATCH_DIR"

LINUX_DIR="${IPU4_UPSTREAM_WATCH_LINUX:-$ROOT/tools/linux-watch}"
LINUX_URL="${IPU4_UPSTREAM_WATCH_URL:-https://github.com/torvalds/linux.git}"
DAYS="${IPU4_UPSTREAM_WATCH_DAYS:-180}"

# A blobless clone of master: commits and trees for the path-limited
# rev-list, blobs fetched on demand by format-patch.
if [[ ! -e "$LINUX_DIR/HEAD" ]]; then
	git init -q --bare "$LINUX_DIR"
	git -C "$LINUX_DIR" remote add origin "$LINUX_URL"
	git -C "$LINUX_DIR" config remote.origin.promisor true
	git -C "$LINUX_DIR" config remote.origin.partialclonefilter blob:none
fi
echo ">>> fetching master (last $DAYS days, blobless)"
git -C "$LINUX_DIR" fetch -q --no-tags --filter=blob:none \
	--shallow-since="$(date -u -d "-$DAYS days" +%Y-%m-%d)" \
	origin +refs/heads/master:refs/remotes/origin/master

# --- read state ---------------------------------------------------------------

if [[ ! -f "$STATE_FILE" ]]; then
	echo "error: $STATE_FILE missing; create it before running watcher" >&2
	exit 1
fi

master_old="${IPU4_UPSTREAM_WATCH_SINCE:-$(jq -r '.last_seen["master"]' "$STATE_FILE")}"
processed_ids_file="$OUT/processed_ids.txt"
jq -r '.processed_patch_ids[]?' "$STATE_FILE" > "$processed_ids_file" || true

master_new=$(git -C "$LINUX_DIR" rev-parse origin/master)

is_seeding=0
if [[ "$master_old" == "$SENTINEL" ]]; then
	master_old="$master_new"; is_seeding=1
fi

if [[ "$master_old" != "$master_new" ]] &&
   ! git -C "$LINUX_DIR" merge-base --is-ancestor "$master_old" "$master_new" 2>/dev/null; then
	echo "error: last_seen.master $master_old is not in the fetched history;" \
		"raise IPU4_UPSTREAM_WATCH_DAYS" >&2
	exit 1
fi

# --- enumerate candidates -----------------------------------------------------

list_new_commits() {
	local old="$1" new="$2"
	if [[ -z "$old" || "$old" == "null" || "$old" == "$new" ]]; then
		return
	fi
	git -C "$LINUX_DIR" rev-list --reverse "$old..$new" -- "$UPSTREAM_REL"
}

mapfile -t master_cands_raw < <(list_new_commits "$master_old" "$master_new")

patch_id_of() {
	git -C "$LINUX_DIR" show "$1" | git -C "$LINUX_DIR" patch-id --stable | awk '{print $1}'
}

# Commits whose patch-id we already processed (picked by hand or by an
# earlier run) are dropped.
master_cands=()
for sha in "${master_cands_raw[@]:-}"; do
	[[ -z "$sha" ]] && continue
	pid=$(patch_id_of "$sha")
	if grep -qxF "$pid" "$processed_ids_file" 2>/dev/null; then
		continue
	fi
	master_cands+=("$sha")
done
candidates=("${master_cands[@]:-}")
[[ -z "${candidates[0]:-}" ]] && candidates=()

echo ">>> ${#candidates[@]} candidate commit(s) (seeding=$is_seeding)"

# --- prepare bot branch -------------------------------------------------------

date_tag=$(date -u +%Y-%m-%d)
branch="claude/upstream-watch/$date_tag"
git -C "$ROOT" fetch origin main --quiet || true
echo ">>> creating $branch from $BASE_REF"
git -C "$ROOT" checkout -B "$branch" "$BASE_REF"

applied_nd="$OUT/applied.ndjson"
conflict_nd="$OUT/conflict.ndjson"
na_nd="$OUT/na.ndjson"
: > "$applied_nd"; : > "$conflict_nd"; : > "$na_nd"

emit() {
	local f="$1" sha="$2" subject="$3" files="$4" rej="${5:-}"
	jq -nc \
		--arg sha "$sha" --arg subject "$subject" \
		--arg files "$files" --arg rej "$rej" \
		'{sha: $sha, subject: $subject,
		  files: ($files | split(",") | map(select(length>0))),
		  reject_files: ($rej | split(",") | map(select(length>0)))}' \
		>> "$f"
}

# --- process candidates -------------------------------------------------------

for sha in "${candidates[@]}"; do
	subject=$(git -C "$LINUX_DIR" log -1 --format=%s "$sha")
	mapfile -t touched < <(git -C "$LINUX_DIR" show --name-only --format= "$sha" | sed '/^$/d')
	mapfile -t added < <(git -C "$LINUX_DIR" show --diff-filter=A --name-only --format= "$sha" | sed '/^$/d')

	# Files the fork carries, and files the commit adds (new upstream
	# support, e.g. IPU4P): both are the fork's to take.
	overlap=()
	for p in "${touched[@]}"; do
		[[ "$p" == "$UPSTREAM_REL/"* ]] || continue
		base="${p#"$UPSTREAM_REL/"}"
		if is_local_only "$base"; then continue; fi
		# The fork's build files are its own.
		[[ "$base" == Makefile || "$base" == Kconfig ]] && continue
		if [[ ! -e "$ROOT/$LOCAL_REL/$base" ]] &&
		   ! printf '%s\n' "${added[@]:-}" | grep -qxF "$p"; then
			continue
		fi
		overlap+=("$base")
	done

	files_csv=$(IFS=,; echo "${touched[*]}")

	if (( ${#overlap[@]} == 0 )); then
		echo "  $sha  N/A       $subject"
		emit "$na_nd" "$sha" "$subject" "$files_csv"
		continue
	fi

	rm -f "$PATCH_DIR"/*.patch
	git -C "$LINUX_DIR" format-patch -1 "$sha" --full-index \
		--relative="$UPSTREAM_REL/" \
		-o "$PATCH_DIR" -- "$UPSTREAM_REL/" \
		":(exclude)$UPSTREAM_REL/Makefile" \
		":(exclude)$UPSTREAM_REL/Kconfig" >/dev/null
	patch_file=$(ls "$PATCH_DIR"/*.patch | head -n1)

	# Copy the patch's pre- and postimage blobs in, so git am --3way can
	# merge (and the result never points at objects only mainline has).
	grep -oE '^index [0-9a-f]+\.\.[0-9a-f]+' "$patch_file" \
		| awk '{print $2}' | tr '.' '\n' | awk 'NF' | sort -u \
		| while read -r oid; do
			[[ "$oid" =~ ^0+$ ]] && continue
			git -C "$ROOT" cat-file -e "$oid" 2>/dev/null && continue
			git -C "$LINUX_DIR" cat-file blob "$oid" \
				| git -C "$ROOT" hash-object -w --stdin >/dev/null
		done
	if git -C "$ROOT" am --3way --directory="$LOCAL_REL" \
		"$patch_file" >/dev/null 2>&1; then
		msg=$(git -C "$ROOT" log -1 --format=%B)
		git -C "$ROOT" commit -q --amend \
			-m "$msg" -m "(cherry picked from commit $sha)"
		echo "  $sha  applied   $subject"
		emit "$applied_nd" "$sha" "$subject" "$(IFS=,; echo "${overlap[*]}")"
	else
		# Capture unmerged paths from the index before aborting.
		mapfile -t conflicted < <(
			git -C "$ROOT" diff --name-only --diff-filter=U 2>/dev/null \
				| sed "s|^$LOCAL_REL/||"
		)
		git -C "$ROOT" am --abort >/dev/null 2>&1 || true
		echo "  $sha  CONFLICT  $subject"
		emit "$conflict_nd" "$sha" "$subject" \
			"$(IFS=,; echo "${overlap[*]}")" \
			"$(IFS=,; echo "${conflicted[*]:-}")"
	fi
done

# --- bump state (always) ------------------------------------------------------

new_processed_ids=$(mktemp)
{
	cat "$processed_ids_file" 2>/dev/null || true
	for sha in "${master_cands[@]:-}"; do
		[[ -z "$sha" ]] && continue
		patch_id_of "$sha"
	done
} | awk 'NF' | sort -u > "$new_processed_ids"

jq \
	--arg m "$master_new" \
	--rawfile ids "$new_processed_ids" \
	'.schema = 2
	 | .last_seen = {master: $m}
	 | .processed_patch_ids = ($ids | split("\n") | map(select(length>0)))' \
	"$STATE_FILE" > "$STATE_FILE.tmp"
mv "$STATE_FILE.tmp" "$STATE_FILE"
rm -f "$new_processed_ids"

# Did the state file actually change versus the base branch?
state_changed=0
if ! git -C "$ROOT" diff --quiet "$BASE_REF" -- "$STATE_FILE"; then
	state_changed=1
	git -C "$ROOT" add "$STATE_FILE"
	git -C "$ROOT" commit -m "upstream-watch: bump state to $date_tag" >/dev/null
fi

# --- decide whether to open a PR ---------------------------------------------

n_applied=$(wc -l < "$applied_nd" || echo 0)
n_conflict=$(wc -l < "$conflict_nd" || echo 0)
n_total=$(( n_applied + n_conflict ))

# State drift alone (upstream moved without touching forked files) is not
# worth a PR; the range is simply re-scanned next run.
if (( n_total == 0 && is_seeding == 0 )); then
	echo ">>> nothing to do (no applicable candidates)"
	exit 0
fi

# Identifies the commits a PR asks to triage, so an unmerged PR is not
# re-opened every run for the same commits.
key=$(jq -rs 'map(.sha) | sort | join(" ")' "$applied_nd" "$conflict_nd")
marker="<!-- upstream-watch-commits: ${key:-seed} -->"

# --- render PR body -----------------------------------------------------------

pr_body="$OUT/pr-body.md"
MASTER_OLD="$master_old" MASTER_NEW="$master_new" \
	"$HERE/render-pr-body.sh" "$applied_nd" "$conflict_nd" "$na_nd" > "$pr_body"
printf '\n%s\n' "$marker" >> "$pr_body"

echo
echo "=== PR body ==="
cat "$pr_body"
echo "==============="
echo

if [[ "$DRY_RUN" == "1" ]]; then
	echo ">>> dry run: skipping push + PR creation (branch $branch is local-only)"
	exit 0
fi

open_prs=$(gh pr list --state open --limit 100 \
	--json number,headRefName,body \
	--jq '[.[] | select(.headRefName | startswith("claude/upstream-watch/"))]')
dup=$(jq -r --arg m "$marker" \
	'.[] | select(.body | contains($m)) | .number' <<<"$open_prs" | head -n1)
if [[ -n "$dup" ]]; then
	echo ">>> open PR #$dup already covers these commits; nothing to do"
	exit 0
fi

git -C "$ROOT" push -u origin "$branch"

if (( is_seeding == 1 && n_total == 0 )); then
	title="$date_tag upstream-watch: seed state"
else
	title="$date_tag upstream sync: $n_applied clean, $n_conflict conflicts"
fi
url=$(gh pr create --base "${BASE_REF#origin/}" --head "$branch" \
	--title "$title" --body-file "$pr_body")
echo "$url"

# The new PR re-lists everything still unmerged, so older ones are stale.
jq -r --arg b "$branch" '.[] | select(.headRefName != $b) | .number' \
	<<<"$open_prs" | while read -r n; do
	gh pr close "$n" --delete-branch --comment "Superseded by $url."
done
