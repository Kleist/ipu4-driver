#!/usr/bin/env bash
# Daily upstream IPU6 commit watcher. Detects new commits on linux-6.12.y and
# Linus master that touch drivers/media/pci/intel/ipu6/, attempts to apply
# each to kernel/ipu4/ via git am, and (in normal mode) opens a single PR
# with the clean cherry-picks stacked. Conflicts and N/A commits are
# enumerated in the PR body for manual triage.
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

ensure_linux_clone "$ROOT"
deepen_linux_clone "$ROOT"
LINUX_DIR="$ROOT/tools/linux"

# Stable mirror carries the moving linux-6.12.y branch; bootstrap's default
# upstream URL (torvalds) only has master. Add a 'stable' remote idempotently.
if ! git -C "$LINUX_DIR" remote get-url stable >/dev/null 2>&1; then
	git -C "$LINUX_DIR" remote add stable \
		https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git
fi
echo ">>> fetching origin/master and stable/linux-6.12.y"
# `bootstrap.sh` clones origin with a tag-only refspec
# (`+refs/tags/v6.12:refs/tags/v6.12`), so a plain `git fetch origin master`
# lands in FETCH_HEAD only — `refs/remotes/origin/master` is never created
# and the rev-parse below fails with "unknown revision". Spell the
# destination explicitly. The stable remote was added with the default
# refspec, so its plain fetch is fine.
git -C "$LINUX_DIR" fetch origin master:refs/remotes/origin/master --quiet
git -C "$LINUX_DIR" fetch stable linux-6.12.y --quiet

# --- read state ---------------------------------------------------------------

if [[ ! -f "$STATE_FILE" ]]; then
	echo "error: $STATE_FILE missing; create it before running watcher" >&2
	exit 1
fi

stable_old=$(jq -r '.last_seen["linux-6.12.y"]' "$STATE_FILE")
master_old="${IPU4_UPSTREAM_WATCH_SINCE:-$(jq -r '.last_seen["master"]' "$STATE_FILE")}"
processed_ids_file="$OUT/processed_ids.txt"
jq -r '.processed_patch_ids[]?' "$STATE_FILE" > "$processed_ids_file" || true

stable_new=$(git -C "$LINUX_DIR" rev-parse stable/linux-6.12.y)
master_new=$(git -C "$LINUX_DIR" rev-parse origin/master)

is_seeding=0
if [[ "$stable_old" == "$SENTINEL" ]]; then
	stable_old="$stable_new"; is_seeding=1
fi
if [[ "$master_old" == "$SENTINEL" ]]; then
	master_old="$master_new"; is_seeding=1
fi

# --- enumerate candidates -----------------------------------------------------

list_new_commits() {
	local old="$1" new="$2"
	if [[ -z "$old" || "$old" == "null" || "$old" == "$new" ]]; then
		return
	fi
	git -C "$LINUX_DIR" rev-list --reverse "$old..$new" -- "$UPSTREAM_REL"
}

mapfile -t stable_cands < <(list_new_commits "$stable_old" "$stable_new")
mapfile -t master_cands_raw < <(list_new_commits "$master_old" "$master_new")

patch_id_of() {
	git -C "$LINUX_DIR" show "$1" | git -C "$LINUX_DIR" patch-id --stable | awk '{print $1}'
}

# Master commits whose patch-id matches a stable commit picked in this same
# run, or matches an id we already processed in a previous run, are dropped.
stable_ids_file="$OUT/stable_ids.txt"
: > "$stable_ids_file"
for sha in "${stable_cands[@]:-}"; do
	[[ -z "$sha" ]] && continue
	patch_id_of "$sha" >> "$stable_ids_file"
done

master_cands=()
for sha in "${master_cands_raw[@]:-}"; do
	[[ -z "$sha" ]] && continue
	pid=$(patch_id_of "$sha")
	if grep -qxF "$pid" "$stable_ids_file" 2>/dev/null \
	   || grep -qxF "$pid" "$processed_ids_file" 2>/dev/null; then
		continue
	fi
	master_cands+=("$sha")
done

candidates=()
for sha in "${stable_cands[@]:-}" "${master_cands[@]:-}"; do
	[[ -n "$sha" ]] && candidates+=("$sha")
done

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

	overlap=()
	for p in "${touched[@]}"; do
		[[ "$p" == "$UPSTREAM_REL/"* ]] || continue
		base="${p#"$UPSTREAM_REL/"}"
		if is_local_only "$base"; then continue; fi
		[[ -e "$ROOT/$LOCAL_REL/$base" ]] || continue
		overlap+=("$base")
	done

	files_csv=$(IFS=,; echo "${touched[*]}")

	if (( ${#overlap[@]} == 0 )); then
		echo "  $sha  N/A       $subject"
		emit "$na_nd" "$sha" "$subject" "$files_csv"
		continue
	fi

	rm -f "$PATCH_DIR"/*.patch
	git -C "$LINUX_DIR" format-patch -1 "$sha" \
		--relative="$UPSTREAM_REL/" \
		-o "$PATCH_DIR" >/dev/null
	patch_file=$(ls "$PATCH_DIR"/*.patch | head -n1)

	if git -C "$ROOT" am --3way --directory="$LOCAL_REL" "$patch_file" >/dev/null 2>&1; then
		git -C "$ROOT" commit --amend --no-edit \
			--trailer "Upstream-commit: $sha" >/dev/null
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
	--arg s "$stable_new" --arg m "$master_new" \
	--rawfile ids "$new_processed_ids" \
	'.last_seen["linux-6.12.y"] = $s
	 | .last_seen["master"] = $m
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

if (( n_total == 0 && state_changed == 0 )); then
	echo ">>> nothing to do (no candidates, no state drift)"
	exit 0
fi

# --- render PR body -----------------------------------------------------------

pr_body="$OUT/pr-body.md"
STABLE_OLD="$stable_old" STABLE_NEW="$stable_new" \
	MASTER_OLD="$master_old" MASTER_NEW="$master_new" \
	"$HERE/render-pr-body.sh" "$applied_nd" "$conflict_nd" "$na_nd" > "$pr_body"

echo
echo "=== PR body ==="
cat "$pr_body"
echo "==============="
echo

if [[ "$DRY_RUN" == "1" ]]; then
	echo ">>> dry run: skipping push + PR creation (branch $branch is local-only)"
	exit 0
fi

git -C "$ROOT" push -u origin "$branch"

if (( is_seeding == 1 && n_total == 0 )); then
	title="$date_tag upstream-watch: seed state"
else
	title="$date_tag upstream sync: $n_applied clean, $n_conflict conflicts"
fi
gh pr create --base "${BASE_REF#origin/}" --head "$branch" \
	--title "$title" --body-file "$pr_body"
