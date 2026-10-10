#!/usr/bin/env bash
# Manage per-PR dashboard previews.
#
# actions/deploy-pages publishes one artifact as the whole site, so a
# PR cannot deploy into a subdirectory on its own. Previews are kept
# on the `pages-previews` branch (one directory per PR, single orphan
# commit, force-pushed) and pages.yml copies that branch into
# site/pr-preview/ on every production deploy.
#
#   previews.sh publish <pr> <site-dir>   replace pr-<pr>/ with <site-dir>
#   previews.sh remove  <pr>              drop pr-<pr>/
#   previews.sh merge   <site-dir>        copy open PRs' previews into
#                                         <site-dir>/pr-preview/
#   previews.sh comment <pr> <body> [--if-exists]
#                                         create/update the PR's sticky
#                                         preview comment
#
# Runs inside a checkout whose `origin` can be pushed to (publish /
# remove) or fetched from (merge). `merge` uses `gh` to skip previews
# of PRs that are no longer open; if gh fails, nothing is pruned.
set -euo pipefail

BRANCH="${IPU4_PREVIEWS_BRANCH:-pages-previews}"
REF="refs/remotes/origin/$BRANCH"

die() { echo "previews.sh: $*" >&2; exit 1; }

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# Fetch the previews branch; prints its SHA, or nothing if absent.
fetch_branch() {
  if git ls-remote --exit-code origin "refs/heads/$BRANCH" >/dev/null 2>&1; then
    git fetch -q origin "+refs/heads/$BRANCH:$REF"
    git rev-parse "$REF"
  fi
}

# update <pr> [site-dir]: rewrite the branch with pr-<pr>/ replaced by
# site-dir (or removed). Retries on lease failures so concurrent PR
# runs can't clobber each other's directories.
update() {
  local pr="$1" site="${2:-}" dir="pr-$1"
  local verb=remove stage lease tree commit attempt
  stage="$TMP/stage"
  if [[ -n "$site" ]]; then
    verb=update
    mkdir -p "$stage"
    cp -r "$site" "$stage/$dir"
  fi

  for attempt in 1 2 3 4 5; do
    lease="$(fetch_branch)"
    export GIT_INDEX_FILE="$TMP/index"
    rm -f "$GIT_INDEX_FILE"
    if [[ -n "$lease" ]]; then
      git read-tree "$lease"
    else
      git read-tree --empty
    fi
    git rm -r -q --cached --ignore-unmatch -- "$dir"
    if [[ -n "$site" ]]; then
      git --work-tree="$stage" add -f -A -- "$dir"
    fi
    tree="$(git write-tree)"
    unset GIT_INDEX_FILE

    if [[ -n "$lease" && "$tree" == "$(git rev-parse "$lease^{tree}")" ]]; then
      echo "previews.sh: $dir unchanged; nothing to push"
      return 0
    fi
    commit="$(git -c user.name="github-actions[bot]" \
        -c user.email="41898282+github-actions[bot]@users.noreply.github.com" \
        commit-tree "$tree" -m "pages-previews: $verb $dir")"
    if git push -q --force-with-lease="refs/heads/$BRANCH:$lease" \
        origin "$commit:refs/heads/$BRANCH"; then
      echo "previews.sh: $verb $dir on $BRANCH: pushed"
      return 0
    fi
    echo "previews.sh: push raced (attempt $attempt); retrying" >&2
    sleep $((attempt * 2))
  done
  die "could not update $BRANCH after 5 attempts"
}

merge() {
  local site="$1" lease open d n
  lease="$(fetch_branch)"
  if [[ -z "$lease" ]]; then
    echo "previews.sh: no $BRANCH branch; no previews to merge"
    return 0
  fi
  mkdir -p "$site/pr-preview"
  git archive "$lease" | tar -x -C "$site/pr-preview"
  if open="$(gh pr list --state open --limit 500 --json number --jq '.[].number')"; then
    for d in "$site"/pr-preview/pr-*; do
      [[ -d "$d" ]] || continue
      n="${d##*/pr-}"
      if ! grep -qx "$n" <<<"$open"; then
        echo "previews.sh: dropping preview of closed PR #$n"
        rm -rf "$d"
      fi
    done
  else
    echo "previews.sh: gh pr list failed; keeping every preview" >&2
  fi
  echo "previews.sh: merged $(find "$site/pr-preview" -mindepth 1 -maxdepth 1 -type d | wc -l) preview(s)"
}

comment() {
  local pr="$1" body="$2" if_exists="${3:-}" marker='<!-- pages-preview -->' id
  id="$(gh api --paginate "repos/{owner}/{repo}/issues/$pr/comments" \
      --jq ".[] | select(.body | startswith(\"$marker\")) | .id" | head -1)"
  body="$marker"$'\n'"$body"
  if [[ -n "$id" ]]; then
    gh api -X PATCH "repos/{owner}/{repo}/issues/comments/$id" -f body="$body" >/dev/null
  elif [[ "$if_exists" != "--if-exists" ]]; then
    gh api "repos/{owner}/{repo}/issues/$pr/comments" -f body="$body" >/dev/null
  fi
}

case "${1:-}" in
  publish)
    [[ $# -eq 3 ]] || die "usage: previews.sh publish <pr> <site-dir>"
    [[ "$2" =~ ^[0-9]+$ ]] || die "bad PR number: $2"
    [[ -d "$3" ]] || die "no such site dir: $3"
    update "$2" "$3"
    ;;
  remove)
    [[ $# -eq 2 ]] || die "usage: previews.sh remove <pr>"
    [[ "$2" =~ ^[0-9]+$ ]] || die "bad PR number: $2"
    update "$2"
    ;;
  merge)
    [[ $# -eq 2 ]] || die "usage: previews.sh merge <site-dir>"
    merge "$2"
    ;;
  comment)
    [[ $# -eq 3 || ( $# -eq 4 && "$4" == "--if-exists" ) ]] \
      || die "usage: previews.sh comment <pr> <body> [--if-exists]"
    [[ "$2" =~ ^[0-9]+$ ]] || die "bad PR number: $2"
    comment "$2" "$3" "${4:-}"
    ;;
  *)
    die "usage: previews.sh {publish <pr> <site-dir>|remove <pr>|merge <site-dir>|comment <pr> <body> [--if-exists]}"
    ;;
esac
