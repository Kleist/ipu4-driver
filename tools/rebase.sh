#!/usr/bin/env bash
# Weekly rebase: pull linux-6.12.y, rebase the IPU4 branch, run the test
# tiers, push if green.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
LINUX_DIR="$ROOT/tools/linux"

if [[ ! -d "$LINUX_DIR/.git" ]]; then
	echo "tools/linux/ missing; run tools/bootstrap.sh first" >&2
	exit 1
fi

BRANCH="${IPU4_LINUX_BRANCH:-ipu4-6.12}"
STABLE_URL="${IPU4_STABLE_URL:-https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git}"
STABLE_BRANCH="${IPU4_STABLE_BRANCH:-linux-6.12.y}"

cd "$LINUX_DIR"
git remote get-url stable >/dev/null 2>&1 || git remote add stable "$STABLE_URL"
git fetch --depth=1 stable "$STABLE_BRANCH"
git checkout "$BRANCH"
git rebase "stable/$STABLE_BRANCH"

cd "$ROOT"
tools/tests/kunit.sh
tools/tests/e2e.sh

# Caller decides whether to push.
echo ">>> rebase + tests green on $BRANCH"
