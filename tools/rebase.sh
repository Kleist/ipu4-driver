#!/usr/bin/env bash
# Weekly rebase-cron entry point. Re-bootstraps the forked Linux and
# QEMU trees (picking up whatever moved on linux-6.12.y since the last
# run), rebuilds, and runs every smoke tier we have today. Caller
# decides whether to push anything; this script's job is green / red.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
LINUX_DIR="$ROOT/tools/linux"

# Point bootstrap at linux-stable on the stable-tree mirror, which
# carries the linux-6.12.y BRANCH. The default bootstrap URL is
# torvalds/linux — that mirror only tracks mainline. Pick up the
# latest 6.12.y point release each week by fetching the branch tip.
export IPU4_LINUX_URL="${IPU4_LINUX_URL:-https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git}"
export IPU4_LINUX_TAG="${IPU4_LINUX_TAG:-linux-6.12.y}"

# Fresh tree every week — avoids accidental state carrying over from
# the previous run and is how CI exercises the full cold path.
if [[ -d "$LINUX_DIR/.git" ]]; then
	echo ">>> wiping stale $LINUX_DIR"
	rm -rf "$LINUX_DIR"
fi

"$ROOT"/tools/bootstrap.sh
"$ROOT"/tools/build.sh
"$ROOT"/tools/tests/kunit.sh
"$ROOT"/tools/build-qemu.sh
"$ROOT"/tools/build-kernel.sh
"$ROOT"/tools/rootfs/build.sh
"$ROOT"/tools/tests/vm-smoke.sh
"$ROOT"/tools/tests/probe-smoke.sh
"$ROOT"/tools/tests/streamon-smoke.sh

echo ">>> weekly rebase + all smoke tiers green on $IPU4_LINUX_TAG"
