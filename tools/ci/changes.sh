#!/usr/bin/env bash
# Decide which CI areas a change touches, so PR workflows skip the
# expensive jobs whose inputs didn't change.
#
# Usage:
#   tools/ci/changes.sh <base> <head>   # diff base...head (merge-base)
#   tools/ci/changes.sh --files         # changed paths on stdin
#   tools/ci/changes.sh --all           # every area (push / dispatch)
#
# Prints one `<area>=true|false` line per area, suitable for
# >> "$GITHUB_OUTPUT". Areas:
#
#   kunit     ci.yml build-and-kunit legs (fork driver build + KUnit)
#   pytest    ci.yml pytest job (Python trace / dashboard tools)
#   vm_smoke  vm-smoke.yml (fork driver in the full VM + MMIO compare)
#   ipu6      vm-ipu6.yml (the test ladder, every leg)
#
# Unknown paths select nothing: keep the lists below in step with what
# each job actually reads (bootstrap / build / test scripts and the
# cache keys in the workflows). Anything that can't be diffed selects
# everything, so a classifier failure costs runner time, not coverage.
set -euo pipefail

AREAS=(kunit pytest vm_smoke ipu6)
declare -A HIT=()

emit() {
	local a
	for a in "${AREAS[@]}"; do
		echo "$a=${HIT[$a]:-false}"
	done
}

all() {
	local a
	for a in "${AREAS[@]}"; do
		HIT[$a]=true
	done
}

classify() {
	local f="$1"
	case "$f" in
	# CI plumbing shared by every job.
	.github/actions/setup-harness/* | .github/workflows/changes.yml | tools/ci/*)
		all ;;
	# Kernel pins feed both fork-flavor workflows.
	.github/kernel-pins/*)
		HIT[kunit]=true; HIT[vm_smoke]=true ;;
	.github/workflows/ci.yml)
		HIT[kunit]=true; HIT[pytest]=true ;;
	.github/workflows/build-and-kunit.yml)
		HIT[kunit]=true ;;
	.github/workflows/vm-smoke.yml | .github/workflows/vm-smoke-reusable.yml)
		HIT[vm_smoke]=true ;;
	.github/workflows/vm-ipu6.yml)
		HIT[ipu6]=true ;;

	# Fork driver + its in-tree seeding / build / KUnit runner. KUnit's
	# gcov harvest boots tools/run-vm.sh with tools/rootfs/init.kunit-cov.
	# The driver sources and tools/build.sh (flavor dispatch) also feed
	# the ladder's ipu4-out-of-tree leg.
	kernel/ipu4/* | tools/build.sh)
		HIT[kunit]=true; HIT[vm_smoke]=true; HIT[ipu6]=true ;;
	kernel/* | tools/linux-patches/* | tools/tests/kunit.sh)
		HIT[kunit]=true; HIT[vm_smoke]=true ;;
	tools/bootstrap.sh | tools/vm-kconfig.sh | tools/run-vm.sh | tools/rootfs/*)
		HIT[kunit]=true; HIT[vm_smoke]=true; HIT[ipu6]=true ;;

	# QEMU model + guest image inputs shared by both VM flows.
	tools/qemu-patches/* | tools/build-qemu.sh | tools/build-kernel.sh | \
	tools/firmware/* | data/* | tools/tests/vm-smoke.sh | tools/tests/streamon.c)
		HIT[vm_smoke]=true; HIT[ipu6]=true ;;
	# ladder.sh checks probe writes against the silicon trace.
	tools/trace/check_writes.py)
		HIT[pytest]=true; HIT[vm_smoke]=true; HIT[ipu6]=true ;;

	# ipu6 and out-of-tree flavors (the ladder).
	patches/ipu6-ipu4/* | tools/ipu6/* | tools/out-of-tree/* | \
	tools/tests/ipu6/* | tools/virt-sensor/*)
		HIT[ipu6]=true ;;

	# Python tools and their tests.
	tests/* | pyproject.toml | tools/tests/pytest.sh)
		HIT[pytest]=true ;;
	tools/trace/* | postprocess_trace.py)
		HIT[pytest]=true; HIT[vm_smoke]=true ;;
	tools/dashboard/*.py)
		HIT[pytest]=true ;;

	# Remaining fork VM-test inputs (streamon / probe / mmiotrace /
	# compare-mmio scripts and baselines, coverage collection).
	tools/tests/* | tools/coverage/*)
		HIT[vm_smoke]=true ;;
	esac
}

case "${1:-}" in
--all)
	all ;;
--files)
	while IFS= read -r f; do
		[[ -n "$f" ]] && classify "$f"
	done ;;
"")
	echo "usage: $0 <base> <head> | --files | --all" >&2
	exit 2 ;;
*)
	if ! files="$(git diff --name-only "$1...$2")"; then
		echo "::warning::cannot diff $1...$2; running every job" >&2
		all
	else
		while IFS= read -r f; do
			[[ -n "$f" ]] && classify "$f"
		done <<<"$files"
	fi ;;
esac

emit
