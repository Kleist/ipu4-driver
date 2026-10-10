# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test checkpatch

test_checkpatch() {
	local cp="$LINUX_DIR/scripts/checkpatch.pl" p rc=0
	if [[ -z "$PATCH_DIR" ]]; then
		echo "no series on the $VARIANT variant's base; nothing to check"
		return 0
	fi
	shopt -s nullglob
	local patches=("$PATCH_DIR"/*.patch)
	shopt -u nullglob
	if (( ${#patches[@]} == 0 )); then
		echo "no patches in ${PATCH_DIR#"$ROOT"/}; nothing to check"
		return 0
	fi
	if [[ ! -f "$cp" ]]; then
		echo "missing $cp; run tools/ipu6/bootstrap.sh first"
		return 1
	fi
	for p in "${patches[@]}"; do
		echo ">>> checkpatch --strict ${p#"$ROOT"/}"
		(cd "$LINUX_DIR" && perl "$cp" --strict --color=never "$p") || rc=1
	done
	return "$rc"
}
