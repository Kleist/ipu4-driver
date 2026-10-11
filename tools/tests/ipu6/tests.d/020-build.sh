# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test build series-applies

test_build() {
	local log="$OUT/build.compile.log"
	# tools/<flavor>/build.sh.
	IPU4_IPU6_CLEAN=1 IPU4_IPU6_BUILD_LOG="$log" "$ROOT/tools/build.sh" ||
		return 1
	# "cleanly": no compiler warnings (W=1 for the ipu6 flavor, the
	# module's own level for out-of-tree). modpost's "WARNING:" lines
	# about unresolved symbols are expected (no vmlinux) and not matched.
	if grep -E '(warning|error):' "$log"; then
		echo "!!! compiler warnings in the build (see above)"
		return 1
	fi
}
