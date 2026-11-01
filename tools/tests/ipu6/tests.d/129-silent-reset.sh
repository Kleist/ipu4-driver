# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test silent-reset stream-requeue

# The out-of-tree driver's silent reset: on the module-load boot (snippet
# tools/rootfs/init.ipu6-modules.d/129-silent-reset.loaded.sh) a stream on
# Capture 0 is paused after its first frame, force_need_reset is set and
# Capture 1 is opened, which restarts the running stream; the stream must
# then go on delivering frames. Drivers without the silent reset report
# "skip", which is no PASS.
test_silent-reset() {
	local log="$ROOT/tools/tests/out/ipu6-module-load.serial"
	if [[ -z "${WANT[module-load]:-}" ]]; then
		test_module-load > /dev/null || return 1
	fi
	grep -E "^MODLOAD:silent-reset|^STREAM:(paused|requeue|fail)" "$log"
	grep -q '^MODLOAD:silent-reset ok' "$log" && grep -q '^MODLOAD: PASS' "$log"
}
