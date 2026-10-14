# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test stream-requeue stream-on

# Buffers queued while the stream runs are captured: on the module-load
# boot (snippet tools/rootfs/init.ipu6-modules.d/128-stream-requeue.loaded.sh)
# streamon gives each dequeued buffer back with VIDIOC_QBUF and takes the
# next frame, three times (STREAMON_REQUEUE=3), which sends a capture
# command per buffer to the firmware on a running stream.
test_stream-requeue() {
	local log="$ROOT/tools/tests/out/ipu6-module-load.serial"
	if [[ -z "${WANT[module-load]:-}" ]]; then
		test_module-load > /dev/null || return 1
	fi
	grep -E "^MODLOAD:stream-requeue|^STREAM:(requeue|fail)" "$log"
	grep -q '^MODLOAD:stream-requeue ok' "$log" && grep -q '^MODLOAD: PASS' "$log"
}
