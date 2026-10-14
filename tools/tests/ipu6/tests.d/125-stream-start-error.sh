# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test stream-start-error stream-on

# The driver unwinds a stream whose firmware start fails. A boot of its
# own (the stream boot's init, tools/rootfs/init.ipu6-stream) with the
# model's fail-stream-starts=1, which answers the first STREAM_START with
# a firmware error and no frame: the driver must log "stream start
# error", close the firmware stream and fail VIDIOC_STREAMON
# (STREAM:fail step=streamon), and the modules must then unload without
# an oops. Its 9p share is separate, so its gcov counters do not replace
# the stream boot's.
test_stream-start-error() {
	local log="$ROOT/tools/tests/out/ipu6-stream-error.serial"
	local share="$ROOT/tools/tests/out/stream-error-share"
	if [[ -z "${WANT[vm-boot]:-}" ]]; then
		"$ROOT/tools/build-kernel.sh" || return 1
	fi
	"$ROOT/tools/virt-sensor/build.sh" || return 1
	IPU4_INIT=ipu6-stream "$ROOT/tools/rootfs/build.sh" || return 1
	rm -rf "$share"
	mkdir -p "$share"
	timeout --preserve-status "${IPU4_SMOKE_TIMEOUT:-90}" \
		env IPU4_MODEL_PROPS=fail-stream-starts=1 IPU4_TESTS_SHARE="$share" \
		"$ROOT/tools/run-vm.sh" > "$log" 2>&1
	[[ -f "$share/gcov-ipu6-stream.tar" ]] &&
		mv "$share/gcov-ipu6-stream.tar" \
			"$ROOT/tools/tests/out/gcov-ipu6-stream-error.tar"
	grep -E "^(STREAMVM|STREAM):|stream start error" "$log"
	if grep -q '^STREAMVM:fail' "$log"; then
		echo "!!! STREAMVM:fail markers (see above)"
		return 1
	fi
	grep -q 'stream start error' "$log" &&
		grep -q '^STREAM:fail step=streamon' "$log" &&
		grep -q '^STREAMVM:oops none' "$log" &&
		grep -q '^STREAMVM: DONE' "$log"
}
