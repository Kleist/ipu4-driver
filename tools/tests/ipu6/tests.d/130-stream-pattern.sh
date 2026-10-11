# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test stream-pattern stream-on

# The first captured frame holds the QEMU model's test pattern. The model
# answers each capture of the stream boot with SOF and PIN_DATA_READY
# after writing byte[k] = k & 0xff into the first 64 KiB of the buffer
# through the IPU MMU (its IOVA walker), and streamon
# (tools/tests/streamon.c) checks byte[k] == (k + sequence) & 0xff at a
# few offsets of the dequeued buffer (sequence 0 for the first frame),
# as the fork's streamon-smoke does (STREAM:pattern_ok). Passes on STREAM:pattern_ok and
# STREAM:done: the buffer the driver handed to the firmware is the one
# userspace got back, mapped through the ISYS MMU to the same pages.
test_stream-pattern() {
	boot_stream_vm || return 1
	stream_log_ok || return 1
	grep -q '^STREAM:pattern_ok ' "$STREAM_LOG" &&
		grep -q '^STREAM:done ' "$STREAM_LOG"
}
