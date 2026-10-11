# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test fw-open isys-irq

# The ISYS firmware is opened through SysCom and answers the first stream
# open. The stream boot (stream-boot.sh, tools/rootfs/init.ipu6-stream)
# streams from CSI-2 port 0; at VIDIOC_STREAMON the driver opens the
# firmware (ipu6_isys_fw_open(): SPC start, SysCom config and state in
# ISYS DMEM) and sends STREAM_OPEN for stream 0, which the QEMU model
# answers with STREAM_OPEN_DONE on the message receive queue. Passes
# when:
#  - the driver received STREAM_OPEN_DONE for stream 0 and finished the
#    open ("start stream: open complete"). The model finds the send queue
#    by its DMEM pointer register, as the firmware does, so it answers
#    only when the driver uses IPU4's SysCom layout;
#  - the stream boot's writes to ISYS DMEM (BAR + 0x108000) start with
#    silicon's from the firmware open to the STREAM_OPEN_DONE handling
#    (data/trace.txt lines 600-622, check_writes.py --prefix): package
#    directory (0x40, secure mode), SysCom state and command UNINIT, the
#    stream 0 send queue write pointer at register 10 (0x108028) and the
#    message receive queue read pointer at register 29 (0x108074). Those
#    two registers follow from the queue pointer base (6) and the number
#    of message send queues (8), which the model does not check. Left out:
#    the SysCom config address (+0x004, an IOVA) and the proxy send
#    queue's pointers (+0x018/+0x01c), where the guest sends two iwake
#    proxy tokens that silicon never sends (plan, PR 11 learnings).
# What happens after the open (stream start, CSI-2 receiver, frames, a
# kernel warning when stream-on fails) is for the later stream tests.
test_fw-open() {
	boot_stream_vm || return 1
	stream_log_ok || return 1
	if grep -E 'stream open (time out|error)' "$STREAM_LOG"; then
		return 1
	fi
	grep -q 'FW resp 00 STREAM_OPEN_DONE, stream 0,' "$STREAM_LOG" &&
		grep -q 'start stream: open complete' "$STREAM_LOG" &&
		python3 "$ROOT/tools/trace/check_writes.py" \
			--expected "$ROOT/data/trace.txt" --expected-lines 600-622 \
			--prefix --actual "$STREAM_WRITES" \
			0x108000-0x108004 0x108008-0x108018 0x108020-0x108100
}
