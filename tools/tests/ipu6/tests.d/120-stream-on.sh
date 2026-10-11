# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test stream-on fw-open

# VIDIOC_STREAMON succeeds and frames arrive. On the stream boot
# (stream-boot.sh, tools/rootfs/init.ipu6-stream), streamon's STREAMON
# opens and starts the firmware stream and then enables the CSI-2
# receiver of port 0 (ipu6_isys_csi2_enable_streams()). Passes when:
#  - streamon reports STREAM:streamon and STREAM:dqbuf (a frame came
#    back) and the boot's oops scan is clean (STREAMVM:oops none): a
#    failing STREAMON leaves a vb2 warning behind (plan, PR 11 and PR 12
#    learnings);
#  - the boot's writes to the receiver's enable and configuration
#    registers (port 0, BAR + 0x164000 and + 0x164008), the buttress
#    LTR/DID register (0x68) and the ISYS proxy send queue's pointers
#    (DMEM + 0x018/+0x01c) are exactly silicon's during its stream
#    (data/trace.txt lines 600-1113): RX_CONFIG 0x3 and RX_ENABLE 1 at
#    stream on, both 0 at stream off, and no iWake proxy token or
#    LTR/DID write. The QEMU model accepts any receiver write and
#    delivers frames whether the receiver is enabled or not, so this
#    is what checks the receiver programming;
#  - the receiver error interrupt setup (CSIRX EDGE, LEVEL_NOT_PULSE,
#    MASK, ENABLE at port + 0x500/0x514/0x504/0x510) starts with
#    silicon's (lines 651-668, 0x1ffff, --prefix: the driver masks them
#    again at stream off, silicon does not). CSIRX status/clear are left
#    out: the firmware response handler reads and clears them per frame.
# The receiver timing (+0x2c..) and lane count (+0x04) follow the
# sensor (silicon: one lane at 320 MHz, the virt-sensor: four at
# 450 MHz) and are not compared.
# The ipu6 variant checks the same markers and compares the CSI-2 port 0
# stream on/off and MCD PHY power-up writes with the upstream driver's
# (../upstream_writes.py; the PHY tables are read from the upstream source).
# The ipu4p variant likewise compares port 0's receiver enable, config and
# IRQ control writes with the IPU4P series' (../ipu4p_writes.py).
test_stream-on() {
	boot_stream_vm || return 1
	stream_log_ok || return 1
	local rc=0
	grep -Eq '^STREAM:streamon([[:space:]]|$)' "$STREAM_LOG" || {
		echo "!!! no STREAM:streamon"
		rc=1
	}
	grep -Eq '^STREAM:dqbuf([[:space:]]|$)' "$STREAM_LOG" || {
		echo "!!! no frame (STREAM:dqbuf)"
		rc=1
	}
	grep -q '^STREAMVM:oops none' "$STREAM_LOG" || {
		echo "!!! kernel oops/warning in the stream boot"
		rc=1
	}
	if [[ "$IPU4_VARIANT" == ipu6 || "$IPU4_VARIANT" == ipu4p ]]; then
		local up="$ROOT/tools/tests/ipu6/upstream_writes.py" ranges
		[[ "$IPU4_VARIANT" == ipu4p ]] &&
			up="$ROOT/tools/tests/ipu6/ipu4p_writes.py"
		read -ra ranges < <(python3 "$up" --ranges stream-on)
		python3 "$ROOT/tools/trace/check_writes.py" \
			--expected <(python3 "$up" stream-on) \
			--actual "$STREAM_WRITES" "${ranges[@]}" || rc=1
		return "$rc"
	fi
	python3 "$ROOT/tools/trace/check_writes.py" \
		--expected "$ROOT/data/trace.txt" --expected-lines 600-1113 \
		--actual "$STREAM_WRITES" \
		0x68-0x6c 0x108018-0x108020 0x164000-0x164004 0x164008-0x16400c ||
		rc=1
	python3 "$ROOT/tools/trace/check_writes.py" \
		--expected "$ROOT/data/trace.txt" --expected-lines 651-668 \
		--prefix --actual "$STREAM_WRITES" \
		0x164500-0x164508 0x164510-0x164518 || rc=1
	return "$rc"
}
