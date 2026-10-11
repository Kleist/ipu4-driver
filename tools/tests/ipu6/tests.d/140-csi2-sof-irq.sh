# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.
# IPU4P takes frame starts from the firmware too, so the ipu4p variant
# also covers the error IRQ: the model reports the same error through
# SIP0 (bit 3) and port 0's CSI IRQ controls. Passes when the driver logs
# it and the ISR acknowledged SIP0's port 0 bit and cleared the error in
# port 0's CTRL0 (+0x66400).

ladder_test csi2-sof-irq stream-on

# A CSI-2 frame start (FS) interrupt is observed on port 0. IPU4's CSI-2
# receiver raises FS/FE of virtual channels 0-3 on the port's CSI2PART
# interrupt controller (port + 0x400). The driver enables them at stream
# start and logs them at dev_dbg (plan decision 3: the firmware's SOF/EOF
# responses stay the source of the V4L2 events). On the stream boot
# (stream-boot.sh), the QEMU model latches FS_VC0 in port 0's CSI2PART
# status and raises port 0's UNISPART bit with every frame it delivers;
# the status reads back only the enabled bits. Passes when:
#  - the driver logs "csi2-0: frame start, vc 0" (isys dyndbg): an FS
#    interrupt of port 0 arrived while the frame interrupts were enabled
#    (streamon's second buffer is captured after the receiver is on);
#  - the boot's writes to CSI2PART EDGE, MASK, ENABLE, LEVEL_NOT_PULSE
#    (port + 0x400/0x404/0x410/0x414) and to the CSI2S2M controller
#    (port + 0x600..0x614) are exactly silicon's during its stream
#    (data/trace.txt lines 600-1113): 0x3333 (FS/FE of VC 0-3) and
#    level 0 at stream on (lines 664-668), CSI2S2M and CSI2PART mask and
#    enable 0 at stream off (lines 1078-1081).
# CSI2PART CLEAR is left out: the interrupt handler writes it per
# interrupt, and the model also raises port 0's interrupt for the frame
# the firmware delivers before the receiver is enabled.
#
# The out-of-tree driver's handler logs nothing; it acknowledges the
# interrupt by writing the status back to CSI2PART CLEAR, so a CLEAR
# write with FS of VC 0 (bit 0) other than the enable's 0x3333 is the
# frame start.
csi2_fs_acked() {
	python3 - "$STREAM_WRITES" <<'PY'
import re, sys
for line in open(sys.argv[1]):
    m = re.search(r"addr (0x[0-9a-f]+) value (0x[0-9a-f]+)", line)
    if not m:
        continue
    addr, val = (int(x, 16) for x in m.groups())
    if addr & 0xffffff == 0x16440c and val & 1 and val != 0x3333:
        sys.exit(0)
sys.exit(1)
PY
}

# Upstream IPU6 keeps the CSI-2 sync IRQs masked and takes frame starts
# from the firmware, so the ipu6 variant covers the port's error IRQ
# instead: the model reports a corrected packet header error with each
# frame. Passes when the ISR dispatched port 0 (writes 0x3 to CTRL0
# CLEAR), the driver logs the error, and the sync MASK is only written 0.
test_csi2-sof-irq() {
	boot_stream_vm || return 1
	stream_log_ok || return 1
	local rc=0
	if [[ "$IPU4_VARIANT" == ipu4p ]]; then
		grep -q 'csi2-0 info: Single packet header error corrected' \
			"$STREAM_LOG" || {
			echo "!!! no CSI-2 receiver error reported on port 0"
			rc=1
		}
		python3 - "$ROOT/tools/trace" "$STREAM_WRITES" <<-'PY' || rc=1
			import sys
			sys.path.insert(0, sys.argv[1])
			from check_writes import read_writes
			with open(sys.argv[2]) as f:
			    writes = list(read_writes(f))
			if (0x166d0c, 0x8) not in writes:
			    sys.exit("!!! ISR never acknowledged port 0 in SIP0")
			if (0x16640c, 0x1) not in writes:
			    sys.exit("!!! ISR never cleared port 0's error in CTRL0")
		PY
		return "$rc"
	fi
	if [[ "$IPU4_VARIANT" == ipu6 ]]; then
		local up="$ROOT/tools/tests/ipu6/upstream_writes.py" ranges
		grep -q 'csi2-0 error: Single packet header error corrected' \
			"$STREAM_LOG" || {
			echo "!!! no CSI-2 receiver error reported on port 0"
			rc=1
		}
		python3 - "$ROOT/tools/trace" "$STREAM_WRITES" <<-'PY' || rc=1
			import sys
			sys.path.insert(0, sys.argv[1])
			from check_writes import read_writes
			with open(sys.argv[2]) as f:
			    if (0x23820c, 0x3) not in read_writes(f):
			        sys.exit("!!! ISR never cleared port 0 in CTRL0")
		PY
		read -ra ranges < <(python3 "$up" --ranges csi2-sof-irq)
		python3 "$ROOT/tools/trace/check_writes.py" \
			--expected <(python3 "$up" csi2-sof-irq) \
			--actual "$STREAM_WRITES" "${ranges[@]}" || rc=1
		return "$rc"
	fi
	if [[ "$FLAVOR" == out-of-tree ]]; then
		csi2_fs_acked || {
			echo "!!! no CSI-2 frame start interrupt acknowledged on port 0"
			rc=1
		}
	else
		grep -Eq "$IPU_MODNAME\.isys\.[0-9]+: csi2-0: frame start, vc 0" \
			"$STREAM_LOG" || {
			echo "!!! no CSI-2 frame start interrupt on port 0"
			rc=1
		}
	fi
	python3 "$ROOT/tools/trace/check_writes.py" \
		--expected "$ROOT/data/trace.txt" --expected-lines 600-1113 \
		--actual "$STREAM_WRITES" \
		0x164400-0x164408 0x164410-0x164418 0x164600-0x164618 || rc=1
	return "$rc"
}
