# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

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

test_csi2-sof-irq() {
	boot_stream_vm || return 1
	stream_log_ok || return 1
	local rc=0
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
