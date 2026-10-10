# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test isys-irq probe-mmu

# The ISYS runtime resume sets up the IPU4 ISYS interrupts as silicon
# does. The probe init forces one resume (snippet
# tools/rootfs/init.ipu6-probe.d/085-isys-irq.bound.sh; PROBE:isys-resume:
# the power/control "on"/"auto" cycle worked, so the TSC sync passed). Its
# register writes are compared with silicon's second ISYS resume in
# data/trace.txt, lines 487-599 (ISYS power-up to the end of the
# interrupt setup; the first ISYS MMU init at lines 269-369 comes
# without one, and from line 600 on the firmware is started and the
# ISR runs). Compared, in order:
#  - ISYS MMU0/MMU1 (BAR + 0x1e0000/0x1e0100) +0x08..+0x8c, as in
#    probe-mmu (page-table address and TLB invalidate left out);
#  - 0x68, IPU6's fabric control (LTR/DID values): silicon never writes
#    it, so the guest must not either;
#  - 0x88, the TSC sync request (one, no retries);
#  - 0x17c000-0x17c41c, the UNISPART interrupt controller: EDGE, LEVEL,
#    CLEAR, MASK and ENABLE = the 6 CSI-2 port bits + the firmware bit
#    (0x40001878), then the software interrupt registers.
test_isys-irq() {
	boot_probe_vm || return 1
	probe_log_ok &&
		grep -q '^PROBE:isys-resume ok' "$PROBE_LOG" &&
		python3 "$ROOT/tools/trace/check_writes.py" \
			--expected "$ROOT/data/trace.txt" --expected-lines 487-599 \
			--actual "$PROBE_WRITES" \
			0x1e0008-0x1e0090 0x1e0108-0x1e0190 0x68-0x6c 0x88-0x8c \
			0x17c000-0x17c420
}
