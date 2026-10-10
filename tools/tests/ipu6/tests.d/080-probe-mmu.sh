# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test probe-mmu probe-buttress

# The PSYS MMU hardware init in probe programs the IPU4 MMU tables: the
# guest's writes to the PSYS MMU configuration registers are the ones
# silicon makes in data/trace.txt, in the same order. The QEMU model
# accepts any value there, so this compares the probe boot's MMIO write
# log (QEMU trace event) with the silicon trace instead of looking at
# how probe went. Per MMU (MMU0 at BAR + 0x4b0000, MMU1 at + 0x4b0100,
# MMU1R at + 0x4b0600) the compared range is +0x08..+0x8c: INFO and the
# 16 L1 and 16 L2 stream start addresses, i.e. everything
# __ipu6_mmu_hw_init() writes except the page-table address at +0x04.
# Left out on purpose: +0x00 (TLB invalidate; upstream writes
# 0xffffffff, silicon 0x1 / 0xffff) and the address translator ZLW
# registers from +0x100 (the reference driver programs them, upstream
# ipu6 does not). The ISYS MMU (+0x1e0000) is only initialised at ISYS
# runtime resume (first stream open); isys-irq checks it.
test_probe-mmu() {
	boot_probe_vm || return 1
	probe_log_ok &&
		python3 "$ROOT/tools/trace/check_writes.py" \
			--expected "$ROOT/data/trace.txt" --actual "$PROBE_WRITES" \
			0x4b0008-0x4b0090 0x4b0108-0x4b0190 0x4b0608-0x4b0690
}
