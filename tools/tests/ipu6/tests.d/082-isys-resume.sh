# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test isys-resume probe-buttress

# A forced ISYS runtime resume (snippet
# tools/rootfs/init.ipu6-probe.d/085-isys-irq.bound.sh) worked: the ISYS
# went active, synced the TSC with the buttress and suspended again.
# Variant-neutral; isys-irq additionally compares the IPU4 resume's
# register writes with the silicon trace, which does not apply to the
# other QEMU model variants.
test_isys-resume() {
	boot_probe_vm || return 1
	probe_log_ok && grep -q '^PROBE:isys-resume ok' "$PROBE_LOG"
}
