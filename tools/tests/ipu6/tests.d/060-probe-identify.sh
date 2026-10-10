# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test probe-identify module-load

# Probe identified the IPU the QEMU model variant presents (requested its
# firmware, intel/ipu/ipu4_fw.bin for IPU4 or ipu6_fw.bin for IPU6,
# rather than rejecting the device). Probe itself may still fail further
# on; that is what the later probe-* tests check.
test_probe-identify() {
	boot_probe_vm || return 1
	probe_log_ok && grep -q '^PROBE:identify ok' "$PROBE_LOG"
}
