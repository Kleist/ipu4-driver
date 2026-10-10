# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test probe-identify module-load

# Probe identified the IPU4 (requested intel/ipu/ipu4_fw.bin rather than
# rejecting the device). Probe itself may still fail further on; that is
# what the later probe-* tests check.
test_probe-identify() {
	boot_probe_vm || return 1
	probe_log_ok && grep -q '^PROBE:identify ok' "$PROBE_LOG"
}
