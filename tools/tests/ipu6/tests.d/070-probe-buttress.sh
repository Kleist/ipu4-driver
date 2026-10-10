# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test probe-buttress probe-identify

# Buttress init and ISYS/PSYS power-up succeed: probe got through the
# PSYS runtime resume (which powers up ISYS, its parent, first) to the
# PSYS MMU init, both devices were runtime-suspended (powered down)
# afterwards, and no "Change power status timeout" / "power down
# failed!" was logged from bind to rmmod.
test_probe-buttress() {
	boot_probe_vm || return 1
	probe_log_ok &&
		grep -q '^PROBE:buttress ok' "$PROBE_LOG" &&
		grep -q '^PROBE:pwr-errors none' "$PROBE_LOG"
}
