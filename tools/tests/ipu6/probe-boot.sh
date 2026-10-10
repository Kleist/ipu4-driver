# shellcheck shell=bash
# Shared probe boot of the probe-* tests; sourced by tools/tests/ipu6/ladder.sh.

# Boot with tools/rootfs/init.ipu6-probe: load the modules as
# module-load does (intel-ipu6 probes 8086:5a88 inside its insmod),
# report how far probe got through PROBE: markers, unbind, unload and scan
# dmesg. Serial log: tools/tests/out/ipu6-probe.serial. The probe-* tests
# share one boot per ladder run; the first one to run does it.
# Tests run in a subshell (piped into tee), so the "already booted" state
# is a stamp file, removed when the ladder starts.
# The boot also logs every MMIO write to the IPU BAR (QEMU's
# memory_region_ops_write trace event, IPU4_MMIO_WRITE_LOG in run-vm.sh)
# to tools/tests/out/ipu6-probe.mmio-writes, for tests that compare
# register values with the silicon trace (tools/trace/check_writes.py).
PROBE_LOG="$ROOT/tools/tests/out/ipu6-probe.serial"
PROBE_WRITES="$ROOT/tools/tests/out/ipu6-probe.mmio-writes"
PROBE_STAMP="$OUT/probe-boot.stamp"
boot_probe_vm() {
	if [[ -e "$PROBE_STAMP" ]]; then
		echo ">>> reusing the probe boot of this run (${PROBE_LOG#"$ROOT"/})"
		return 0
	fi
	if [[ -z "${WANT[vm-boot]:-}" ]]; then
		IPU4_FLAVOR=ipu6 "$ROOT/tools/build-kernel.sh" || return 1
	fi
	"$ROOT/tools/virt-sensor/build.sh" || return 1
	IPU4_FLAVOR=ipu6 IPU4_INIT=ipu6-probe "$ROOT/tools/rootfs/build.sh" ||
		return 1
	rm -f "$PROBE_LOG" "$PROBE_WRITES"
	# The raw log has every region's writes (~15 MB, mostly serial and
	# VGA); keep only the IPU BAR's.
	timeout --preserve-status "${IPU4_SMOKE_TIMEOUT:-90}" \
		env IPU4_FLAVOR=ipu6 IPU4_MMIO_WRITE_LOG="$PROBE_WRITES.all" \
		"$ROOT/tools/run-vm.sh" > "$PROBE_LOG" 2>&1
	grep -F "name 'ipu4-bar0'" "$PROBE_WRITES.all" > "$PROBE_WRITES"
	rm -f "$PROBE_WRITES.all"
	touch "$PROBE_STAMP"
}

# Common checks of the probe boot: the init ran to the end without
# PROBE:fail markers (module load, unload) or an oops.
probe_log_ok() {
	grep -E '^PROBE:|intel-ipu6|intel_ipu6|virt-sensor' "$PROBE_LOG" |
		grep -vE 'dma_alloc|dmamap|mapping entry'
	if grep -q '^PROBE:fail' "$PROBE_LOG"; then
		echo "!!! PROBE:fail markers (see above)"
		return 1
	fi
	grep -q '^PROBE: DONE' "$PROBE_LOG" &&
		grep -q '^PROBE:oops none' "$PROBE_LOG"
}
