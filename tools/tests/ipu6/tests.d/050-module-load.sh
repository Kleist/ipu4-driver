# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test module-load vm-boot

# Boot the vm-boot guest kernel with tools/rootfs/init.ipu6-modules:
# load the vb2 modules, the harness's virt-sensor (tools/virt-sensor/),
# intel-ipu6 and intel-ipu6-isys, check that 8086:5a88 is there, unload,
# and scan dmesg. Whether probe binds is up to the probe-* tests. The
# guest prints
# MODLOAD: PASS|FAIL. Rebuilds the kernel only when vm-boot (which does
# that) is not part of this run.
test_module-load() {
	local log="$ROOT/tools/tests/out/ipu6-module-load.serial"
	if [[ -z "${WANT[vm-boot]:-}" ]]; then
		IPU4_FLAVOR=ipu6 "$ROOT/tools/build-kernel.sh" || return 1
	fi
	"$ROOT/tools/virt-sensor/build.sh" || return 1
	IPU4_FLAVOR=ipu6 IPU4_INIT=ipu6-modules "$ROOT/tools/rootfs/build.sh" ||
		return 1
	rm -f "$log"
	timeout --preserve-status "${IPU4_SMOKE_TIMEOUT:-90}" \
		env IPU4_FLAVOR=ipu6 "$ROOT/tools/run-vm.sh" > "$log" 2>&1
	grep -E '^MODLOAD:|virt-sensor|ipu6' "$log"
	grep -q '^MODLOAD: PASS' "$log"
}
