# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test vm-boot build

# Full guest kernel (bzImage + modules, same .config as `build`) +
# initramfs with the vm-smoke init, booted under the QEMU IPU4 model;
# passes when the guest sees 8086:5a88 on its PCI bus. Needs the patched
# QEMU (tools/bootstrap.sh --qemu-only && tools/build-qemu.sh).
test_vm-boot() {
	local qemu="${IPU4_QEMU_BIN:-$ROOT/tools/qemu/build/qemu-system-x86_64}"
	if [[ ! -x "$qemu" ]]; then
		echo "missing $qemu; run tools/bootstrap.sh --qemu-only &&" \
			"tools/build-qemu.sh"
		return 1
	fi
	IPU4_FLAVOR=ipu6 "$ROOT/tools/build-kernel.sh" || return 1
	IPU4_FLAVOR=ipu6 "$ROOT/tools/tests/vm-smoke.sh"
}
