# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test frame-desc stream-on

# The driver streams from a sensor that reports a CSI-2 frame descriptor:
# it takes VC and data type from the descriptor instead of deriving them
# from the format, as the stream boot's virt sensor makes it do. On the
# module-load boot (snippet tools/rootfs/init.ipu6-modules.d/
# 135-frame-desc.loaded.sh) the guest turns on the virt sensor's
# frame_desc parameter and streams one frame: the driver must log the
# descriptor ("Framedesc: stream 0") and the frame must hold the model's
# pattern.
test_frame-desc() {
	local log="$ROOT/tools/tests/out/ipu6-module-load.serial"
	if [[ -z "${WANT[module-load]:-}" ]]; then
		test_module-load > /dev/null || return 1
	fi
	grep -E "^MODLOAD:frame-desc|Framedesc|^STREAM:(fail|pattern)" "$log"
	grep -q '^MODLOAD:frame-desc ok' "$log" && grep -q '^MODLOAD: PASS' "$log"
}
