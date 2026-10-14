# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test stream-formats stream-on

# The driver streams formats other than RGB888: on the module-load boot
# (snippet tools/rootfs/init.ipu6-modules.d/132-stream-formats.loaded.sh)
# streamon sets the virt sensor and CSI-2 receiver to RGB565, UYVY,
# RAW8/10/12 Bayer and Y10 in turn (STREAMON_FMT), with unpacked and
# packed capture formats, and each frame must hold the model's pattern.
test_stream-formats() {
	local log="$ROOT/tools/tests/out/ipu6-module-load.serial"
	if [[ -z "${WANT[module-load]:-}" ]]; then
		test_module-load > /dev/null || return 1
	fi
	grep -E "^MODLOAD:stream-formats|^STREAM:format" "$log"
	grep -q '^MODLOAD:stream-formats ok' "$log" && grep -q '^MODLOAD: PASS' "$log"
}
