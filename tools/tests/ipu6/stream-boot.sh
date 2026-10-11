# shellcheck shell=bash
# Shared stream boot of the stream tests; sourced by tools/tests/ipu6/ladder.sh.

# Boot with tools/rootfs/init.ipu6-stream: load the modules, capture one
# frame from CSI-2 port 0 with /bin/streamon (STREAM: markers), unbind,
# unload and scan dmesg. Serial log: tools/tests/out/ipu6-stream.serial;
# the IPU BAR's MMIO writes go to tools/tests/out/ipu6-stream.mmio-writes
# as in the probe boot (probe-boot.sh). The stream tests share one boot
# per ladder run; the first one to run does it ("already booted" is a
# stamp file, removed when the ladder starts, as for the probe boot).
STREAM_LOG="$ROOT/tools/tests/out/ipu6-stream.serial"
STREAM_WRITES="$ROOT/tools/tests/out/ipu6-stream.mmio-writes"
STREAM_STAMP="$OUT/stream-boot.stamp"
boot_stream_vm() {
	if [[ -e "$STREAM_STAMP" ]]; then
		echo ">>> reusing the stream boot of this run (${STREAM_LOG#"$ROOT"/})"
		return 0
	fi
	if [[ -z "${WANT[vm-boot]:-}" ]]; then
		"$ROOT/tools/build-kernel.sh" || return 1
	fi
	"$ROOT/tools/virt-sensor/build.sh" || return 1
	IPU4_INIT=ipu6-stream "$ROOT/tools/rootfs/build.sh" ||
		return 1
	rm -f "$STREAM_LOG" "$STREAM_WRITES"
	timeout --preserve-status "${IPU4_SMOKE_TIMEOUT:-90}" \
		env IPU4_MMIO_WRITE_LOG="$STREAM_WRITES.all" \
		"$ROOT/tools/run-vm.sh" > "$STREAM_LOG" 2>&1
	grep -F "name 'ipu4-bar0'" "$STREAM_WRITES.all" > "$STREAM_WRITES"
	rm -f "$STREAM_WRITES.all"
	touch "$STREAM_STAMP"
}

# Common checks of the stream boot: the init ran to the end without
# STREAMVM:fail markers (module load, finding the nodes, unload). How far
# the stream got is up to each test (STREAM: markers, driver output), and
# so is the oops scan (STREAMVM:oops none|found): a stream that fails
# part-way can leave a kernel warning that only the tests for that part
# should fail on.
stream_log_ok() {
	grep -E "^(STREAMVM|STREAM|SUBDEV|MEDIA):|$IPU_PCI_DRIVER|$IPU_MODNAME|virt-sensor" \
		"$STREAM_LOG" | grep -vE 'dma_alloc|dmamap|mapping entry'
	if grep -q '^STREAMVM:fail' "$STREAM_LOG"; then
		echo "!!! STREAMVM:fail markers (see above)"
		return 1
	fi
	grep -q '^STREAMVM: DONE' "$STREAM_LOG"
}
