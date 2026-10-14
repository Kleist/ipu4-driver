# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test auth-error module-load

# The driver unwinds a probe whose firmware authentication fails. A boot
# of its own (the module-load boot's init, tools/rootfs/init.ipu6-modules)
# with the model's fail-cse-auth=on: the firmware starts unauthenticated,
# so probe sends BOOT_LOAD and AUTH_RUN, and the CSE fails AUTH_RUN. The
# driver must log "FW authentication failed" and leave the IPU unbound,
# and the modules must then unload without an oops. Its 9p share is
# separate, so its gcov counters do not replace the module-load boot's.
test_auth-error() {
	local log="$ROOT/tools/tests/out/ipu6-auth-error.serial"
	local share="$ROOT/tools/tests/out/auth-error-share"
	if [[ -z "${WANT[vm-boot]:-}" ]]; then
		"$ROOT/tools/build-kernel.sh" || return 1
	fi
	"$ROOT/tools/virt-sensor/build.sh" || return 1
	IPU4_INIT=ipu6-modules "$ROOT/tools/rootfs/build.sh" || return 1
	rm -rf "$share"
	mkdir -p "$share"
	timeout --preserve-status "${IPU4_SMOKE_TIMEOUT:-90}" \
		env IPU4_MODEL_PROPS=fail-cse-auth=on IPU4_TESTS_SHARE="$share" \
		"$ROOT/tools/run-vm.sh" > "$log" 2>&1
	[[ -f "$share/gcov-ipu6-modules.tar" ]] &&
		mv "$share/gcov-ipu6-modules.tar" \
			"$ROOT/tools/tests/out/gcov-ipu6-auth-error.tar"
	grep -E "^MODLOAD:(fail|info|ok rmmod)|^MODLOAD: |CSE|FW authentication" "$log"
	grep -q 'FW authentication failed' "$log" &&
		grep -q '^MODLOAD:info .* unbound' "$log" &&
		! grep -q '^MODLOAD:fail rmmod' "$log" &&
		! grep -q '^MODLOAD:fail kernel log' "$log"
}
