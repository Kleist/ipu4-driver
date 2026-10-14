# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test pci-reauth module-load

# Firmware re-authentication after a PCI reset. The platform has
# authenticated the firmware before the driver loads (SECURITY_CTL reads
# AUTH_DONE on silicon and in the model), so a normal probe skips the
# CSE exchange. On the module-load boot (snippet tools/rootfs/
# init.ipu6-modules.d/055-pci-reauth.loaded.sh) the guest resets the
# function (the model's PCI Advanced Features FLR), whose reset_done()
# drops the authentication (FW_RESET_CTL), then unbinds and rebinds the
# driver: the new probe sends BOOT_LOAD and AUTH_RUN to the model's CSE
# and must log "CSE authenticate_run done". The module-load boot does not
# capture register writes, so this does not disturb the write checks of
# the probe and stream boots.
test_pci-reauth() {
	local log="$ROOT/tools/tests/out/ipu6-module-load.serial"
	if [[ -z "${WANT[module-load]:-}" ]]; then
		test_module-load > /dev/null || return 1
	fi
	grep -E "^MODLOAD:(pci-reauth|info reset_method)|CSE|authenticat" "$log"
	grep -q '^MODLOAD:pci-reauth ok' "$log" && grep -q '^MODLOAD: PASS' "$log"
}
