# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-modules before the unload; $ipu is the
# IPU's sysfs path, say() prints a MODLOAD: marker.
#
# MODLOAD:pci-reauth ok|fail <why>  a PCI function reset dropped the
#                                   firmware authentication and the
#                                   next probe re-ran it through the CSE

if [ -z "$ipu" ] || [ ! -e "$ipu/driver" ]; then
	say "pci-reauth fail (driver not bound)"
else
	say "info reset_method $(cat "$ipu/reset_method" 2>/dev/null)"
	# reset_done() resets the CSE's authentication state; the next
	# probe then sends BOOT_LOAD and AUTH_RUN.
	if echo 1 > "$ipu/reset"; then
		echo "${ipu##*/}" > "/sys/bus/pci/drivers/$IPU_PCI_DRIVER/unbind"
		echo "${ipu##*/}" > "/sys/bus/pci/drivers/$IPU_PCI_DRIVER/bind"
		if dmesg | grep -q 'CSE authenticate_run done'; then
			say "pci-reauth ok"
		else
			say "pci-reauth fail (no CSE authenticate_run)"
		fi
	else
		say "pci-reauth fail (reset)"
	fi
	dmesg | grep -iE 'CSE|authenticat|IPC|reset' | grep -E "$IPU_PCI_DRIVER|${ipu##*/}"
fi
