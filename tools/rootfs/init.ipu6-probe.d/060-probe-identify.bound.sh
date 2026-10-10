# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-probe (README.md here); $ipu is the IPU
# PCI device (e.g. 0000:00:04.0), say() prints a PROBE: marker.

# Identification: the device switch in ipu6_pci_probe() either rejects
# the ID ("Unsupported IPU6 device 5a88") or picks the IPU4 settings,
# among them the firmware file. probe requests that file right after
# buttress init, and the firmware loader logs its name whether the file
# exists ("Loading firmware from .../intel/ipu/ipu4_fw.bin", dyndbg) or
# not ("Direct firmware load for intel/ipu/ipu4_fw.bin failed"). So this
# check also needs probe to get through buttress init, which it does from
# the first patch of the series on.
if dmesg | grep -q "Unsupported IPU6 device 5a88"; then
	say "identify fail (Unsupported IPU6 device 5a88)"
elif dmesg | grep -F "$ipu" | grep -q "intel/ipu/ipu4_fw.bin"; then
	say "identify ok"
else
	say "identify fail (no request for intel/ipu/ipu4_fw.bin)"
fi
