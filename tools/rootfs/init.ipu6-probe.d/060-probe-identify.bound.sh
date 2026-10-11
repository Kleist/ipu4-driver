# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-probe (README.md here); $ipu is the IPU
# PCI device (e.g. 0000:00:04.0), $want its PCI device ID (0x5a88 for
# IPU4), say() prints a PROBE: marker.

# Identification: the device switch in ipu6_pci_probe() either rejects
# the ID ("Unsupported IPU6 device <id>") or picks the variant's
# settings, among them the firmware file: ipu4_fw.bin for the IPU4
# (5a88, through the series), ipu6_fw.bin for the IPU6 the ipu6 QEMU
# variant presents (9a19, upstream), ipu4p_fw.bin for the ipu4p
# variant's IPU4P (8a19). probe requests that file right after buttress
# init, and the firmware loader logs its name whether the
# file exists ("Loading firmware from .../<fw>", dyndbg) or not ("Direct
# firmware load for <fw> failed"). So this check also needs probe to get
# through buttress init, which it does for IPU4 from the first patch of
# the series on.
case "$want" in
0x8a19) fw=intel/ipu/ipu4p_fw.bin ;;
0x9a19) fw=intel/ipu/ipu6_fw.bin ;;
*) fw=intel/ipu/ipu4_fw.bin ;;
esac
did=${want#0x}
if dmesg | grep -q "Unsupported IPU6 device $did"; then
	say "identify fail (Unsupported IPU6 device $did)"
elif dmesg | grep -F "$ipu" | grep -qF "$fw"; then
	say "identify ok"
else
	say "identify fail (no request for $fw)"
fi
