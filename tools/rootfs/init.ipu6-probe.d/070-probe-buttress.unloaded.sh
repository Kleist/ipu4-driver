# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-probe (README.md here); $ipu is the IPU
# PCI device (e.g. 0000:00:04.0), say() prints a PROBE: marker.

# After unbind and rmmod: no buttress power-up/-down error from bind to
# rmmod (the power-down at unbind included).
if dmesg | grep -E 'Change power status timeout|power down failed'; then
	say "pwr-errors found"
else
	say "pwr-errors none"
fi
