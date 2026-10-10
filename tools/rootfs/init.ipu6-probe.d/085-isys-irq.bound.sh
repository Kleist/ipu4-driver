# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-probe (README.md here); $ipu is the IPU
# PCI device (e.g. 0000:00:04.0), say() prints a PROBE: marker.
#
# PROBE:isys-resume ok|fail  a forced ISYS runtime resume worked and the
#                            ISYS suspended again

# ISYS runtime resume. The ISYS driver initialises the ISYS MMU, syncs
# the TSC and sets up its interrupts at runtime resume, which normally
# happens at the first stream start. Force one through power/control:
# "on" resumes the device, "auto" lets it suspend again. Whether the
# registers were written right is checked on the host against the
# silicon trace (isys-irq test); this only reports whether the resume
# worked. A failed resume leaves runtime_status at "error" (or
# "suspended") instead of "active".
isys_resumed=fail
for d in /sys/bus/auxiliary/devices/intel_ipu6.isys.*; do
	[ -e "$d/power/control" ] || continue
	echo on > "$d/power/control"
	st=$(cat "$d/power/runtime_status")
	echo auto > "$d/power/control"
	i=0
	while [ "$i" -lt 50 ] &&
	      [ "$(cat "$d/power/runtime_status")" != suspended ]; do
		sleep 0.1
		i=$((i + 1))
	done
	if dmesg | grep -qi 'tsc sync'; then
		isys_resumed="fail (TSC sync)"
	elif [ "$st" != active ]; then
		isys_resumed="fail (runtime_status $st)"
	elif [ "$(cat "$d/power/runtime_status")" != suspended ]; then
		isys_resumed="fail (not suspended again)"
	else
		isys_resumed=ok
	fi
done
say "isys-resume $isys_resumed"
dmesg | grep -iE 'intel_ipu6\.isys.*(error|fail|timeout)|tsc sync'
