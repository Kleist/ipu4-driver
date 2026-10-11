# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-probe (README.md here); $ipu is the IPU
# PCI device (e.g. 0000:00:04.0), say() prints a PROBE: marker.

# Buttress power. probe powers up ISYS and PSYS by runtime-resuming the
# PSYS device (the ISYS device is its parent): ipu6_buttress_power()
# writes IS/PS_FREQ_CTL and polls the PWR_STATE FSM, logging "Change
# power status timeout" if it does not get there. The PSYS MMU hardware
# init runs right after that and logs the trash buffer it maps, which is
# the positive marker for power-up. At the end of probe both devices are
# runtime-suspended again, which powers them down through the same FSM;
# a failed power-down leaves them active ("power down failed!").
aux=/sys/bus/auxiliary/devices
if dmesg | grep -q "$IPU_MODNAME\.psys\..*iova trash buffer for MMUID"; then
	powered_up=1
else
	powered_up=
fi
suspended=
i=0
while [ "$i" -lt 50 ]; do
	n=0
	for d in "$aux/$IPU_MODNAME".isys.* "$aux/$IPU_MODNAME".psys.*; do
		[ "$(cat "$d/power/runtime_status" 2>/dev/null)" = suspended ] &&
			n=$((n + 1))
	done
	if [ "$n" = 2 ]; then
		suspended=1
		break
	fi
	sleep 0.1
	i=$((i + 1))
done
for d in "$aux/$IPU_MODNAME".*; do
	[ -e "$d" ] || continue
	say "info ${d##*/} $(cat "$d/power/runtime_status")"
done
if dmesg | grep -q 'Change power status timeout'; then
	say "buttress fail (Change power status timeout)"
elif [ -z "$powered_up" ]; then
	say "buttress fail (PSYS runtime resume did not reach the MMU init)"
elif [ -z "$suspended" ]; then
	say "buttress fail (ISYS/PSYS not runtime-suspended after probe)"
else
	say "buttress ok"
fi
