# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-modules before the unload; say()
# prints a MODLOAD: marker.
#
# MODLOAD:stream-formats ok|fail <why>  a frame with the model's pattern
#                                       in each of the virt sensor's
#                                       other formats

sf_video=
sf_subdev=
sf_sensor=
for n in /sys/class/video4linux/*; do
	case "$(cat "$n/name" 2>/dev/null)" in
	"$IPU_ENTITY ISYS Capture 0") sf_video="/dev/${n##*/}" ;;
	"$IPU_ENTITY CSI2 0") sf_subdev="/dev/${n##*/}" ;;
	ipu4-virt-sensor*) sf_sensor="/dev/${n##*/}" ;;
	esac
done
if [ -z "$sf_video" ] || [ -z "$sf_subdev" ] || [ -z "$sf_sensor" ]; then
	say "stream-formats fail (no sensor, CSI2 0 subdev or capture node)"
else
	# media bus code:pixel format; RGB888 last puts the sensor back.
	sf_bad=
	for f in 0x1017:RGBP 0x200f:UYVY 0x3001:BA81 0x300a:BA10 \
		0x3012:pRCC 0x200a:Y10P 0x100a:BGR3; do
		if STREAMON_FMT=$f STREAMON_SENSOR=$sf_sensor \
			streamon "$sf_video" /dev/media0 "$sf_subdev" |
			grep -q '^STREAM:pattern_ok'; then
			echo "STREAM:format $f ok"
		else
			echo "STREAM:format $f fail"
			sf_bad="$sf_bad $f"
		fi
	done
	if [ -z "$sf_bad" ]; then
		say "stream-formats ok"
	else
		say "stream-formats fail (no pattern for$sf_bad)"
	fi
fi
