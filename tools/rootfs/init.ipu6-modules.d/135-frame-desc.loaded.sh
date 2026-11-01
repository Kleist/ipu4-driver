# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-modules before the unload; say()
# prints a MODLOAD: marker.
#
# MODLOAD:frame-desc ok|fail <why>  a stream with the virt sensor's CSI-2
#                                   frame descriptor delivered the
#                                   model's pattern

fd_param=/sys/module/virt_sensor/parameters/frame_desc
[ -e "$fd_param" ] || fd_param=
fd_video=
fd_subdev=
for n in /sys/class/video4linux/*; do
	case "$(cat "$n/name" 2>/dev/null)" in
	"$IPU_ENTITY ISYS Capture 0") fd_video="/dev/${n##*/}" ;;
	"$IPU_ENTITY CSI2 0") fd_subdev="/dev/${n##*/}" ;;
	esac
done
if [ -z "$fd_param" ]; then
	say "frame-desc fail (no frame_desc parameter)"
elif [ -z "$fd_video" ] || [ -z "$fd_subdev" ]; then
	say "frame-desc fail (no CSI2 0 subdev or capture node)"
else
	echo 1 > "$fd_param"
	# The descriptor the driver got, logged at debug level by the
	# driver (out-of-tree, ipu6 before 7.4) or by the v4l2 core.
	echo 'format "Framedesc" +p' > /proc/dynamic_debug/control 2>/dev/null
	echo 'file v4l2-subdev.c format "stream %u, code" +p' \
		> /proc/dynamic_debug/control 2>/dev/null
	if streamon "$fd_video" /dev/media0 "$fd_subdev" | grep '^STREAM:pattern_ok' &&
	   dmesg | grep -qE 'Framedesc: stream 0|	stream 0, code .*, vc 0'; then
		say "frame-desc ok"
	else
		say "frame-desc fail (no pattern or no descriptor)"
	fi
	echo 0 > "$fd_param"
	dmesg | grep -E 'Framedesc|	stream 0, code'
fi
