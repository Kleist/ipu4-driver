# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-probe (README.md here); say() prints a
# PROBE: marker.
#
# PROBE:v4l2-query ran|fail <why>  /bin/v4l2-query ran against CSI-2
#                                  port 0's subdev and capture node; its
#                                  QUERY: lines are checked on the host

q_video=
q_subdev=
for n in /sys/class/video4linux/*; do
	case "$(cat "$n/name" 2>/dev/null)" in
	"$IPU_ENTITY ISYS Capture 0") q_video="/dev/${n##*/}" ;;
	"$IPU_ENTITY CSI2 0") q_subdev="/dev/${n##*/}" ;;
	esac
done
if [ -n "$q_video" ] && [ -n "$q_subdev" ]; then
	v4l2-query "$q_video" "$q_subdev"
	say "v4l2-query ran"
else
	say "v4l2-query fail (no CSI2 0 subdev or capture node)"
fi
