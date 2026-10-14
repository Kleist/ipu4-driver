# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-modules before the unload; say()
# prints a MODLOAD: marker.
#
# MODLOAD:stream-requeue ok|fail <why>  buffers queued while streaming
#                                       came back with frames

rq_video=
rq_subdev=
for n in /sys/class/video4linux/*; do
	case "$(cat "$n/name" 2>/dev/null)" in
	"$IPU_ENTITY ISYS Capture 0") rq_video="/dev/${n##*/}" ;;
	"$IPU_ENTITY CSI2 0") rq_subdev="/dev/${n##*/}" ;;
	esac
done
if [ -z "$rq_video" ] || [ -z "$rq_subdev" ]; then
	say "stream-requeue fail (no CSI2 0 subdev or capture node)"
else
	mkdir -p /tmp
	STREAMON_REQUEUE=3 streamon "$rq_video" /dev/media0 "$rq_subdev" \
		> /tmp/requeue.log
	grep -E '^STREAM:(requeue|fail|pattern)' /tmp/requeue.log
	if grep -q '^STREAM:requeue_ok n=3' /tmp/requeue.log; then
		say "stream-requeue ok"
	else
		say "stream-requeue fail (see STREAM: lines)"
	fi
fi
