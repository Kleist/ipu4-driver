# shellcheck shell=sh disable=SC2154
# Sourced by tools/rootfs/init.ipu6-modules before the unload; say()
# prints a MODLOAD: marker.
#
# MODLOAD:silent-reset ok|skip|fail <why>  opening a capture node while
#                                          the ISYS needs a reset restarts
#                                          the running stream, which then
#                                          goes on delivering frames

sr_param=/sys/module/${IPU_MODNAME}_isys/parameters/force_need_reset
sr_video0=
sr_video1=
sr_subdev=
for n in /sys/class/video4linux/*; do
	case "$(cat "$n/name" 2>/dev/null)" in
	"$IPU_ENTITY ISYS Capture 0") sr_video0="/dev/${n##*/}" ;;
	"$IPU_ENTITY ISYS Capture 1") sr_video1="/dev/${n##*/}" ;;
	"$IPU_ENTITY CSI2 0") sr_subdev="/dev/${n##*/}" ;;
	esac
done
if [ ! -e "$sr_param" ]; then
	say "silent-reset skip (no force_need_reset parameter)"
elif [ -z "$sr_video0" ] || [ -z "$sr_video1" ] || [ -z "$sr_subdev" ]; then
	say "silent-reset fail (no CSI2 0 subdev or capture nodes)"
else
	mkdir -p /tmp
	rm -f /tmp/sr-go
	STREAMON_PAUSE=/tmp/sr-go STREAMON_REQUEUE=3 \
		streamon "$sr_video0" /dev/media0 "$sr_subdev" \
		> /tmp/silent-reset.log &
	sr_pid=$!
	for _ in 1 2 3 4 5 6 7 8 9 10; do
		grep -q '^STREAM:paused' /tmp/silent-reset.log && break
		sleep 1
	done
	# The open of another node restarts the paused stream (video_open).
	echo 1 > "$sr_param"
	: < "$sr_video1"
	touch /tmp/sr-go
	wait "$sr_pid"
	grep -E '^STREAM:(paused|requeue|fail)' /tmp/silent-reset.log
	if ! grep -q '^STREAM:requeue_ok n=3' /tmp/silent-reset.log; then
		say "silent-reset fail (no frames after the restart)"
	elif ! dmesg | grep -q 'restarting other streams done: 0'; then
		say "silent-reset fail (no restart in the kernel log)"
	else
		say "silent-reset ok"
	fi
fi
