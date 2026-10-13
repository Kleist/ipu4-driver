# shellcheck shell=bash
# Sourced by tools/tests/ipu6/ladder.sh; see tools/tests/ipu6/tests.d/README.md.

ladder_test v4l2-query probe-identify

# The capture node's and CSI-2 subdev's query and format ioctls work
# while the driver is bound: format and frame size enumeration, get/try/
# set format, CREATE_BUFS sizing, mbus code enumeration, FRAME_SYNC event
# subscription and the source pad's crop selection. None of them touches
# the hardware. The probe init runs /bin/v4l2-query (tools/tests/
# v4l2-query.c; snippet tools/rootfs/init.ipu6-probe.d/
# 090-v4l2-query.bound.sh), which prints a QUERY: line per step. The
# metadata format steps are reported but not required.
V4L2_QUERY_STEPS=(querycap enum_fmt_vid enum_framesizes g_fmt_vid try_fmt_vid
	s_fmt_vid create_bufs enum_mbus_code subscribe_event g_sel_crop
	g_sel_crop_default g_sel_crop_bounds s_sel_crop)

test_v4l2-query() {
	boot_probe_vm || return 1
	grep -E '^(QUERY:|PROBE:v4l2-query)' "$PROBE_LOG"
	probe_log_ok && grep -q '^PROBE:v4l2-query ran' "$PROBE_LOG" || return 1
	local s rc=0
	for s in "${V4L2_QUERY_STEPS[@]}"; do
		if ! grep -q "^QUERY:$s ok" "$PROBE_LOG"; then
			echo "!!! QUERY:$s did not succeed"
			rc=1
		fi
	done
	return "$rc"
}
