#!/bin/bash -eu
TD=/sys/kernel/tracing

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
TRACE_FILE="/mnt/storage/trace.txt"
TRACE_TYPE=mmiotrace
STREAM_TOOL=yavta
RESOLUTION="800x800"
MIPI_BRIDGE_CHAN="0-000e"
IPU_CSI_CHAN=0
VIDEO_OUTPUT=12

function usage() {
        echo "Usage:"
        echo "   $0 [-r WxH][-c <channel>][-t <trace_type>][-s <stream_tool][-f <output_file>]"
        echo ""
        echo "Runs a trace of the IPU4 driver on target"
        echo ""
        echo "Options:"
        echo "   -r  Resolution on the format WxH, (default $RESOLUTION)"
        echo "   -c  Channel 0 or 1 (default 0)"
        echo "   -t  Trace type e.g., mmiotrace/i2c/function/function_graph (default mmiotrace)"
        echo "   -s  Tool used to start streaming, yavta (default) or gstreamer"
        echo "   -f  Trace output file (default $TRACE_FILE)"
        echo ""
        echo "   -h  Show this help text"
        echo ""
}

while getopts "hr:t:s:c:" opt; do
  case $opt in
    r)
      if [ "$OPTARG" == "400x400" ]; then
        RESOLUTION="$OPTARG"
      elif [ "$OPTARG" == "800x800" ]; then
        RESOLUTION="$OPTARG"
      else
        echo "Error: Resolution $OPTARG not supported"
        exit 1
      fi
    ;;
    t)
        TRACE_TYPE="$OPTARG"
    ;;
    s)
        STREAM_TOOL="$OPTARG"
    ;;
    c)
      if [ "$OPTARG" == "0" ]; then
        MIPI_BRIDGE_CHAN="0-000e"
        IPU_CSI_CHAN=0
        VIDEO_OUTPUT=12
      elif [ "$OPTARG" == "1" ]; then
        MIPI_BRIDGE_CHAN="3-000e"
        IPU_CSI_CHAN=4
        VIDEO_OUTPUT=13
      else
        echo "Error: Channel $OPTARG not supported"
        exit 1
      fi
    ;;
    h)
        usage
        exit 0
    ;;
    \?)
        echo "Error: Invalid option -$OPTARG" >&2
        usage
        exit 1
    ;;
  esac
done

WIDTH=$(echo "$RESOLUTION" | cut -dx -f1)
HEIGHT=$(echo "$RESOLUTION" | cut -dx -f2)
VIDEO_DEVICE="/dev/video$VIDEO_OUTPUT"


echo "Tracing $TRACE_TYPE from running $STREAM_TOOL"

echo "Tracing $TRACE_TYPE to file: $TRACE_FILE"

. "$SCRIPT_DIR/trace_functions.sh"

echo "Trying to unload drivers"
echo -n 0000:00:03.0 > /sys/bus/pci/drivers/intel-ipu4/unbind || true
modprobe -r intel_ipu4_isys || true
modprobe -r intel_ipu4 || true
modprobe -r ambu_tc358748 || true

trace_init

lcd_source 1 || true

modprobe -a videobuf2-v4l2 videobuf2-dma-contig
modprobe ambu-tc358748

trace_start $TRACE_TYPE $TRACE_FILE

trace_modprobe intel-ipu4
trace_modprobe intel-ipu4-isys

tracer trace_marker "media-ctl 0"
media-ctl -v \
  -V "\
    \"tc358748 $MIPI_BRIDGE_CHAN\"    :0 [fmt:RGB888_1X24/$RESOLUTION],\
    \"Intel IPU4 CSI2 $IPU_CSI_CHAN\"  :0 [fmt:RGB888_1X24/$RESOLUTION],\
    \"Intel IPU4 CSI2 $IPU_CSI_CHAN\"  :1 [fmt:RGB888_1X24/$RESOLUTION]\
    "\
  -l "\
    \"tc358748 $MIPI_BRIDGE_CHAN\"    :0 -> \"Intel IPU4 CSI2 $IPU_CSI_CHAN\" :0 [1],\
    \"Intel IPU4 CSI2 $IPU_CSI_CHAN\"  :1 -> \"Intel IPU4 ISYS Capture $VIDEO_OUTPUT\" :0 [5]\
  "

tracer trace_marker "video stream begin"
if [ "$STREAM_TOOL" = gstreamer ]; then
  GST_DEBUG=6 gst-launch-1.0 v4l2src device=$VIDEO_DEVICE num-buffers=10 ! video/x-raw,width="$WIDTH",height="$HEIGHT",format=BGRx ! fbdevsink
else
  timeout 20s yavta --data-prefix -c10 -n4 -I -s "$RESOLUTION" --file=/tmp/frame-#.bin -f XBGR32 $VIDEO_DEVICE || echo "yavta failed"
fi

tracer trace_marker "video stream end"
sleep 1
trace_stop
if [ "$STREAM_TOOL" = yavta ]; then
  # First frame often contains some noise, so let's get the second here:
  bgr0_to_image --width "$WIDTH" --height "$HEIGHT" /tmp/frame-000001.bin /tmp/frame-000001.png
  hexdump -C /tmp/frame-000001.bin -n 32
fi