#!/bin/bash -ue

TD=/sys/kernel/tracing
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
TRACER_PID=0

tracer() {
        echo "tracer: $1 = $2"
        echo "$2" >"$TD/$1"
}

trace_modprobe() {
        MODULE="$1"
        echo "modprobe'ing $@"
        tracer trace_marker "modprobe $1 begin"
        modprobe "$@"
        sleep 1
        tracer trace_marker "modprobe $1 end"
}

trace_init() {
        mount -t tracefs nodev $TD || true
        tracer current_tracer nop
        echo "Increasing trace buffers"
        echo 655360 >/sys/kernel/debug/tracing/buffer_size_kb
}

trace_start() {
        TRACE_TYPE="$1"
        TRACE_FILE="$2"
        case "$TRACE_TYPE" in
        mmiotrace)
                tracer current_tracer mmiotrace
        ;;

        i2c)
                tracer current_tracer nop
                echo 1 > /sys/kernel/tracing/events/i2c/enable
                echo 1 > /sys/kernel/tracing/tracing_on
        ;;

        function)
                echo "*:mod:tc358746 *:mod:intel_ipu* fwnode*" >$TD/set_ftrace_filter
                tracer current_tracer function
        ;;

        function_graph)
                trace_modprobe intel-ipu4
                trace_modprobe intel-ipu4-isys

                echo "*:mod:tc358746 *:mod:intel_ipu* fwnode* v4l2_subdev_*" >$TD/set_ftrace_filter
                echo "!ipu6_dma_alloc !ipu6_dma_free !ipu6_mmu_map !ipu6_mmu_unmap !ipu6_dma_mmap !tlb_invalidate !ipu6_mmu_iova_to_phys !__dma_clear_buffer" >>$TD/set_ftrace_filter
                echo 1 > /sys/kernel/tracing/options/funcgraph-retval || echo "funggraph-retval not available"
                tracer current_tracer function_graph
        ;;
        *)
                echo "Unknown trace type $TRACE_TYPE"
                exit 1
        ;;
        esac


        cat $TD/trace_pipe >"$TRACE_FILE" &
        TRACER_PID=$!
        echo "Trace started with PID $TRACER_PID"
}

trace_stop() {
        echo "Killing tracer PID $TRACER_PID"
        kill $TRACER_PID
}