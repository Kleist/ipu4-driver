#!/bin/bash -ue
if [ "$#" != "1" ]; then
    echo "Usage: $0 output_path"
    echo ""
    echo "Copies and splits mmiotrace from the target with .ssh/config 'Host ab'"
    echo "The output is stored in 'output_path/(kernel_major_version)' as multiple files"
fi

OUTPUT_PATH="$1"

strip_invalidate() {
    # Useless use of cat - but it makes it all align :)
    cat "$1" \
        | grep -v -E "MARK.*_invalidate.*" \
        | sed -e "s/.*psys.*mmu.*/(stripped psys mmu)/" \
            -e "s/.*isys.*mmu.*/(stripped isys mmu)/" \
        | uniq -c
}

scp ab:/mnt/storage/trace.txt $OUTPUT_PATH/trace.txt
./postprocess_trace.py $OUTPUT_PATH/trace.txt  > $OUTPUT_PATH/pp_all.txt
strip_invalidate $OUTPUT_PATH/pp_all.txt >$OUTPUT_PATH/stripped_all.txt

grep "^W 4.*" $OUTPUT_PATH/pp_all.txt|sort|uniq -c>$OUTPUT_PATH/sorted_writes.txt

for mod in "intel-ipu4" "intel-ipu4-isys"; do 
    ./postprocess_trace.py $OUTPUT_PATH/trace.txt --begin "modprobe. *$mod.* begin" --end "modprobe .*$mod.* end" > $OUTPUT_PATH/pp_${mod}.txt
    strip_invalidate $OUTPUT_PATH/pp_${mod}.txt >$OUTPUT_PATH/stripped_${mod}.txt
done

./postprocess_trace.py $OUTPUT_PATH/trace.txt --begin "video stream begin" --end "video stream done" > $OUTPUT_PATH/pp_stream.txt
strip_invalidate $OUTPUT_PATH/pp_stream.txt >$OUTPUT_PATH/stripped_stream.txt

./postprocess_trace.py $OUTPUT_PATH/trace.txt --begin "start_streaming begin" --end "start_streaming end" > $OUTPUT_PATH/pp_start_streaming.txt
strip_invalidate $OUTPUT_PATH/pp_start_streaming.txt >$OUTPUT_PATH/stripped_start_streaming.txt

dump_state_contexts="$(sed -n "s/.*ipu_dump_state in context \(.*\) begin/\1/p" "$OUTPUT_PATH/trace.txt")"

echo "Dump state contexts:"
echo "$dump_state_contexts"

while IFS= read -r dump_state_context; do
    echo "Splitting $dump_state_context"
    ./postprocess_trace.py $OUTPUT_PATH/trace.txt --begin "ipu_dump_state in context $dump_state_context begin" --end "ipu_dump_state in context $dump_state_context end" > "$OUTPUT_PATH/pp_$dump_state_context.txt"
    strip_invalidate "$OUTPUT_PATH/pp_$dump_state_context.txt" >"$OUTPUT_PATH/stripped_$dump_state_context.txt"
done <<< "$dump_state_contexts"
