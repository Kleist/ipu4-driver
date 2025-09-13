#!/bin/env python3
"""
python functions for extracting and analysing frame counts

This module should not depend on a test agent or DUT being available
"""

import os
import statistics

# In order to catch out of order frames, we don't want small decrements to
# be seen as the counter resetting. This threshold is the minimum the count
# should be lower, before we consider the counter to be reset.
UNWRAP_THRESHOLD=50

def unwrap(counts: list) -> list:
    """
    frame counts are 8 bits, but 0 is skipped, so the sequence of a perfect frame count
    wrapping is [254, 255, 1, 2 ]

    This function assumes any decreasing frame count is wrapping, and adds 255 after each wrap.

    Examples (run with doctest):

    >>> unwrap([1,2,3])
    [1, 2, 3]

    >>> unwrap([255, 1])
    [255, 256]

    >>> unwrap([255, 1, 255, 1])
    [255, 256, 510, 511]

    # Counter going less than UNWRAP_THRESHOLD back is not considered a wrap
    >>> UNWRAP_THRESHOLD # Adding as test to verify test is up to date
    50
    >>> unwrap([255, 250])
    [255, 250]

    # Counter going more than UNWRAP_THRESHOLD back is considered wrapping
    >>> unwrap([52, 1])
    [52, 256]
"""
    last = counts[0]
    unwrapped = []
    offset = 0
    for count in counts:
        if count+offset < last-UNWRAP_THRESHOLD:
            offset += 255
        last = count+offset
        unwrapped.append(last)
    return unwrapped

def jumps(frame_counts: list) -> list:
    """
    Returns the minimum, average and maximum distance between frame_counts

    A perfect frame count will have the values [min: 1, avg: 1, max: 1]

    Examples (run with doctest):
    >>> jumps([1, 2])
    [1]

    >>> jumps([1, 2, 6])
    [1, 4]

    >>> jumps([255, 1])
    [1]
    """
    unwrapped = unwrap(frame_counts)
    return [y-x for x,y in zip(unwrapped, unwrapped[1:])]

def stats(counts: list) -> dict:
    """
    >>> stats([1, 2])
    {'min': 1, 'mean': 1, 'max': 1}

    >>> stats([1, 3])
    {'min': 2, 'mean': 2, 'max': 2}

    >>> stats([1, 3, 7])
    {'min': 2, 'mean': 3, 'max': 4}

    >>> stats([255,1])
    {'min': 1, 'mean': 1, 'max': 1}
    """
    frame_jumps = jumps(unwrap(counts))
    return {
        'min': min(frame_jumps), 
        'mean': statistics.mean(frame_jumps),
        'max': max(frame_jumps)
    }

def extract(output_path: str, resolution: int) -> list:
    """
    Extract the frame count from a raw capture of BGR0 frames

    The frame number is stored in the third pixel of every frame in the following manner
    {R[7:2], frame_number[7:6], G[7:3], frame_number[5:3], B[7:3], frame_number[2:0]} 
    """
    BYTES_PER_PIXEL=4 # BGRx format
    frame_size = BYTES_PER_PIXEL*resolution*resolution

    def get_frame_count_pixel_offset(frame_no: int) -> int:
        return frame_size*frame_no+BYTES_PER_PIXEL*2

    def get_frame_count_from_pixel_data(bgr: bytes) -> int:
        B = bgr[0]
        G = bgr[1]
        R = bgr[2]
        return (((R & 0x03) << 6) | ((G & 0x07) << 3) | (B & 0x07))

    file_size_in_bytes = os.stat(output_path).st_size
    frame_count = file_size_in_bytes / frame_size
    fc_list = []
    with open(output_path, 'rb') as f:
        for frame_no in range(int(frame_count)):
            offset = get_frame_count_pixel_offset(frame_no)
            # Read BGR pixel values
            f.seek(offset)
            bgr = f.read(3)
            frame_counter = get_frame_count_from_pixel_data(bgr)
            fc_list.append(frame_counter)
    return fc_list

def print_array(name, values):
    value_str = ", ".join("%3d" % v for v in values)
    print(f"{name:>6}: {value_str}")

def print_counts(counts):
    unwrapped = unwrap(counts)
    frame_jumps = jumps(unwrapped)
    STEP=20
    for offset in range(0, len(counts), STEP):
        print_array("Raw", counts[offset:offset+STEP])
        print_array("Unwrap", unwrapped[offset:offset+STEP])
        print_array("Jumps", frame_jumps[offset:offset+STEP])
        print("")

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("file")
    parser.add_argument("resolution", type=int)
    parser.add_argument("-v","--verbose", action="store_true")
    args = parser.parse_args()
    counts = extract(args.file, args.resolution)
    if args.verbose:
        print_counts(counts)

    frame_stats = stats(counts)
    for k,v in frame_stats.items():
        print(f"{k:5}: {v:5.1f}")

if __name__ == "__main__":
    main()