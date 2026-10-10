#!/usr/bin/env python3
"""Check that a capture makes the same IPU4 register writes as silicon.

Compares the MMIO *writes* that fall into a set of BAR-offset ranges in
two captures, in order, and passes only when both write sequences are
identical and not empty. The reference is the silicon trace
(``data/trace.txt``); the capture under test is either another mmiotrace
(the fork flavor's ``tools/tests/mmiotrace.sh``) or a QEMU
``memory_region_ops_write`` trace log (``IPU4_MMIO_WRITE_LOG`` in
``tools/run-vm.sh``, used by the ipu6 flavor's ladder). The format is
detected per line, so either file can be either kind.

Addresses are compared as offsets into the IPU BAR:

  * mmiotrace: ``W <width> <time> <map-id> 0x<addr> 0x<value> ...``,
    relative to the BAR of the first ``MAP`` line.
  * QEMU log: ``memory_region_ops_write cpu <n> mr 0x.. addr 0x<addr>
    value 0x<value> size <n> name '<region>'``, only for the region named
    by ``--region``. QEMU logs absolute addresses; a PCI BAR is aligned
    to its size, so the offset is the address modulo ``--bar-size``.

Unlike ``compare.py`` (a coverage report over every access), this is a
gate for a handful of registers whose values the model accepts blindly,
so wrong values would otherwise go unnoticed. Pick ranges that the
driver writes deterministically (configuration tables), not addresses
or counters.

Usage:
  check_writes.py --expected data/trace.txt --actual capture.log \\
      0x4b0008-0x4b0090 0x4b0108-0x4b0190

Exit status: 0 when the sequences match, 1 when they differ or the
expected side has no writes in the ranges, 2 on usage errors.
"""

import argparse
import difflib
import re
import sys

BAR_SIZE = 0x1000000  # 16 MiB, the IPU4 BAR (data/trace.txt MAP line).

_QEMU_RE = re.compile(
    r"memory_region_ops_write\s.*\baddr (0x[0-9a-fA-F]+) "
    r"value (0x[0-9a-fA-F]+) size \d+ name '([^']*)'")


def parse_range(text):
    """Parse ``START-END`` (hex or decimal, END exclusive)."""
    try:
        start, end = (int(part, 0) for part in text.split("-", 1))
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"bad range '{text}' (expected START-END, e.g. 0x8-0x90)")
    if end <= start:
        raise argparse.ArgumentTypeError(f"empty range '{text}'")
    return start, end


def read_writes(lines, region="ipu4-bar0", bar_size=BAR_SIZE):
    """Yield (bar_offset, value) for every write to the IPU BAR."""
    bar = None
    for line in lines:
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "MAP" and bar is None and len(fields) >= 6:
            bar, size = int(fields[3], 16), int(fields[5], 16)
            continue
        if fields[0] == "W" and len(fields) >= 6 and bar is not None:
            addr = int(fields[4], 16)
            if bar <= addr < bar + size:
                yield addr - bar, int(fields[5], 16)
            continue
        m = _QEMU_RE.search(line)
        if m and m.group(3) == region:
            yield int(m.group(1), 16) % bar_size, int(m.group(2), 16)


def in_ranges(writes, ranges):
    return [(off, val) for off, val in writes
            if any(start <= off < end for start, end in ranges)]


def fmt(write):
    return f"+0x{write[0]:06x} = 0x{write[1]:x}"


def compare(expected, actual, out):
    """Print the comparison to ``out``; return True when it passes."""
    if not expected:
        print("no expected writes in the ranges; check the ranges", file=out)
        return False
    if expected == actual:
        print(f"PASS: all {len(expected)} writes match", file=out)
        return True
    print(f"FAIL: expected {len(expected)} writes, got {len(actual)}",
          file=out)
    for line in difflib.unified_diff([fmt(w) for w in expected],
                                     [fmt(w) for w in actual],
                                     "expected", "actual", lineterm="", n=1):
        print(line, file=out)
    return False


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Compare register writes in BAR ranges between two "
                    "captures (mmiotrace or QEMU memory_region_ops_write "
                    "log).")
    ap.add_argument("--expected", required=True,
                    help="reference capture, usually data/trace.txt")
    ap.add_argument("--actual", required=True, help="capture under test")
    ap.add_argument("--region", default="ipu4-bar0",
                    help="QEMU memory region name of the IPU BAR")
    ap.add_argument("ranges", nargs="+", type=parse_range,
                    metavar="START-END", help="BAR offset range to compare")
    args = ap.parse_args(argv)

    with open(args.expected, errors="replace") as f:
        expected = in_ranges(read_writes(f, args.region), args.ranges)
    with open(args.actual, errors="replace") as f:
        actual = in_ranges(read_writes(f, args.region), args.ranges)
    return 0 if compare(expected, actual, sys.stdout) else 1


if __name__ == "__main__":
    sys.exit(main())
