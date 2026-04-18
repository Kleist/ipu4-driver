#!/usr/bin/env python3
"""Diff two mmiotrace captures from the QEMU fuzzing loop.

Usage: tools/trace/diff.py OLD NEW

Each input is a ftrace mmiotrace file as emitted by the guest. The diff is
register-address oriented, not line oriented: it reports how far into the
probe sequence each run got by listing the last BAR+offset that each run
touched, and the prefix they share.
"""
from __future__ import annotations

import re
import sys
from dataclasses import dataclass

# mmiotrace line: "R 4 0x8000 0xfeb00010 0xb00710ad 0"
# columns: op width phys_addr virt_addr value pid
_RE = re.compile(
    r"^\s*([RW])\s+\d+\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\b"
)


@dataclass(frozen=True)
class Access:
    op: str
    offset: int
    value: int

    def fmt(self) -> str:
        return f"{self.op} +0x{self.offset:06x} = 0x{self.value:08x}"


def parse(path: str) -> list[Access]:
    out: list[Access] = []
    with open(path) as f:
        for line in f:
            m = _RE.match(line)
            if not m:
                continue
            out.append(
                Access(op=m.group(1), offset=int(m.group(2), 16), value=int(m.group(4), 16))
            )
    return out


def common_prefix_len(a: list[Access], b: list[Access]) -> int:
    n = min(len(a), len(b))
    for i in range(n):
        if a[i] != b[i]:
            return i
    return n


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    old = parse(argv[1])
    new = parse(argv[2])
    prefix = common_prefix_len(old, new)
    print(f"old: {len(old)} accesses")
    print(f"new: {len(new)} accesses")
    print(f"shared prefix: {prefix}")
    if prefix < len(old):
        print(f"old diverged at #{prefix}: {old[prefix].fmt()}")
    if prefix < len(new):
        print(f"new diverged at #{prefix}: {new[prefix].fmt()}")
    if len(new) > len(old):
        print(f"progress: {len(new) - len(old)} new accesses past old end-of-trace")
        for a in new[len(old) : len(old) + 10]:
            print(f"  {a.fmt()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
