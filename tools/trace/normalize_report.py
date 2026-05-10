#!/usr/bin/env python3
"""Redact intrinsically-volatile fields in a compare-mmio report so the
text can be diffed across runs without false positives.

The compare-mmio gate (``tools/tests/compare-mmio.sh``) does a byte-equal
diff of a freshly-generated ``report.txt`` against a committed baseline.
Some registers — TSC counters, FW_COM ring positions, PWR_STATE FSM
samples — return values that depend on wall-clock pacing under the QEMU
model and therefore drift every run. CLAUDE.md calls these out as
"intrinsic divergence, not missing handlers."

This module rewrites those rows' ``qemu:`` value lists to the literal
string ``<volatile>`` and rewrites the ``qemu accesses: <N>`` total to
``qemu accesses:    <volatile>``. Stable rows are left untouched, so a
real regression (e.g. a previously-handled register starts returning a
different value) still trips the gate.

The allowlist file format is one ``<region>|<offset>`` per line, with
``#`` comments. ``|`` is the separator because no region label produced
by ``compare.py`` contains it (regions today: ``buttress``, ``isys spc``,
``isys dmem``, ``csi2 port0``, ``isys0 mmu``).
"""
from __future__ import annotations

import argparse
import sys


VOLATILE_PLACEHOLDER = '<volatile>'


def load_volatile(path):
    """Return a set of ``(region, offset_lower)`` tuples from an
    allowlist file. Lines may contain ``#`` comments and blanks."""
    entries = set()
    with open(path) as fh:
        for raw in fh:
            line = raw.split('#', 1)[0].strip()
            if not line:
                continue
            if '|' not in line:
                raise ValueError(
                    f"{path}: malformed entry {raw!r} — expected "
                    f"'<region>|<offset>'"
                )
            region, offset = line.split('|', 1)
            entries.add((region.strip(), offset.strip().lower()))
    return entries


def _parse_address_row(line):
    """Parse the indented ``  <region:14> <offset:6>  <name>`` row that
    ``compare.py::_fmt_addr`` emits inside the read-value-mismatches
    section. Returns ``(region, offset_lower)`` or ``None`` if the line
    isn't an address row."""
    # _fmt_addr layout: exactly 2 leading spaces, region left-padded
    # to 14, 1 space, offset right-padded to 6, optional "  <name>".
    # The silicon/qemu value lines have 4 leading spaces, so the
    # exactly-two-space check excludes them — without it a long
    # silicon/qemu payload whose mid-string happens to span "0x..."
    # at indices 17-22 gets misidentified as an address row.
    if len(line) < 23 or not line.startswith('  ') or line[2] == ' ':
        return None
    region = line[2:16].rstrip()
    offset = line[17:23].strip()
    if not offset.startswith('0x'):
        return None
    if not region:
        return None
    return (region, offset.lower())


def normalize(text, volatile):
    """Return ``text`` with volatile rows redacted. ``volatile`` is the
    set returned by ``load_volatile``."""
    out = []
    section = None  # 'unimpl' | 'mismatch' | 'extra' | None
    expect_volatile_qemu = False
    for line in text.splitlines():
        if line.startswith('qemu accesses:'):
            out.append(f'qemu accesses:    {VOLATILE_PLACEHOLDER}')
            continue
        if line.startswith('== unimplemented'):
            section = 'unimpl'
            expect_volatile_qemu = False
            out.append(line)
            continue
        if line.startswith('== read-value mismatches'):
            section = 'mismatch'
            expect_volatile_qemu = False
            out.append(line)
            continue
        if line.startswith('== addresses only in qemu'):
            section = 'extra'
            expect_volatile_qemu = False
            out.append(line)
            continue
        if section == 'mismatch':
            parsed = _parse_address_row(line)
            if parsed is not None:
                expect_volatile_qemu = parsed in volatile
                out.append(line)
                continue
            if expect_volatile_qemu and line.startswith('    qemu:'):
                out.append(f'    qemu:    {VOLATILE_PLACEHOLDER}')
                expect_volatile_qemu = False
                continue
        out.append(line)
    # Preserve a trailing newline if the input had one (splitlines drops
    # it). compare.py's format_diff doesn't add one, so str.endswith('\n')
    # is the test.
    suffix = '\n' if text.endswith('\n') else ''
    return '\n'.join(out) + suffix


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        '--volatile', required=True,
        help='Path to the volatile-allowlist file (region|offset, one per line).',
    )
    parser.add_argument(
        '--in', dest='in_path', default='-',
        help='Path to the report.txt to normalize. "-" reads stdin (default).',
    )
    parser.add_argument(
        '--out', default='-',
        help='Where to write the normalized report. "-" writes stdout (default).',
    )
    args = parser.parse_args(argv)

    volatile = load_volatile(args.volatile)
    if args.in_path == '-':
        text = sys.stdin.read()
    else:
        with open(args.in_path) as fh:
            text = fh.read()

    normalized = normalize(text, volatile)

    if args.out == '-':
        sys.stdout.write(normalized)
    else:
        with open(args.out, 'w') as fh:
            fh.write(normalized)
    return 0


if __name__ == '__main__':
    sys.exit(main())
