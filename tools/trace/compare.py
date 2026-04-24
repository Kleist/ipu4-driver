#!/usr/bin/env python3
"""Diff an MMIO trace captured on real IPU4 silicon against one captured
under the QEMU model, and classify the divergences.

Both inputs are JSON Lines as emitted by ``postprocess_trace.py --json``.
A records-only stream (with ``reg_name`` / ``decoded`` fields populated)
keeps this script decoupled from the raw ftrace mmiotrace format and
makes it straight-forward to feed synthetic traces from unit tests.

The diff is structural, not line-aligned: we bucket accesses by
(address, op) and compare per-bucket properties. Timing-sensitive
differences (QEMU being slower than silicon, different retry counts,
etc.) would dominate any strict line-by-line diff and drown out the
signal we actually want — "which registers does the real driver touch
that the model still answers with ``LOG_UNIMP``?".

Classifications:

  1. unimplemented: addresses the silicon trace touched that the QEMU
     trace never touched. Sorted by access count descending — that's
     the worklist for extending ``tools/qemu-patches/hw/misc/ipu4.c``.
  2. value_mismatch: both streams read the same address but returned
     disjoint value sets. The M4.5 ``BTRS_PWR_STATE_IS_PWR_RDY`` shift
     bug is exactly this: silicon returned an IPU4 layout, QEMU was
     returning an IPU6 layout.
  3. extra_in_qemu: addresses QEMU accessed that silicon didn't. Usually
     benign (different init sequence) but worth reporting so the model
     doesn't drift silently.

Exit codes: 0 if no divergences, 1 if any unimplemented address or any
value mismatch, 2 on argument / parse errors. ``extra_in_qemu`` alone
does not fail the diff — by itself it's informational.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Iterable


# The IPU4 exposes a 16 MiB BAR starting at 0x90000000 in the mmiotrace
# capture. Accesses outside this window in ``data/trace.txt`` come from
# other PCI devices (igb, i915, pcieport, ...) and are noise for this
# tool; filter them out so ``lookup_address`` collisions at the top of
# the region table don't pollute the diff.
IPU4_BAR_START = 0x90000000
IPU4_BAR_END = 0x91000000


def is_ipu4_address(address):
    return IPU4_BAR_START <= address < IPU4_BAR_END


def load_records(path_or_file):
    """Yield JSON records from a JSONL file path or an already-open file,
    skipping records outside the IPU4 BAR. Records must at minimum carry
    ``address`` and ``op`` keys."""
    if hasattr(path_or_file, 'read'):
        fh = path_or_file
        close = False
    else:
        fh = open(path_or_file)
        close = True
    try:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            if 'address' not in rec or 'op' not in rec:
                continue
            if not is_ipu4_address(rec['address']):
                continue
            yield rec
    finally:
        if close:
            fh.close()


@dataclass
class _Bucket:
    """Per-(address, op) aggregation. ``values`` is the set of distinct
    payloads — for reads that's what the device returned, for writes
    that's what the driver chose to write."""
    address: int = 0
    op: str = ''
    region: str | None = None
    reg_name: str | None = None
    offset: int | None = None
    count: int = 0
    values: set = field(default_factory=set)


def _bucket_records(records):
    buckets = {}
    for rec in records:
        key = (rec['address'], rec['op'])
        b = buckets.setdefault(key, _Bucket(
            address=rec['address'],
            op=rec['op'],
            region=rec.get('region'),
            reg_name=rec.get('reg_name'),
            offset=rec.get('offset'),
        ))
        b.count += 1
        b.values.add(rec['value'])
    return buckets


@dataclass
class Diff:
    silicon_total: int = 0
    qemu_total: int = 0
    unimplemented: list[dict] = field(default_factory=list)
    value_mismatches: list[dict] = field(default_factory=list)
    extra_in_qemu: list[dict] = field(default_factory=list)

    def is_clean(self):
        return not self.unimplemented and not self.value_mismatches


def diff(silicon_records, qemu_records) -> Diff:
    silicon_records = list(silicon_records)
    qemu_records = list(qemu_records)
    silicon_buckets = _bucket_records(silicon_records)
    qemu_buckets = _bucket_records(qemu_records)

    # Group by address (op-agnostic) for the "has QEMU seen this
    # address at all?" question. Individual-op mismatches are detected
    # separately below for value-divergence classification.
    qemu_addresses = {addr for (addr, _op) in qemu_buckets}

    d = Diff(silicon_total=len(silicon_records), qemu_total=len(qemu_records))

    # (1) unimplemented
    unimpl_agg = {}
    for (addr, op), b in silicon_buckets.items():
        if addr in qemu_addresses:
            continue
        agg = unimpl_agg.setdefault(addr, {
            'address': addr,
            'region': b.region,
            'reg_name': b.reg_name,
            'offset': b.offset,
            'reads': 0,
            'writes': 0,
        })
        if op == 'R':
            agg['reads'] = b.count
        else:
            agg['writes'] = b.count
    d.unimplemented = sorted(
        unimpl_agg.values(),
        key=lambda row: row['reads'] + row['writes'],
        reverse=True,
    )

    # (2) value_mismatch: read-side value divergence. Write-side is
    # driver-owned, so we only diff reads here — QEMU's read return
    # value is what the model has to get right.
    for (addr, op), b in silicon_buckets.items():
        if op != 'R':
            continue
        qb = qemu_buckets.get((addr, 'R'))
        if qb is None:
            continue  # unimplemented, already reported
        if b.values == qb.values:
            continue
        only_silicon = sorted(b.values - qb.values)
        only_qemu = sorted(qb.values - b.values)
        d.value_mismatches.append({
            'address': addr,
            'region': b.region,
            'reg_name': b.reg_name,
            'offset': b.offset,
            'silicon_values': sorted(b.values),
            'qemu_values': sorted(qb.values),
            'only_silicon': only_silicon,
            'only_qemu': only_qemu,
        })
    d.value_mismatches.sort(key=lambda row: row['address'])

    # (3) extra_in_qemu: addresses QEMU touched but silicon didn't.
    silicon_addresses = {addr for (addr, _op) in silicon_buckets}
    extra_agg = {}
    for (addr, op), b in qemu_buckets.items():
        if addr in silicon_addresses:
            continue
        agg = extra_agg.setdefault(addr, {
            'address': addr,
            'region': b.region,
            'reg_name': b.reg_name,
            'offset': b.offset,
            'reads': 0,
            'writes': 0,
        })
        if op == 'R':
            agg['reads'] = b.count
        else:
            agg['writes'] = b.count
    d.extra_in_qemu = sorted(
        extra_agg.values(),
        key=lambda row: row['reads'] + row['writes'],
        reverse=True,
    )

    return d


def _fmt_offset(offset):
    return f"0x{offset:03x}" if offset is not None else '?'


def _fmt_addr(row):
    region = row.get('region') or '?'
    name = row.get('reg_name') or ''
    off_str = _fmt_offset(row.get('offset'))
    if name:
        return f"{region:<14} {off_str:>6}  {name}"
    return f"{region:<14} {off_str:>6}"


def _fmt_values(values):
    if not values:
        return '(none)'
    if len(values) <= 4:
        return ' '.join(f'0x{v:x}' for v in values)
    preview = ' '.join(f'0x{v:x}' for v in values[:4])
    return f"{preview} ... (+{len(values) - 4} more)"


def format_diff(d: Diff) -> str:
    out = []
    out.append(f"silicon accesses: {d.silicon_total}")
    out.append(f"qemu accesses:    {d.qemu_total}")
    out.append('')

    out.append(f"== unimplemented addresses ({len(d.unimplemented)}) ==")
    if not d.unimplemented:
        out.append('  (none)')
    else:
        out.append(f"  {'region':<14} {'offset':>6}  {'name':<30} {'R':>4} {'W':>4}")
        for row in d.unimplemented:
            region = row['region'] or '?'
            off_str = _fmt_offset(row['offset'])
            name = row['reg_name'] or ''
            out.append(
                f"  {region:<14} {off_str:>6}  {name:<30} "
                f"{row['reads']:>4} {row['writes']:>4}"
            )
    out.append('')

    out.append(f"== read-value mismatches ({len(d.value_mismatches)}) ==")
    if not d.value_mismatches:
        out.append('  (none)')
    else:
        for row in d.value_mismatches:
            out.append(f"  {_fmt_addr(row)}")
            out.append(f"    silicon: {_fmt_values(row['silicon_values'])}")
            out.append(f"    qemu:    {_fmt_values(row['qemu_values'])}")
    out.append('')

    out.append(f"== addresses only in qemu ({len(d.extra_in_qemu)}) ==")
    if not d.extra_in_qemu:
        out.append('  (none)')
    else:
        for row in d.extra_in_qemu:
            region = row['region'] or '?'
            off_str = _fmt_offset(row['offset'])
            name = row['reg_name'] or ''
            out.append(
                f"  {region:<14} {off_str:>6}  {name:<30} "
                f"{row['reads']:>4} {row['writes']:>4}"
            )

    return '\n'.join(out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('silicon', help='JSONL from postprocess_trace.py on real capture')
    parser.add_argument('qemu', help='JSONL from postprocess_trace.py on QEMU capture')
    parser.add_argument(
        '--json', action='store_true',
        help='Emit the full diff as a single JSON document to stdout '
             'instead of the human-readable report.',
    )
    args = parser.parse_args(argv)

    silicon = load_records(args.silicon)
    qemu = load_records(args.qemu)
    d = diff(silicon, qemu)

    if args.json:
        print(json.dumps({
            'silicon_total': d.silicon_total,
            'qemu_total': d.qemu_total,
            'unimplemented': d.unimplemented,
            'value_mismatches': [
                {k: v for k, v in row.items()}
                for row in d.value_mismatches
            ],
            'extra_in_qemu': d.extra_in_qemu,
        }))
    else:
        print(format_diff(d))

    return 0 if d.is_clean() else 1


if __name__ == '__main__':
    sys.exit(main())
