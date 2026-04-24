#!/usr/bin/env python3
"""Decode an mmiotrace of the IPU4 into human- and machine-readable form.

Three output modes:

  (default) one line per W/R with the region, register name, and value
            decoded into named bitfields when the driver header defines
            them.
  --summary per-register histogram of access counts + distinct values.
  --json    JSON Lines with the same information in a form ``compare.py``
            and other downstream tools can consume.

Bitfield decoders are derived at startup from the kernel driver headers
under ``kernel/ipu4/``; if those headers are absent (e.g. when running
the script against a bare trace dump outside the repo) decoding falls
back to address-only output.
"""

import argparse
import errno
import json
import os
import pathlib
import re
import socket
import sys
from collections import Counter, defaultdict, namedtuple

AddressRegion = namedtuple('AddressRegion', ['begin', 'max_offset', 'name', 'named_regs'])
def address_region(begin, max_offset, name, named_regs=None):
    if named_regs is None:
        named_regs = []
    return AddressRegion(begin=begin, max_offset=max_offset, name=name, named_regs=named_regs)

SPC_REGS = {
    0x0: 'SPC_STATUS_CTRL',
}

CSI2_REGS = {
    0x000: 'RX_ENABLE',
    0x008: 'RX_CONFIG',
    0x400: 'PART_IRQ_EDGE',
    0x404: 'PART_IRQ_MASK',
    0x408: 'PART_IRQ_STATUS',
    0x40c: 'PART_IRQ_CLEAR',
    0x410: 'PART_IRQ_ENABLE',
    0x414: 'PART_IRQ_LEVEL_NOT_PULSE',
    0x500: 'RX_IRQ_EDGE',
    0x504: 'RX_IRQ_MASK',
    0x508: 'RX_IRQ_STATUS',
    0x50c: 'RX_IRQ_CLEAR',
    0x510: 'RX_IRQ_ENABLE',
    0x514: 'RX_IRQ_LEVEL_NOT_PULSE',
    0x600: 'S2M_IRQ_EDGE',
    0x604: 'S2M_IRQ_MASK',
    0x608: 'S2M_IRQ_STATUS',
    0x60c: 'S2M_IRQ_CLEAR',
    0x610: 'S2M_IRQ_ENABLE',
    0x614: 'S2M_IRQ_LEVEL_NOT_PULSE',
}

NAMED_REGIONS = [
    address_region(0x90400000 + 0x340000,   0x300, 'psys isp3'),
    address_region(0x90400000 + 0x2C0000,   0x300, 'psys isp2'),
    address_region(0x90400000 + 0x240000,   0x300, 'psys isp1'),
    address_region(0x90400000 + 0x1C0000,   0x300, 'psys isp0'),

    address_region(0x904b0600,   0x400, 'psys2 mmu'),
    address_region(0x904b0100,   0x500, 'psys1 mmu'),
    address_region(0x904b0000,   0x100, 'psys0 mmu'),

    address_region(0x90460000,   0x300, 'psys gpdev'),
    address_region(0x90430000,   0x300, 'psys spp0'),
    address_region(0x90420000,   0x300, 'psys spp1'),

    address_region(0x90408000,  0x8000, 'psys dmem', {
        0x0: 'BOOTLOADER_STATUS_OFFSET',
    }),
    address_region(0x90400000,  0x8008, 'psys spc '),

    address_region(0x901e0100,   0x400, 'isys1 mmu'),
    address_region(0x901e0000,   0x100, 'isys0 mmu'),


    address_region(0x9017c000,   0x500, 'isys unispart', {
        0x00: 'IRQ_EDGE',
        0x04: 'IRQ_MASK',
        0x08: 'IRQ_STATUS',
        0x0C: 'IRQ_CLEAR',
        0x10: 'IRQ_ENABLE',
        0x14: 'IRQ_LEVEL_NOT_PULSE',
        0x414: 'SW_IRQ',
        0x418: 'SW_IRQ_MUX',
    }),

    address_region(0x9016C800,   0x1000, 'csi2 port5', CSI2_REGS),
    address_region(0x9016C000,   0x1000, 'csi2 port4', CSI2_REGS),
    address_region(0x90167000,   0x1000, 'csi2 port3', CSI2_REGS),
    address_region(0x90166000,   0x1000, 'csi2 port2', CSI2_REGS),
    address_region(0x90165000,   0x1000, 'csi2 port1', CSI2_REGS),
    address_region(0x90164000,   0x1000, 'csi2 port0', CSI2_REGS),

    address_region(0x90108000,   0x100, 'isys dmem',{
        0x08: 'SYSCOM_STATE',
        0x28: 'FW_COM_SEND_WR_POS',
        0x2c: 'FW_COM_SEND_RD_POS',
        0x70: 'FW_COM_RECV_WR_POS',
        0x74: 'FW_COM_RECV_RD_POS',
    }),
    address_region(0x90100000,   0x100, 'isys spc ', SPC_REGS),
    address_region(0x90000000, 0xf7498, 'buttress ', {
        0x8: 'WDT',
        0xC: 'BTRS_CTRL',
        0x30: 'FW_RESET_CTL',
        0x34: 'IS_FREQ_CTL',
        0x38: 'PS_FREQ_CTL',
        0x50: 'ISH2IUCSR',
        0x54: 'ISH2IUDB0',
        0x58: 'ISH2IUDATA0',
        0x5C: 'PWR_STATE',
        0x78: 'FW_SOURCE_BASE_LO',
        0x7C: 'FW_SOURCE_BASE_HI',
        0x80: 'FW_SOURCE_BASE_SIZE',
        0x88: 'FABRIC_CMD',
        0x90: 'ISR_STATUS',
        0x94: 'ISR_ENABLED_STATUS',
        0x98: 'ISR_ENABLE',
        0x9c: 'ISR_CLEAR',
        0x100: 'IU2CSEDB0',
        0x104: 'IU2CSEDATA0',
        0x108: 'IU2CSECSR',
        0x10C: 'IU2ISHDB0',
        0x110: 'IU2ISHDATA0',
        0x114: 'IU2ISHDATA1',
        0x118: 'IU2ISHCSR',
        0x120: 'TSW_CTL',
        0x164: 'TSC_LO',
        0x168: 'TSC_HI',
        0x300: 'SECURITY_CTL',
        0x304: 'CSE2IUDB0',
        0x308: 'CSE2IUDATA0',
        0x30C: 'CSE2IUCSR',
    }),
]


# ---------------------------------------------------------------------------
# Header parser → field tables.
#
# Driver headers under kernel/ipu4/ already name every bitfield the silicon
# exposes. Parse the ones we know about into a flat {name: int} map, then
# fold the names into per-register field lists so the decoder can annotate
# raw register values as ``PWR_STATE=0x0fa02003 [IS_PWR_FSM=0xa(IS_RDY) ...]``.
# ---------------------------------------------------------------------------

HEADER_FILES = (
    'ipu4-platform-buttress-regs.h',
    'ipu6-platform-buttress-regs.h',
    'ipu4-platform-regs.h',
    'ipu6-platform-regs.h',
    'ipu4-platform-isys-csi2-reg.h',
    'ipu6-platform-isys-csi2-reg.h',
    'ipu6-fw-com.h',
)

# When multiple headers define the same logical field (e.g. IPU4 vs IPU6
# shifts for PWR_STATE.IS_PWR_FSM) we keep the one with the highest-priority
# prefix. IPU4 first because this driver targets IPU4 silicon.
_PREFIX_PRIORITY = (
    'IPU4_BUTTRESS_',
    'IPU4_',
    'BUTTRESS_',
    'IPU6SE_',
    'IPU6_BUTTRESS_',
    'IPU6_',
    '',
)

# Only the registers that show up in NAMED_REGIONS are candidates for
# field decoding; skip anything else so the parser doesn't pair up
# unrelated defines that happen to share a fragment of a register name.
_DECODABLE_REGS = {
    'PWR_STATE', 'FW_RESET_CTL', 'BTRS_CTRL',
    'ISR_STATUS', 'ISR_ENABLE', 'ISR_CLEAR', 'ISR_ENABLED_STATUS',
    'IU2CSECSR', 'CSE2IUCSR', 'IU2ISHCSR', 'ISH2IUCSR',
    'SECURITY_CTL',
    'IRQ_EDGE', 'IRQ_MASK', 'IRQ_STATUS', 'IRQ_CLEAR', 'IRQ_ENABLE',
    'PART_IRQ_STATUS', 'PART_IRQ_ENABLE',
    'RX_IRQ_STATUS', 'RX_IRQ_ENABLE',
    'S2M_IRQ_STATUS', 'S2M_IRQ_ENABLE',
}

_DEFINE_RE = re.compile(r'^\s*#\s*define\s+(\w+)\s+(.+?)\s*(?:/\*.*\*/)?\s*$')

_SIMPLE_INT_RE = re.compile(r'^(0x[0-9a-fA-F]+|[0-9]+)$')
_BIT_RE = re.compile(r'^BIT\((\d+)\)$')
_GENMASK_RE = re.compile(r'^GENMASK\((\d+)\s*,\s*(\d+)\)$')
_SHIFT_EXPR_RE = re.compile(r'^\((0x[0-9a-fA-F]+|\d+)\s*<<\s*(\d+)\)$')


def _eval_simple_expr(expr):
    """Return the integer value of ``expr`` or None if it's not a form we
    understand. Silently skips parametric macros, nested base+offset
    expressions, and anything else that isn't a constant literal."""
    expr = expr.strip()
    # Drop a trailing C-style comment if the define had one, e.g.
    #   #define X 0x5c  /* power state */
    expr = re.sub(r'\s*/\*.*$', '', expr)
    m = _SIMPLE_INT_RE.match(expr)
    if m:
        return int(m.group(1), 0)
    m = _BIT_RE.match(expr)
    if m:
        return 1 << int(m.group(1))
    m = _GENMASK_RE.match(expr)
    if m:
        hi, lo = int(m.group(1)), int(m.group(2))
        return ((1 << (hi + 1)) - 1) & ~((1 << lo) - 1)
    m = _SHIFT_EXPR_RE.match(expr)
    if m:
        return int(m.group(1), 0) << int(m.group(2))
    return None


def parse_header(path):
    """Parse ``#define NAME SIMPLE_EXPR`` lines from a header file.

    Returns a ``{name: int}`` dict. Function-like macros (``#define FOO(x)``)
    and defines whose RHS is not a simple literal expression are skipped.
    """
    defs = {}
    try:
        content = pathlib.Path(path).read_text()
    except OSError:
        return defs
    # Merge line-continuations so GENMASK arguments on the next line parse.
    content = content.replace('\\\n', '')
    for line in content.split('\n'):
        # Filter out function-like macros early: "#define FOO(x) ..." where
        # the open paren is glued to the name (no whitespace between name
        # and paren). A space/tab before the paren means this is a normal
        # define whose value happens to be parenthesised (GENMASK args,
        # ``(0xf << 20)`` shift expressions, etc.) — keep those.
        if re.match(r'^\s*#\s*define\s+\w+\(', line):
            continue
        m = _DEFINE_RE.match(line)
        if not m:
            continue
        name, expr = m.group(1), m.group(2)
        val = _eval_simple_expr(expr)
        if val is not None:
            defs[name] = val
    return defs


def _lowest_set_bit(value):
    return (value & -value).bit_length() - 1 if value else 0


def _prefix_rank(prefix):
    """Lower rank → higher priority."""
    for i, p in enumerate(_PREFIX_PRIORITY):
        if p and prefix.startswith(p):
            return i
    return len(_PREFIX_PRIORITY)


def build_field_tables(defines):
    """Turn a flat ``{name: value}`` map into per-register field tables.

    Output shape::

        {
            'PWR_STATE': [
                ('IS_PWR_FSM', shift, mask, {0xa: 'IS_RDY', ...}),
                ('PS_PWR_FSM', shift, mask, {0xf: 'PS_PWR_UP', ...}),
                ('HH_STATUS', shift, mask, {}),
            ],
            ...
        }

    Pairing heuristic:

      * A name matching ``<prefix>_<REG>_<FIELD>_SHIFT`` and one matching
        ``<prefix>_<REG>_<FIELD>_MASK`` with the same ``<prefix>_<REG>_<FIELD>``
        become a field.
      * A name matching ``<prefix>_<REG>_<FIELD>_MASK`` without a SHIFT
        counterpart (typical for ``GENMASK(hi, lo)``) becomes a field with
        shift derived from the lowest set bit of the mask.
      * Any other name matching ``<prefix>_<REG>_<FIELD>_<VALUE_NAME>`` with
        a small integer value is attached as an enum value for that field.
      * When the same (REG, FIELD) appears under multiple prefixes, the
        ``_PREFIX_PRIORITY`` winner is kept (IPU4 before IPU6).
    """
    # Pass 1: discover (reg, field) pairs from defines ending in _SHIFT
    # or _MASK. Group by prefix so IPU4-vs-IPU6 collisions can be resolved.
    # shift_mask[(reg, field)][prefix] = {'SHIFT': int, 'MASK': int}
    shift_mask = defaultdict(lambda: defaultdict(dict))

    for reg in _DECODABLE_REGS:
        needle = f'_{reg}_'
        for name, value in defines.items():
            idx = name.find(needle)
            if idx < 0:
                continue
            prefix = name[:idx + 1]  # include trailing underscore
            rest = name[idx + len(needle):]
            if rest.endswith('_SHIFT'):
                field = rest[:-len('_SHIFT')]
                if field:
                    shift_mask[(reg, field)][prefix]['SHIFT'] = value
            elif rest.endswith('_MASK'):
                field = rest[:-len('_MASK')]
                if field:
                    shift_mask[(reg, field)][prefix]['MASK'] = value

    # Pick a single prefix per (reg, field) — prefer the highest-priority
    # one that actually defines a MASK (shift-only entries decode nothing).
    winners = {}
    for (reg, field), per_prefix in shift_mask.items():
        candidates = [p for p, parts in per_prefix.items() if 'MASK' in parts]
        if not candidates:
            continue
        winners[(reg, field)] = min(candidates, key=_prefix_rank)

    # If an IPU4-specific define exists for any field of a register, that
    # register's layout is authoritative for IPU4 silicon — drop fields
    # whose winner is IPU6-only. Without this, IPU6 bit assignments that
    # have no IPU4 counterpart (e.g. ``IPU6_BUTTRESS_PWR_STATE_IS_PWR``
    # at bits 4:3) would be shown in IPU4 traces where those bits mean
    # something else entirely.
    regs_with_ipu4 = {
        reg for (reg, _field), prefix in winners.items()
        if prefix.startswith('IPU4_')
    }
    winners = {
        (reg, field): prefix
        for (reg, field), prefix in winners.items()
        if reg not in regs_with_ipu4 or prefix.startswith('IPU4_')
    }

    # Pass 2: attach enum-style value constants to their owning field.
    # A define named ``<prefix>_<REG>_<FIELD>_<VALUE_NAME>`` whose value
    # is a small integer is an enum label for the field iff we picked up
    # ``<prefix>_<REG>_<FIELD>_{SHIFT,MASK}`` already. Merge enums from
    # every prefix we saw for that field so the IPU6 header's richer
    # state-name list still gets used even when the IPU4 prefix wins
    # the shift/mask lottery.
    enums_by_rf = defaultdict(dict)
    for reg in _DECODABLE_REGS:
        needle = f'_{reg}_'
        for name, value in defines.items():
            idx = name.find(needle)
            if idx < 0:
                continue
            rest = name[idx + len(needle):]
            if rest.endswith('_SHIFT') or rest.endswith('_MASK') or not rest:
                continue
            if not (0 <= value < 0x100):
                continue
            # Find the longest known field name that is a prefix of rest.
            field_candidates = [
                f for (r, f) in shift_mask if r == reg and rest.startswith(f + '_')
            ]
            if not field_candidates:
                continue
            field = max(field_candidates, key=len)
            tail = rest[len(field) + 1:]
            enums_by_rf[(reg, field)].setdefault(value, tail)

    tables = defaultdict(list)
    for (reg, field), best_prefix in winners.items():
        parts = shift_mask[(reg, field)][best_prefix]
        mask = parts['MASK']
        shift = parts.get('SHIFT', _lowest_set_bit(mask))
        enum_map = enums_by_rf.get((reg, field), {})
        tables[reg].append((field, shift, mask, enum_map))

    # Stable ordering: by shift ascending so output reads LSB-first.
    for reg in tables:
        tables[reg].sort(key=lambda f: f[1])
    return dict(tables)


def _load_field_tables():
    header_dir = pathlib.Path(__file__).resolve().parent / 'kernel' / 'ipu4'
    env_override = os.environ.get('IPU4_HEADER_DIR')
    if env_override:
        header_dir = pathlib.Path(env_override)
    defines = {}
    for fname in HEADER_FILES:
        defines.update(parse_header(header_dir / fname))
    return build_field_tables(defines)


FIELD_TABLES = _load_field_tables()


def decode_bitfields(reg_name, value):
    """Return a human-readable bitfield breakdown of ``value`` for ``reg_name``.

    Returns an empty string if the register has no decoder or if the driver
    headers were unavailable at import time.
    """
    fields = FIELD_TABLES.get(reg_name)
    if not fields:
        return ''
    parts = []
    for field, shift, mask, enums in fields:
        field_val = (value & mask) >> shift
        if field_val == 0:
            # Skip zero fields to keep the default output tight; a fully-zero
            # register still shows the raw value. Callers that want every
            # field (e.g. JSON mode) go through decode_bitfields_dict() below.
            continue
        if field_val in enums:
            parts.append(f'{field}=0x{field_val:x}({enums[field_val]})')
        else:
            parts.append(f'{field}=0x{field_val:x}')
    return ' '.join(parts)


def decode_bitfields_dict(reg_name, value):
    """Same extraction as ``decode_bitfields`` but returning a structured
    ``{field: {'value': int, 'enum': str|None}}`` for JSON output."""
    fields = FIELD_TABLES.get(reg_name)
    if not fields:
        return {}
    out = {}
    for field, shift, mask, enums in fields:
        field_val = (value & mask) >> shift
        out[field] = {
            'value': field_val,
            'enum': enums.get(field_val),
        }
    return out


# ---------------------------------------------------------------------------
# Address lookup + per-line processing.
# ---------------------------------------------------------------------------

LookupResult = namedtuple(
    'LookupResult',
    ['region', 'reg_name', 'offset', 'display'],
)


def reg_address_translate(offset, named_regs):
    hex_value = f"0x{offset:03x}"
    if offset in named_regs:
        return f"{named_regs[offset]}({hex_value})"
    return hex_value


def lookup_address(address):
    """Return detailed lookup info for ``address`` or None if no region
    matches (the post-header mmiotrace lines for unrelated devices)."""
    for region in NAMED_REGIONS:
        if address >= region.begin:
            offset = address - region.begin
            if offset > region.max_offset:
                print(
                    f'Warning: offset seems too big: 0x{offset:x} > '
                    f'0x{region.max_offset:x} (region "{region.name}" '
                    f'full address 0x{address:x})',
                    file=sys.stderr,
                )
            named_regs = region.named_regs if isinstance(region.named_regs, dict) else {}
            reg_name = named_regs.get(offset)
            display = f"{region.name} {reg_address_translate(offset, region.named_regs)}"
            return LookupResult(
                region=region.name,
                reg_name=reg_name,
                offset=offset,
                display=display,
            )
    return None


def address_translate(address):
    """String-only shim for the legacy call sites (tests, external users)."""
    res = lookup_address(address)
    return res.display if res else None


def _parse_rw(line):
    """Parse a trace W/R line into a structured record, or None."""
    parts = line.split(' ')
    if len(parts) < 6 or parts[0] not in ('R', 'W'):
        return None
    try:
        timestamp = float(parts[2])
        address = int(parts[4], 16)
        value = int(parts[5], 16)
    except ValueError:
        return None
    lookup = lookup_address(address)
    region = lookup.region if lookup else None
    reg_name = lookup.reg_name if lookup else None
    offset = lookup.offset if lookup else None
    return {
        'op': parts[0],
        'timestamp': timestamp,
        'address': address,
        'region': region,
        'offset': offset,
        'reg_name': reg_name,
        'value': value,
    }


def process(line):
    elems = line.split(' ')
    if elems[0] in ['W', 'R']:
        elems[2] = "(timestamp)"
        address = int(elems[4], base=16)
        lookup = lookup_address(address)
        elems[4] = lookup.display if lookup else f'0x{address:x}'
        del elems[6:]
        del elems[3]
        # Annotate the value with decoded bitfields when we have a name.
        if lookup and lookup.reg_name:
            try:
                raw_value = int(elems[-1], 16)
            except ValueError:
                raw_value = None
            if raw_value is not None:
                decoded = decode_bitfields(lookup.reg_name, raw_value)
                if decoded:
                    elems[-1] = f'{elems[-1]} [{decoded}]'
    elif elems[0] in ['MARK', 'MAP']:
        elems[1] = "(timestamp)"
        if elems[2] == 'In':
            elems[4] = elems[4].replace('ipu6_', 'ipuX_')
            elems[4] = elems[4].replace('ipu_', 'ipuX_')
        if 0 == elems[2].find('bus_pm_runtime'):
            # The old driver registers bus drivers for each MMU, the new registers them for isys/psis
            elems[3] = elems[3].replace('intel-ipu4-mmu0', 'intel_ipu4.isys.24')
            elems[3] = elems[3].replace('intel-ipu4-mmu1', 'intel_ipu4.psys.24')
    return elems


# ---------------------------------------------------------------------------
# --summary and --json modes.
# ---------------------------------------------------------------------------


def summarise(lines):
    """Walk an iterable of trace lines and build per-register counters.

    Returns a list of ``dict`` rows sorted by total access count descending,
    suitable for either table-printing or asserting against in tests.
    """
    counts = Counter()
    values_seen = defaultdict(set)
    names = {}
    for line in lines:
        rec = _parse_rw(line.strip())
        if rec is None or rec['region'] is None:
            continue
        key = (rec['region'], rec['offset'])
        counts[(key, rec['op'])] += 1
        values_seen[key].add(rec['value'])
        if rec['reg_name']:
            names[key] = rec['reg_name']

    rows = []
    all_keys = {k for (k, _op) in counts}
    for key in all_keys:
        region, offset = key
        rows.append({
            'region': region,
            'offset': offset,
            'name': names.get(key, ''),
            'reads': counts[(key, 'R')],
            'writes': counts[(key, 'W')],
            'distinct_values': len(values_seen[key]),
        })
    rows.sort(key=lambda r: (r['reads'] + r['writes'], r['reads']), reverse=True)
    return rows


def format_summary(rows):
    header = f"{'region':<14} {'offset':>8}  {'name':<28} {'R':>5} {'W':>5} {'distinct':>8}"
    out = [header, '-' * len(header)]
    for r in rows:
        out.append(
            f"{r['region']:<14} {r['offset']:>#8x}  {r['name']:<28} "
            f"{r['reads']:>5} {r['writes']:>5} {r['distinct_values']:>8}"
        )
    return '\n'.join(out)


def record_to_json(rec):
    """Enrich a raw W/R record with decoded bitfields for JSON output."""
    out = dict(rec)
    out['decoded'] = (
        decode_bitfields_dict(rec['reg_name'], rec['value'])
        if rec['reg_name'] else {}
    )
    return out


# ---------------------------------------------------------------------------
# CLI entry point.
# ---------------------------------------------------------------------------


def _run_default(lines, begin_pattern, end_pattern, out):
    began = begin_pattern is None
    count = 0
    for line in lines:
        if (begin_pattern is None) or began or begin_pattern.search(line):
            if not began:
                print(f"Begin found: {line.strip()}", file=sys.stderr)
            began = True
            count += 1
            elems = process(line.strip())
            print(' '.join(elems), file=out)
            if end_pattern and end_pattern.search(line):
                print(f"End found({count} lines): {line.strip()}", file=sys.stderr)
                break


def _run_json(lines, begin_pattern, end_pattern, out):
    began = begin_pattern is None
    for line in lines:
        if (begin_pattern is None) or began or begin_pattern.search(line):
            began = True
            rec = _parse_rw(line.strip())
            if rec is not None:
                print(json.dumps(record_to_json(rec)), file=out)
            if end_pattern and end_pattern.search(line):
                break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('tracefile')
    parser.add_argument('--begin', help="Regex for first line to process", default=None)
    parser.add_argument('--end', help="Regex for last line to process", default=None)
    parser.add_argument('--summary', action='store_true',
                        help="Emit a per-register histogram instead of a "
                             "line-by-line dump.")
    parser.add_argument('--json', action='store_true',
                        help="Emit JSON Lines (one object per W/R access) "
                             "instead of the human-readable dump.")
    args = parser.parse_args()

    try:
        end_pattern = re.compile(args.end) if args.end else None
        begin_pattern = re.compile(args.begin) if args.begin else None

        with open(args.tracefile, 'r') as f:
            lines = list(f)

        if args.summary:
            # Summary ignores --begin/--end; it's meant to characterise the
            # whole capture. Callers who want a slice pre-filter externally.
            print(format_summary(summarise(lines)))
        elif args.json:
            _run_json(lines, begin_pattern, end_pattern, sys.stdout)
        else:
            _run_default(lines, begin_pattern, end_pattern, sys.stdout)
    except socket.error as e:
        if e.errno != errno.EPIPE:
            raise
        # EPIPE happens e.g. when downstream pipe stop (e.g. piping into `head`)
        exit(0)

if __name__ == "__main__":
    main()
