#!/usr/bin/env python3

import argparse
import errno
import socket
import sys
import re


from collections import namedtuple

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

def reg_address_translate(offset, named_regs):
    hex_value = f"0x{offset:03x}"
    if offset in named_regs:
        return f"{named_regs[offset]}({hex_value})"
    return hex_value

def address_translate(address):
    for region in NAMED_REGIONS:
        if address >= region.begin:
            offset = address-region.begin
            if offset > region.max_offset:
                print(f'Warning: offset seems too big: 0x{offset:x} > 0x{region.max_offset:x} (region "{region.name}" full address 0x{address:x})', file=sys.stderr)
            reg_addr = reg_address_translate(offset, region.named_regs)
            return f"{region.name} {reg_addr}"

def process(line):
    elems = line.split(' ')
    if elems[0] in ['W', 'R']:
        elems[2] = "(timestamp)"
        elems[4] = address_translate(int(elems[4], base=16))
        del elems[6:]
        del elems[3]
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('tracefile')
    parser.add_argument('--begin', help="Regex for first line to process", default=None)
    parser.add_argument('--end', help="Regex for last line to process", default=None)
    args = parser.parse_args()

    try:
        end_pattern = re.compile(args.end) if args.end else None
        begin_pattern = re.compile(args.begin) if args.begin else None
        began = begin_pattern is None
        lines = 0
        for line in open(args.tracefile, 'r'):
            if (begin_pattern is None) or began or begin_pattern.search(line):
                if not began:
                    print(f"Begin found: {line.strip()}", file=sys.stderr)
                began = True
                lines += 1

                elems = process(line.strip())
                outline = ' '.join(elems)
                print(outline)

                if end_pattern and end_pattern.search(line):
                    print(f"End found({lines} lines): {line.strip()}", file=sys.stderr)
                    break
    except socket.error as e:
        if e.errno != errno.EPIPE:
            raise
        # EPIPE happens e.g. when downstream pipe stop (e.g. piping into `head`)
        exit(0)

if __name__ == "__main__":
    main()

def test_strip_timestamp():
    elems = process("W 4 123 0x0 0x0 0x0 0")
    assert elems[2] == '(timestamp)'
    assert len(elems) == 5

    elems = process("R 4 123 0x0 0x0 0x0 0")
    assert elems[2] == '(timestamp)'

    elems = process("MAP 4 123 0x0 0x0 0x0 0")
    assert elems[1] == '(timestamp)'

    elems = process("MARK 4 123 0x0 0x0 0x0 0")
    assert elems[1] == '(timestamp)'

def test_offset_names():
    elems = process("R 4 1.00 1 0x90000318 0x0 0x0 0")
    assert(elems[3] == "buttress  0x318")

def test_address_translate():
    assert address_translate(0x901e0004) == "isys0 mmu 0x004"
    assert address_translate(0x901e0204) == "isys1 mmu 0x104"
    assert address_translate(0x904b0004) == "psys0 mmu 0x004"
    assert address_translate(0x904b0104) == "psys1 mmu 0x004"
    assert address_translate(0x904b0400) == "psys1 mmu 0x300"
    assert address_translate(0x90100000) == "isys spc  0x000"
    assert address_translate(0x90400000) == "psys spc  0x000"

    assert address_translate(0x90108000) == "isys dmem 0x000"
    assert address_translate(0x90408000) == "psys dmem 0x000"

def test_replace_ipu_function_names():
    elems = process("MARK 123 In function ipu_something")
    assert elems[4] == "ipuX_something"

    elems = process("MARK 123 In function ipu6_something")
    assert elems[4] == "ipuX_something"

def test_named_regions_is_sorted():
    sorted_regions = list(reversed(sorted(NAMED_REGIONS)))
    assert sorted_regions == NAMED_REGIONS
