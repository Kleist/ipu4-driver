#!/usr/bin/env python3
"""Register writes Ruslan Bay's IPU4P driver makes on an IPU4P (8086:8a19).

There is no IPU4P silicon trace, so the ipu4p variant's write-comparing
ladder tests (probe-mmu, isys-irq) check the guest against what
patches/ipu6-ipu4p/ writes, computed here from the driver's own tables.
Every value names the place it comes from; update it when the series
changes. Same interface as upstream_writes.py (the ipu6 variant's).

Usage:
  ipu4p_writes.py SET            the expected writes, one QEMU
                                 memory_region_ops_write log line each
  ipu4p_writes.py --ranges SET   the BAR ranges they cover

SET is probe-mmu, isys-irq or stream-on.
"""

import sys

# ipu6-mmu-hw.c: REG_INFO; ipu6-platform-regs.h: IPU4P_MMU_L1/L2_STREAM_
# ID_REG_OFFSET, IPU4P_INFO_REQUEST_DESTINATION_PRIMARY.
REG_INFO = 0x08
L1_OFF = 0x0c
L2_OFF = 0x4c
PRIMARY = 1 << 4

# ipu6-mmu-hw.c ipu4p_{isys,psys}_mmu_hwdata: (base, info_bits,
# l1_block_sz, l2_block_sz), base = IPU4P_{ISYS,PSYS}_OFFSET +
# IPU4P_{ISYS,PSYS}_IOMMU*_OFFSET. __ipu6_mmu_hw_init() writes, per MMU
# in table order, the page-table address (+0x04, not compared: it is an
# allocation), info_bits, then the cumulative block start address of
# each L1 stream and each L2 stream.
ISYS_MMUS = [
    (0x1e0000, PRIMARY, [], []),
    (0x1e0100, 0, [5, 16, 6, 6, 6, 6, 6, 8, 0, 0, 0, 0, 0, 0, 0, 5],
     [2] * 16),
]
PSYS_MMUS = [
    (0x4b0000, PRIMARY, [], []),
    (0x4b0100, 0, [2, 5, 4, 2, 2, 10, 5, 16, 10, 5, 0, 0, 0, 0, 0, 3],
     [2] * 16),
    (0x4b0600, 0, [2, 6, 5, 16, 16, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 3],
     [2] * 16),
]


def mmu_hw_init(mmus):
    """Writes of __ipu6_mmu_hw_init() from REG_INFO on, and their ranges."""
    writes, ranges = [], []
    for base, info, l1, l2 in mmus:
        writes.append((base + REG_INFO, info))
        for off, sizes in ((L1_OFF, l1), (L2_OFF, l2)):
            addr = 0
            for j, size in enumerate(sizes):
                writes.append((base + off + 4 * j, addr))
                addr += size
        ranges.append((base + REG_INFO, base + 0x100))
    return writes, ranges


def isys_resume():
    """Writes of isys_runtime_pm_resume() and the suspend after it."""
    writes, ranges = mmu_hw_init(ISYS_MMUS)
    # ipu6_buttress_start_tsc_sync(): BUTTRESS_REG_FABRIC_CMD =
    # BUTTRESS_FABRIC_CMD_START_TSC_SYNC, once (the model's handshake
    # completes at the first poll).
    writes.append((0x88, 0x1))
    ranges.append((0x88, 0x8c))
    # ipu4p_isys_irq_setup(), ipu4p-isys-csi2-regs.h, ISYS base 0x100000:
    # per IRQ block, the mask into EDGE, MASK, ENABLE and LEVEL_NOT_PULSE
    # (+0x00, +0x04, +0x10, +0x14), then CLEAR (+0x0c) = 0xffffffff;
    # then UNISPART SW_IRQ_REG and SW_IRQ_MUX_REG = 0. The suspend
    # writes no LTR/DID on IPU4P.
    unispart = 1 << 22 | 1 << 3 | 1 << 4  # IRQ_SW | CSI2(0) | CSI2(1)
    blocks = [
        (0x17c000, unispart),  # UNISPART
        (0x1b0c00, 0),         # ISA_ACC
        (0x1be200, 0),         # A
        (0x166d00, 1 << 3),    # SIP0, SIP0_CSI2_IRQ
        (0x16ed00, 0x7f),      # SIP1, SIP1_CSI2_IRQ_MASK
    ]
    for base, mask in blocks:
        writes += [(base + off, mask) for off in (0x00, 0x04, 0x10, 0x14)]
        writes.append((base + 0x0c, 0xffffffff))
        ranges.append((base, base + 0x18))
    writes += [(0x17c414, 0), (0x17c418, 0)]
    ranges.append((0x17c414, 0x17c41c))
    return writes, ranges


def stream_on():
    """Receiver writes of ipu4p_isys_phy_set_power() on port 0.

    CSI-2 port 0 at ISYS + ipu4p_port_offsets[0] (0x64300), its IRQ
    controls at ISYS + IPU4P_ISYS_CSI_IRQ_CTRL{,0}_BASE(0) (0x66300,
    0x66400). Left out: the timing and lane count (they follow the
    sensor), the IRQ CLEAR registers (the ISR writes them per interrupt)
    and the LTR/DID writes (0x68; set_iwake_ltrdid() derives them at run
    time).
    """
    rx = 0x164300
    ctrl, ctrl0 = 0x166300, 0x166400
    active, errors = 1, 0xffff  # CSI_IRQ_ACTIVE, CSI_ERROR_MASK
    # Stream on: RX_CONFIG |= DISABLE_BYTE_CLK_GATING | RELEASE_LP11
    # (read back as 0), RX_ENABLE = 0, ipu4p_isys_csi_irq_enable() (EDGE,
    # LEVEL_NOT_PULSE, MASK, ENABLE per block), then RX_ENABLE = 1.
    writes = [(rx + 0x08, 0x3), (rx, 0)]
    for base, mask in ((ctrl, active), (ctrl0, errors)):
        writes += [(base, mask), (base + 0x14, 0), (base + 0x04, mask),
                   (base + 0x10, mask)]
    writes.append((rx, 1))
    # Stream off: RX_CONFIG cleared, RX_ENABLE = 0,
    # ipu4p_isys_csi_irq_disable() (MASK, ENABLE per block).
    writes += [(rx + 0x08, 0), (rx, 0)]
    for base in (ctrl, ctrl0):
        writes += [(base + 0x04, 0), (base + 0x10, 0)]
    ranges = [(rx, rx + 0x04), (rx + 0x08, rx + 0x0c)]
    for base in (ctrl, ctrl0):
        ranges += [(base, base + 0x0c), (base + 0x10, base + 0x18)]
    return writes, ranges


SETS = {
    # Probe: only the PSYS MMUs are initialised; the ISYS ones wait for
    # the ISYS resume.
    "probe-mmu": lambda: mmu_hw_init(PSYS_MMUS),
    "isys-irq": isys_resume,
    "stream-on": stream_on,
}


def main(argv):
    ranges_only = argv[:1] == ["--ranges"]
    if ranges_only:
        argv = argv[1:]
    if len(argv) != 1 or argv[0] not in SETS:
        print(__doc__, file=sys.stderr)
        return 2
    writes, ranges = SETS[argv[0]]()
    if ranges_only:
        print(" ".join(f"0x{s:x}-0x{e:x}" for s, e in ranges))
        return 0
    for addr, val in writes:
        print(f"memory_region_ops_write addr 0x{addr:x} value 0x{val:x} "
              "size 4 name 'ipu4-bar0'")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
