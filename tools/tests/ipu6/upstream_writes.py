#!/usr/bin/env python3
"""Register writes the upstream IPU6 driver makes on an IPU6 (8086:9a19).

There is no IPU6 silicon trace in this repo, so the ipu6 variant's
write-comparing ladder tests (probe-mmu, isys-irq) check the guest
against what mainline drivers/media/pci/intel/ipu6/ (tools/ipu6/
pin.upstream) writes, computed here from the driver's own tables. Every
value names the place it comes from; when the pin moves and the driver
changes, update it here.

Usage:
  upstream_writes.py SET            the expected writes, one QEMU
                                    memory_region_ops_write log line each
                                    (tools/trace/check_writes.py --expected)
  upstream_writes.py --ranges SET   the BAR ranges they cover

SET is probe-mmu, isys-irq, fw-open, stream-on or csi2-sof-irq. stream-on reads the PHY
tables from the upstream source in $IPU4_IPU6_LINUX_DIR (default
tools/linux-ipu6-upstream).
"""

import os
import re
import sys

# ipu6-mmu.c: REG_INFO; ipu6-platform-regs.h: IPU6_MMU_L1/L2_STREAM_ID_
# REG_OFFSET, IPU6_PSYS_MMU1W_L2_STREAM_ID_REG_OFFSET.
REG_INFO = 0x08
L1_OFF = 0x0c
L2_OFF = 0x4c
PSYS_MMU1W_L2_OFF = 0x8c
IOSF = 1 << 9  # IPU6_INFO_REQUEST_DESTINATION_IOSF

# ipu6.c ipu6_isys_ipdata / ipu6_psys_ipdata .hw_variant.mmu_hw:
# (base, info_bits, l1_block_sz, l2_block_sz, l2_stream_id_reg_offset).
# IPU6_UNIFIED_OFFSET is 0. ipu6_mmu_hw_init() writes, per MMU in table
# order, the page-table address (+0x04, not compared: it is an
# allocation), info_bits, then the cumulative block start address of
# each L1 stream and each L2 stream.
ISYS_MMUS = [
    (0x2e0000, IOSF, [3, 8] + [2] * 6 + [1] * 8, [2] * 16, L2_OFF),
    (0x2e0500, 0, [2] * 13 + [1, 1, 4], [2] * 16, L2_OFF),
    (0x2e0a00, 0, [], [], L2_OFF),  # IOMMUI
]
PSYS_MMUS = [
    (0x1b0000, IOSF, [2] * 16, [2] * 16, L2_OFF),
    (0x1b0700, 0,
     [1] + [2] * 14 + [10, 5, 4, 14, 6, 4, 14, 6, 4, 8, 4, 2, 1, 1, 1, 1, 14],
     [2] * 32, PSYS_MMU1W_L2_OFF),
    (0x1b0e00, 0, [1, 4, 4, 4, 4, 16, 8, 4, 32, 16, 16, 2, 2, 2, 1, 12],
     [2] * 16, L2_OFF),
    (0x1b1500, 0, [], [], L2_OFF),  # IOMMUI
]


def mmu_hw_init(mmus):
    """Writes of ipu6_mmu_hw_init() from REG_INFO on, and their ranges."""
    writes, ranges = [], []
    for base, info, l1, l2, l2_off in mmus:
        writes.append((base + REG_INFO, info))
        for off, sizes in ((L1_OFF, l1), (l2_off, l2)):
            addr = 0
            for j, size in enumerate(sizes):
                writes.append((base + off + 4 * j, addr))
                addr += size
        # Everything the MMU block has after the page-table address.
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
    # isys_setup_hw(): 8 CSI-2 ports on 9a19 (IPU6_TGL_ISYS_CSI2_NPORTS),
    # IPU6_ISYS_UNISPART_IRQ_CSI2(port) = 0x3 << (port * 4), into
    # IPU6_REG_ISYS_CSI_TOP_CTRL0_IRQ_{EDGE,LEVEL_NOT_PULSE,MASK,ENABLE},
    # then CLEAR = GENMASK(19, 0).
    csi = 0
    for port in range(8):
        csi |= 0x3 << (port * 4)
    writes += [(0x238200, csi), (0x238214, csi), (0x238204, csi),
               (0x238210, csi), (0x23820c, 0xfffff)]
    ranges.append((0x238200, 0x238218))
    # ISYS_UNISPART_IRQS = IRQ_SW (BIT 22) | CSI0 (BIT 2) | CSI1 (BIT 3)
    # into IPU6_REG_ISYS_UNISPART_IRQ_{EDGE,LEVEL_NOT_PULSE}, CLEAR =
    # GENMASK(28, 0), {MASK,ENABLE}; SW_IRQ_REG and SW_IRQ_MUX_REG = 0;
    # IPU6_REG_ISYS_CDC_THRESHOLD(i) = cdc_fifo_threshold {6, 8, 2}.
    unispart = 1 << 22 | 1 << 2 | 1 << 3
    writes += [(0x27c000, unispart), (0x27c014, unispart),
               (0x27c00c, 0x1fffffff), (0x27c004, unispart),
               (0x27c010, unispart), (0x27c414, 0), (0x27c418, 0),
               (0x27c400, 6), (0x27c404, 8), (0x27c408, 2)]
    ranges.append((0x27c000, 0x27c420))
    # set_iwake_ltrdid() into IPU6_BUTTRESS_FABIC_CONTROL (0x68), union
    # fabric_ctrl: ltr_val [9:0], ltr_scale [12:10], did_val [25:16],
    # did_scale [28:26], the rest read back as 0. Resume, LTR_ISYS_ON:
    # LTR_DID_PKGC_2R (20), LTR_SCALE_1024NS (2), DID_SCALE_1US (2).
    # The suspend that follows, LTR_ISYS_OFF: LTR_DID_VAL_MAX (1023),
    # LTR_SCALE_DEFAULT (5), DID_SCALE_1US.
    def fabric_ctrl(ltr, ltr_scale, did, did_scale):
        return ltr | ltr_scale << 10 | did << 16 | did_scale << 26
    writes += [(0x68, fabric_ctrl(20, 2, 20, 2)),
               (0x68, fabric_ctrl(1023, 5, 1023, 2))]
    ranges.append((0x68, 0x6c))
    return writes, ranges


def fw_open():
    """Writes from the ISYS firmware open to the STREAM_OPEN_DONE handling.

    Compared as a prefix: the stream start follows.
    """
    spc = 0x210000           # IPU6_ISYS_SPC_OFFSET, SPC_STATUS_CTRL at +0
    dmem = 0x200000          # IPU6_ISYS_DMEM_OFFSET, 4-byte registers
    boot = 0x4000            # BUTTRESS_FW_BOOT_PARAMS_0, ISYS offset 0
    ready, start, run = 1 << 5, 1 << 1, 1 << 3
    icache_invalidate = 1 << 12
    writes = [
        # ipu6_configure_spc(): SPC_STATUS_CTRL |= ICACHE_INVALIDATE on
        # an idle SPC (READY); secure mode: DMEM[0] =
        # IPU6_PKG_DIR_IMR_OFFSET.
        (spc, ready | icache_invalidate),
        (dmem + 0x0, 0x40),
        # ipu6_fw_com_open(): TUNIT_CFG_DWR_REG = TUNIT_MAGIC_PATTERN,
        # SYSCOM_COMMAND_REG = SYSCOM_COMMAND_UNINIT, boot param
        # SYSCOM_STATE_ID = SYSCOM_STATE_UNINIT (SYSCOM_CONFIG_ID, an
        # IOVA, is not compared), start_sp(): START | RUN |
        # ICACHE_INVALIDATE (no prefetch on 9a19).
        (dmem + 0x4, 0x5a5a5a5a),
        (dmem + 0x8, 0x57a7f000),
        (boot + 0x4, 0x57a7e000),
        (spc, start | run | icache_invalidate),
    ]
    # ipu6_fw_com_prepare() hands out cursor registers from
    # SYSCOM_QPR_BASE_REG (4), a write/read pair per queue: input queues
    # proxy, dev, then one per stream (IPU6_ISYS_NUM_STREAMS = 12), then
    # output queues proxy, msg.
    proxy_send_wr = 4
    stream0_send_wr = 4 + 2 * 2
    proxy_recv_rd = 4 + 2 * (2 + 12) + 1
    msg_recv_rd = proxy_recv_rd + 2
    # Stream on: three iwake proxy writes (GDA_ENABLE_IWAKE_INDEX,
    # GDA_IWAKE_THRESHOLD_INDEX, GDA_IRQ_CRITICAL_THRESHOLD_INDEX), each
    # one token sent and its answer consumed.
    for n in range(1, 4):
        writes += [(dmem + 4 * proxy_send_wr, n),
                   (dmem + 4 * proxy_recv_rd, n)]
    # STREAM_OPEN on stream 0's queue, then STREAM_OPEN_DONE consumed.
    writes += [(dmem + 4 * stream0_send_wr, 1),
               (dmem + 4 * msg_recv_rd, 1)]
    ranges = [(boot + 0x4, boot + 0x8), (dmem, dmem + 0x100),
              (spc, spc + 0x4)]
    return writes, ranges


def phy_table(name):
    """The {reg, val} entries of a phy_reg table in ipu6-isys-mcd-phy.c."""
    root = os.path.join(os.path.dirname(__file__), "..", "..", "..")
    linux = os.environ.get("IPU4_IPU6_LINUX_DIR",
                           os.path.join(root, "tools", "linux-ipu6-upstream"))
    with open(os.path.join(linux, "drivers/media/pci/intel/ipu6",
                           "ipu6-isys-mcd-phy.c")) as f:
        src = f.read()
    body = re.search(r"phy_reg %s\[\] = \{(.*?)\n\};" % name, src, re.S)
    return [(int(r, 16), int(v, 16)) for r, v in
            re.findall(r"\{(0x[0-9a-f]+), (0x[0-9a-f]+)\}", body.group(1))
            if int(r, 16)]


def stream_on():
    """Writes of one stream on and off on CSI-2 port 0, 2 lanes."""
    port = 0x220000          # CSI_REG_PORT_BASE(0)
    irq, sync = port + 0x80, port + 0xa0  # CSI_PORT_REG_BASE_IRQ_CSI(_SYNC)
    status, mask, clear, enable, lnp = 0x8, 0x4, 0xc, 0x10, 0x14
    drv = lambda i: 0x238018 + 4 * i     # CSI_REG_HUB_DRV_ACCESS_PORT
    fw = lambda i: 0x237000 + 4 * i      # .._FW_ACCESS_PORT, ofs 0x17000
    phy_ctl, phy_base = 0x238008, 0x10000  # PHY 0
    pwr_en, reset = 1 << 0, 1 << 4
    err = 0xfffff            # IPU6_CSI_RX_ERROR_IRQ_MASK
    nlanes = 2               # virt-sensor on 9a19
    # ipu6_isys_csi2_set_stream(on): port reset, port clocks of all 8
    # ports, error IRQs, sync IRQs, FE and PPI2CSI.
    writes = [(port, 1), (port, 0)]
    for i in range(8):
        writes += [(drv(i), 1), (fw(i), 1)]
    writes += [(irq + status, err), (irq + mask, err), (irq + clear, err),
               (irq + lnp, err), (irq + enable, err),
               (sync + status, 0xffffffff), (sync + mask, 0),
               (sync + clear, 0xffffffff), (sync + lnp, 0),
               (sync + enable, 0xffffffff),
               (port + 0x284, 0),           # FE_MODE
               (port + 0x288, 0),           # FE_MUX_CTRL = CSI_SENSOR_INPUT
               (port + 0x290, 0x3),         # SYNC_CNTR_SEL: LINE | FRAME
               (port + 0x204, (nlanes - 1) << 3),  # PPI_INTF lanes
               (port + 0x200, 1), (port + 0x280, 1)]
    # ipu6_isys_mcd_phy_set_power(on): PWR_EN, reset deasserted,
    # common_init_regs, the 2-lane table of PHY port 1 (driver port 0 is
    # PHY port 1), reset asserted.
    writes += [(phy_ctl, pwr_en), (phy_ctl, pwr_en)]
    writes += [(phy_base + r, v) for r, v in phy_table("common_init_regs")]
    writes += [(phy_base + r, v) for r, v in phy_table("x2_port1_config_regs")]
    writes += [(phy_ctl, pwr_en | reset)]
    # ipu6_isys_csi2_set_stream(off), PHY power off, port clocks off.
    writes += [(port + 0x280, 0), (port + 0x200, 0), (irq + enable, 0),
               (irq + clear, err), (sync + enable, 0),
               (sync + clear, 0xffffffff), (phy_ctl, 0), (fw(0), 0),
               (drv(0), 0)]
    # The IRQ handler also writes both CLEAR registers, at times that
    # depend on interrupt timing: leave them out.
    writes = [w for w in writes if w[0] not in (irq + clear, sync + clear)]
    ranges = [(port, irq + clear), (irq + clear + 4, sync + clear),
              (sync + clear + 4, port + 0x1000),
              (phy_base, phy_base + 0x8000), (fw(0), fw(8)),
              (phy_ctl, phy_ctl + 4), (drv(0), drv(8))]
    return writes, ranges


def csi2_sync_masked():
    """The CSI-2 sync (FS/FE) IRQs stay masked: their MASK is written once,
    0, at stream on; frame starts come as the firmware's FRAME_SOF."""
    sync_mask = 0x220000 + 0xa0 + 0x4
    return [(sync_mask, 0)], [(sync_mask, sync_mask + 4)]


SETS = {
    # Probe: only the PSYS MMUs are initialised (ipu6_pci_probe ->
    # ipu6_mmu_hw_init(psys)); the ISYS ones wait for the ISYS resume.
    "probe-mmu": lambda: mmu_hw_init(PSYS_MMUS),
    "isys-irq": isys_resume,
    "fw-open": fw_open,
    "stream-on": stream_on,
    "csi2-sof-irq": csi2_sync_masked,
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
