/*
 * Intel IPU4 CSI2 receiver — see ipu4-csi2.h for the interface.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "migration/vmstate.h"

#include "ipu4-csi2.h"
#include "ipu4-irq-trio.h"

/* Per-port register offsets within port_base..port_base+0x800. Port 0
 * lives at CSI2P0_BASE. */
#define CSI2_PORT_RX_ENABLE          0x000
#define CSI2_PORT_RX_NOF_LANES       0x004
#define CSI2_PORT_RX_CONFIG          0x008
#define CSI2_PORT_DLY_TERMEN_C       0x02c
#define CSI2_PORT_DLY_SETTLE_C       0x030
#define CSI2_PORT_DLY_TERMEN_D0      0x034
#define CSI2_PORT_DLY_SETTLE_D0      0x038
#define CSI2_PORT_PART_IRQ_BASE      0x400
#define CSI2_PORT_RX_IRQ_BASE        0x500
#define CSI2_PORT_S2M_IRQ_BASE       0x600

#define CSI2_PORT_SIZE               0x1000

/* CSI2 ports 1..5 live at 0x65000..0x6d800; we don't model them
 * because data/trace.txt only exercises port 0. */
#define CSI2_PORTS_1_5_RANGE_BEGIN   0x65000
#define CSI2_PORTS_1_5_RANGE_END     0x6d800

const VMStateDescription vmstate_ipu4_csi2_port = {
    .name = "ipu4/csi2/port",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(rx_enable, Ipu4Csi2Port),
        VMSTATE_UINT32(rx_nof_lanes, Ipu4Csi2Port),
        VMSTATE_UINT32(rx_config, Ipu4Csi2Port),
        VMSTATE_UINT32(dly_termen_c, Ipu4Csi2Port),
        VMSTATE_UINT32(dly_settle_c, Ipu4Csi2Port),
        VMSTATE_UINT32(dly_termen_d0, Ipu4Csi2Port),
        VMSTATE_UINT32(dly_settle_d0, Ipu4Csi2Port),
        VMSTATE_STRUCT(part, Ipu4Csi2Port, 0,
                       vmstate_ipu4_irq_trio, Ipu4IrqTrio),
        VMSTATE_STRUCT(rx, Ipu4Csi2Port, 0,
                       vmstate_ipu4_irq_trio, Ipu4IrqTrio),
        VMSTATE_STRUCT(s2m, Ipu4Csi2Port, 0,
                       vmstate_ipu4_irq_trio, Ipu4IrqTrio),
        VMSTATE_END_OF_LIST()
    }
};

const VMStateDescription vmstate_ipu4_csi2 = {
    .name = "ipu4/csi2",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(port0, Ipu4Csi2, 0,
                       vmstate_ipu4_csi2_port, Ipu4Csi2Port),
        VMSTATE_END_OF_LIST()
    }
};

static void ipu4_csi2_port_reset(Ipu4Csi2Port *p)
{
    p->rx_enable = 0;
    p->rx_nof_lanes = 0;
    p->rx_config = 0;
    p->dly_termen_c = 0;
    p->dly_settle_c = 0;
    p->dly_termen_d0 = 0;
    p->dly_settle_d0 = 0;
    ipu4_irq_trio_reset(&p->part);
    ipu4_irq_trio_reset(&p->rx);
    ipu4_irq_trio_reset(&p->s2m);
}

void ipu4_csi2_reset(Ipu4Csi2 *c)
{
    ipu4_csi2_port_reset(&c->port0);
}

/* Decode an address inside a CSI2 port. *trio_out is set to NULL for
 * non-trio registers; otherwise to the trio and `*trio_off` to the
 * offset within it. Returns the per-port offset, or -1 if `addr` is
 * outside the port. */
static bool csi2_port_decode(const Ipu4Csi2Port *p, hwaddr off,
                             const Ipu4IrqTrio **trio_out,
                             hwaddr *trio_off)
{
    if (off >= CSI2_PORT_PART_IRQ_BASE &&
        off < CSI2_PORT_PART_IRQ_BASE + IPU4_IRQ_TRIO_SIZE) {
        *trio_out = &p->part;
        *trio_off = off - CSI2_PORT_PART_IRQ_BASE;
        return true;
    }
    if (off >= CSI2_PORT_RX_IRQ_BASE &&
        off < CSI2_PORT_RX_IRQ_BASE + IPU4_IRQ_TRIO_SIZE) {
        *trio_out = &p->rx;
        *trio_off = off - CSI2_PORT_RX_IRQ_BASE;
        return true;
    }
    if (off >= CSI2_PORT_S2M_IRQ_BASE &&
        off < CSI2_PORT_S2M_IRQ_BASE + IPU4_IRQ_TRIO_SIZE) {
        *trio_out = &p->s2m;
        *trio_off = off - CSI2_PORT_S2M_IRQ_BASE;
        return true;
    }
    *trio_out = NULL;
    return false;
}

static bool csi2_port_read(const Ipu4Csi2Port *p, hwaddr off, uint64_t *val)
{
    const Ipu4IrqTrio *trio;
    hwaddr trio_off;

    switch (off) {
    case CSI2_PORT_RX_ENABLE:     *val = p->rx_enable;     return true;
    case CSI2_PORT_RX_NOF_LANES:  *val = p->rx_nof_lanes;  return true;
    case CSI2_PORT_RX_CONFIG:     *val = p->rx_config;     return true;
    case CSI2_PORT_DLY_TERMEN_C:  *val = p->dly_termen_c;  return true;
    case CSI2_PORT_DLY_SETTLE_C:  *val = p->dly_settle_c;  return true;
    case CSI2_PORT_DLY_TERMEN_D0: *val = p->dly_termen_d0; return true;
    case CSI2_PORT_DLY_SETTLE_D0: *val = p->dly_settle_d0; return true;
    }

    if (csi2_port_decode(p, off, &trio, &trio_off)) {
        switch (trio_off) {
        case IPU4_IRQ_TRIO_EDGE:    *val = trio->edge;   return true;
        case IPU4_IRQ_TRIO_MASK:    *val = trio->mask;   return true;
        case IPU4_IRQ_TRIO_STATUS:
            /* CSI2 STATUS reads are gated by ENABLE (matches pre-
             * refactor behaviour for all three trios). */
            *val = trio->status & trio->enable;
            return true;
        case IPU4_IRQ_TRIO_ENABLE:  *val = trio->enable; return true;
        case IPU4_IRQ_TRIO_LEVEL:   *val = trio->level;  return true;
        }
    }
    return false;
}

static bool csi2_port_write(Ipu4Csi2Port *p, hwaddr off, uint64_t val)
{
    Ipu4IrqTrio *trio;
    hwaddr trio_off;

    switch (off) {
    case CSI2_PORT_RX_ENABLE:     p->rx_enable     = val; return true;
    case CSI2_PORT_RX_NOF_LANES:  p->rx_nof_lanes  = val; return true;
    case CSI2_PORT_RX_CONFIG:     p->rx_config     = val; return true;
    case CSI2_PORT_DLY_TERMEN_C:  p->dly_termen_c  = val; return true;
    case CSI2_PORT_DLY_SETTLE_C:  p->dly_settle_c  = val; return true;
    case CSI2_PORT_DLY_TERMEN_D0: p->dly_termen_d0 = val; return true;
    case CSI2_PORT_DLY_SETTLE_D0: p->dly_settle_d0 = val; return true;
    }

    /* csi2_port_decode returns a const pointer; cast away const
     * because writes legitimately mutate the same per-trio storage. */
    if (csi2_port_decode(p, off, (const Ipu4IrqTrio **)&trio, &trio_off)) {
        return ipu4_irq_trio_write(trio, trio_off, val);
    }
    return false;
}

bool ipu4_csi2_mmio_read(const Ipu4Csi2 *c, hwaddr addr, uint64_t *val)
{
    if (addr >= CSI2P0_BASE && addr < CSI2P0_BASE + CSI2_PORT_SIZE) {
        return csi2_port_read(&c->port0, addr - CSI2P0_BASE, val);
    }
    if (addr >= CSI2_PORTS_1_5_RANGE_BEGIN &&
        addr < CSI2_PORTS_1_5_RANGE_END) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: CSI2 port>=1 read +0x%06" HWADDR_PRIx
                      " — only port 0 is modelled; data/trace.txt "
                      "only exercised port 0, so this means a new "
                      "driver path hit an unmodelled port.\n", addr);
        *val = 0;
        return true;
    }
    return false;
}

bool ipu4_csi2_mmio_write(Ipu4Csi2 *c, hwaddr addr, uint64_t val)
{
    if (addr >= CSI2P0_BASE && addr < CSI2P0_BASE + CSI2_PORT_SIZE) {
        return csi2_port_write(&c->port0, addr - CSI2P0_BASE, val);
    }
    if (addr >= CSI2_PORTS_1_5_RANGE_BEGIN &&
        addr < CSI2_PORTS_1_5_RANGE_END) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: CSI2 port>=1 write +0x%06" HWADDR_PRIx
                      " val=0x%" PRIx64 " — only port 0 is modelled; "
                      "data/trace.txt only exercised port 0, so this "
                      "means a new driver path hit an unmodelled port.\n",
                      addr, val);
        return true;
    }
    return false;
}
