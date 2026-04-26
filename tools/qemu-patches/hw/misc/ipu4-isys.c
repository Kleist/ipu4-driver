/*
 * Intel IPU4 ISYS subsystem — see ipu4-isys.h for the interface.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "migration/vmstate.h"

#include "ipu4-isys.h"
#include "ipu4-irq-trio.h"
#include "ipu4-fw-isys.h"

/* ISYS unispart IRQ block. The driver derives the address as
 * ISYS_BASE (0x100000) + UNISPART_OFFSET (0x7c000) = 0x17c000 from
 * BAR0 (kernel/ipu4/ipu4-platform-regs.h:49 + ISYS base), and the
 * silicon trace confirms it.
 *
 *   STATUS         read returns raw status (unmasked)
 *   CLEAR          write-1-to-clear bits of status
 *   EDGE/MASK/ENABLE/LEVEL_NOT_PULSE  latched R/W
 *   SW_IRQ         trigger register; the driver writes 0 on silicon
 *                  so the model treats it as a no-op for now.
 *   SW_IRQ_MUX     latched R/W
 */
#define IS_UNISPART_BASE                 0x17c000
#define IS_UNISPART_IRQ_TRIO_BASE        (IS_UNISPART_BASE + 0x000)
#define IS_UNISPART_SW_IRQ               (IS_UNISPART_BASE + 0x414)
#define IS_UNISPART_SW_IRQ_MUX           (IS_UNISPART_BASE + 0x418)

/* SPC status/control register lives at offset 0 of each SPC region.
 * IPU4 maps ISYS SPC at BAR+0x100000 and PSYS SPC at BAR+0x400000
 * (kernel/ipu4/ipu4-platform-regs.h:26-27). Bit map
 * (kernel/ipu4/ipu6-platform-regs.h:81-87):
 *   BIT(1)  START
 *   BIT(3)  RUN
 *   BIT(5)  READY
 *   BIT(12) ICACHE_INVALIDATE
 *   BIT(13) ICACHE_PREFETCH
 * `query_sp()` waits for `(val & (READY | START)) == READY` — i.e.
 * READY set, START cleared. To make that test pass on the first
 * read the model latches whatever the driver writes but always
 * folds in READY and folds out START on read.
 */
#define IS_SPC_STATUS_CTRL               0x100000
#define PS_SPC_STATUS_CTRL               0x400000
#define SPC_STATUS_START                 (1u << 1)
#define SPC_STATUS_READY                 (1u << 5)

const VMStateDescription vmstate_ipu4_isys = {
    .name = "ipu4/isys",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(unispart_irq, Ipu4Isys, 0,
                       vmstate_ipu4_irq_trio, Ipu4IrqTrio),
        VMSTATE_UINT32(unispart_sw_irq_mux, Ipu4Isys),
        VMSTATE_UINT32(is_spc_status_ctrl, Ipu4Isys),
        VMSTATE_UINT32(ps_spc_status_ctrl, Ipu4Isys),
        VMSTATE_UINT32_ARRAY(dmem, Ipu4Isys, IS_DMEM_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

void ipu4_isys_reset(Ipu4Isys *i)
{
    ipu4_irq_trio_reset(&i->unispart_irq);
    i->unispart_sw_irq_mux = 0;
    i->is_spc_status_ctrl = 0;
    i->ps_spc_status_ctrl = 0;
    memset(i->dmem, 0, sizeof(i->dmem));
}

void ipu4_isys_signal_sw_irq(Ipu4Isys *i)
{
    i->unispart_irq.status |= ISYS_UNISPART_IRQ_SW;
}

static bool unispart_read(const Ipu4Isys *i, hwaddr addr, uint64_t *val)
{
    if (addr >= IS_UNISPART_IRQ_TRIO_BASE &&
        addr < IS_UNISPART_IRQ_TRIO_BASE + IPU4_IRQ_TRIO_SIZE) {
        hwaddr off = addr - IS_UNISPART_IRQ_TRIO_BASE;
        switch (off) {
        case IPU4_IRQ_TRIO_EDGE:    *val = i->unispart_irq.edge;   return true;
        case IPU4_IRQ_TRIO_MASK:    *val = i->unispart_irq.mask;   return true;
        case IPU4_IRQ_TRIO_STATUS:
            /* Raw status — see the comment in ipu4-isys.h. The
             * driver's ipu4_isys_isr reads STATUS and dispatches on
             * bit 30 directly without re-AND-ing with ENABLE; masking
             * here would silently drop the SW IRQ the syscom layer
             * synthesises if isys_setup_hw() hasn't yet programmed
             * ENABLE. */
            *val = i->unispart_irq.status;
            return true;
        case IPU4_IRQ_TRIO_ENABLE:  *val = i->unispart_irq.enable; return true;
        case IPU4_IRQ_TRIO_LEVEL:   *val = i->unispart_irq.level;  return true;
        }
    }
    if (addr == IS_UNISPART_SW_IRQ_MUX) {
        *val = i->unispart_sw_irq_mux;
        return true;
    }
    return false;
}

static bool unispart_write(Ipu4Isys *i, hwaddr addr, uint64_t val)
{
    if (addr >= IS_UNISPART_IRQ_TRIO_BASE &&
        addr < IS_UNISPART_IRQ_TRIO_BASE + IPU4_IRQ_TRIO_SIZE) {
        return ipu4_irq_trio_write(&i->unispart_irq,
                                   addr - IS_UNISPART_IRQ_TRIO_BASE, val);
    }
    if (addr == IS_UNISPART_SW_IRQ) {
        /* Silicon writes 0 here on every path (51 accesses, all 0x0).
         * On real hardware this is presumably a trigger that raises
         * the bits in SW_IRQ_MUX into IRQ_STATUS, but with value 0
         * nothing changes. Absorb the write without modelling the
         * mux routing until a driver path actually sets bits here. */
        if (val != 0) {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: unispart SW_IRQ written non-zero "
                          "val=0x%" PRIx64 " — model's zero-only-absorb "
                          "assumption broken; SW_IRQ_MUX routing is "
                          "unmodelled. See tools/notes/registers.md "
                          "(0x17c414).\n", val);
        }
        return true;
    }
    if (addr == IS_UNISPART_SW_IRQ_MUX) {
        i->unispart_sw_irq_mux = val;
        return true;
    }
    return false;
}

bool ipu4_isys_mmio_read(const Ipu4Isys *i, hwaddr addr, uint64_t *val)
{
    if (unispart_read(i, addr, val)) {
        return true;
    }
    if (addr == IS_SPC_STATUS_CTRL) {
        /* See the bit-map comment near the constant: force READY on,
         * START off so query_sp() reads "SPC is idle and ready"
         * regardless of what the driver kicked into the latch. */
        *val = (i->is_spc_status_ctrl & ~SPC_STATUS_START) | SPC_STATUS_READY;
        return true;
    }
    if (addr == PS_SPC_STATUS_CTRL) {
        *val = (i->ps_spc_status_ctrl & ~SPC_STATUS_START) | SPC_STATUS_READY;
        return true;
    }
    if (addr >= IS_DMEM_BASE && addr < IS_DMEM_BASE + IS_DMEM_SIZE) {
        /* SEND_RD_POS echoes SEND_WR_POS so the driver sees
         * "firmware has caught up" and doesn't block on the ring
         * filling. SYSCOM_STATE always returns READY so
         * ipu6_fw_com_ready()'s 500ms poll terminates on the
         * first read — the firmware-handshake shortcut documented
         * in the Step 1 plan. All other slots return their
         * latched value. */
        if (addr == IS_DMEM_FW_COM_SEND_RD_POS) {
            *val = i->dmem[(IS_DMEM_FW_COM_SEND_WR_POS - IS_DMEM_BASE) / 4];
        } else if (addr == IS_DMEM_SYSCOM_STATE) {
            *val = SYSCOM_STATE_READY;
        } else {
            *val = i->dmem[(addr - IS_DMEM_BASE) / 4];
        }
        return true;
    }
    return false;
}

bool ipu4_isys_mmio_write(Ipu4Isys *i, hwaddr addr, uint64_t val)
{
    if (unispart_write(i, addr, val)) {
        return true;
    }
    if (addr == IS_SPC_STATUS_CTRL) {
        i->is_spc_status_ctrl = val;
        return true;
    }
    if (addr == PS_SPC_STATUS_CTRL) {
        i->ps_spc_status_ctrl = val;
        return true;
    }
    /* DMEM writes are NOT handled here — the syscom layer needs to
     * see them first to apply SYSCOM_CONFIG_REG / send-bump side
     * effects, then it stores into i->dmem directly. */
    return false;
}
