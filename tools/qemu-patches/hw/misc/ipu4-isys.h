/*
 * Intel IPU4 ISYS subsystem — unispart IRQ block, ISYS/PSYS SPC
 * status/ctrl latches, and the ISYS DMEM syscom ring-pointer window.
 *
 * Mirrors the ISYS register footprint inferred from
 * kernel/ipu4/ipu4-platform-regs.h plus the IS_DMEM offsets the
 * driver derives via FW_COM_WR_REG / FW_COM_RD_REG (ipu6-fw-com.h).
 *
 * The DMEM storage is owned here, but write-side syscom side-effects
 * (SYSCOM_CONFIG_REG latch, send-cursor bump dispatch) are handled by
 * the syscom layer, which reaches into Ipu4Isys::dmem directly. The
 * read path is entirely module-local — SEND_RD_POS echoes SEND_WR_POS
 * and SYSCOM_STATE always returns READY.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef HW_MISC_IPU4_ISYS_H
#define HW_MISC_IPU4_ISYS_H

#include "exec/hwaddr.h"
#include "migration/vmstate.h"

#include "ipu4-irq-trio.h"

/* ISYS DMEM syscom ring-pointer block at BAR+0x108000 (from
 * postprocess_trace.py's NAMED_REGIONS; the driver derives offsets
 * via FW_COM_WR_REG / FW_COM_RD_REG in kernel/ipu4/ipu6-fw-com.h).
 * The first 0x100 bytes are firmware-parameter + ring-pointer slots
 * the driver writes during FW bringup and polls during streaming.
 */
#define IS_DMEM_BASE                     0x108000
#define IS_DMEM_SIZE                     0x100
#define IS_DMEM_SYSCOM_STATE             (IS_DMEM_BASE + 0x008)
#define IS_DMEM_FW_COM_SEND_WR_POS       (IS_DMEM_BASE + 0x028)
#define IS_DMEM_FW_COM_SEND_RD_POS       (IS_DMEM_BASE + 0x02c)
#define IS_DMEM_FW_COM_RECV_WR_POS       (IS_DMEM_BASE + 0x070)
#define IS_DMEM_FW_COM_RECV_RD_POS       (IS_DMEM_BASE + 0x074)

/* Bit raised in unispart IRQ STATUS by the syscom layer when posting a
 * FW response. ipu4_isys_signal_sw_irq() abstracts the bit-set. */
#define ISYS_UNISPART_IRQ_SW             (1u << 30)

typedef struct Ipu4Isys {
    /* Unispart IRQ block at BAR+0x17c000.
     *
     * STATUS reads return RAW status (unmasked). `status` stays zero
     * under QEMU because we don't model the ISYS hardware that
     * normally raises these interrupts (frame generator, CSI2 port
     * events, …); the syscom layer toggles ISYS_UNISPART_IRQ_SW
     * directly via ipu4_isys_signal_sw_irq. */
    Ipu4IrqTrio unispart_irq;
    uint32_t unispart_sw_irq_mux;

    /* ISYS / PSYS SPC status-control latches at BAR+0x100000 and
     * BAR+0x400000. Reads return the latched value with SPC_STATUS_READY
     * forced on / SPC_STATUS_START forced off so query_sp() succeeds on
     * the first poll. PSYS SPC lives here as a structural twin of ISYS
     * SPC; it is not otherwise an ISYS register. */
    uint32_t is_spc_status_ctrl;
    uint32_t ps_spc_status_ctrl;

    /* ISYS DMEM syscom window. */
    uint32_t dmem[IS_DMEM_SIZE / 4];
} Ipu4Isys;

extern const VMStateDescription vmstate_ipu4_isys;

void ipu4_isys_reset(Ipu4Isys *i);

bool ipu4_isys_mmio_read(const Ipu4Isys *i, hwaddr addr, uint64_t *val);
bool ipu4_isys_mmio_write(Ipu4Isys *i, hwaddr addr, uint64_t val);

/* Set ISYS_UNISPART_IRQ_SW in unispart_irq.status; called by syscom
 * after posting a FW response so ipu4_isys_isr drains it. */
void ipu4_isys_signal_sw_irq(Ipu4Isys *i);

#endif /* HW_MISC_IPU4_ISYS_H */
