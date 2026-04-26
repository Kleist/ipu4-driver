/*
 * Intel IPU4 buttress block — power state FSM, ISR, IPC, TSC,
 * security control, FW reset / source registers.
 *
 * Mirrors the BTRS_* registers documented in
 * kernel/ipu4/ipu6-platform-buttress-regs.h and the FSM behaviour of
 * kernel/ipu4/ipu6-buttress.c. The buttress block lives at BAR+0 in
 * the device model.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef HW_MISC_IPU4_BUTTRESS_H
#define HW_MISC_IPU4_BUTTRESS_H

#include "exec/hwaddr.h"
#include "migration/vmstate.h"

/* Latched-state for the buttress block. Embedded inside Ipu4State. */
typedef struct Ipu4Buttress {
    uint32_t btrs_ctrl;
    uint32_t fw_reset_ctl;
    uint32_t is_freq_ctl;
    uint32_t ps_freq_ctl;

    /* PWR_STATE FSM flags. Set when the driver writes IS/PS_FREQ_CTL
     * with BUTTRESS_FREQ_CTL_START (BIT(31)); cleared on a zero write.
     * PWR_STATE reads compose bits[23:20] = IS_RDY / bits[28:24] =
     * PS_PWR_UP from these. */
    bool is_powered;
    bool ps_powered;

    uint32_t fabric_cmd;

    /* Latched on read of BTRS_REG_PWR_STATE; preserved for snapshot
     * continuity, not consulted by reads. */
    uint32_t pwr_state;

    uint32_t isr_enable;
    uint32_t isr_status;
    uint32_t fw_src_lo;
    uint32_t fw_src_hi;
    uint32_t fw_src_size;
    uint32_t security_ctl;
    uint32_t iu2cse_csr;
    uint32_t cse2iu_csr;
    uint32_t cse2iu_data0;
} Ipu4Buttress;

extern const VMStateDescription vmstate_ipu4_buttress;

void ipu4_buttress_reset(Ipu4Buttress *b);

/* Buttress register MMIO. Returns true if `addr` falls inside the
 * buttress register set (caller should not fall through to subsequent
 * blocks); false otherwise. */
bool ipu4_buttress_mmio_read(Ipu4Buttress *b, hwaddr addr, uint64_t *val);
bool ipu4_buttress_mmio_write(Ipu4Buttress *b, hwaddr addr, uint64_t val);

/* Set BTRS_ISR_IS_IRQ in isr_status. The syscom layer calls this
 * after posting a FW response so the driver's ipu6_buttress_isr
 * dispatches into ipu4_isys_isr. The matching BTRS_REG_ISR_CLEAR W1C
 * path is handled inside the module. */
void ipu4_buttress_signal_is_irq(Ipu4Buttress *b);

#endif /* HW_MISC_IPU4_BUTTRESS_H */
