/*
 * Intel IPU4 buttress block — see ipu4-buttress.h for the interface.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"

#include "ipu4-buttress.h"

/* Buttress registers (from kernel/ipu4/ipu6-platform-buttress-regs.h).
 * Offsets are from BAR0; the buttress block lives at BAR+0. */
#define BTRS_REG_WDT               0x008
#define BTRS_REG_BTRS_CTRL         0x00c
#define BTRS_REG_FW_RESET_CTL      0x030
#define BTRS_REG_IS_FREQ_CTL       0x034
#define BTRS_REG_PS_FREQ_CTL       0x038
#define BTRS_REG_PWR_STATE         0x05c
#define BTRS_REG_FW_SOURCE_BASE_LO 0x078
#define BTRS_REG_FW_SOURCE_BASE_HI 0x07c
#define BTRS_REG_FW_SOURCE_SIZE    0x080
#define BTRS_REG_FABRIC_CMD        0x088
#define BTRS_REG_ISR_STATUS        0x090
#define BTRS_REG_ISR_ENABLED_STATUS 0x094
#define BTRS_REG_ISR_ENABLE        0x098
#define BTRS_REG_ISR_CLEAR         0x09c
#define BTRS_REG_IU2CSEDB0         0x100
#define BTRS_REG_IU2CSEDATA0       0x104
#define BTRS_REG_IU2CSECSR         0x108
#define BTRS_REG_TSC_LO            0x164
#define BTRS_REG_TSC_HI            0x168
#define BTRS_REG_SECURITY_CTL      0x300
#define BTRS_REG_CSE2IUDB0         0x304
#define BTRS_REG_CSE2IUDATA0       0x308
#define BTRS_REG_CSE2IUCSR         0x30c
#define BTRS_REG_SKU               0x314

#define BTRS_FW_RESET_CTL_START    (1u << 0)
#define BTRS_FW_RESET_CTL_DONE     (1u << 1)

/* PWR_STATE FSM bits (kernel/ipu4/ipu4-platform-buttress-regs.h:11-22).
 *   bits [1:0]    PWR_RDY = 3       — buttress always ready
 *   bits [13:12]  HH_DONE = 2       — TSC-sync done
 *   bits [23:20]  IS_PWR_FSM   IDLE=0, IS_RDY=0xa
 *   bits [28:24]  PS_PWR_FSM   IDLE=0, PS_PWR_UP=0xf
 * The driver triggers transitions by writing IS_FREQ_CTL / PS_FREQ_CTL
 * with BUTTRESS_FREQ_CTL_START (BIT(31)) for power-on or zero for
 * power-off (kernel/ipu4/ipu6-buttress.c:466-505). */
#define BTRS_PWR_STATE_PWR_RDY     (3u << 0)
#define BTRS_PWR_STATE_HH_DONE     (2u << 12)
#define BTRS_PWR_STATE_IS_RDY      (0xau << 20)
#define BTRS_PWR_STATE_PS_PWR_UP   (0xfu << 24)
#define BTRS_FREQ_CTL_START        (1u << 31)

/* IPU4 SP→host syscom delivery uses BTRS_REG_ISR_STATUS bit 0 to wake
 * `ipu6_buttress_isr`, which then dispatches to `ipu4_isys_isr`
 * (ipu6-isys.c:375). */
#define BTRS_ISR_IS_IRQ            (1u << 0)

const VMStateDescription vmstate_ipu4_buttress = {
    .name = "ipu4/buttress",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(btrs_ctrl, Ipu4Buttress),
        VMSTATE_UINT32(fw_reset_ctl, Ipu4Buttress),
        VMSTATE_UINT32(is_freq_ctl, Ipu4Buttress),
        VMSTATE_UINT32(ps_freq_ctl, Ipu4Buttress),
        VMSTATE_BOOL(is_powered, Ipu4Buttress),
        VMSTATE_BOOL(ps_powered, Ipu4Buttress),
        VMSTATE_UINT32(fabric_cmd, Ipu4Buttress),
        VMSTATE_UINT32(pwr_state, Ipu4Buttress),
        VMSTATE_UINT32(isr_enable, Ipu4Buttress),
        VMSTATE_UINT32(isr_status, Ipu4Buttress),
        VMSTATE_UINT32(fw_src_lo, Ipu4Buttress),
        VMSTATE_UINT32(fw_src_hi, Ipu4Buttress),
        VMSTATE_UINT32(fw_src_size, Ipu4Buttress),
        VMSTATE_UINT32(security_ctl, Ipu4Buttress),
        VMSTATE_UINT32(iu2cse_csr, Ipu4Buttress),
        VMSTATE_UINT32(cse2iu_csr, Ipu4Buttress),
        VMSTATE_UINT32(cse2iu_data0, Ipu4Buttress),
        VMSTATE_END_OF_LIST()
    }
};

void ipu4_buttress_reset(Ipu4Buttress *b)
{
    /* Hardware-default reset values from data/trace.txt. BTRS_CTRL
     * reads 0x10 before any write (bit 4, fixed by silicon); the
     * driver never writes it back, so the reset must match or
     * compare.py sees a read-value mismatch. */
    b->btrs_ctrl = 0x10;
    b->fw_reset_ctl = 0;
    b->is_freq_ctl = 0;
    b->ps_freq_ctl = 0;
    b->is_powered = false;
    b->ps_powered = false;
    b->fabric_cmd = 0;
    b->pwr_state = 0;
    b->isr_enable = 0;
    b->isr_status = 0;
    b->fw_src_lo = 0;
    b->fw_src_hi = 0;
    b->fw_src_size = 0;
    /* Silicon's SECURITY_CTL reads 0x37002 before any driver write —
     * the value has BUTTRESS_SECURITY_CTL_FW_SECURE_MODE (BIT(16))
     * set, so the real silicon runs the IPU4 in secure mode. Our
     * model can't complete the CSE IPC authentication handshake
     * that secure mode requires, so reset to 0 instead and let the
     * driver fall into the non-secure branch (same shortcut the M3
     * buttress rows in registers.md document). compare.py will
     * flag a persistent value_mismatch here until secure-mode CSE
     * IPC is modelled; that's intentional. */
    b->security_ctl = 0;
    b->iu2cse_csr = 0;
    b->cse2iu_csr = 0;
    b->cse2iu_data0 = 0;
}

bool ipu4_buttress_mmio_read(Ipu4Buttress *b, hwaddr addr, uint64_t *val)
{
    switch (addr) {
    case BTRS_REG_WDT:
        /* Silicon returns 0xfff0fff on reads (likely a timeout/fuse
         * constant; value lifted verbatim from data/trace.txt). The
         * driver doesn't write it back, so a fixed return is enough.
         * Writes are still absorbed as watchdog kicks in mmio_write. */
        *val = 0xfff0fff;
        return true;
    case BTRS_REG_BTRS_CTRL:
        *val = b->btrs_ctrl;
        return true;
    case BTRS_REG_FW_RESET_CTL:
        /* Once the driver kicks START, latch DONE so the poll terminates. */
        if (b->fw_reset_ctl & BTRS_FW_RESET_CTL_START) {
            b->fw_reset_ctl |= BTRS_FW_RESET_CTL_DONE;
        }
        *val = b->fw_reset_ctl;
        return true;
    case BTRS_REG_PWR_STATE: {
        /* Compose PWR_STATE from per-island FSM flags. PWR_RDY and
         * HH_DONE are tied high — buttress and TSC-sync always ready
         * since we don't model their FSMs. */
        uint32_t v = BTRS_PWR_STATE_PWR_RDY | BTRS_PWR_STATE_HH_DONE;
        if (b->is_powered) {
            v |= BTRS_PWR_STATE_IS_RDY;
        }
        if (b->ps_powered) {
            v |= BTRS_PWR_STATE_PS_PWR_UP;
        }
        b->pwr_state = v;
        *val = v;
        return true;
    }
    case BTRS_REG_ISR_STATUS:
    case BTRS_REG_ISR_ENABLED_STATUS:
        /* Pre-refactor model returned (status & enable) for both reads;
         * preserved verbatim. The driver writes BTRS_REG_ISR_ENABLE
         * with bit 0 set during probe so the syscom IRQ propagates. */
        *val = b->isr_status & b->isr_enable;
        return true;
    case BTRS_REG_ISR_ENABLE:
        *val = b->isr_enable;
        return true;
    case BTRS_REG_SECURITY_CTL:
        *val = b->security_ctl;
        return true;
    case BTRS_REG_IU2CSECSR:
        *val = b->iu2cse_csr;
        return true;
    case BTRS_REG_CSE2IUCSR:
        *val = b->cse2iu_csr;
        return true;
    case BTRS_REG_CSE2IUDATA0:
        *val = b->cse2iu_data0;
        return true;
    case BTRS_REG_SKU:
        *val = 0; /* unfused */
        return true;
    case BTRS_REG_TSC_LO:
    case BTRS_REG_TSC_HI: {
        /* 64-bit free-running timestamp counter. data/trace.txt shows
         * the driver reading HI, LO, HI in sequence and retrying when
         * the two HI samples disagree; we re-sample QEMU_CLOCK_VIRTUAL
         * per read rather than latching a shared snapshot. The HI half
         * (top 32 bits of nanoseconds) only flips every ~4.3 seconds
         * of guest time, so the chance of an in-flight HI rollover
         * between two adjacent MMIO accesses in the same vcpu thread
         * is effectively nil — the driver's retry path stays cold. */
        uint64_t tsc = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        *val = (addr == BTRS_REG_TSC_HI) ? (tsc >> 32) : (uint32_t)tsc;
        return true;
    }
    }
    return false;
}

bool ipu4_buttress_mmio_write(Ipu4Buttress *b, hwaddr addr, uint64_t val)
{
    switch (addr) {
    case BTRS_REG_BTRS_CTRL:
        b->btrs_ctrl = val;
        return true;
    case BTRS_REG_WDT:
        /* Watchdog kick; ignore. */
        return true;
    case BTRS_REG_FW_RESET_CTL:
        b->fw_reset_ctl = val;
        return true;
    case BTRS_REG_IS_FREQ_CTL:
        /* Driver writes the BUTTRESS_FREQ_CTL_START bit (BIT(31))
         * along with the divisor / QoS-floor / ICCMAX fields to
         * power up an island, or zero to power it down
         * (kernel/ipu4/ipu6-buttress.c:466-505). Mirror the START
         * bit into is_powered so PWR_STATE reads track the driver's
         * intent — without this, ipu6_buttress_power(off) times out
         * polling for IS_PWR_FSM == IDLE and the auto-suspend
         * recovery strands isys->power == 0. */
        b->is_freq_ctl = val;
        b->is_powered = (val & BTRS_FREQ_CTL_START) != 0;
        return true;
    case BTRS_REG_PS_FREQ_CTL:
        b->ps_freq_ctl = val;
        b->ps_powered = (val & BTRS_FREQ_CTL_START) != 0;
        return true;
    case BTRS_REG_FABRIC_CMD:
        /* Single-shot fabric command write (data/trace.txt shows one
         * write of 0x1 during init). No driver readback; latch and
         * move on. */
        b->fabric_cmd = val;
        return true;
    case BTRS_REG_FW_SOURCE_BASE_LO:
        b->fw_src_lo = val;
        return true;
    case BTRS_REG_FW_SOURCE_BASE_HI:
        b->fw_src_hi = val;
        return true;
    case BTRS_REG_FW_SOURCE_SIZE:
        b->fw_src_size = val;
        return true;
    case BTRS_REG_ISR_ENABLE:
        b->isr_enable = val;
        return true;
    case BTRS_REG_ISR_CLEAR:
        /* Write-1-to-clear. */
        b->isr_status &= ~(uint32_t)val;
        return true;
    case BTRS_REG_SECURITY_CTL:
        b->security_ctl = val;
        return true;
    case BTRS_REG_IU2CSEDB0:
    case BTRS_REG_IU2CSEDATA0:
    case BTRS_REG_IU2CSECSR:
        /* IPC send path. The reset handshake is a no-op: echo CSR bits
         * back via CSE2IU* so ipu6_buttress_ipc_reset() completes. */
        if (addr == BTRS_REG_IU2CSECSR) {
            b->iu2cse_csr = val;
            b->cse2iu_csr = val; /* echo */
            b->cse2iu_data0 = 0;
        }
        return true;
    case BTRS_REG_CSE2IUDB0:
    case BTRS_REG_CSE2IUCSR:
        /* IPC receive side: the driver acks by writing here; clear state. */
        b->cse2iu_csr = 0;
        return true;
    }
    return false;
}

void ipu4_buttress_signal_is_irq(Ipu4Buttress *b)
{
    b->isr_status |= BTRS_ISR_IS_IRQ;
}
