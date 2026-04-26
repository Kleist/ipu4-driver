/*
 * Intel IPU4 IRQ-trio helper.
 *
 * The unispart and CSI2 IRQ blocks share a 6-register layout —
 * EDGE / MASK / STATUS / CLEAR (W1C) / ENABLE / LEVEL_NOT_PULSE — at
 * a fixed per-block base. This header collapses the latched state and
 * the write semantics so per-block .c files only need to dispatch on
 * the trio's base offset.
 *
 * Read semantics differ between blocks (unispart returns raw STATUS,
 * CSI2 returns STATUS & ENABLE), so reads are intentionally not
 * abstracted — callers do their own switch on `off` and compose the
 * correct value from the trio fields.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef HW_MISC_IPU4_IRQ_TRIO_H
#define HW_MISC_IPU4_IRQ_TRIO_H

#include "exec/hwaddr.h"
#include "migration/vmstate.h"

/* Per-trio register offsets, identical for unispart and every CSI2
 * IRQ block (kernel/ipu4/ipu4-platform-isys-csi2-reg.h:31-44 +
 * ipu4-platform-regs.h:49). */
#define IPU4_IRQ_TRIO_EDGE      0x000
#define IPU4_IRQ_TRIO_MASK      0x004
#define IPU4_IRQ_TRIO_STATUS    0x008
#define IPU4_IRQ_TRIO_CLEAR     0x00c
#define IPU4_IRQ_TRIO_ENABLE    0x010
#define IPU4_IRQ_TRIO_LEVEL     0x014
#define IPU4_IRQ_TRIO_SIZE      0x018

typedef struct Ipu4IrqTrio {
    uint32_t edge;
    uint32_t mask;
    uint32_t status;
    uint32_t enable;
    uint32_t level;
} Ipu4IrqTrio;

extern const VMStateDescription vmstate_ipu4_irq_trio;

static inline void ipu4_irq_trio_reset(Ipu4IrqTrio *t)
{
    t->edge = 0;
    t->mask = 0;
    t->status = 0;
    t->enable = 0;
    t->level = 0;
}

/* Apply a write to register `off` within the trio. Latches EDGE /
 * MASK / ENABLE / LEVEL_NOT_PULSE; treats CLEAR as W1C against
 * status. Returns true if `off` was a known trio register, false
 * otherwise. STATUS itself is read-only — writes are dropped on the
 * caller's floor (they fall through to the unimplemented log).
 */
static inline bool ipu4_irq_trio_write(Ipu4IrqTrio *t,
                                       hwaddr off, uint64_t val)
{
    switch (off) {
    case IPU4_IRQ_TRIO_EDGE:
        t->edge = val;
        return true;
    case IPU4_IRQ_TRIO_MASK:
        t->mask = val;
        return true;
    case IPU4_IRQ_TRIO_CLEAR:
        t->status &= ~(uint32_t)val;
        return true;
    case IPU4_IRQ_TRIO_ENABLE:
        t->enable = val;
        return true;
    case IPU4_IRQ_TRIO_LEVEL:
        t->level = val;
        return true;
    }
    return false;
}

#endif /* HW_MISC_IPU4_IRQ_TRIO_H */
