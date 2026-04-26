/*
 * Intel IPU4 IRQ-trio helper — vmstate descriptor.
 *
 * The reset and write helpers are inline in ipu4-irq-trio.h; this
 * file exists so the vmstate descriptor has external linkage that
 * unispart and CSI2 can share via VMSTATE_STRUCT.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "qemu/osdep.h"
#include "migration/vmstate.h"

#include "ipu4-irq-trio.h"

const VMStateDescription vmstate_ipu4_irq_trio = {
    .name = "ipu4/irq_trio",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(edge, Ipu4IrqTrio),
        VMSTATE_UINT32(mask, Ipu4IrqTrio),
        VMSTATE_UINT32(status, Ipu4IrqTrio),
        VMSTATE_UINT32(enable, Ipu4IrqTrio),
        VMSTATE_UINT32(level, Ipu4IrqTrio),
        VMSTATE_END_OF_LIST()
    }
};
