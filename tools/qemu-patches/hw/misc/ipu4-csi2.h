/*
 * Intel IPU4 CSI2 receiver block.
 *
 * Models port 0 (the only port exercised in data/trace.txt) plus a
 * "this is unmodelled" fallback for ports 1..5. Mirrors the kernel
 * register layout in kernel/ipu4/ipu4-platform-isys-csi2-reg.h.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef HW_MISC_IPU4_CSI2_H
#define HW_MISC_IPU4_CSI2_H

#include "exec/hwaddr.h"
#include "migration/vmstate.h"

#include "ipu4-irq-trio.h"

/* CSI2 port 0 base. Bases for ports 1..5 are 0x65000 (p1), 0x66000
 * (p2), 0x67000 (p3), 0x6c000 (p4), 0x6c800 (p5)
 * (kernel/ipu4/ipu4-platform-isys-csi2-reg.h:13-15). Only port 0 is
 * modelled — data/trace.txt only exercised port 0; any access to the
 * other ports' range logs LOG_UNIMP. */
#define CSI2P0_BASE                      0x64000

/* Per-port CSI2 state. Writes to an unmodelled register inside the
 * port's range fall through to the unimplemented-access logger. */
typedef struct Ipu4Csi2Port {
    uint32_t rx_enable;
    uint32_t rx_nof_lanes;
    uint32_t rx_config;
    uint32_t dly_termen_c;
    uint32_t dly_settle_c;
    uint32_t dly_termen_d0;
    uint32_t dly_settle_d0;

    /* IRQ trios: PART (CSI2 partial frame), RX (CSI RX), S2M
     * (stream-to-memory). Same layout as the unispart trio. */
    Ipu4IrqTrio part;
    Ipu4IrqTrio rx;
    Ipu4IrqTrio s2m;
} Ipu4Csi2Port;

typedef struct Ipu4Csi2 {
    Ipu4Csi2Port port0;
} Ipu4Csi2;

extern const VMStateDescription vmstate_ipu4_csi2;

void ipu4_csi2_reset(Ipu4Csi2 *c);

/* Returns true when `addr` falls in any CSI2 register range —
 * including the unmodelled-port log range — so the caller stops
 * dispatching. `*val` is set on a port-0 hit; on an unmodelled-port
 * read it stays 0 (matching pre-refactor behaviour). */
bool ipu4_csi2_mmio_read(const Ipu4Csi2 *c, hwaddr addr, uint64_t *val);
bool ipu4_csi2_mmio_write(Ipu4Csi2 *c, hwaddr addr, uint64_t val);

#endif /* HW_MISC_IPU4_CSI2_H */
