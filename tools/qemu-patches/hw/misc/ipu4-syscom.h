/*
 * Intel IPU4 syscom (firmware command/response) layer.
 *
 * Owns the queue descriptor cache, the FW response posting pipeline,
 * the Step-4 frame delivery path, and the DMEM-write side-effects
 * that drive the cache + send-cursor bump dispatch. Mirrors the
 * in-driver ipu6_fw_com.c + ipu4_isys_isr layering.
 *
 * The layer has no Ipu4State dependency; instead the caller bundles
 * pointers to its peers (mmu, isys, buttress, pdev) into an
 * Ipu4SyscomCtx and passes it through every entry point. This keeps
 * the dispatch direction one-way: ipu4.c → syscom → {mmu, isys,
 * buttress}.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef HW_MISC_IPU4_SYSCOM_H
#define HW_MISC_IPU4_SYSCOM_H

#include "exec/hwaddr.h"
#include "hw/pci/pci_device.h"
#include "migration/vmstate.h"

#include "ipu4-fw-isys.h"
#include "ipu4-isys.h"
#include "ipu4-mmu.h"
#include "ipu4-buttress.h"

typedef struct Ipu4Syscom {
    /* IPU IOVA of the syscom config struct, latched from a DMEM[1]
     * (SYSCOM_CONFIG_REG) write. Subsequent send-queue activity
     * triggers a full read of the config + queue descriptors. */
    uint32_t config_iova;

    /* Cached msg-send queue descriptors per stream (loaded lazily on
     * first send-cursor write so the cache picks up the descriptor
     * after the driver finished initialising it). send_q_loaded
     * stays false until both the config addr is known and a stream's
     * descriptor was successfully read; loaded state is per-stream so
     * a partial setup doesn't poison the others. */
    Ipu4FwSysQueue send_q[IPU4_MAX_MSG_STREAMS];
    bool send_q_loaded[IPU4_MAX_MSG_STREAMS];

    /* Cached msg-recv queue descriptor (single output queue at index
     * IPU4_BASE_MSG_RECV_QUEUE_INDEX of the absolute queue table). */
    Ipu4FwSysQueue recv_q;
    bool recv_q_loaded;

    /* Last-seen wr cursor per msg-send queue. Each driver-side bump
     * pushes (new_wr - prev) % q->size tokens; we read each new
     * token's send_type out of the input ring and dispatch on that
     * (Step 3 — counting alone doesn't survive STREAM_CAPTURE
     * interleaving between STREAM_START and STREAM_FLUSH). */
    uint32_t send_wr_seen[IPU4_MAX_MSG_STREAMS];
} Ipu4Syscom;

/* The syscom layer's external dependencies, bundled. The caller
 * builds one of these on the stack per MMIO callback so syscom
 * helpers don't need direct access to Ipu4State. */
typedef struct Ipu4SyscomCtx {
    Ipu4Syscom *sc;
    Ipu4Isys *isys;
    const Ipu4MmuRegs *mmu;
    Ipu4Buttress *buttress;
    PCIDevice *pdev;
} Ipu4SyscomCtx;

extern const VMStateDescription vmstate_ipu4_syscom;

void ipu4_syscom_reset(Ipu4Syscom *sc);

/* Apply a DMEM write. Updates the syscom config-iova / cache
 * invalidations, latches val into Ipu4Isys::dmem[reg], and (if the
 * write was a wr-cursor bump) dispatches send-token responses. The
 * caller owns the IS_DMEM_BASE/SIZE range check; this helper assumes
 * `addr` is within it. */
void ipu4_syscom_dmem_write(const Ipu4SyscomCtx *ctx,
                            hwaddr addr, uint64_t val);

#endif /* HW_MISC_IPU4_SYSCOM_H */
