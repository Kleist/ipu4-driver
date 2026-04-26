/*
 * Intel IPU4 syscom layer — see ipu4-syscom.h for the interface.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/pci/msi.h"
#include "migration/vmstate.h"

#include "ipu4-syscom.h"
#include "ipu4-fw-isys.h"
#include "ipu4-isys.h"
#include "ipu4-mmu.h"
#include "ipu4-buttress.h"

/* Step-4 deterministic frame pattern: write `total` bytes starting at
 * `base_iova`, where byte[k] = (k + seq) & 0xff. Bounded by
 * IPU4_FRAME_PATTERN_BYTES (64 KB) — see comment in
 * ipu4_mmu_dma_write_iova_pattern about why we don't walk the full
 * vb2 buffer. */
#define IPU4_FRAME_PATTERN_BYTES (64 * 1024)

const VMStateDescription vmstate_ipu4_syscom = {
    .name = "ipu4/syscom",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(config_iova, Ipu4Syscom),
        VMSTATE_UINT32_ARRAY(send_wr_seen, Ipu4Syscom, IPU4_MAX_MSG_STREAMS),
        VMSTATE_END_OF_LIST()
        /* The send_q[]/recv_q descriptor cache and the *_loaded
         * flags are intentionally not migrated; they're rebuilt
         * lazily from the IPU MMU on the next send-cursor write. */
    }
};

void ipu4_syscom_reset(Ipu4Syscom *sc)
{
    sc->config_iova = 0;
    memset(sc->send_q, 0, sizeof(sc->send_q));
    memset(sc->send_q_loaded, 0, sizeof(sc->send_q_loaded));
    memset(&sc->recv_q, 0, sizeof(sc->recv_q));
    sc->recv_q_loaded = false;
    memset(sc->send_wr_seen, 0, sizeof(sc->send_wr_seen));
}

/* Read the `i`-th queue descriptor from the IOVA-addressed queue array
 * pointed to by `array_iova`. */
static bool load_queue_desc(const Ipu4SyscomCtx *ctx, uint32_t array_iova,
                            unsigned int i, Ipu4FwSysQueue *out)
{
    return ipu4_mmu_dma_read_iova(ctx->mmu, ctx->pdev,
                                  array_iova + i * sizeof(Ipu4FwSysQueue),
                                  out, sizeof(*out));
}

/* Lazy lookup of the msg-send queue descriptor for stream `stream`.
 * Returns a pointer into the cache, or NULL if the descriptor isn't
 * loadable yet (no syscom config addr written, MMU not programmed,
 * etc.). */
static Ipu4FwSysQueue *get_send_queue(const Ipu4SyscomCtx *ctx,
                                      unsigned int stream)
{
    Ipu4Syscom *sc = ctx->sc;
    Ipu4FwSyscomConfig cfg;
    unsigned int q_index;

    if (stream >= IPU4_MAX_MSG_STREAMS) {
        return NULL;
    }
    if (sc->send_q_loaded[stream]) {
        return &sc->send_q[stream];
    }
    if (sc->config_iova == 0) {
        return NULL;
    }
    if (!ipu4_mmu_dma_read_iova(ctx->mmu, ctx->pdev,
                                sc->config_iova, &cfg, sizeof(cfg))) {
        return NULL;
    }

    q_index = IPU4_BASE_MSG_SEND_QUEUES + stream;
    if (!load_queue_desc(ctx, cfg.input_queue, q_index,
                         &sc->send_q[stream])) {
        return NULL;
    }
    sc->send_q_loaded[stream] = true;
    sc->send_wr_seen[stream] = 0;
    return &sc->send_q[stream];
}

/* Lazy lookup of the single msg-recv queue (output queue index 1 in
 * the absolute table = output_queue array index 1). */
static Ipu4FwSysQueue *get_recv_queue(const Ipu4SyscomCtx *ctx)
{
    Ipu4Syscom *sc = ctx->sc;
    Ipu4FwSyscomConfig cfg;

    if (sc->recv_q_loaded) {
        return &sc->recv_q;
    }
    if (sc->config_iova == 0) {
        return NULL;
    }
    if (!ipu4_mmu_dma_read_iova(ctx->mmu, ctx->pdev,
                                sc->config_iova, &cfg, sizeof(cfg))) {
        return NULL;
    }
    /* output_queue[1] is the msg-recv slot; output_queue[0] is proxy. */
    if (!load_queue_desc(ctx, cfg.output_queue, 1, &sc->recv_q)) {
        return NULL;
    }
    sc->recv_q_loaded = true;
    return &sc->recv_q;
}

/* Post a pre-filled response token on the msg-recv queue and raise
 * the IS-side MSI so the driver's ISR drains it. Caller fills `resp`
 * before calling — at minimum `type` and `stream_handle`; pin and
 * timestamp fields stay zeroed for the simple ack types and get
 * populated for PIN_DATA_READY. */
static void post_response_full(const Ipu4SyscomCtx *ctx,
                               const Ipu4FwIsysRespInfo *resp)
{
    Ipu4FwSysQueue *rq = get_recv_queue(ctx);
    uint32_t *recv_wr_slot;
    uint32_t wr;

    if (!rq || rq->size == 0 || rq->token_size < sizeof(*resp)) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: cannot post FW response type=%u stream=%u "
                      "(recv queue not loaded or token_size too small)\n",
                      resp->type, resp->stream_handle);
        return;
    }

    /* Locate the slot the recv-side wr cursor points at. The driver
     * caches the wr/rd cursors in DMEM at q->wr_reg*4 / q->rd_reg*4. */
    if (rq->wr_reg >= IS_DMEM_SIZE / 4) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: recv queue wr_reg=%u out of DMEM range\n",
                      rq->wr_reg);
        return;
    }
    recv_wr_slot = &ctx->isys->dmem[rq->wr_reg];
    wr = *recv_wr_slot;
    if (wr >= rq->size) {
        wr = 0;
    }

    /* DMA the response into the slot. If the IOVA can't be walked
     * (MMU not programmed), bail out — the driver will time out, but
     * better that than corrupting an unrelated guest page. */
    if (!ipu4_mmu_dma_write_iova(ctx->mmu, ctx->pdev,
                                 rq->vied_address + wr * rq->token_size,
                                 resp, sizeof(*resp))) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: failed to DMA response to recv slot wr=%u "
                      "iova=0x%x — IPU MMU mapping missing?\n",
                      wr, rq->vied_address);
        return;
    }

    /* Advance the firmware-owned wr cursor. */
    wr = (wr + 1) % rq->size;
    *recv_wr_slot = wr;

    /* Raise the IS-side IRQ. The buttress dispatch + ISYS unispart
     * SW-IRQ bit are both required for ipu6_buttress_isr →
     * ipu4_isys_isr → isys_isr_one to drain the response. */
    ipu4_isys_signal_sw_irq(ctx->isys);
    ipu4_buttress_signal_is_irq(ctx->buttress);

    if (msi_enabled(ctx->pdev)) {
        msi_notify(ctx->pdev, 0);
    }
}

/* Wrapper for simple ack types (OPEN_DONE, START_*_ACK, FLUSH_ACK,
 * CLOSE_ACK) that only need the type and stream_handle filled.
 * buf_handle stays 0 — the driver's ipu6_put_fw_msg_buf is a no-op
 * for buf_handle == 0 (kernel/ipu4/ipu6-isys.c:1185-1192). */
static void post_response(const Ipu4SyscomCtx *ctx, uint8_t resp_type,
                          uint8_t stream_handle)
{
    Ipu4FwIsysRespInfo resp = {
        .type = resp_type,
        .stream_handle = stream_handle,
    };
    post_response_full(ctx, &resp);
}

/* Step-4: synthesise a PIN_DATA_READY response that the driver's
 * ipu6_isys_queue_buf_ready() (kernel/ipu4/ipu6-isys-queue.c:935)
 * matches against the buffer's IPU IOVA via `info->pin.addr`.
 * timestamp[] = 0 so get_sof_sequence_by_timestamp() falls into the
 * `time == 0` branch and returns `atomic_read(stream->sequence) - 1`,
 * which equals 0 after our preceding FRAME_SOF response increments
 * stream->sequence from 0 to 1 — that's the value the streamon-smoke
 * pattern check (k + seq) was written for. */
static void post_pin_data_ready(const Ipu4SyscomCtx *ctx,
                                uint8_t stream_handle, uint8_t pin_id,
                                uint32_t pin_addr, uint64_t pin_buf_id)
{
    Ipu4FwIsysRespInfo resp = {
        .type = IPU4_FW_RESP_STREAM_PIN_DATA_READY,
        .stream_handle = stream_handle,
        .pin_id = pin_id,
        .pin = {
            .out_buf_id = pin_buf_id,
            .addr = pin_addr,
        },
    };
    post_response_full(ctx, &resp);
}

/* Step-4: deliver a frame for every non-zero output pin in the
 * `frame_buff_set` payload that the driver attached to a
 * STREAM_START_AND_CAPTURE / STREAM_CAPTURE token. For each pin we
 * (a) DMA-write IPU4_FRAME_PATTERN_BYTES of the deterministic pattern
 *     `byte[k] = (k + seq) & 0xff` (seq=0 — see post_pin_data_ready)
 *     into the IOVA-mapped buffer at output_pins[i].addr,
 * (b) post a FRAME_SOF response to bump stream->sequence,
 * (c) post a PIN_DATA_READY response that the driver matches against
 *     the buffer's IOVA. */
static void deliver_frame(const Ipu4SyscomCtx *ctx, uint8_t stream,
                          uint32_t frame_buff_set_iova)
{
    Ipu4FwOutputPin pins[IPU4_MAX_OPINS];
    unsigned int i;

    if (!ipu4_mmu_dma_read_iova(ctx->mmu, ctx->pdev,
                                frame_buff_set_iova, pins, sizeof(pins))) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: failed to DMA-read frame_buff_set at "
                      "iova=0x%x — driver will not see frames\n",
                      frame_buff_set_iova);
        return;
    }

    /* FRAME_SOF first so the driver's atomic_fetch_inc on
     * stream->sequence (ipu6-isys-csi2.c:725) takes effect before
     * PIN_DATA_READY's get_sof_sequence_by_timestamp() reads it. */
    {
        Ipu4FwIsysRespInfo sof = {
            .type = IPU4_FW_RESP_FRAME_SOF,
            .stream_handle = stream,
        };
        post_response_full(ctx, &sof);
    }

    for (i = 0; i < IPU4_MAX_OPINS; i++) {
        if (pins[i].addr == 0) {
            continue;
        }
        ipu4_mmu_dma_write_iova_pattern(ctx->mmu, ctx->pdev,
                                        pins[i].addr,
                                        IPU4_FRAME_PATTERN_BYTES, 0);
        post_pin_data_ready(ctx, stream, (uint8_t)i,
                            pins[i].addr, pins[i].out_buf_id);
    }
}

/* Detect a driver-side bump of the wr cursor for msg-send queue
 * `stream` and synthesise a response per the protocol's request/ack
 * pairing. */
static void handle_send_bump(const Ipu4SyscomCtx *ctx, unsigned int stream,
                             uint32_t new_wr)
{
    Ipu4Syscom *sc = ctx->sc;
    Ipu4FwSysQueue *q = get_send_queue(ctx, stream);
    uint32_t prev = sc->send_wr_seen[stream];
    uint32_t pushed, slot;

    if (!q || q->size == 0) {
        /* Without the descriptor we can't tell wrap from increment;
         * still record the cursor so subsequent bumps are tracked
         * once the descriptor lands. */
        sc->send_wr_seen[stream] = new_wr;
        return;
    }
    if (new_wr == prev) {
        return;
    }
    pushed = (new_wr + q->size - prev) % q->size;
    slot = prev;
    sc->send_wr_seen[stream] = new_wr;

    while (pushed-- > 0) {
        Ipu4FwSendToken token;

        if (q->token_size < sizeof(token) ||
            !ipu4_mmu_dma_read_iova(ctx->mmu, ctx->pdev,
                                    q->vied_address + slot * q->token_size,
                                    &token, sizeof(token))) {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: cannot read send token (stream=%u "
                          "slot=%u vied=0x%x token_size=%u) — driver "
                          "will time out on stream completion.\n",
                          stream, slot, q->vied_address, q->token_size);
            slot = (slot + 1) % q->size;
            continue;
        }
        slot = (slot + 1) % q->size;

        switch (token.send_type) {
        case IPU4_FW_SEND_STREAM_OPEN:
            post_response(ctx, IPU4_FW_RESP_STREAM_OPEN_DONE,
                          (uint8_t)stream);
            break;
        case IPU4_FW_SEND_STREAM_START:
            /* No buffer attached — STREAM_START is the bufless variant,
             * so just complete stream_start_completion. */
            post_response(ctx, IPU4_FW_RESP_STREAM_START_ACK,
                          (uint8_t)stream);
            break;
        case IPU4_FW_SEND_STREAM_START_AND_CAPTURE:
            /* Bundle: ack the start completion AND deliver the frame
             * for the buffer attached via token.payload. */
            post_response(ctx, IPU4_FW_RESP_STREAM_START_AND_CAPTURE_ACK,
                          (uint8_t)stream);
            deliver_frame(ctx, (uint8_t)stream, token.payload);
            break;
        case IPU4_FW_SEND_STREAM_CAPTURE:
            /* Step 4: deliver the frame attached to this token. The
             * driver doesn't wait on a per-CAPTURE completion (the
             * only acks the streamon path waits on are STREAM_OPEN
             * and STREAM_*_START), so a single PIN_DATA_READY per
             * non-zero output_pins[i].addr is sufficient. */
            deliver_frame(ctx, (uint8_t)stream, token.payload);
            break;
        case IPU4_FW_SEND_STREAM_FLUSH:
            /* Driver completes stream_stop_completion
             * (ipu6-isys.c:1452-1453). Without this ack, every
             * STREAMOFF logs "stream stop time out". */
            post_response(ctx, IPU4_FW_RESP_STREAM_FLUSH_ACK,
                          (uint8_t)stream);
            break;
        case IPU4_FW_SEND_STREAM_CLOSE:
            /* Driver completes stream_close_completion
             * (ipu6-isys.c:1438-1439). */
            post_response(ctx, IPU4_FW_RESP_STREAM_CLOSE_ACK,
                          (uint8_t)stream);
            break;
        case IPU4_FW_SEND_STREAM_STOP:
            /* The IPU4 driver routes STREAMOFF through STREAM_FLUSH
             * (kernel/ipu4/ipu6-isys-video.c:750), not STREAM_STOP,
             * so STREAM_STOP arriving here would mean a path we
             * haven't characterised. Log and drop. */
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: STREAM_STOP send on stream=%u "
                          "(driver shouldn't use this opcode)\n",
                          stream);
            break;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: unknown send_type=%u on stream=%u\n",
                          token.send_type, stream);
            break;
        }
    }
}

void ipu4_syscom_dmem_write(const Ipu4SyscomCtx *ctx,
                            hwaddr addr, uint64_t val)
{
    Ipu4Syscom *sc = ctx->sc;
    uint32_t reg = (addr - IS_DMEM_BASE) / 4;

    if (addr == IS_DMEM_FW_COM_SEND_RD_POS) {
        /* The read path echoes SEND_WR_POS here; the driver is only
         * expected to *read* this slot. A write means our
         * "firmware-owned cursor, driver never writes" assumption is
         * wrong — the echo logic would then hide the driver's
         * intended value. */
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: driver wrote SEND_RD_POS val=0x%" PRIx64
                      " — model only echoes SEND_WR_POS on reads; "
                      "this write is lost.\n", val);
    }

    /* DMEM[1] = SYSCOM_CONFIG_REG (kernel/ipu4/ipu6-fw-com.c:101).
     * The driver writes the IPU IOVA of the syscom_config struct
     * here right before kicking cell_start. We re-latch the cached
     * queue descriptors on every write so a fresh ipu6_fw_com_open
     * call (e.g. after silent reset) starts clean. */
    if (reg == 1 /* SYSCOM_CONFIG_REG */) {
        sc->config_iova = val;
        memset(sc->send_q_loaded, 0, sizeof(sc->send_q_loaded));
        sc->recv_q_loaded = false;
        memset(sc->send_wr_seen, 0, sizeof(sc->send_wr_seen));
    }

    /* DMEM[2] = SYSCOM_STATE_REG. Driver writes UNINIT before polling
     * for READY. Treat it as the firmware-init reset marker for the
     * syscom layer — clear the wr-cursor cache so a re-open without
     * a config rewrite still tracks the fresh ring correctly. */
    if (reg == 2 /* SYSCOM_STATE_REG */ &&
        val == SYSCOM_STATE_UNINIT) {
        memset(sc->send_wr_seen, 0, sizeof(sc->send_wr_seen));
    }

    ctx->isys->dmem[reg] = val;

    /* Detect a wr-cursor bump on any of the 8 msg-send queues and
     * synthesise the matching FW response. The msg-send queues live
     * at DMEM regs 10, 12, 14, … 24 (= absolute input queue indices
     * 2..9, see comment in ipu4-fw-isys.h near
     * IPU4_BASE_MSG_SEND_QUEUES). */
    if (reg >= 10 && reg < 10 + IPU4_MAX_MSG_STREAMS * 2 &&
        ((reg - 10) & 1) == 0) {
        unsigned int stream = (reg - 10) / 2;
        handle_send_bump(ctx, stream, val);
    }
}
