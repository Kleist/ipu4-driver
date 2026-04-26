/*
 * Intel IPU4 (0x8086:0x5a88) emulated PCI device.
 *
 * This is a skeleton model. It advertises the right VID/DID and a 16 MB
 * BAR0, responds to a small set of buttress registers with the minimum
 * behavior required to keep ipu6_pci_probe() moving forward, and logs
 * every unknown MMIO access so the M3 fuzzing loop has a target list.
 *
 * Register semantics are documented in tools/notes/registers.md as they
 * are inferred from the driver. Do not extend this file without adding
 * a note there explaining the new behavior.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msi.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

#include "ipu4-fw-isys.h"
#include "ipu4-mmu.h"
#include "ipu4-buttress.h"
#include "ipu4-csi2.h"
#include "ipu4-isys.h"

#define TYPE_IPU4 "ipu4"
OBJECT_DECLARE_SIMPLE_TYPE(Ipu4State, IPU4)

#define IPU4_PCI_VENDOR_ID  0x8086
#define IPU4_PCI_DEVICE_ID  0x5a88
#define IPU4_BAR_SIZE       (16 * MiB)

/* Buttress block (BTRS_*, IPC echo, PWR_STATE, TSC, FW reset/source,
 * ISR W1C) lives in ipu4-buttress.{c,h}. */

/* ISYS unispart IRQ block, ISYS/PSYS SPC status-ctrl latches, and
 * the ISYS DMEM syscom window live in ipu4-isys.{c,h}. */

/* CSI2 port 0 + the unmodelled-port log range live in ipu4-csi2.{c,h}. */

/* SYSCOM_STATE values + protocol queue layout live in ipu4-fw-isys.h. */

/* ISYS / PSYS MMU windows + IOVA walker live in ipu4-mmu.{c,h}. */

/* IPU4 SP→host syscom delivery raises both the buttress IS-side IRQ
 * (via ipu4_buttress_signal_is_irq) and the ISYS unispart SW IRQ bit
 * (via ipu4_isys_signal_sw_irq) before the MSI fires. The driver's
 * ipu4_isys_isr matches the unispart bit to decide there's a FW
 * software event to drain (ipu6-isys.c:375). */

/* FW protocol opcodes, response/queue/token struct shapes and the
 * IPU4_BASE_MSG_SEND_QUEUES / IPU4_MAX_MSG_STREAMS / SYSCOM_QPR_BASE_REG
 * constants live in ipu4-fw-isys.h. */

struct Ipu4State {
    PCIDevice parent_obj;
    MemoryRegion bar0;

    /* Buttress block (BTRS_*, IPC echo, PWR_STATE, TSC, ISR). */
    Ipu4Buttress buttress;

    /* ISYS subsystem (unispart IRQ, ISYS+PSYS SPC, DMEM syscom window). */
    Ipu4Isys isys;

    /* CSI2 receiver block (port 0 + unmodelled-port log range). */
    Ipu4Csi2 csi2;

    /* ISYS / PSYS MMU page-table programming windows + ISYS MMU L1
     * page-table base pfn used by the IOVA walker. */
    Ipu4MmuRegs mmu;

    /* IPU IOVA of the syscom config struct, latched from a DMEM[1]
     * (SYSCOM_CONFIG_REG) write. Subsequent send-queue activity
     * triggers a full read of the config + queue descriptors. */
    uint32_t syscom_config_iova;

    /* Cached msg-send queue descriptors per stream (loaded lazily on
     * first send-cursor write so the cache picks up the descriptor
     * after the driver finished initialising it). is_send_q_loaded
     * stays false until both the config addr is known and a stream's
     * descriptor was successfully read; loaded state is per-stream so
     * a partial setup doesn't poison the others. */
    Ipu4FwSysQueue is_send_q[IPU4_MAX_MSG_STREAMS];
    bool is_send_q_loaded[IPU4_MAX_MSG_STREAMS];

    /* Cached msg-recv queue descriptor (single output queue at index
     * IPU4_BASE_MSG_RECV_QUEUE_INDEX of the absolute queue table). */
    Ipu4FwSysQueue is_recv_q;
    bool is_recv_q_loaded;

    /* Last-seen wr cursor per msg-send queue. Each driver-side bump
     * pushes (new_wr - prev) % q->size tokens; we read each new
     * token's send_type out of the input ring and dispatch on that
     * (Step 3 — counting alone doesn't survive STREAM_CAPTURE
     * interleaving between STREAM_START and STREAM_FLUSH). */
    uint32_t is_send_wr_seen[IPU4_MAX_MSG_STREAMS];
};

/* IOVA walker + DMA helpers live in ipu4-mmu.{c,h}. The Step-4 frame
 * pattern bound is local to the syscom frame-delivery path here. */
#define IPU4_FRAME_PATTERN_BYTES (64 * 1024)

/* Read the `i`-th queue descriptor from the IOVA-addressed queue array
 * pointed to by `array_iova`. */
static bool ipu4_load_queue_desc(Ipu4State *s, uint32_t array_iova,
                                 unsigned int i, Ipu4FwSysQueue *out)
{
    return ipu4_mmu_dma_read_iova(&s->mmu, &s->parent_obj,
                                  array_iova + i * sizeof(Ipu4FwSysQueue),
                                  out, sizeof(*out));
}

/* Lazy lookup of the msg-send queue descriptor for stream `stream`. Returns
 * a pointer into the cache, or NULL if the descriptor isn't loadable yet
 * (no syscom config addr written, MMU not programmed, etc.).
 */
static Ipu4FwSysQueue *ipu4_get_send_queue(Ipu4State *s, unsigned int stream)
{
    Ipu4FwSyscomConfig cfg;
    unsigned int q_index;

    if (stream >= IPU4_MAX_MSG_STREAMS) {
        return NULL;
    }
    if (s->is_send_q_loaded[stream]) {
        return &s->is_send_q[stream];
    }
    if (s->syscom_config_iova == 0) {
        return NULL;
    }
    if (!ipu4_mmu_dma_read_iova(&s->mmu, &s->parent_obj,
                                s->syscom_config_iova, &cfg, sizeof(cfg))) {
        return NULL;
    }

    q_index = IPU4_BASE_MSG_SEND_QUEUES + stream;
    if (!ipu4_load_queue_desc(s, cfg.input_queue, q_index,
                              &s->is_send_q[stream])) {
        return NULL;
    }
    s->is_send_q_loaded[stream] = true;
    s->is_send_wr_seen[stream] = 0;
    return &s->is_send_q[stream];
}

/* Lazy lookup of the single msg-recv queue (output queue index 1 in the
 * absolute table = output_queue array index 1).
 */
static Ipu4FwSysQueue *ipu4_get_recv_queue(Ipu4State *s)
{
    Ipu4FwSyscomConfig cfg;

    if (s->is_recv_q_loaded) {
        return &s->is_recv_q;
    }
    if (s->syscom_config_iova == 0) {
        return NULL;
    }
    if (!ipu4_mmu_dma_read_iova(&s->mmu, &s->parent_obj,
                                s->syscom_config_iova, &cfg, sizeof(cfg))) {
        return NULL;
    }
    /* output_queue[1] is the msg-recv slot; output_queue[0] is proxy. */
    if (!ipu4_load_queue_desc(s, cfg.output_queue, 1, &s->is_recv_q)) {
        return NULL;
    }
    s->is_recv_q_loaded = true;
    return &s->is_recv_q;
}

/* Post a pre-filled response token on the msg-recv queue and raise
 * the IS-side MSI so the driver's ISR drains it. Caller fills `resp`
 * before calling — at minimum `type` and `stream_handle`; pin and
 * timestamp fields stay zeroed for the simple ack types and get
 * populated for PIN_DATA_READY. The response is DMA-written into the
 * recv-queue's host backing (translated through the IPU MMU), the
 * recv WR cursor in DMEM is bumped, and both buttress + unispart IRQ
 * status bits are set so `ipu6_buttress_isr` → `ipu4_isys_isr` →
 * `isys_isr_one` traverses the same path silicon would.
 */
static void ipu4_post_response_full(Ipu4State *s,
                                    const Ipu4FwIsysRespInfo *resp)
{
    Ipu4FwSysQueue *rq = ipu4_get_recv_queue(s);
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
     * caches the wr/rd cursors in DMEM at q->wr_reg*4 / q->rd_reg*4.
     */
    if (rq->wr_reg >= IS_DMEM_SIZE / 4) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: recv queue wr_reg=%u out of DMEM range\n",
                      rq->wr_reg);
        return;
    }
    recv_wr_slot = &s->isys.dmem[rq->wr_reg];
    wr = *recv_wr_slot;
    if (wr >= rq->size) {
        wr = 0;
    }

    /* DMA the response into the slot. If the IOVA can't be walked
     * (MMU not programmed), bail out — the driver will time out, but
     * better that than corrupting an unrelated guest page. */
    if (!ipu4_mmu_dma_write_iova(&s->mmu, &s->parent_obj,
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
     * ipu4_isys_isr → isys_isr_one to drain the response. msi_init()
     * was called in realize() and pci_alloc_irq_vectors() runs at
     * driver probe, so msi_enabled() is reliably true by the time
     * the first response posts. */
    ipu4_isys_signal_sw_irq(&s->isys);
    ipu4_buttress_signal_is_irq(&s->buttress);

    if (msi_enabled(&s->parent_obj)) {
        msi_notify(&s->parent_obj, 0);
    }
}

/* Wrapper for simple ack types (OPEN_DONE, START_*_ACK, FLUSH_ACK,
 * CLOSE_ACK) that only need the type and stream_handle filled.
 * buf_handle stays 0 — the driver's ipu6_put_fw_msg_buf is a no-op
 * for buf_handle == 0 (kernel/ipu4/ipu6-isys.c:1185-1192).
 */
static void ipu4_post_response(Ipu4State *s, uint8_t resp_type,
                               uint8_t stream_handle)
{
    Ipu4FwIsysRespInfo resp = {
        .type = resp_type,
        .stream_handle = stream_handle,
    };
    ipu4_post_response_full(s, &resp);
}

/* Step-4: synthesise a PIN_DATA_READY response that the driver's
 * ipu6_isys_queue_buf_ready() (kernel/ipu4/ipu6-isys-queue.c:935)
 * matches against the buffer's IPU IOVA via `info->pin.addr`.
 * timestamp[] = 0 so get_sof_sequence_by_timestamp() falls into the
 * `time == 0` branch and returns `atomic_read(stream->sequence) - 1`,
 * which equals 0 after our preceding FRAME_SOF response increments
 * stream->sequence from 0 to 1 — that's the value the streamon-smoke
 * pattern check (k + seq) was written for.
 */
static void ipu4_post_pin_data_ready(Ipu4State *s, uint8_t stream_handle,
                                     uint8_t pin_id, uint32_t pin_addr,
                                     uint64_t pin_buf_id)
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
    ipu4_post_response_full(s, &resp);
}

/* Step-4: deliver a frame for every non-zero output pin in the
 * `frame_buff_set` payload that the driver attached to a
 * STREAM_START_AND_CAPTURE / STREAM_CAPTURE token. For each pin we
 * (a) DMA-write IPU4_FRAME_PATTERN_BYTES of the deterministic pattern
 *     `byte[k] = (k + seq) & 0xff` (seq=0 — see ipu4_post_pin_data_ready)
 *     into the IOVA-mapped buffer at output_pins[i].addr,
 * (b) post a FRAME_SOF response to bump stream->sequence (so
 *     get_sof_sequence_by_timestamp() returns 0 for the first frame),
 * (c) post a PIN_DATA_READY response that the driver matches against
 *     the buffer's IOVA.
 *
 * The frame_buff_set layout (kernel/ipu4/ipu6-fw-isys.h:688) starts
 * with output_pins[IPU4_MAX_OPINS=6], each 16 bytes packed —
 * Ipu4FwOutputPin in this file. We only read the first 96 bytes;
 * process_group_light + the trailing u8 flags don't gate frame
 * delivery in our model. IPU4_MAX_OPINS lives in ipu4-fw-isys.h.
 */

static void ipu4_deliver_frame(Ipu4State *s, uint8_t stream,
                               uint32_t frame_buff_set_iova)
{
    Ipu4FwOutputPin pins[IPU4_MAX_OPINS];
    unsigned int i;

    if (!ipu4_mmu_dma_read_iova(&s->mmu, &s->parent_obj,
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
        ipu4_post_response_full(s, &sof);
    }

    for (i = 0; i < IPU4_MAX_OPINS; i++) {
        if (pins[i].addr == 0) {
            continue;
        }
        ipu4_mmu_dma_write_iova_pattern(&s->mmu, &s->parent_obj,
                                        pins[i].addr,
                                        IPU4_FRAME_PATTERN_BYTES, 0);
        ipu4_post_pin_data_ready(s, stream, (uint8_t)i,
                                 pins[i].addr, pins[i].out_buf_id);
    }
}

/* Detect a driver-side bump of the wr cursor for msg-send queue `stream`
 * and synthesise a response per the protocol's request/ack pairing.
 *
 * The driver places a token at slot wr-1 (mod size) and then bumps the
 * WR cursor — see ipu6_send_get_token / ipu6_send_put_token in
 * kernel/ipu4/ipu6-fw-com.c:331-369. We DMA-read each pushed token's
 * 16-byte header out of the input ring (vied_address + slot * 16) and
 * dispatch on its send_type (Ipu4FwSendToken.send_type at byte 12).
 */
static void ipu4_handle_send_bump(Ipu4State *s, unsigned int stream,
                                  uint32_t new_wr)
{
    Ipu4FwSysQueue *q = ipu4_get_send_queue(s, stream);
    uint32_t prev = s->is_send_wr_seen[stream];
    uint32_t pushed, slot;

    if (!q || q->size == 0) {
        /* Without the descriptor we can't tell wrap from increment;
         * still record the cursor so subsequent bumps are tracked
         * once the descriptor lands. */
        s->is_send_wr_seen[stream] = new_wr;
        return;
    }
    if (new_wr == prev) {
        return;
    }
    pushed = (new_wr + q->size - prev) % q->size;
    slot = prev;
    s->is_send_wr_seen[stream] = new_wr;

    while (pushed-- > 0) {
        Ipu4FwSendToken token;

        if (q->token_size < sizeof(token) ||
            !ipu4_mmu_dma_read_iova(&s->mmu, &s->parent_obj,
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
            ipu4_post_response(s, IPU4_FW_RESP_STREAM_OPEN_DONE,
                               (uint8_t)stream);
            break;
        case IPU4_FW_SEND_STREAM_START:
            /* No buffer attached — STREAM_START is the bufless variant,
             * so just complete stream_start_completion. */
            ipu4_post_response(s, IPU4_FW_RESP_STREAM_START_ACK,
                               (uint8_t)stream);
            break;
        case IPU4_FW_SEND_STREAM_START_AND_CAPTURE:
            /* Bundle: ack the start completion AND deliver the frame
             * for the buffer attached via token.payload. The driver's
             * isys_isr_one drains responses one at a time per IRQ, so
             * the ordering doesn't matter — both fire before
             * start_stream_firmware's wait_for_completion_timeout
             * returns to streamon. */
            ipu4_post_response(s,
                               IPU4_FW_RESP_STREAM_START_AND_CAPTURE_ACK,
                               (uint8_t)stream);
            ipu4_deliver_frame(s, (uint8_t)stream, token.payload);
            break;
        case IPU4_FW_SEND_STREAM_CAPTURE:
            /* Step 4: deliver the frame attached to this token. The
             * driver doesn't wait on a per-CAPTURE completion (the
             * only acks the streamon path waits on are STREAM_OPEN
             * and STREAM_*_START), so a single PIN_DATA_READY per
             * non-zero output_pins[i].addr is sufficient. */
            ipu4_deliver_frame(s, (uint8_t)stream, token.payload);
            break;
        case IPU4_FW_SEND_STREAM_FLUSH:
            /* Driver completes stream_stop_completion
             * (ipu6-isys.c:1452-1453). Without this ack, every
             * STREAMOFF logs "stream stop time out". */
            ipu4_post_response(s, IPU4_FW_RESP_STREAM_FLUSH_ACK,
                               (uint8_t)stream);
            break;
        case IPU4_FW_SEND_STREAM_CLOSE:
            /* Driver completes stream_close_completion
             * (ipu6-isys.c:1438-1439). After CLOSE_ACK the driver
             * may re-STREAMON the same stream handle; the input
             * queue's wr/rd cursors keep going (firmware-com layer
             * doesn't reset them on close), so our wr_seen tracker
             * naturally follows. No per-stream state to reset. */
            ipu4_post_response(s, IPU4_FW_RESP_STREAM_CLOSE_ACK,
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

static uint64_t ipu4_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    Ipu4State *s = IPU4(opaque);
    uint64_t val = 0;

    if (ipu4_buttress_mmio_read(&s->buttress, addr, &val)) {
        return val;
    }
    if (ipu4_isys_mmio_read(&s->isys, addr, &val)) {
        return val;
    }
    if (ipu4_mmu_mmio_read(&s->mmu, addr, &val)) {
        return val;
    }
    if (ipu4_csi2_mmio_read(&s->csi2, addr, &val)) {
        return val;
    }

    qemu_log_mask(LOG_UNIMP,
                  "ipu4: read unimpl +0x%06" HWADDR_PRIx
                  " size=%u\n", addr, size);
    return 0;
}

static void ipu4_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    Ipu4State *s = IPU4(opaque);

    if (ipu4_buttress_mmio_write(&s->buttress, addr, val)) {
        return;
    }
    if (addr >= IS_DMEM_BASE && addr < IS_DMEM_BASE + IS_DMEM_SIZE) {
        uint32_t reg = (addr - IS_DMEM_BASE) / 4;

        if (addr == IS_DMEM_FW_COM_SEND_RD_POS) {
            /* The read path echoes SEND_WR_POS here; the driver is
             * only expected to *read* this slot. A write means our
             * "firmware-owned cursor, driver never writes"
             * assumption is wrong — the echo logic would then hide
             * the driver's intended value. */
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: driver wrote SEND_RD_POS val=0x%"
                          PRIx64 " — model only echoes SEND_WR_POS "
                          "on reads; this write is lost.\n", val);
        }

        /* DMEM[1] = SYSCOM_CONFIG_REG (kernel/ipu4/ipu6-fw-com.c:101).
         * The driver writes the IPU IOVA of the syscom_config struct
         * here right before kicking cell_start. We re-latch the
         * cached queue descriptors on every write so a fresh
         * ipu6_fw_com_open call (e.g. after silent reset) starts
         * clean. */
        if (reg == 1 /* SYSCOM_CONFIG_REG */) {
            s->syscom_config_iova = val;
            memset(s->is_send_q_loaded, 0, sizeof(s->is_send_q_loaded));
            s->is_recv_q_loaded = false;
            memset(s->is_send_wr_seen, 0, sizeof(s->is_send_wr_seen));
        }

        /* DMEM[2] = SYSCOM_STATE_REG. Driver writes UNINIT before
         * polling for READY. Treat it as the firmware-init reset
         * marker for the syscom layer — clear the wr-cursor cache
         * so a re-open without a config rewrite still tracks the
         * fresh ring correctly. */
        if (reg == 2 /* SYSCOM_STATE_REG */ &&
            val == SYSCOM_STATE_UNINIT) {
            memset(s->is_send_wr_seen, 0, sizeof(s->is_send_wr_seen));
        }

        s->isys.dmem[reg] = val;

        /* Detect a wr-cursor bump on any of the 8 msg-send queues
         * and synthesise the matching FW response. The msg-send
         * queues live at DMEM regs 10, 12, 14, … 24 (= absolute
         * input queue indices 2..9, see comment above near
         * IPU4_BASE_MSG_SEND_QUEUES). */
        if (reg >= 10 && reg < 10 + IPU4_MAX_MSG_STREAMS * 2 &&
            ((reg - 10) & 1) == 0) {
            unsigned int stream = (reg - 10) / 2;
            ipu4_handle_send_bump(s, stream, val);
        }
        return;
    }
    if (ipu4_isys_mmio_write(&s->isys, addr, val)) {
        return;
    }
    if (ipu4_mmu_mmio_write(&s->mmu, addr, val)) {
        return;
    }
    if (ipu4_csi2_mmio_write(&s->csi2, addr, val)) {
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                  "ipu4: write unimpl +0x%06" HWADDR_PRIx
                  " val=0x%" PRIx64 " size=%u\n",
                  addr, val, size);
}

static const MemoryRegionOps ipu4_mmio_ops = {
    .read = ipu4_mmio_read,
    .write = ipu4_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void ipu4_realize(PCIDevice *pdev, Error **errp)
{
    Ipu4State *s = IPU4(pdev);

    pci_config_set_interrupt_pin(pdev->config, 1);

    /* The driver does pci_alloc_irq_vectors(dev, 1, 1, PCI_IRQ_MSI).
     * Expose 1 MSI vector; the IPU6EP branch in ipu6.c also calls
     * pci_disable_msi() first, so the capability must be present even
     * when the device only uses INTx at runtime. */
    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    memory_region_init_io(&s->bar0, OBJECT(s), &ipu4_mmio_ops, s,
                          "ipu4-bar0", IPU4_BAR_SIZE);
    pci_register_bar(pdev, 0,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &s->bar0);
}

static void ipu4_exit(PCIDevice *pdev)
{
    msi_uninit(pdev);
}

static void ipu4_reset(DeviceState *dev)
{
    Ipu4State *s = IPU4(dev);

    ipu4_buttress_reset(&s->buttress);
    ipu4_isys_reset(&s->isys);
    ipu4_csi2_reset(&s->csi2);
    ipu4_mmu_reset(&s->mmu);
    s->syscom_config_iova = 0;
    memset(s->is_send_q, 0, sizeof(s->is_send_q));
    memset(s->is_send_q_loaded, 0, sizeof(s->is_send_q_loaded));
    memset(&s->is_recv_q, 0, sizeof(s->is_recv_q));
    s->is_recv_q_loaded = false;
    memset(s->is_send_wr_seen, 0, sizeof(s->is_send_wr_seen));
}

static const VMStateDescription vmstate_ipu4 = {
    .name = "ipu4",
    .version_id = 13,
    .minimum_version_id = 13,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, Ipu4State),
        VMSTATE_STRUCT(buttress, Ipu4State, 0, vmstate_ipu4_buttress,
                       Ipu4Buttress),
        VMSTATE_STRUCT(isys, Ipu4State, 0, vmstate_ipu4_isys, Ipu4Isys),
        VMSTATE_STRUCT(csi2, Ipu4State, 0, vmstate_ipu4_csi2, Ipu4Csi2),
        VMSTATE_STRUCT(mmu, Ipu4State, 0, vmstate_ipu4_mmu, Ipu4MmuRegs),
        VMSTATE_UINT32_V(syscom_config_iova, Ipu4State, 8),
        VMSTATE_UINT32_ARRAY_V(is_send_wr_seen, Ipu4State,
                               IPU4_MAX_MSG_STREAMS, 8),
        VMSTATE_END_OF_LIST()
    }
};

static void ipu4_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = ipu4_realize;
    k->exit = ipu4_exit;
    k->vendor_id = IPU4_PCI_VENDOR_ID;
    k->device_id = IPU4_PCI_DEVICE_ID;
    k->revision = 0x00;
    k->class_id = PCI_CLASS_MULTIMEDIA_OTHER;

    dc->desc = "Intel IPU4 (emulated)";
    dc->vmsd = &vmstate_ipu4;
    dc->reset = ipu4_reset;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo ipu4_info = {
    .name = TYPE_IPU4,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(Ipu4State),
    .class_init = ipu4_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        {}
    },
};

static void ipu4_register_types(void)
{
    type_register_static(&ipu4_info);
}

type_init(ipu4_register_types)
