/*
 * Intel IPU4 firmware/syscom protocol types — opcodes, response struct
 * shapes, and queue-layout constants shared by the IPU4 device model.
 *
 * Mirrors the in-driver definitions in kernel/ipu4/ipu6-fw-isys.h and
 * ipu6-fw-com.h (the IPU4 fork preserves the IPU6 names). This header
 * is the QEMU-side counterpart: when the on-the-wire format changes on
 * the kernel side, update the matching definitions here.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef HW_MISC_IPU4_FW_ISYS_H
#define HW_MISC_IPU4_FW_ISYS_H

/* This header relies on the including .c file having already included
 * "qemu/osdep.h" — QEMU style forbids osdep.h from .h files. osdep.h
 * brings in <stdint.h> for uint*_t and the QEMU_PACKED /
 * QEMU_BUILD_BUG_ON macros used below. */

/* SYSCOM_STATE values (kernel/ipu4/ipu6-fw-com.c:44). The driver
 * writes UNINIT and polls SYSCOM_STATE until it reads READY before
 * proceeding past `ipu6_fw_com_open()` / `ipu6_fw_com_ready()`. The
 * model returns READY unconditionally on read so the poll completes
 * on the first iteration — the real SPC handshake isn't simulated.
 */
#define SYSCOM_STATE_UNINIT              0x57A7E000
#define SYSCOM_STATE_READY               0x57A7E001
#define SYSCOM_STATE_INACTIVE            0x57A7E002

/* Syscom queue layout (kernel/ipu4/ipu6-fw-com.c:101 + ipu6-fw-isys.h):
 *   reg 0..5  management slots (PKG_DIR, SYSCOM_CONFIG, …, VTL0)
 *   reg 6+    pairs of (wr_reg, rd_reg) per queue, in input-then-output order
 * IPU4 has 1 proxy + 1 dev + 8 msg input queues = queue indices 0..9.
 * The msg-send queue for stream N is at index IPU4_BASE_MSG_SEND_QUEUES + N
 * = 2 + N, occupying register pair (10+2N, 11+2N) — DMEM byte offsets
 * (0x028 + 8N, 0x02c + 8N). The single msg-recv queue follows the input
 * block at index 11 (1 proxy + 0 dev + 1 msg = output indices 0..1, with
 * the msg-recv at output index 1). Mapped as input.10/11 = wr/rd of the
 * 11th queue overall = absolute reg 28/29 (DMEM 0x070/0x074), matching
 * the IS_DMEM_FW_COM_RECV_*_POS macros in the device model.
 */
#define IPU4_BASE_MSG_SEND_QUEUES        2
#define IPU4_BASE_MSG_RECV_QUEUE_INDEX   11   /* in the absolute queue table */
#define IPU4_MAX_MSG_STREAMS             8
#define SYSCOM_QPR_BASE_REG              6

/* Per-stream, per-direction queue counts the syscom layer needs to
 * size its caches (proxy + dev + msg inputs; proxy + msg outputs). */
#define IPU4_NUM_INPUT_QUEUES  (1 + 1 + IPU4_MAX_MSG_STREAMS)
#define IPU4_NUM_OUTPUT_QUEUES (1 + 1)

/* IPU6 ISYS firmware send-token opcodes (kernel/ipu4/ipu6-fw-isys.h:131).
 * The driver's call sequence is well-defined per stream lifecycle but
 * STREAM_CAPTUREs interleave freely between STREAM_START and
 * STREAM_FLUSH (ipu6_isys_stream_start() drains any pre-queued buffers
 * after start_stream_firmware returns; buf_queue() pushes a CAPTURE
 * for every fresh QBUF), so we read the token's send_type directly
 * out of the input ring instead of counting bumps. The token is
 * `struct ipu6_fw_send_queue_token { u64 buf_handle; u32 payload;
 * u16 send_type; u16 stream_id; }` — 16 bytes packed
 * (ipu6-fw-isys.h:806).
 */
enum {
    IPU4_FW_SEND_STREAM_OPEN              = 0,
    IPU4_FW_SEND_STREAM_START             = 1,
    IPU4_FW_SEND_STREAM_START_AND_CAPTURE = 2,
    IPU4_FW_SEND_STREAM_CAPTURE           = 3,
    IPU4_FW_SEND_STREAM_STOP              = 4,
    IPU4_FW_SEND_STREAM_FLUSH             = 5,
    IPU4_FW_SEND_STREAM_CLOSE             = 6,
};

enum {
    IPU4_FW_RESP_STREAM_OPEN_DONE              = 0,
    IPU4_FW_RESP_STREAM_START_ACK              = 1,
    IPU4_FW_RESP_STREAM_START_AND_CAPTURE_ACK  = 2,
    IPU4_FW_RESP_STREAM_CAPTURE_ACK            = 3,
    IPU4_FW_RESP_STREAM_STOP_ACK               = 4,
    IPU4_FW_RESP_STREAM_FLUSH_ACK              = 5,
    IPU4_FW_RESP_STREAM_CLOSE_ACK              = 6,
    IPU4_FW_RESP_STREAM_PIN_DATA_READY         = 7,
    IPU4_FW_RESP_FRAME_SOF                     = 9,
};

/* IPU6 FW response struct sizes — verified against the IPU4 (non-IPU6)
 * branch in kernel/ipu4/ipu6-fw-isys.h:746-758. Used to lay out the
 * response token we DMA into the recv ring. `__attribute__((packed))`
 * isn't strictly required by the driver but keeps the layout robust if
 * QEMU is ever built with an unusual default alignment.
 */
typedef struct QEMU_PACKED Ipu4FwOutputPin {
    uint64_t out_buf_id;
    uint32_t addr;
    uint32_t compress;
} Ipu4FwOutputPin;

typedef struct QEMU_PACKED Ipu4FwParamPin {
    uint64_t param_buf_id;
    uint32_t addr;
    uint32_t _pad;
} Ipu4FwParamPin;

typedef struct QEMU_PACKED Ipu4FwErrorInfo {
    uint32_t error;
    uint32_t error_details;
} Ipu4FwErrorInfo;

typedef struct QEMU_PACKED Ipu4FwIsysRespInfo {
    uint64_t buf_handle;
    Ipu4FwOutputPin pin;
    Ipu4FwParamPin process_group_light;
    Ipu4FwErrorInfo error_info;
    uint32_t timestamp[2];
    uint8_t stream_handle;
    uint8_t type;
    uint8_t pin_id;
    uint8_t acc_id;
    uint8_t frame_counter;
    uint8_t written_direct;
    uint8_t _pad[2];   /* round to 64 bytes for token_size alignment */
} Ipu4FwIsysRespInfo;

QEMU_BUILD_BUG_ON(sizeof(Ipu4FwIsysRespInfo) != 64);

/* Mirror of ipu6_fw_syscom_config (kernel/ipu4/ipu6-fw-com.c:61). The
 * driver writes the IPU IOVA of an instance of this struct into
 * DMEM[1] (SYSCOM_CONFIG_REG) before kicking cell_start; we walk the
 * IPU MMU page tables to translate the IOVA, then read the struct.
 */
typedef struct QEMU_PACKED Ipu4FwSyscomConfig {
    uint32_t firmware_address;
    uint32_t num_input_queues;
    uint32_t num_output_queues;
    uint32_t input_queue;     /* IOVA of an array of Ipu4FwSysQueue */
    uint32_t output_queue;    /* IOVA of an array of Ipu4FwSysQueue */
    uint32_t specific_addr;
    uint32_t specific_size;
} Ipu4FwSyscomConfig;

/* Mirror of ipu6_fw_sys_queue (kernel/ipu4/ipu6-fw-com.c:26). */
typedef struct QEMU_PACKED Ipu4FwSysQueue {
    uint64_t host_address;    /* kernel virtual; not used by the model */
    uint32_t vied_address;    /* IPU IOVA of the queue's token storage */
    uint32_t size;            /* tokens per queue + 1 */
    uint32_t token_size;
    uint32_t wr_reg;          /* DMEM register index */
    uint32_t rd_reg;
    uint32_t _align;
} Ipu4FwSysQueue;

/* Mirror of ipu6_fw_send_queue_token (kernel/ipu4/ipu6-fw-isys.h:806).
 * Step 3 reads this off the input ring to dispatch by send_type
 * (FLUSH/CLOSE among others) — counting bumps doesn't work because
 * STREAM_CAPTUREs interleave between START and FLUSH. */
typedef struct QEMU_PACKED Ipu4FwSendToken {
    uint64_t buf_handle;
    uint32_t payload;
    uint16_t send_type;
    uint16_t stream_id;
} Ipu4FwSendToken;

QEMU_BUILD_BUG_ON(sizeof(Ipu4FwSendToken) != 16);

/* frame_buff_set layout (kernel/ipu4/ipu6-fw-isys.h:688) starts with
 * output_pins[IPU4_MAX_OPINS=6], each 16 bytes packed (Ipu4FwOutputPin).
 * The device model only reads the first 96 bytes; process_group_light
 * + the trailing u8 flags don't gate frame delivery. */
#define IPU4_MAX_OPINS 6

#endif /* HW_MISC_IPU4_FW_ISYS_H */
