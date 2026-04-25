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
#include "qemu/timer.h"
#include "qemu/units.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msi.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

#define TYPE_IPU4 "ipu4"
OBJECT_DECLARE_SIMPLE_TYPE(Ipu4State, IPU4)

#define IPU4_PCI_VENDOR_ID  0x8086
#define IPU4_PCI_DEVICE_ID  0x5a88
#define IPU4_BAR_SIZE       (16 * MiB)

/* Buttress registers (from kernel/ipu4/ipu6-platform-buttress-regs.h).
 * Offsets are from BAR0; the buttress block itself lives at BAR+0 in the
 * device model, which matches the way the driver derives its regs. */
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

#define BTRS_FW_RESET_CTL_START    BIT(0)
#define BTRS_FW_RESET_CTL_DONE     BIT(1)

/* ISYS unispart IRQ block. The driver derives the address as
 * ISYS_BASE (0x100000) + UNISPART_OFFSET (0x7c000) = 0x17c000 from
 * BAR0 (kernel/ipu4/ipu4-platform-regs.h:49 + ISYS base), and the
 * silicon trace confirms it. Same W1C / enable / level-select shape
 * as the buttress ISR trio:
 *
 *   STATUS         read returns `status & enable`
 *   CLEAR          write-1-to-clear bits of `status`
 *   EDGE/MASK/ENABLE/LEVEL_NOT_PULSE/SW_IRQ_MUX  latched R/W
 *   SW_IRQ         trigger register; the driver writes 0 on silicon
 *                  so the model treats it as a no-op for now.
 *
 * `status` stays zero under QEMU because we don't model the ISYS
 * hardware that normally raises these interrupts (frame generator,
 * CSI2 port events, …). That's a legitimate value_mismatch under
 * compare.py — STATUS reads 0 here, silicon cycles 0/0x40000000 as
 * real IRQs fire — but it's intrinsic to the lack of a backing
 * device, not a handler bug.
 */
#define IS_UNISPART_BASE                 0x17c000
#define IS_UNISPART_IRQ_EDGE             (IS_UNISPART_BASE + 0x000)
#define IS_UNISPART_IRQ_MASK             (IS_UNISPART_BASE + 0x004)
#define IS_UNISPART_IRQ_STATUS           (IS_UNISPART_BASE + 0x008)
#define IS_UNISPART_IRQ_CLEAR            (IS_UNISPART_BASE + 0x00c)
#define IS_UNISPART_IRQ_ENABLE           (IS_UNISPART_BASE + 0x010)
#define IS_UNISPART_IRQ_LEVEL_NOT_PULSE  (IS_UNISPART_BASE + 0x014)
#define IS_UNISPART_SW_IRQ               (IS_UNISPART_BASE + 0x414)
#define IS_UNISPART_SW_IRQ_MUX           (IS_UNISPART_BASE + 0x418)

/* CSI2 ports. Each of the 6 ports has the same register layout
 * (kernel/ipu4/ipu4-platform-isys-csi2-reg.h:19-56) but lives at an
 * irregular base offset (kernel/ipu4/ipu4-platform-isys-csi2-reg.h:13-15):
 *
 *     port 0 — 0x64000   port 3 — 0x67000
 *     port 1 — 0x65000   port 4 — 0x6c000
 *     port 2 — 0x66000   port 5 — 0x6c800
 *
 * Per-port register offsets:
 *
 *   0x000  CSI_RX_ENABLE             R/W latched, bit 0 = enable.
 *   0x004  CSI_RX_NOF_ENABLED_LANES  R/W latched.
 *   0x008  CSI_RX_CONFIG             R/W latched; driver reads back.
 *   0x02c/0x030  DLY_CNT_{TERMEN,SETTLE}_CLANE   R/W latched.
 *   0x034/0x038  DLY_CNT_{TERMEN,SETTLE}_DLANE(0) R/W latched.
 *   0x400-0x414  CSI2PART IRQ trio (W1C status, latched edge/mask/enable).
 *   0x500-0x514  CSI RX IRQ trio.
 *   0x600-0x614  S2M IRQ trio.
 *
 * Step-5b raises FS_VC0 in CSI2 PART_IRQ_STATUS for port 0 every time
 * the model delivers a frame; ports 1..5 stay at 0 status (no backing
 * frame source), which is how silicon's `data/trace.txt` looks for
 * them too — it only exercises port 0.
 */
#define CSI2_PORT_REG_RX_ENABLE                 0x000
#define CSI2_PORT_REG_RX_NOF_LANES              0x004
#define CSI2_PORT_REG_RX_CONFIG                 0x008
#define CSI2_PORT_REG_DLY_TERMEN_C              0x02c
#define CSI2_PORT_REG_DLY_SETTLE_C              0x030
#define CSI2_PORT_REG_DLY_TERMEN_D0             0x034
#define CSI2_PORT_REG_DLY_SETTLE_D0             0x038
#define CSI2_PORT_REG_PART_IRQ_EDGE             0x400
#define CSI2_PORT_REG_PART_IRQ_MASK             0x404
#define CSI2_PORT_REG_PART_IRQ_STATUS           0x408
#define CSI2_PORT_REG_PART_IRQ_CLEAR            0x40c
#define CSI2_PORT_REG_PART_IRQ_ENABLE           0x410
#define CSI2_PORT_REG_PART_IRQ_LEVEL_NOT_PULSE  0x414
#define CSI2_PORT_REG_RX_IRQ_EDGE               0x500
#define CSI2_PORT_REG_RX_IRQ_MASK               0x504
#define CSI2_PORT_REG_RX_IRQ_STATUS             0x508
#define CSI2_PORT_REG_RX_IRQ_CLEAR              0x50c
#define CSI2_PORT_REG_RX_IRQ_ENABLE             0x510
#define CSI2_PORT_REG_RX_IRQ_LEVEL_NOT_PULSE    0x514
#define CSI2_PORT_REG_S2M_IRQ_EDGE              0x600
#define CSI2_PORT_REG_S2M_IRQ_MASK              0x604
#define CSI2_PORT_REG_S2M_IRQ_STATUS            0x608
#define CSI2_PORT_REG_S2M_IRQ_CLEAR             0x60c
#define CSI2_PORT_REG_S2M_IRQ_ENABLE            0x610
#define CSI2_PORT_REG_S2M_IRQ_LEVEL_NOT_PULSE   0x614

#define IPU4_CSI2_NUM_PORTS                     6

/* ISYS DMEM syscom ring-pointer block at BAR+0x108000 (from
 * postprocess_trace.py's NAMED_REGIONS; the driver derives offsets
 * via FW_COM_WR_REG / FW_COM_RD_REG in kernel/ipu4/ipu6-fw-com.h).
 * The first 0x100 bytes are firmware-parameter + ring-pointer slots
 * the driver writes during FW bringup and polls during streaming.
 * We back the whole window with a flat array:
 *
 *   0x008 SYSCOM_STATE     latched
 *   0x028 SEND_WR_POS      latched
 *   0x02c SEND_RD_POS      echoes SEND_WR so the driver sees
 *                          "firmware consumed everything"
 *   0x070 RECV_WR_POS      stays at whatever driver last wrote
 *                          (0 initially). Silicon sees real firmware
 *                          updates; the model doesn't, so compare.py
 *                          will flag this as a value_mismatch until
 *                          firmware simulation lands.
 *   0x074 RECV_RD_POS      latched (driver-written ack cursor)
 *   others (0x00-0x7f)     firmware parameter slots; no readback,
 *                          plain latching is enough.
 */
#define IS_DMEM_BASE                     0x108000
#define IS_DMEM_SIZE                     0x100
#define IS_DMEM_SYSCOM_STATE             (IS_DMEM_BASE + 0x008)
#define IS_DMEM_FW_COM_SEND_WR_POS       (IS_DMEM_BASE + 0x028)
#define IS_DMEM_FW_COM_SEND_RD_POS       (IS_DMEM_BASE + 0x02c)
#define IS_DMEM_FW_COM_RECV_WR_POS       (IS_DMEM_BASE + 0x070)
#define IS_DMEM_FW_COM_RECV_RD_POS       (IS_DMEM_BASE + 0x074)

/* SYSCOM_STATE values (kernel/ipu4/ipu6-fw-com.c:44). The driver
 * writes UNINIT and polls SYSCOM_STATE until it reads READY before
 * proceeding past `ipu6_fw_com_open()` / `ipu6_fw_com_ready()`. The
 * model returns READY unconditionally on read so the poll completes
 * on the first iteration — the real SPC handshake isn't simulated.
 */
#define SYSCOM_STATE_UNINIT              0x57A7E000
#define SYSCOM_STATE_READY               0x57A7E001
#define SYSCOM_STATE_INACTIVE            0x57A7E002

/* SPC status/control register lives at offset 0 of each SPC region.
 * IPU4 maps ISYS SPC at BAR+0x100000 and PSYS SPC at BAR+0x400000
 * (kernel/ipu4/ipu4-platform-regs.h:26-27 + ipu4-platform-regs.h's
 * ISYS/PSYS_BASE). Bit map (kernel/ipu4/ipu6-platform-regs.h:81-87):
 *   BIT(1)  START
 *   BIT(3)  RUN
 *   BIT(5)  READY
 *   BIT(12) ICACHE_INVALIDATE
 *   BIT(13) ICACHE_PREFETCH
 * `query_sp()` waits for `(val & (READY | START)) == READY` — i.e.
 * READY set, START cleared. To make that test pass on the first
 * read the model latches whatever the driver writes but always
 * folds in READY and folds out START on read.
 */
#define IS_SPC_STATUS_CTRL               0x100000
#define PS_SPC_STATUS_CTRL               0x400000
#define SPC_STATUS_START                 (1u << 1)
#define SPC_STATUS_READY                 (1u << 5)

/* ISYS / PSYS MMU windows. postprocess_trace.py breaks each side into
 * sub-regions (isys0/isys1; psys0/psys1/psys2), but the driver treats
 * each side as a single page-table programming window, so we back them
 * with one flat array per side:
 *
 *   ISYS MMU   BAR+0x1e0000..0x1e04ff   (isys0 + isys1)
 *   PSYS MMU   BAR+0x4b0000..0x4b09ff   (psys0 + psys1 + psys2)
 *
 * data/trace.txt writes 208 entries into the ISYS side and 200 into
 * the PSYS side, all during FW bringup — page directory + L1/L2
 * entries + invalidate bits — and does not read any of them back in
 * this capture. Minimum viable handling: latch writes, return latched
 * value on reads. Real streaming (M5b) will need pci_dma_rw() off the
 * PDE writes; that's a later PR.
 */
#define IS_MMU_BASE                      0x1e0000
#define IS_MMU_SIZE                      0x500
#define PS_MMU_BASE                      0x4b0000
#define PS_MMU_SIZE                      0xa00

/* PWR_STATE FSM bits (kernel/ipu4/ipu4-platform-buttress-regs.h:11-22).
 * The driver polls PWR_STATE for ISYS/PSYS power transitions and a
 * one-time TSC-sync handshake. Step 5 models the per-power-island
 * state machine so `bus_pm_runtime_suspend` actually succeeds —
 * pre-Step-5 the model returned BTRS_PWR_STATE_PWR_RDY_ALL on every
 * read, so power-down polls timed out and the auto-suspend recovery
 * path stranded the auxiliary device with isys->power == 0.
 *
 *   bits [1:0]    PWR_RDY = 3       — buttress always ready
 *   bits [13:12]  HH_DONE = 2       — TSC-sync done
 *   bits [23:20]  IS_PWR_FSM
 *                   IDLE   = 0      — IS not powered
 *                   IS_RDY = 0xa    — IS powered up
 *   bits [28:24]  PS_PWR_FSM
 *                   IDLE      = 0
 *                   PS_PWR_UP = 0xf
 *
 * The driver triggers transitions by writing IS_FREQ_CTL / PS_FREQ_CTL
 * with BUTTRESS_FREQ_CTL_START (BIT(31)) set for power-on or zeroed
 * for power-off (kernel/ipu4/ipu6-buttress.c:466-505). The model
 * mirrors the START bit into a per-island powered flag and computes
 * PWR_STATE on read.
 */
#define BTRS_PWR_STATE_PWR_RDY            (3u << 0)
#define BTRS_PWR_STATE_HH_DONE            (2u << 12)
#define BTRS_PWR_STATE_IS_RDY             (0xau << 20)
#define BTRS_PWR_STATE_PS_PWR_UP          (0xfu << 24)
#define BTRS_FREQ_CTL_START               (1u << 31)

/* CSE↔IU IPC CSR token bits (kernel/ipu4/ipu6-platform-buttress-regs.h:184).
 * The driver uses CSR_OUT (IU2CSECSR=0x108) to signal phase tokens to
 * the CSE side and CSR_IN (CSE2IUCSR=0x30c) as the peer-ack channel
 * (W1C-cleared by the driver). The model mirrors a minimal subset
 * sufficient for ipu6_buttress_ipc_reset() to terminate. */
#define BUTTRESS_IU2CSECSR_ENTRY                       (1u << 0)
#define BUTTRESS_IU2CSECSR_EXIT                        (1u << 1)
#define BUTTRESS_IU2CSECSR_ASSERTED_REG_VALID_REQ      (1u << 3)
#define BUTTRESS_IU2CSECSR_ACKED_REG_VALID             (1u << 4)

/* IPU4 ISYS MMU sub-block 0 lives at BAR + ISYS_OFFSET (0x100000) +
 * IOMMU0_OFFSET (0xe0000) = 0x1e0000 (kernel/ipu4/ipu4-platform-regs.h).
 * The L1 page-table physical pfn is programmed at sub-block + 0x004
 * (REG_L1_PHYS in kernel/ipu4/ipu6-mmu.c:51). All sub-blocks within a
 * side mirror the same L1 root, so latching the first write is enough
 * to walk the table from QEMU.
 *
 * Step-2 firmware responder uses this latch to translate the IPU IOVA
 * stored in the syscom config (DMEM[1]) and the queue descriptor table
 * into host physical addresses for `pci_dma_read` / `pci_dma_write`.
 */
#define IS_MMU_SUB0_REG_L1_PHYS          (IS_MMU_BASE + 0x004)
#define ISP_PADDR_SHIFT                  12
#define ISP_PAGE_MASK                    0xfffu
#define ISP_L1PT_SHIFT                   22
#define ISP_L2PT_SHIFT                   12
#define ISP_L2PT_MASK                    0x3ffu

/* IPU4 SP→host syscom delivery uses the buttress IS-side IRQ
 * (BTRS_REG_ISR_STATUS bit 0) to wake `ipu6_buttress_isr`, which
 * dispatches to `ipu4_isys_isr` (ipu6-isys.c:375). That handler reads
 * IS_UNISPART_IRQ_STATUS and matches BIT(30) to decide there's a FW
 * software event to drain. Both bits must therefore be set before
 * raising the MSI.
 */
#define BTRS_ISR_IS_IRQ                  (1u << 0)
#define ISYS_UNISPART_IRQ_SW             (1u << 30)

/* CSI2 port-N event bit in UNISPART_IRQ_STATUS. Driver dispatches to
 * `ipu6_isys_csi2_isr(&isys->csi2[N])` when bit `IPU4_ISYS_UNISPART_IRQ_CSI2(N)`
 * (kernel/ipu4/ipu4-platform-regs.h:33-37) is set in the masked status.
 * Step-5b fires CSI2 port-0 along with the SW-IRQ on every PIN_DATA_READY
 * delivery so the driver's ISR also reads CSI2_PART_IRQ_STATUS, exercising
 * the per-port SOF/EOF event path silicon takes per frame. */
#define ISYS_UNISPART_IRQ_CSI2_PORT0     (1u << 3)

/* CSI2 PART IRQ status bits (kernel/ipu4/ipu4-platform-isys-csi2-reg.h:42).
 * FS_VC(n) = 1 << (n*4); FE_VC(n) = 2 << (n*4). VC0 SOF is bit 0; that's
 * what the model sets to advertise a frame-start to the driver's
 * `ipu6_isys_csi2_isr`. */
#define CSI2_PART_IRQ_FS_VC0             (1u << 0)

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
 * the existing IS_DMEM_FW_COM_RECV_*_POS macros above.
 */
#define IPU4_BASE_MSG_SEND_QUEUES        2
#define IPU4_BASE_MSG_RECV_QUEUE_INDEX   11   /* in the absolute queue table */
#define IPU4_MAX_MSG_STREAMS             8
#define SYSCOM_QPR_BASE_REG              6

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

/* Per-stream, per-direction state the model needs after the FW is up. */
#define IPU4_NUM_INPUT_QUEUES  (1 + 1 + IPU4_MAX_MSG_STREAMS)   /* proxy + dev + msg */
#define IPU4_NUM_OUTPUT_QUEUES (1 + 1)                          /* proxy + msg */

typedef struct Ipu4Csi2Port {
    uint32_t rx_enable, rx_nof_lanes, rx_config;
    uint32_t dly_termen_c, dly_settle_c, dly_termen_d0, dly_settle_d0;
    uint32_t part_edge, part_mask, part_status, part_enable, part_level;
    uint32_t rx_edge, rx_mask, rx_status, rx_enable_irq, rx_level;
    uint32_t s2m_edge, s2m_mask, s2m_status, s2m_enable, s2m_level;
} Ipu4Csi2Port;

/* CSI2 port base offsets — irregular because ports 4 and 5 share a
 * 0x1000 block (ports 0..3 are spaced 0x1000 apart, ports 4 and 5
 * are spaced 0x800). Order matches IPU_CSI_PORT_NUM in
 * kernel/ipu4/ipu6-isys-csi2.c. */
static const hwaddr ipu4_csi2_port_base[IPU4_CSI2_NUM_PORTS] = {
    0x64000, 0x65000, 0x66000, 0x67000, 0x6c000, 0x6c800,
};

struct Ipu4State {
    PCIDevice parent_obj;
    MemoryRegion bar0;

    /* Latched registers. */
    uint32_t btrs_ctrl;
    uint32_t fw_reset_ctl;
    uint32_t is_freq_ctl;
    uint32_t ps_freq_ctl;
    /* Step-5 PWR_STATE FSM flags. Set to true when the driver writes
     * IS_FREQ_CTL / PS_FREQ_CTL with BUTTRESS_FREQ_CTL_START (BIT(31)),
     * cleared when it writes the same register without the bit (the
     * power-down path zeros the entire register). PWR_STATE reads
     * report bits[23:20] = IS_RDY when is_powered is true,
     * bits[28:24] = PS_PWR_UP when ps_powered is true.
     */
    bool is_powered;
    bool ps_powered;
    uint32_t fabric_cmd;
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

    /* ISYS unispart IRQ block. */
    uint32_t is_unispart_irq_edge;
    uint32_t is_unispart_irq_mask;
    uint32_t is_unispart_irq_status;
    uint32_t is_unispart_irq_enable;
    uint32_t is_unispart_irq_level_not_pulse;
    uint32_t is_unispart_sw_irq_mux;

    /* CSI2 ports 0..5. Each port has identical register layout
     * (Ipu4Csi2Port). Step 5 follow-up extends to all 6 ports —
     * pre-this, only port 0 was modelled and ports 1..5 fell through
     * to a LOG_UNIMP fallback. */
    Ipu4Csi2Port csi2[IPU4_CSI2_NUM_PORTS];

    /* ISYS DMEM syscom window (0x108000..0x1080ff). */
    uint32_t is_dmem[IS_DMEM_SIZE / 4];

    /* ISYS / PSYS SPC status-control latches. Reads return the
     * latched value with READY forced on / START forced off so
     * `query_sp()` succeeds on the first poll. */
    uint32_t is_spc_status_ctrl;
    uint32_t ps_spc_status_ctrl;

    /* ISYS / PSYS MMU page-table programming windows. */
    uint32_t is_mmu[IS_MMU_SIZE / 4];
    uint32_t ps_mmu[PS_MMU_SIZE / 4];

    /* ISYS MMU L1 page-table base pfn (latched from BAR+0x1e0004,
     * REG_L1_PHYS in kernel/ipu4/ipu6-mmu.c). Step-2 firmware
     * responder uses this to walk IOVA → host phys for syscom DMA.
     */
    uint32_t is_mmu_l1_pfn;

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

/* Walk the IPU MMU page tables to translate an IPU IOVA into a host
 * physical address suitable for `pci_dma_read` / `pci_dma_write`. The
 * format mirrors the in-driver walker (kernel/ipu4/ipu6-mmu.c):
 *
 *   IOVA[31:22] = l1_idx, IOVA[21:12] = l2_idx, IOVA[11:0] = offset
 *   l1_pt_phys     = is_mmu_l1_pfn << 12
 *   l2_pt_pfn      = *(u32 *)(l1_pt_phys + l1_idx*4)   (27-bit pfn)
 *   target_pfn     = *(u32 *)((l2_pt_pfn << 12) + l2_idx*4)
 *   target_phys    = (target_pfn << 12) + offset
 *
 * Returns false (and leaves *out_phys untouched) if the MMU base
 * hasn't been programmed yet or any walked entry is the all-zero
 * "dummy" pteval — that means the driver hasn't actually mapped this
 * IOVA, and any access would have faulted on real silicon.
 */
static bool ipu4_iova_to_phys(Ipu4State *s, uint32_t iova, hwaddr *out_phys)
{
    uint32_t l1_idx = (iova >> ISP_L1PT_SHIFT) & 0x3ff;
    uint32_t l2_idx = (iova >> ISP_L2PT_SHIFT) & ISP_L2PT_MASK;
    uint32_t offset = iova & ISP_PAGE_MASK;
    hwaddr l1_pt_phys, l2_pt_phys;
    uint32_t l2_pt_pfn = 0, target_pfn = 0;
    MemTxResult res;

    if (s->is_mmu_l1_pfn == 0) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: iova_to_phys(0x%x): L1_PHYS not programmed\n",
                      iova);
        return false;
    }

    l1_pt_phys = (hwaddr)s->is_mmu_l1_pfn << ISP_PADDR_SHIFT;
    res = pci_dma_read(&s->parent_obj, l1_pt_phys + (hwaddr)l1_idx * 4,
                       &l2_pt_pfn, 4);
    if (res != MEMTX_OK || l2_pt_pfn == 0) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: iova_to_phys(0x%x): L1 walk failed "
                      "(res=%d pfn=0x%x)\n", iova, res, l2_pt_pfn);
        return false;
    }

    l2_pt_phys = (hwaddr)l2_pt_pfn << ISP_PADDR_SHIFT;
    res = pci_dma_read(&s->parent_obj, l2_pt_phys + (hwaddr)l2_idx * 4,
                       &target_pfn, 4);
    if (res != MEMTX_OK || target_pfn == 0) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: iova_to_phys(0x%x): L2 walk failed "
                      "(res=%d pfn=0x%x)\n", iova, res, target_pfn);
        return false;
    }

    *out_phys = ((hwaddr)target_pfn << ISP_PADDR_SHIFT) + offset;
    return true;
}

/* Read `len` bytes from the guest-side memory backing IOVA `iova`. */
static bool ipu4_dma_read_iova(Ipu4State *s, uint32_t iova, void *buf,
                               size_t len)
{
    hwaddr phys;

    if (!ipu4_iova_to_phys(s, iova, &phys)) {
        return false;
    }
    return pci_dma_read(&s->parent_obj, phys, buf, len) == MEMTX_OK;
}

static bool ipu4_dma_write_iova(Ipu4State *s, uint32_t iova, const void *buf,
                                size_t len)
{
    hwaddr phys;

    if (!ipu4_iova_to_phys(s, iova, &phys)) {
        return false;
    }
    return pci_dma_write(&s->parent_obj, phys, buf, len) == MEMTX_OK;
}

/* Step-4 deterministic frame pattern: write `total` bytes starting at
 * `base_iova`, where byte[k] = (k + seq) & 0xff. The IOVA range may
 * span many 4K pages backed by physically discontiguous host pages
 * (vb2_dma_sg memops in kernel/ipu4/ipu6-isys-queue.c:1029), so walk
 * page-by-page and translate each IOVA chunk through the IPU MMU.
 *
 * `total` is bounded by `IPU4_FRAME_PATTERN_BYTES` rather than the
 * full vb2 buffer size because we don't have the buffer length in
 * the STREAM_CAPTURE token (only out_buf_id + addr). The Step-4
 * test (tools/tests/streamon.c) only verifies offsets up to 65535
 * so 64 KB is sufficient. Walking until the IPU MMU walk fails
 * would be slightly more general but risks writing past the end of
 * a smaller buffer — bounded chunk is conservative.
 */
#define IPU4_FRAME_PATTERN_BYTES (64 * 1024)
#define IPU4_PAGE_SIZE_BYTES     4096

static void ipu4_dma_write_iova_pattern(Ipu4State *s, uint32_t base_iova,
                                        size_t total, uint8_t seq)
{
    uint8_t buf[IPU4_PAGE_SIZE_BYTES];
    size_t off = 0;

    while (off < total) {
        uint32_t iova = base_iova + (uint32_t)off;
        uint32_t page_off = iova & (IPU4_PAGE_SIZE_BYTES - 1);
        size_t chunk = MIN(total - off,
                           IPU4_PAGE_SIZE_BYTES - (size_t)page_off);
        hwaddr phys;
        size_t i;

        if (!ipu4_iova_to_phys(s, iova, &phys)) {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: frame-pattern write stopped at "
                          "off=%zu (iova=0x%x): page unmapped — "
                          "buffer ends or IPU MMU not programmed\n",
                          off, iova);
            return;
        }
        for (i = 0; i < chunk; i++) {
            buf[i] = (uint8_t)((off + i + seq) & 0xff);
        }
        if (pci_dma_write(&s->parent_obj, phys, buf, chunk) != MEMTX_OK) {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: pci_dma_write failed at "
                          "iova=0x%x phys=0x%" HWADDR_PRIx "\n",
                          iova, phys);
            return;
        }
        off += chunk;
    }
}

/* Read the `i`-th queue descriptor from the IOVA-addressed queue array
 * pointed to by `array_iova`. */
static bool ipu4_load_queue_desc(Ipu4State *s, uint32_t array_iova,
                                 unsigned int i, Ipu4FwSysQueue *out)
{
    return ipu4_dma_read_iova(s,
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
    if (!ipu4_dma_read_iova(s, s->syscom_config_iova, &cfg, sizeof(cfg))) {
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
    if (!ipu4_dma_read_iova(s, s->syscom_config_iova, &cfg, sizeof(cfg))) {
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
    recv_wr_slot = &s->is_dmem[rq->wr_reg];
    wr = *recv_wr_slot;
    if (wr >= rq->size) {
        wr = 0;
    }

    /* DMA the response into the slot. If the IOVA can't be walked
     * (MMU not programmed), bail out — the driver will time out, but
     * better that than corrupting an unrelated guest page. */
    if (!ipu4_dma_write_iova(s,
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
    s->is_unispart_irq_status |= ISYS_UNISPART_IRQ_SW;
    s->isr_status |= BTRS_ISR_IS_IRQ;

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
 * delivery in our model.
 */
#define IPU4_MAX_OPINS 6

static void ipu4_deliver_frame(Ipu4State *s, uint8_t stream,
                               uint32_t frame_buff_set_iova)
{
    Ipu4FwOutputPin pins[IPU4_MAX_OPINS];
    unsigned int i;

    if (!ipu4_dma_read_iova(s, frame_buff_set_iova, pins, sizeof(pins))) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: failed to DMA-read frame_buff_set at "
                      "iova=0x%x — driver will not see frames\n",
                      frame_buff_set_iova);
        return;
    }

    /* Step-5b IRQ event cycling: silicon's CSI2 RX hardware fires a
     * SOF event on every frame-start, which the driver's ISR
     * dispatches to `ipu6_isys_csi2_isr` (ipu6-isys.c:391-398).
     * Mirror that per delivered frame: set the CSI2 PART_IRQ_STATUS
     * FS_VC0 bit and the matching UNISPART_IRQ_STATUS port-0 bit so
     * the driver's loop in `ipu4_isys_isr` reads PART_IRQ_STATUS,
     * does W1C on PART_IRQ_CLEAR, and dispatches the SOF event in
     * the same IRQ cycle that drains the firmware response queue.
     * The per-frame cadence is sufficient — the streamon-smoke test
     * delivers a single frame, so a periodic QEMUTimer would only
     * add lifecycle complexity for the same observable behaviour. */
    s->csi2[0].part_status |= CSI2_PART_IRQ_FS_VC0;
    s->is_unispart_irq_status |= ISYS_UNISPART_IRQ_CSI2_PORT0;

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
        ipu4_dma_write_iova_pattern(s, pins[i].addr,
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
            !ipu4_dma_read_iova(s,
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

/* Resolve a BAR-relative address into (port_index, register_offset)
 * for the CSI2 region. Returns -1 if the address isn't within any
 * port's 0x1000 / 0x800 window. */
static int ipu4_csi2_resolve(hwaddr addr, hwaddr *out_offset)
{
    /* Ports 0..3 each cover a 0x1000 window; ports 4 and 5 split a
     * 0x1000 window into two 0x800 halves. */
    static const hwaddr port_size[IPU4_CSI2_NUM_PORTS] = {
        0x1000, 0x1000, 0x1000, 0x1000, 0x800, 0x800,
    };
    unsigned int i;

    for (i = 0; i < IPU4_CSI2_NUM_PORTS; i++) {
        hwaddr base = ipu4_csi2_port_base[i];
        if (addr >= base && addr < base + port_size[i]) {
            *out_offset = addr - base;
            return (int)i;
        }
    }
    return -1;
}

/* Per-port read dispatch. Returns true if `reg_off` matched a known
 * register; *val is set on success. Unknown offsets within a port's
 * window fall through to the caller's LOG_UNIMP path. */
static bool ipu4_csi2_port_read(Ipu4Csi2Port *p, hwaddr reg_off,
                                uint64_t *val)
{
    switch (reg_off) {
    case CSI2_PORT_REG_RX_ENABLE:                 *val = p->rx_enable; return true;
    case CSI2_PORT_REG_RX_NOF_LANES:              *val = p->rx_nof_lanes; return true;
    case CSI2_PORT_REG_RX_CONFIG:                 *val = p->rx_config; return true;
    case CSI2_PORT_REG_DLY_TERMEN_C:              *val = p->dly_termen_c; return true;
    case CSI2_PORT_REG_DLY_SETTLE_C:              *val = p->dly_settle_c; return true;
    case CSI2_PORT_REG_DLY_TERMEN_D0:             *val = p->dly_termen_d0; return true;
    case CSI2_PORT_REG_DLY_SETTLE_D0:             *val = p->dly_settle_d0; return true;

    case CSI2_PORT_REG_PART_IRQ_EDGE:             *val = p->part_edge; return true;
    case CSI2_PORT_REG_PART_IRQ_MASK:             *val = p->part_mask; return true;
    case CSI2_PORT_REG_PART_IRQ_STATUS:
        *val = p->part_status & p->part_enable;
        return true;
    case CSI2_PORT_REG_PART_IRQ_ENABLE:            *val = p->part_enable; return true;
    case CSI2_PORT_REG_PART_IRQ_LEVEL_NOT_PULSE:   *val = p->part_level; return true;

    case CSI2_PORT_REG_RX_IRQ_EDGE:                *val = p->rx_edge; return true;
    case CSI2_PORT_REG_RX_IRQ_MASK:                *val = p->rx_mask; return true;
    case CSI2_PORT_REG_RX_IRQ_STATUS:
        *val = p->rx_status & p->rx_enable_irq;
        return true;
    case CSI2_PORT_REG_RX_IRQ_ENABLE:              *val = p->rx_enable_irq; return true;
    case CSI2_PORT_REG_RX_IRQ_LEVEL_NOT_PULSE:     *val = p->rx_level; return true;

    case CSI2_PORT_REG_S2M_IRQ_EDGE:               *val = p->s2m_edge; return true;
    case CSI2_PORT_REG_S2M_IRQ_MASK:               *val = p->s2m_mask; return true;
    case CSI2_PORT_REG_S2M_IRQ_STATUS:
        *val = p->s2m_status & p->s2m_enable;
        return true;
    case CSI2_PORT_REG_S2M_IRQ_ENABLE:             *val = p->s2m_enable; return true;
    case CSI2_PORT_REG_S2M_IRQ_LEVEL_NOT_PULSE:    *val = p->s2m_level; return true;
    }
    return false;
}

/* Per-port write dispatch. Returns true if reg_off matched a known
 * register. Latches except for the W1C *_IRQ_CLEAR registers. */
static bool ipu4_csi2_port_write(Ipu4Csi2Port *p, hwaddr reg_off,
                                 uint64_t val)
{
    switch (reg_off) {
    case CSI2_PORT_REG_RX_ENABLE:                 p->rx_enable = val; return true;
    case CSI2_PORT_REG_RX_NOF_LANES:              p->rx_nof_lanes = val; return true;
    case CSI2_PORT_REG_RX_CONFIG:                 p->rx_config = val; return true;
    case CSI2_PORT_REG_DLY_TERMEN_C:              p->dly_termen_c = val; return true;
    case CSI2_PORT_REG_DLY_SETTLE_C:              p->dly_settle_c = val; return true;
    case CSI2_PORT_REG_DLY_TERMEN_D0:             p->dly_termen_d0 = val; return true;
    case CSI2_PORT_REG_DLY_SETTLE_D0:             p->dly_settle_d0 = val; return true;

    case CSI2_PORT_REG_PART_IRQ_EDGE:             p->part_edge = val; return true;
    case CSI2_PORT_REG_PART_IRQ_MASK:             p->part_mask = val; return true;
    case CSI2_PORT_REG_PART_IRQ_CLEAR:
        p->part_status &= ~(uint32_t)val;
        return true;
    case CSI2_PORT_REG_PART_IRQ_ENABLE:           p->part_enable = val; return true;
    case CSI2_PORT_REG_PART_IRQ_LEVEL_NOT_PULSE:  p->part_level = val; return true;

    case CSI2_PORT_REG_RX_IRQ_EDGE:               p->rx_edge = val; return true;
    case CSI2_PORT_REG_RX_IRQ_MASK:               p->rx_mask = val; return true;
    case CSI2_PORT_REG_RX_IRQ_CLEAR:
        p->rx_status &= ~(uint32_t)val;
        return true;
    case CSI2_PORT_REG_RX_IRQ_ENABLE:             p->rx_enable_irq = val; return true;
    case CSI2_PORT_REG_RX_IRQ_LEVEL_NOT_PULSE:    p->rx_level = val; return true;

    case CSI2_PORT_REG_S2M_IRQ_EDGE:              p->s2m_edge = val; return true;
    case CSI2_PORT_REG_S2M_IRQ_MASK:              p->s2m_mask = val; return true;
    case CSI2_PORT_REG_S2M_IRQ_CLEAR:
        p->s2m_status &= ~(uint32_t)val;
        return true;
    case CSI2_PORT_REG_S2M_IRQ_ENABLE:            p->s2m_enable = val; return true;
    case CSI2_PORT_REG_S2M_IRQ_LEVEL_NOT_PULSE:   p->s2m_level = val; return true;
    }
    return false;
}

static uint64_t ipu4_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    Ipu4State *s = IPU4(opaque);
    uint64_t val = 0;

    switch (addr) {
    case BTRS_REG_WDT:
        /* Silicon returns 0xfff0fff on reads (likely a timeout/fuse
         * constant; value lifted verbatim from data/trace.txt). The
         * driver doesn't write it back, so a fixed return is enough.
         * Writes are still absorbed as watchdog kicks in the write
         * handler below. */
        val = 0xfff0fff;
        break;
    case BTRS_REG_BTRS_CTRL:
        val = s->btrs_ctrl;
        break;
    case BTRS_REG_FW_RESET_CTL:
        /* Once the driver kicks START, latch DONE so the poll terminates. */
        if (s->fw_reset_ctl & BTRS_FW_RESET_CTL_START) {
            s->fw_reset_ctl |= BTRS_FW_RESET_CTL_DONE;
        }
        val = s->fw_reset_ctl;
        break;
    case BTRS_REG_PWR_STATE:
        /* Compose the PWR_STATE word from the per-island FSM flags
         * (Step 5). PWR_RDY and HH_DONE are tied high — buttress
         * itself and TSC-sync are always considered ready since we
         * don't model their respective FSMs. */
        val = BTRS_PWR_STATE_PWR_RDY | BTRS_PWR_STATE_HH_DONE;
        if (s->is_powered) {
            val |= BTRS_PWR_STATE_IS_RDY;
        }
        if (s->ps_powered) {
            val |= BTRS_PWR_STATE_PS_PWR_UP;
        }
        s->pwr_state = val;
        break;
    case BTRS_REG_ISR_STATUS:
    case BTRS_REG_ISR_ENABLED_STATUS:
        val = s->isr_status & s->isr_enable;
        break;
    case BTRS_REG_ISR_ENABLE:
        val = s->isr_enable;
        break;
    case BTRS_REG_SECURITY_CTL:
        val = s->security_ctl;
        break;
    case BTRS_REG_IU2CSECSR:
        val = s->iu2cse_csr;
        break;
    case BTRS_REG_CSE2IUCSR:
        val = s->cse2iu_csr;
        break;
    case BTRS_REG_CSE2IUDATA0:
        val = s->cse2iu_data0;
        break;
    case BTRS_REG_SKU:
        val = 0; /* unfused */
        break;
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
        val = (addr == BTRS_REG_TSC_HI) ? (tsc >> 32) : (uint32_t)tsc;
        break;
    }
    case IS_UNISPART_IRQ_EDGE:
        val = s->is_unispart_irq_edge;
        break;
    case IS_UNISPART_IRQ_MASK:
        val = s->is_unispart_irq_mask;
        break;
    case IS_UNISPART_IRQ_STATUS:
        /* Real silicon's UNISPART_IRQ_STATUS is the raw set-bits
         * register: bits accumulate via edge / level events, are
         * cleared via W1C to IRQ_CLEAR, and are independent of the
         * ENABLE mask (which gates whether the bit propagates to
         * the buttress parent IRQ, not whether STATUS reads back
         * the bit). The driver's `ipu4_isys_isr` reads STATUS and
         * dispatches on bit 30 directly without re-AND-ing with
         * ENABLE — so masking here would silently drop the SW IRQ
         * we synthesise from the syscom command parser if the
         * driver hasn't yet run isys_setup_hw() to program the
         * ENABLE register. */
        val = s->is_unispart_irq_status;
        break;
    case IS_UNISPART_IRQ_ENABLE:
        val = s->is_unispart_irq_enable;
        break;
    case IS_UNISPART_IRQ_LEVEL_NOT_PULSE:
        val = s->is_unispart_irq_level_not_pulse;
        break;
    case IS_UNISPART_SW_IRQ_MUX:
        val = s->is_unispart_sw_irq_mux;
        break;


    case IS_SPC_STATUS_CTRL:
        /* See the bit-map comment near the constant: force READY on,
         * START off so query_sp() reads "SPC is idle and ready"
         * regardless of what the driver kicked into the latch. */
        val = (s->is_spc_status_ctrl & ~SPC_STATUS_START) | SPC_STATUS_READY;
        break;
    case PS_SPC_STATUS_CTRL:
        val = (s->ps_spc_status_ctrl & ~SPC_STATUS_START) | SPC_STATUS_READY;
        break;

    default:
        if (addr >= IS_DMEM_BASE && addr < IS_DMEM_BASE + IS_DMEM_SIZE) {
            /* SEND_RD_POS echoes SEND_WR_POS so the driver sees
             * "firmware has caught up" and doesn't block on the ring
             * filling. SYSCOM_STATE always returns READY so
             * ipu6_fw_com_ready()'s 500ms poll terminates on the
             * first read — the firmware-handshake shortcut documented
             * in the Step 1 plan. All other slots return their
             * latched value. */
            if (addr == IS_DMEM_FW_COM_SEND_RD_POS) {
                val = s->is_dmem[(IS_DMEM_FW_COM_SEND_WR_POS - IS_DMEM_BASE) / 4];
            } else if (addr == IS_DMEM_SYSCOM_STATE) {
                val = SYSCOM_STATE_READY;
            } else {
                val = s->is_dmem[(addr - IS_DMEM_BASE) / 4];
            }
            break;
        }
        if (addr >= IS_MMU_BASE && addr < IS_MMU_BASE + IS_MMU_SIZE) {
            /* data/trace.txt does 208 writes into this window during
             * FW bringup and zero reads. If the driver reads here,
             * our "write-only latch" assumption breaks — a real DMA
             * path would need pci_dma_rw() off the stored PDEs and
             * returning the raw latched value could hide that. */
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: ISYS MMU read +0x%06" HWADDR_PRIx
                          " — model's write-only-latch assumption "
                          "broken; DMA backing is unmodelled.\n", addr);
            val = s->is_mmu[(addr - IS_MMU_BASE) / 4];
            break;
        }
        if (addr >= PS_MMU_BASE && addr < PS_MMU_BASE + PS_MMU_SIZE) {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: PSYS MMU read +0x%06" HWADDR_PRIx
                          " — model's write-only-latch assumption "
                          "broken; DMA backing is unmodelled.\n", addr);
            val = s->ps_mmu[(addr - PS_MMU_BASE) / 4];
            break;
        }
        {
            hwaddr csi2_off;
            int csi2_port = ipu4_csi2_resolve(addr, &csi2_off);
            if (csi2_port >= 0) {
                if (ipu4_csi2_port_read(&s->csi2[csi2_port], csi2_off,
                                        &val)) {
                    break;
                }
                qemu_log_mask(LOG_UNIMP,
                              "ipu4: CSI2 port %d read unimpl +0x%03"
                              HWADDR_PRIx "\n", csi2_port, csi2_off);
                return 0;
            }
        }
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: read unimpl +0x%06" HWADDR_PRIx
                      " size=%u\n", addr, size);
        return 0;
    }

    return val;
}

static void ipu4_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    Ipu4State *s = IPU4(opaque);

    switch (addr) {
    case BTRS_REG_BTRS_CTRL:
        s->btrs_ctrl = val;
        break;
    case BTRS_REG_WDT:
        /* Watchdog kick; ignore. */
        break;
    case BTRS_REG_FW_RESET_CTL:
        s->fw_reset_ctl = val;
        break;
    case BTRS_REG_IS_FREQ_CTL:
        /* Driver writes the BUTTRESS_FREQ_CTL_START bit (BIT(31))
         * along with the divisor / QoS-floor / ICCMAX fields to
         * power up an island, or zero to power it down
         * (kernel/ipu4/ipu6-buttress.c:466-505). Mirror the START
         * bit into is_powered so PWR_STATE reads track the driver's
         * intent — without this, ipu6_buttress_power(off) times out
         * polling for IS_PWR_FSM == IDLE and the auto-suspend
         * recovery strands isys->power == 0. */
        s->is_freq_ctl = val;
        s->is_powered = (val & BTRS_FREQ_CTL_START) != 0;
        break;
    case BTRS_REG_PS_FREQ_CTL:
        s->ps_freq_ctl = val;
        s->ps_powered = (val & BTRS_FREQ_CTL_START) != 0;
        break;
    case BTRS_REG_FABRIC_CMD:
        /* Single-shot fabric command write (data/trace.txt shows one
         * write of 0x1 during init). No driver readback; latch and
         * move on. */
        s->fabric_cmd = val;
        break;
    case BTRS_REG_FW_SOURCE_BASE_LO:
        s->fw_src_lo = val;
        break;
    case BTRS_REG_FW_SOURCE_BASE_HI:
        s->fw_src_hi = val;
        break;
    case BTRS_REG_FW_SOURCE_SIZE:
        s->fw_src_size = val;
        break;
    case BTRS_REG_ISR_ENABLE:
        s->isr_enable = val;
        break;
    case BTRS_REG_ISR_CLEAR:
        /* Write-1-to-clear. */
        s->isr_status &= ~val;
        break;
    case BTRS_REG_SECURITY_CTL:
        s->security_ctl = val;
        break;
    case BTRS_REG_IU2CSEDATA0:
        /* Driver-to-CSE command word. Step-1-followup secure-mode
         * model leaves the actual command unchecked because
         * ipu6_buttress_authenticate() short-circuits on the
         * AUTH_DONE bit we pre-set in SECURITY_CTL — no BOOT_LOAD /
         * AUTH_RUN ever ships. Latch for completeness. */
        s->iu2cse_csr = val;
        break;
    case BTRS_REG_IU2CSEDB0:
        /* IU2CSEDB0 with BUSY (BIT(31)) is the "command ready" trigger.
         * In a full secure-mode model we'd drive the CSE-side response
         * dance here; auth_done short-circuits keep it as a latch. */
        break;
    case BTRS_REG_IU2CSECSR:
        /* CSR_OUT — driver writes ENTRY/EXIT/QUERY/VALID-REQ tokens
         * to step the IPC reset state machine
         * (kernel/ipu4/ipu6-buttress.c:67-181). Mirror the minimum set
         * of CSE-side responses so the driver's loop terminates on
         * the success path:
         *   - BIT(0) ENTRY  written → set BIT(1) EXIT in CSR_IN.
         *     Drives the loop straight to the `case EXIT:` branch
         *     which clears state and returns 0.
         *   - BIT(3) ASSERTED_REG_VALID_REQ → set BIT(4)
         *     ACKED_REG_VALID in CSR_IN so
         *     ipu6_buttress_ipc_validity_open() succeeds. (The
         *     auth_done short-circuit means this isn't actually
         *     exercised today, but model it for completeness.)
         *   - BIT(5) DEASSERTED_REG_VALID_REQ → no response needed
         *     (validity_close is fire-and-forget). */
        s->iu2cse_csr = val;
        if (val & BUTTRESS_IU2CSECSR_ENTRY) {
            s->cse2iu_csr |= BUTTRESS_IU2CSECSR_EXIT;
        }
        if (val & BUTTRESS_IU2CSECSR_ASSERTED_REG_VALID_REQ) {
            s->cse2iu_csr |= BUTTRESS_IU2CSECSR_ACKED_REG_VALID;
        }
        break;
    case BTRS_REG_CSE2IUCSR:
        /* CSR_IN is W1C from the driver's side: every bit the driver
         * writes a 1 to is cleared. */
        s->cse2iu_csr &= ~(uint32_t)val;
        break;
    case BTRS_REG_CSE2IUDB0:
        /* Driver clears the incoming doorbell after consuming a
         * response. Latched only — no behaviour. */
        s->cse2iu_data0 = 0;
        break;
    case IS_UNISPART_IRQ_EDGE:
        s->is_unispart_irq_edge = val;
        break;
    case IS_UNISPART_IRQ_MASK:
        s->is_unispart_irq_mask = val;
        break;
    case IS_UNISPART_IRQ_CLEAR:
        /* Write-1-to-clear. */
        s->is_unispart_irq_status &= ~(uint32_t)val;
        break;
    case IS_UNISPART_IRQ_ENABLE:
        s->is_unispart_irq_enable = val;
        break;
    case IS_UNISPART_IRQ_LEVEL_NOT_PULSE:
        s->is_unispart_irq_level_not_pulse = val;
        break;
    case IS_UNISPART_SW_IRQ:
        /* Silicon writes 0 here on every path (51 accesses, all 0x0).
         * On real hardware this is presumably a trigger that raises
         * the bits in SW_IRQ_MUX into IRQ_STATUS, but with value 0
         * nothing changes. Absorb the write without modelling the
         * mux routing until a driver path actually sets bits here. */
        if (val != 0) {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: unispart SW_IRQ written non-zero "
                          "val=0x%" PRIx64 " — model's zero-only-absorb "
                          "assumption broken; SW_IRQ_MUX routing is "
                          "unmodelled. See tools/notes/registers.md "
                          "(0x17c414).\n", val);
        }
        break;
    case IS_UNISPART_SW_IRQ_MUX:
        s->is_unispart_sw_irq_mux = val;
        break;


    case IS_SPC_STATUS_CTRL:
        s->is_spc_status_ctrl = val;
        break;
    case PS_SPC_STATUS_CTRL:
        s->ps_spc_status_ctrl = val;
        break;

    default:
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

            s->is_dmem[reg] = val;

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
            break;
        }
        if (addr >= IS_MMU_BASE && addr < IS_MMU_BASE + IS_MMU_SIZE) {
            if (addr == IS_MMU_SUB0_REG_L1_PHYS) {
                /* Latch the L1 PT base pfn so the syscom DMA helpers
                 * can walk IOVAs. Both ISYS MMU sub-blocks mirror the
                 * same L1 root, so capturing sub-block 0 is sufficient
                 * (kernel/ipu4/ipu6-mmu.c:546-563). */
                s->is_mmu_l1_pfn = val;
            }
            s->is_mmu[(addr - IS_MMU_BASE) / 4] = val;
            break;
        }
        if (addr >= PS_MMU_BASE && addr < PS_MMU_BASE + PS_MMU_SIZE) {
            s->ps_mmu[(addr - PS_MMU_BASE) / 4] = val;
            break;
        }
        {
            hwaddr csi2_off;
            int csi2_port = ipu4_csi2_resolve(addr, &csi2_off);
            if (csi2_port >= 0) {
                if (ipu4_csi2_port_write(&s->csi2[csi2_port], csi2_off,
                                         val)) {
                    break;
                }
                qemu_log_mask(LOG_UNIMP,
                              "ipu4: CSI2 port %d write unimpl +0x%03"
                              HWADDR_PRIx " val=0x%" PRIx64 "\n",
                              csi2_port, csi2_off, val);
                break;
            }
        }
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: write unimpl +0x%06" HWADDR_PRIx
                      " val=0x%" PRIx64 " size=%u\n",
                      addr, val, size);
        break;
    }
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

    /* Hardware-default reset values from data/trace.txt. BTRS_CTRL
     * reads 0x10 before any write (bit 4, fixed by silicon); the
     * driver never writes it back, so the reset needs to match or
     * compare.py sees a read-value mismatch. */
    s->btrs_ctrl = 0x10;
    s->fw_reset_ctl = 0;
    s->is_freq_ctl = 0;
    s->ps_freq_ctl = 0;
    s->is_powered = false;
    s->ps_powered = false;
    s->fabric_cmd = 0;
    s->pwr_state = 0;
    s->isr_enable = 0;
    s->isr_status = 0;
    s->fw_src_lo = 0;
    s->fw_src_hi = 0;
    s->fw_src_size = 0;
    /* Silicon's SECURITY_CTL reads 0x37002 before any driver write.
     * Bit layout (kernel/ipu4/ipu6-platform-buttress-regs.h:127-132):
     *   bit  1   AUTH_DONE
     *   bits 12-14, 17  silicon-specific (fuse / config bits)
     *   bit 16   FW_SECURE_MODE
     * AUTH_DONE is asserted at reset so ipu6_buttress_authenticate()
     * fast-paths into ipu6_buttress_auth_done() and skips the full
     * BOOT_LOAD / AUTH_RUN dance — those would require a richer CSE
     * IPC model than we have. Secure mode (BIT(16)) still routes the
     * driver through ipu6_buttress_ipc_reset() at probe time, which
     * the BIT(1)→BIT(0) CSR ping-pong below handles. */
    s->security_ctl = 0x37002;
    s->iu2cse_csr = 0;
    s->cse2iu_csr = 0;
    s->cse2iu_data0 = 0;
    s->is_unispart_irq_edge = 0;
    s->is_unispart_irq_mask = 0;
    s->is_unispart_irq_status = 0;
    s->is_unispart_irq_enable = 0;
    s->is_unispart_irq_level_not_pulse = 0;
    s->is_unispart_sw_irq_mux = 0;
    memset(s->csi2, 0, sizeof(s->csi2));
    memset(s->is_dmem, 0, sizeof(s->is_dmem));
    memset(s->is_mmu, 0, sizeof(s->is_mmu));
    memset(s->ps_mmu, 0, sizeof(s->ps_mmu));
    s->is_spc_status_ctrl = 0;
    s->ps_spc_status_ctrl = 0;
    s->is_mmu_l1_pfn = 0;
    s->syscom_config_iova = 0;
    memset(s->is_send_q, 0, sizeof(s->is_send_q));
    memset(s->is_send_q_loaded, 0, sizeof(s->is_send_q_loaded));
    memset(&s->is_recv_q, 0, sizeof(s->is_recv_q));
    s->is_recv_q_loaded = false;
    memset(s->is_send_wr_seen, 0, sizeof(s->is_send_wr_seen));
}

static const VMStateDescription vmstate_ipu4_csi2_port = {
    .name = "ipu4-csi2-port",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(rx_enable, Ipu4Csi2Port),
        VMSTATE_UINT32(rx_nof_lanes, Ipu4Csi2Port),
        VMSTATE_UINT32(rx_config, Ipu4Csi2Port),
        VMSTATE_UINT32(dly_termen_c, Ipu4Csi2Port),
        VMSTATE_UINT32(dly_settle_c, Ipu4Csi2Port),
        VMSTATE_UINT32(dly_termen_d0, Ipu4Csi2Port),
        VMSTATE_UINT32(dly_settle_d0, Ipu4Csi2Port),
        VMSTATE_UINT32(part_edge, Ipu4Csi2Port),
        VMSTATE_UINT32(part_mask, Ipu4Csi2Port),
        VMSTATE_UINT32(part_status, Ipu4Csi2Port),
        VMSTATE_UINT32(part_enable, Ipu4Csi2Port),
        VMSTATE_UINT32(part_level, Ipu4Csi2Port),
        VMSTATE_UINT32(rx_edge, Ipu4Csi2Port),
        VMSTATE_UINT32(rx_mask, Ipu4Csi2Port),
        VMSTATE_UINT32(rx_status, Ipu4Csi2Port),
        VMSTATE_UINT32(rx_enable_irq, Ipu4Csi2Port),
        VMSTATE_UINT32(rx_level, Ipu4Csi2Port),
        VMSTATE_UINT32(s2m_edge, Ipu4Csi2Port),
        VMSTATE_UINT32(s2m_mask, Ipu4Csi2Port),
        VMSTATE_UINT32(s2m_status, Ipu4Csi2Port),
        VMSTATE_UINT32(s2m_enable, Ipu4Csi2Port),
        VMSTATE_UINT32(s2m_level, Ipu4Csi2Port),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ipu4 = {
    .name = "ipu4",
    .version_id = 10,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, Ipu4State),
        VMSTATE_UINT32(btrs_ctrl, Ipu4State),
        VMSTATE_UINT32(fw_reset_ctl, Ipu4State),
        VMSTATE_UINT32(pwr_state, Ipu4State),
        VMSTATE_UINT32(isr_enable, Ipu4State),
        VMSTATE_UINT32(isr_status, Ipu4State),
        VMSTATE_UINT32(fw_src_lo, Ipu4State),
        VMSTATE_UINT32(fw_src_hi, Ipu4State),
        VMSTATE_UINT32(fw_src_size, Ipu4State),
        VMSTATE_UINT32(security_ctl, Ipu4State),
        VMSTATE_UINT32(iu2cse_csr, Ipu4State),
        VMSTATE_UINT32(cse2iu_csr, Ipu4State),
        VMSTATE_UINT32(cse2iu_data0, Ipu4State),
        VMSTATE_UINT32_V(is_freq_ctl, Ipu4State, 2),
        VMSTATE_UINT32_V(ps_freq_ctl, Ipu4State, 2),
        VMSTATE_UINT32_V(fabric_cmd, Ipu4State, 2),
        VMSTATE_UINT32_V(is_unispart_irq_edge, Ipu4State, 3),
        VMSTATE_UINT32_V(is_unispart_irq_mask, Ipu4State, 3),
        VMSTATE_UINT32_V(is_unispart_irq_status, Ipu4State, 3),
        VMSTATE_UINT32_V(is_unispart_irq_enable, Ipu4State, 3),
        VMSTATE_UINT32_V(is_unispart_irq_level_not_pulse, Ipu4State, 3),
        VMSTATE_UINT32_V(is_unispart_sw_irq_mux, Ipu4State, 3),
        VMSTATE_STRUCT_ARRAY(csi2, Ipu4State, IPU4_CSI2_NUM_PORTS, 10,
                             vmstate_ipu4_csi2_port, Ipu4Csi2Port),
        VMSTATE_UINT32_ARRAY_V(is_dmem, Ipu4State, IS_DMEM_SIZE / 4, 5),
        VMSTATE_UINT32_ARRAY_V(is_mmu, Ipu4State, IS_MMU_SIZE / 4, 6),
        VMSTATE_UINT32_ARRAY_V(ps_mmu, Ipu4State, PS_MMU_SIZE / 4, 6),
        VMSTATE_UINT32_V(is_spc_status_ctrl, Ipu4State, 7),
        VMSTATE_UINT32_V(ps_spc_status_ctrl, Ipu4State, 7),
        VMSTATE_UINT32_V(is_mmu_l1_pfn, Ipu4State, 8),
        VMSTATE_UINT32_V(syscom_config_iova, Ipu4State, 8),
        VMSTATE_UINT32_ARRAY_V(is_send_wr_seen, Ipu4State,
                               IPU4_MAX_MSG_STREAMS, 8),
        VMSTATE_BOOL_V(is_powered, Ipu4State, 9),
        VMSTATE_BOOL_V(ps_powered, Ipu4State, 9),
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
