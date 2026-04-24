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

/* CSI2 port 0. Bases for ports 1..5 are in
 * kernel/ipu4/ipu4-platform-isys-csi2-reg.h:13-15 (0x65000, 0x66000,
 * 0x67000, 0x6C000, 0x6C800 — note ports 4/5 live at their own offsets).
 * Silicon touches only port 0 in data/trace.txt; we model it explicitly
 * and leave the other ports unimplemented until a trace covers them.
 * Per-port register offsets (from
 * kernel/ipu4/ipu4-platform-isys-csi2-reg.h:19-56):
 *
 *   0x000  CSI_RX_ENABLE             R/W latched, bit 0 = enable.
 *   0x004  CSI_RX_NOF_ENABLED_LANES  R/W latched.
 *   0x008  CSI_RX_CONFIG             R/W latched; driver reads back.
 *   0x02c/0x030         DLY_CNT_{TERMEN,SETTLE}_CLANE   R/W latched.
 *   0x034/0x038         DLY_CNT_{TERMEN,SETTLE}_DLANE(0) R/W latched.
 *   0x400-0x414  CSI2PART IRQ trio (same W1C shape as unispart).
 *   0x500-0x514  CSI RX IRQ trio.
 *   0x600-0x614  S2M IRQ trio.
 *
 * Same caveat as the unispart IRQ: `status` stays 0 because nothing
 * in the model raises CSI2 interrupts. That's a backing-device gap,
 * not a handler bug — compare.py will flag the corresponding STATUS
 * reads as value_mismatch until frame generation lands.
 */
#define CSI2P0_BASE                      0x64000
#define CSI2P0_RX_ENABLE                 (CSI2P0_BASE + 0x000)
#define CSI2P0_RX_NOF_LANES              (CSI2P0_BASE + 0x004)
#define CSI2P0_RX_CONFIG                 (CSI2P0_BASE + 0x008)
#define CSI2P0_DLY_TERMEN_C              (CSI2P0_BASE + 0x02c)
#define CSI2P0_DLY_SETTLE_C              (CSI2P0_BASE + 0x030)
#define CSI2P0_DLY_TERMEN_D0             (CSI2P0_BASE + 0x034)
#define CSI2P0_DLY_SETTLE_D0             (CSI2P0_BASE + 0x038)

#define CSI2P0_PART_IRQ_EDGE             (CSI2P0_BASE + 0x400)
#define CSI2P0_PART_IRQ_MASK             (CSI2P0_BASE + 0x404)
#define CSI2P0_PART_IRQ_STATUS           (CSI2P0_BASE + 0x408)
#define CSI2P0_PART_IRQ_CLEAR            (CSI2P0_BASE + 0x40c)
#define CSI2P0_PART_IRQ_ENABLE           (CSI2P0_BASE + 0x410)
#define CSI2P0_PART_IRQ_LEVEL_NOT_PULSE  (CSI2P0_BASE + 0x414)

#define CSI2P0_RX_IRQ_EDGE               (CSI2P0_BASE + 0x500)
#define CSI2P0_RX_IRQ_MASK               (CSI2P0_BASE + 0x504)
#define CSI2P0_RX_IRQ_STATUS             (CSI2P0_BASE + 0x508)
#define CSI2P0_RX_IRQ_CLEAR              (CSI2P0_BASE + 0x50c)
#define CSI2P0_RX_IRQ_ENABLE             (CSI2P0_BASE + 0x510)
#define CSI2P0_RX_IRQ_LEVEL_NOT_PULSE    (CSI2P0_BASE + 0x514)

#define CSI2P0_S2M_IRQ_EDGE              (CSI2P0_BASE + 0x600)
#define CSI2P0_S2M_IRQ_MASK              (CSI2P0_BASE + 0x604)
#define CSI2P0_S2M_IRQ_STATUS            (CSI2P0_BASE + 0x608)
#define CSI2P0_S2M_IRQ_CLEAR             (CSI2P0_BASE + 0x60c)
#define CSI2P0_S2M_IRQ_ENABLE            (CSI2P0_BASE + 0x610)
#define CSI2P0_S2M_IRQ_LEVEL_NOT_PULSE   (CSI2P0_BASE + 0x614)

/* CSI2 ports 1..5 live at
 * 0x65000 (p1), 0x66000 (p2), 0x67000 (p3), 0x6c000 (p4), 0x6c800 (p5)
 * (kernel/ipu4/ipu4-platform-isys-csi2-reg.h:13-15). We don't model
 * them because data/trace.txt only exercises port 0; any access to
 * this range means a driver configuration we haven't captured and
 * the "port 0 only" assumption in the CSI2 block needs rework.
 */
#define CSI2_PORTS_1_5_RANGE_BEGIN       0x65000
#define CSI2_PORTS_1_5_RANGE_END         0x6d800

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

/* PWR_STATE target values the driver polls for. IPU4 uses:
 *   - bits 13:12  HH (TSC-sync) status: DONE = 2
 *   - bits 23:20  IS (ISYS) power FSM: IS_RDY = 0xa
 *   - bits 28:24  PS (PSYS) power FSM: PS_PWR_UP = 0xf
 * Reading PWR_STATE always returns all of them asserted so every
 * `readl_poll_timeout()` in the driver terminates on the first read.
 */
#define BTRS_PWR_STATE_PWR_RDY_ALL      0x0fa02003

struct Ipu4State {
    PCIDevice parent_obj;
    MemoryRegion bar0;

    /* Latched registers. */
    uint32_t btrs_ctrl;
    uint32_t fw_reset_ctl;
    uint32_t is_freq_ctl;
    uint32_t ps_freq_ctl;
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

    /* CSI2 port 0. */
    uint32_t csi2p0_rx_enable;
    uint32_t csi2p0_rx_nof_lanes;
    uint32_t csi2p0_rx_config;
    uint32_t csi2p0_dly_termen_c;
    uint32_t csi2p0_dly_settle_c;
    uint32_t csi2p0_dly_termen_d0;
    uint32_t csi2p0_dly_settle_d0;
    uint32_t csi2p0_part_edge, csi2p0_part_mask, csi2p0_part_status;
    uint32_t csi2p0_part_enable, csi2p0_part_level;
    uint32_t csi2p0_rx_edge, csi2p0_rx_mask, csi2p0_rx_status;
    uint32_t csi2p0_rx_enable_irq, csi2p0_rx_level;
    uint32_t csi2p0_s2m_edge, csi2p0_s2m_mask, csi2p0_s2m_status;
    uint32_t csi2p0_s2m_enable, csi2p0_s2m_level;

    /* ISYS DMEM syscom window (0x108000..0x1080ff). */
    uint32_t is_dmem[IS_DMEM_SIZE / 4];
};

static uint64_t ipu4_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    Ipu4State *s = IPU4(opaque);
    uint64_t val = 0;

    switch (addr) {
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
        /* Report "all power islands ready" on every read. The driver
         * polls this for ISYS, PSYS, and TSC-sync readiness. */
        val = BTRS_PWR_STATE_PWR_RDY_ALL;
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
        val = s->is_unispart_irq_status & s->is_unispart_irq_enable;
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

    case CSI2P0_RX_ENABLE:        val = s->csi2p0_rx_enable; break;
    case CSI2P0_RX_NOF_LANES:     val = s->csi2p0_rx_nof_lanes; break;
    case CSI2P0_RX_CONFIG:        val = s->csi2p0_rx_config; break;
    case CSI2P0_DLY_TERMEN_C:     val = s->csi2p0_dly_termen_c; break;
    case CSI2P0_DLY_SETTLE_C:     val = s->csi2p0_dly_settle_c; break;
    case CSI2P0_DLY_TERMEN_D0:    val = s->csi2p0_dly_termen_d0; break;
    case CSI2P0_DLY_SETTLE_D0:    val = s->csi2p0_dly_settle_d0; break;

    case CSI2P0_PART_IRQ_EDGE:    val = s->csi2p0_part_edge; break;
    case CSI2P0_PART_IRQ_MASK:    val = s->csi2p0_part_mask; break;
    case CSI2P0_PART_IRQ_STATUS:
        val = s->csi2p0_part_status & s->csi2p0_part_enable;
        break;
    case CSI2P0_PART_IRQ_ENABLE:  val = s->csi2p0_part_enable; break;
    case CSI2P0_PART_IRQ_LEVEL_NOT_PULSE: val = s->csi2p0_part_level; break;

    case CSI2P0_RX_IRQ_EDGE:      val = s->csi2p0_rx_edge; break;
    case CSI2P0_RX_IRQ_MASK:      val = s->csi2p0_rx_mask; break;
    case CSI2P0_RX_IRQ_STATUS:
        val = s->csi2p0_rx_status & s->csi2p0_rx_enable_irq;
        break;
    case CSI2P0_RX_IRQ_ENABLE:    val = s->csi2p0_rx_enable_irq; break;
    case CSI2P0_RX_IRQ_LEVEL_NOT_PULSE: val = s->csi2p0_rx_level; break;

    case CSI2P0_S2M_IRQ_EDGE:     val = s->csi2p0_s2m_edge; break;
    case CSI2P0_S2M_IRQ_MASK:     val = s->csi2p0_s2m_mask; break;
    case CSI2P0_S2M_IRQ_STATUS:
        val = s->csi2p0_s2m_status & s->csi2p0_s2m_enable;
        break;
    case CSI2P0_S2M_IRQ_ENABLE:   val = s->csi2p0_s2m_enable; break;
    case CSI2P0_S2M_IRQ_LEVEL_NOT_PULSE: val = s->csi2p0_s2m_level; break;

    default:
        if (addr >= IS_DMEM_BASE && addr < IS_DMEM_BASE + IS_DMEM_SIZE) {
            /* SEND_RD_POS echoes SEND_WR_POS so the driver sees
             * "firmware has caught up" and doesn't block on the ring
             * filling. All other slots return their latched value. */
            if (addr == IS_DMEM_FW_COM_SEND_RD_POS) {
                val = s->is_dmem[(IS_DMEM_FW_COM_SEND_WR_POS - IS_DMEM_BASE) / 4];
            } else {
                val = s->is_dmem[(addr - IS_DMEM_BASE) / 4];
            }
            break;
        }
        if (addr >= CSI2_PORTS_1_5_RANGE_BEGIN &&
            addr < CSI2_PORTS_1_5_RANGE_END) {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: CSI2 port>=1 read +0x%06" HWADDR_PRIx
                          " — only port 0 is modelled; data/trace.txt "
                          "only exercised port 0, so this means a new "
                          "driver path hit an unmodelled port.\n", addr);
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: read unimpl +0x%06" HWADDR_PRIx
                          " size=%u\n", addr, size);
        }
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
        /* Driver writes a clock divisor (low byte) plus the ICCMAX
         * level (bit 31 set when active). Latched only — there's no
         * actual clock to scale in the model, and the driver doesn't
         * read this back to validate. */
        s->is_freq_ctl = val;
        break;
    case BTRS_REG_PS_FREQ_CTL:
        s->ps_freq_ctl = val;
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
    case BTRS_REG_IU2CSEDB0:
    case BTRS_REG_IU2CSEDATA0:
    case BTRS_REG_IU2CSECSR:
        /* IPC send path. The reset handshake is a no-op: echo CSR bits
         * back via CSE2IU* so ipu6_buttress_ipc_reset() completes. */
        if (addr == BTRS_REG_IU2CSECSR) {
            s->iu2cse_csr = val;
            s->cse2iu_csr = val; /* echo */
            s->cse2iu_data0 = 0;
        }
        break;
    case BTRS_REG_CSE2IUDB0:
    case BTRS_REG_CSE2IUCSR:
        /* IPC receive side: the driver acks by writing here; clear state. */
        s->cse2iu_csr = 0;
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

    case CSI2P0_RX_ENABLE:        s->csi2p0_rx_enable = val; break;
    case CSI2P0_RX_NOF_LANES:     s->csi2p0_rx_nof_lanes = val; break;
    case CSI2P0_RX_CONFIG:        s->csi2p0_rx_config = val; break;
    case CSI2P0_DLY_TERMEN_C:     s->csi2p0_dly_termen_c = val; break;
    case CSI2P0_DLY_SETTLE_C:     s->csi2p0_dly_settle_c = val; break;
    case CSI2P0_DLY_TERMEN_D0:    s->csi2p0_dly_termen_d0 = val; break;
    case CSI2P0_DLY_SETTLE_D0:    s->csi2p0_dly_settle_d0 = val; break;

    case CSI2P0_PART_IRQ_EDGE:    s->csi2p0_part_edge = val; break;
    case CSI2P0_PART_IRQ_MASK:    s->csi2p0_part_mask = val; break;
    case CSI2P0_PART_IRQ_CLEAR:
        s->csi2p0_part_status &= ~(uint32_t)val;
        break;
    case CSI2P0_PART_IRQ_ENABLE:  s->csi2p0_part_enable = val; break;
    case CSI2P0_PART_IRQ_LEVEL_NOT_PULSE: s->csi2p0_part_level = val; break;

    case CSI2P0_RX_IRQ_EDGE:      s->csi2p0_rx_edge = val; break;
    case CSI2P0_RX_IRQ_MASK:      s->csi2p0_rx_mask = val; break;
    case CSI2P0_RX_IRQ_CLEAR:
        s->csi2p0_rx_status &= ~(uint32_t)val;
        break;
    case CSI2P0_RX_IRQ_ENABLE:    s->csi2p0_rx_enable_irq = val; break;
    case CSI2P0_RX_IRQ_LEVEL_NOT_PULSE: s->csi2p0_rx_level = val; break;

    case CSI2P0_S2M_IRQ_EDGE:     s->csi2p0_s2m_edge = val; break;
    case CSI2P0_S2M_IRQ_MASK:     s->csi2p0_s2m_mask = val; break;
    case CSI2P0_S2M_IRQ_CLEAR:
        s->csi2p0_s2m_status &= ~(uint32_t)val;
        break;
    case CSI2P0_S2M_IRQ_ENABLE:   s->csi2p0_s2m_enable = val; break;
    case CSI2P0_S2M_IRQ_LEVEL_NOT_PULSE: s->csi2p0_s2m_level = val; break;

    default:
        if (addr >= IS_DMEM_BASE && addr < IS_DMEM_BASE + IS_DMEM_SIZE) {
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
            s->is_dmem[(addr - IS_DMEM_BASE) / 4] = val;
            break;
        }
        if (addr >= CSI2_PORTS_1_5_RANGE_BEGIN &&
            addr < CSI2_PORTS_1_5_RANGE_END) {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: CSI2 port>=1 write +0x%06" HWADDR_PRIx
                          " val=0x%" PRIx64 " — only port 0 is modelled; "
                          "data/trace.txt only exercised port 0, so this "
                          "means a new driver path hit an unmodelled port.\n",
                          addr, val);
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: write unimpl +0x%06" HWADDR_PRIx
                          " val=0x%" PRIx64 " size=%u\n",
                          addr, val, size);
        }
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

    s->btrs_ctrl = 0;
    s->fw_reset_ctl = 0;
    s->is_freq_ctl = 0;
    s->ps_freq_ctl = 0;
    s->fabric_cmd = 0;
    s->pwr_state = 0;
    s->isr_enable = 0;
    s->isr_status = 0;
    s->fw_src_lo = 0;
    s->fw_src_hi = 0;
    s->fw_src_size = 0;
    s->security_ctl = 0;
    s->iu2cse_csr = 0;
    s->cse2iu_csr = 0;
    s->cse2iu_data0 = 0;
    s->is_unispart_irq_edge = 0;
    s->is_unispart_irq_mask = 0;
    s->is_unispart_irq_status = 0;
    s->is_unispart_irq_enable = 0;
    s->is_unispart_irq_level_not_pulse = 0;
    s->is_unispart_sw_irq_mux = 0;
    s->csi2p0_rx_enable = 0;
    s->csi2p0_rx_nof_lanes = 0;
    s->csi2p0_rx_config = 0;
    s->csi2p0_dly_termen_c = 0;
    s->csi2p0_dly_settle_c = 0;
    s->csi2p0_dly_termen_d0 = 0;
    s->csi2p0_dly_settle_d0 = 0;
    s->csi2p0_part_edge = s->csi2p0_part_mask = s->csi2p0_part_status = 0;
    s->csi2p0_part_enable = s->csi2p0_part_level = 0;
    s->csi2p0_rx_edge = s->csi2p0_rx_mask = s->csi2p0_rx_status = 0;
    s->csi2p0_rx_enable_irq = s->csi2p0_rx_level = 0;
    s->csi2p0_s2m_edge = s->csi2p0_s2m_mask = s->csi2p0_s2m_status = 0;
    s->csi2p0_s2m_enable = s->csi2p0_s2m_level = 0;
    memset(s->is_dmem, 0, sizeof(s->is_dmem));
}

static const VMStateDescription vmstate_ipu4 = {
    .name = "ipu4",
    .version_id = 5,
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
        VMSTATE_UINT32_V(csi2p0_rx_enable, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_rx_nof_lanes, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_rx_config, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_dly_termen_c, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_dly_settle_c, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_dly_termen_d0, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_dly_settle_d0, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_part_edge, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_part_mask, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_part_status, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_part_enable, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_part_level, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_rx_edge, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_rx_mask, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_rx_status, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_rx_enable_irq, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_rx_level, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_s2m_edge, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_s2m_mask, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_s2m_status, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_s2m_enable, Ipu4State, 4),
        VMSTATE_UINT32_V(csi2p0_s2m_level, Ipu4State, 4),
        VMSTATE_UINT32_ARRAY_V(is_dmem, Ipu4State, IS_DMEM_SIZE / 4, 5),
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
