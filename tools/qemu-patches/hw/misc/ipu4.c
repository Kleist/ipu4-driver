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
#define BTRS_REG_PWR_STATE         0x05c
#define BTRS_REG_FW_SOURCE_BASE_LO 0x078
#define BTRS_REG_FW_SOURCE_BASE_HI 0x07c
#define BTRS_REG_FW_SOURCE_SIZE    0x080
#define BTRS_REG_ISR_STATUS        0x090
#define BTRS_REG_ISR_ENABLED_STATUS 0x094
#define BTRS_REG_ISR_ENABLE        0x098
#define BTRS_REG_ISR_CLEAR         0x09c
#define BTRS_REG_IU2CSEDB0         0x100
#define BTRS_REG_IU2CSEDATA0       0x104
#define BTRS_REG_IU2CSECSR         0x108
#define BTRS_REG_SECURITY_CTL      0x300
#define BTRS_REG_CSE2IUDB0         0x304
#define BTRS_REG_CSE2IUDATA0       0x308
#define BTRS_REG_CSE2IUCSR         0x30c
#define BTRS_REG_SKU               0x314

#define BTRS_FW_RESET_CTL_START    BIT(0)
#define BTRS_FW_RESET_CTL_DONE     BIT(1)

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
    default:
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: read unimpl +0x%06" HWADDR_PRIx " size=%u\n",
                      addr, size);
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
    default:
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

    s->btrs_ctrl = 0;
    s->fw_reset_ctl = 0;
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
}

static const VMStateDescription vmstate_ipu4 = {
    .name = "ipu4",
    .version_id = 1,
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
