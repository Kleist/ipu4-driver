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
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msi.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

#include "ipu4-mmu.h"
#include "ipu4-buttress.h"
#include "ipu4-csi2.h"
#include "ipu4-isys.h"
#include "ipu4-syscom.h"

#define TYPE_IPU4 "ipu4"
OBJECT_DECLARE_SIMPLE_TYPE(Ipu4State, IPU4)

#define IPU4_PCI_VENDOR_ID  0x8086
#define IPU4_PCI_DEVICE_ID  0x5a88
#define IPU4_BAR_SIZE       (16 * MiB)

/* Per-block register handling lives in:
 *   ipu4-buttress.{c,h}    BTRS_* + IPC echo + PWR_STATE + TSC + ISR W1C
 *   ipu4-isys.{c,h}        unispart IRQ + ISYS/PSYS SPC + DMEM window
 *   ipu4-csi2.{c,h}        CSI2 port 0 + ports 1..5 unmodelled-port log
 *   ipu4-mmu.{c,h}         IS_MMU/PS_MMU windows + IOVA walker + DMA helpers
 *   ipu4-syscom.{c,h}      FW protocol response posting + frame delivery
 *   ipu4-fw-isys.h         FW protocol struct shapes / opcode enums
 *   ipu4-irq-trio.{c,h}    shared 5-field IRQ trio used by isys + csi2
 */

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

    /* Syscom (firmware command/response) layer: queue descriptor cache,
     * SYSCOM_CONFIG_REG latch, send-cursor wr_seen tracker. */
    Ipu4Syscom syscom;
};

/* Bundle the syscom layer's external dependencies for handoff into
 * the syscom helpers (which deliberately don't know Ipu4State). */
static inline Ipu4SyscomCtx ipu4_syscom_ctx(Ipu4State *s)
{
    return (Ipu4SyscomCtx) {
        .sc = &s->syscom,
        .isys = &s->isys,
        .mmu = &s->mmu,
        .buttress = &s->buttress,
        .pdev = &s->parent_obj,
    };
}

/* Syscom queue cache, FW response posting, frame delivery and
 * send-cursor bump dispatch live in ipu4-syscom.{c,h}. */

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
        Ipu4SyscomCtx ctx = ipu4_syscom_ctx(s);
        ipu4_syscom_dmem_write(&ctx, addr, val);
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
    ipu4_syscom_reset(&s->syscom);
}

static const VMStateDescription vmstate_ipu4 = {
    .name = "ipu4",
    .version_id = 14,
    .minimum_version_id = 14,
    .fields = (const VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, Ipu4State),
        VMSTATE_STRUCT(buttress, Ipu4State, 0, vmstate_ipu4_buttress,
                       Ipu4Buttress),
        VMSTATE_STRUCT(isys, Ipu4State, 0, vmstate_ipu4_isys, Ipu4Isys),
        VMSTATE_STRUCT(csi2, Ipu4State, 0, vmstate_ipu4_csi2, Ipu4Csi2),
        VMSTATE_STRUCT(mmu, Ipu4State, 0, vmstate_ipu4_mmu, Ipu4MmuRegs),
        VMSTATE_STRUCT(syscom, Ipu4State, 0, vmstate_ipu4_syscom,
                       Ipu4Syscom),
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
