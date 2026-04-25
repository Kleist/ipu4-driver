/*
 * Intel IPU4 page-table walker + MMU register windows.
 *
 * See ipu4-mmu.h for the interface and design notes.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "migration/vmstate.h"

#include "ipu4-mmu.h"

const VMStateDescription vmstate_ipu4_mmu = {
    .name = "ipu4/mmu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(is_mmu, Ipu4MmuRegs, IS_MMU_SIZE / 4),
        VMSTATE_UINT32_ARRAY(ps_mmu, Ipu4MmuRegs, PS_MMU_SIZE / 4),
        VMSTATE_UINT32(is_mmu_l1_pfn, Ipu4MmuRegs),
        VMSTATE_END_OF_LIST()
    }
};

void ipu4_mmu_reset(Ipu4MmuRegs *r)
{
    memset(r->is_mmu, 0, sizeof(r->is_mmu));
    memset(r->ps_mmu, 0, sizeof(r->ps_mmu));
    r->is_mmu_l1_pfn = 0;
}

bool ipu4_mmu_addr_in_window(hwaddr addr)
{
    return (addr >= IS_MMU_BASE && addr < IS_MMU_BASE + IS_MMU_SIZE) ||
           (addr >= PS_MMU_BASE && addr < PS_MMU_BASE + PS_MMU_SIZE);
}

bool ipu4_mmu_mmio_read(const Ipu4MmuRegs *r, hwaddr addr, uint64_t *val)
{
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
        *val = r->is_mmu[(addr - IS_MMU_BASE) / 4];
        return true;
    }
    if (addr >= PS_MMU_BASE && addr < PS_MMU_BASE + PS_MMU_SIZE) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: PSYS MMU read +0x%06" HWADDR_PRIx
                      " — model's write-only-latch assumption "
                      "broken; DMA backing is unmodelled.\n", addr);
        *val = r->ps_mmu[(addr - PS_MMU_BASE) / 4];
        return true;
    }
    return false;
}

bool ipu4_mmu_mmio_write(Ipu4MmuRegs *r, hwaddr addr, uint64_t val)
{
    if (addr >= IS_MMU_BASE && addr < IS_MMU_BASE + IS_MMU_SIZE) {
        if (addr == IS_MMU_SUB0_REG_L1_PHYS) {
            /* Latch the L1 PT base pfn so the syscom DMA helpers
             * can walk IOVAs. Both ISYS MMU sub-blocks mirror the
             * same L1 root, so capturing sub-block 0 is sufficient
             * (kernel/ipu4/ipu6-mmu.c:546-563). */
            r->is_mmu_l1_pfn = val;
        }
        r->is_mmu[(addr - IS_MMU_BASE) / 4] = val;
        return true;
    }
    if (addr >= PS_MMU_BASE && addr < PS_MMU_BASE + PS_MMU_SIZE) {
        r->ps_mmu[(addr - PS_MMU_BASE) / 4] = val;
        return true;
    }
    return false;
}

bool ipu4_mmu_iova_to_phys(const Ipu4MmuRegs *r, PCIDevice *pdev,
                           uint32_t iova, hwaddr *out_phys)
{
    uint32_t l1_idx = (iova >> ISP_L1PT_SHIFT) & 0x3ff;
    uint32_t l2_idx = (iova >> ISP_L2PT_SHIFT) & ISP_L2PT_MASK;
    uint32_t offset = iova & ISP_PAGE_MASK;
    hwaddr l1_pt_phys, l2_pt_phys;
    uint32_t l2_pt_pfn = 0, target_pfn = 0;
    MemTxResult res;

    if (r->is_mmu_l1_pfn == 0) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: iova_to_phys(0x%x): L1_PHYS not programmed\n",
                      iova);
        return false;
    }

    l1_pt_phys = (hwaddr)r->is_mmu_l1_pfn << ISP_PADDR_SHIFT;
    res = pci_dma_read(pdev, l1_pt_phys + (hwaddr)l1_idx * 4,
                       &l2_pt_pfn, 4);
    if (res != MEMTX_OK || l2_pt_pfn == 0) {
        qemu_log_mask(LOG_UNIMP,
                      "ipu4: iova_to_phys(0x%x): L1 walk failed "
                      "(res=%d pfn=0x%x)\n", iova, res, l2_pt_pfn);
        return false;
    }

    l2_pt_phys = (hwaddr)l2_pt_pfn << ISP_PADDR_SHIFT;
    res = pci_dma_read(pdev, l2_pt_phys + (hwaddr)l2_idx * 4,
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

bool ipu4_mmu_dma_read_iova(const Ipu4MmuRegs *r, PCIDevice *pdev,
                            uint32_t iova, void *buf, size_t len)
{
    hwaddr phys;

    if (!ipu4_mmu_iova_to_phys(r, pdev, iova, &phys)) {
        return false;
    }
    return pci_dma_read(pdev, phys, buf, len) == MEMTX_OK;
}

bool ipu4_mmu_dma_write_iova(const Ipu4MmuRegs *r, PCIDevice *pdev,
                             uint32_t iova, const void *buf, size_t len)
{
    hwaddr phys;

    if (!ipu4_mmu_iova_to_phys(r, pdev, iova, &phys)) {
        return false;
    }
    return pci_dma_write(pdev, phys, buf, len) == MEMTX_OK;
}

void ipu4_mmu_dma_write_iova_pattern(const Ipu4MmuRegs *r, PCIDevice *pdev,
                                     uint32_t base_iova, size_t total,
                                     uint8_t seq)
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

        if (!ipu4_mmu_iova_to_phys(r, pdev, iova, &phys)) {
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
        if (pci_dma_write(pdev, phys, buf, chunk) != MEMTX_OK) {
            qemu_log_mask(LOG_UNIMP,
                          "ipu4: pci_dma_write failed at "
                          "iova=0x%x phys=0x%" HWADDR_PRIx "\n",
                          iova, phys);
            return;
        }
        off += chunk;
    }
}
