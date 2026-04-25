/*
 * Intel IPU4 page-table walker + MMU register windows.
 *
 * Splits out the IS_MMU / PS_MMU register windows and the IOVA → host
 * phys walker that translates the IPU IOVAs the firmware command
 * parser embeds in syscom buffers. Mirrors the in-driver walker in
 * kernel/ipu4/ipu6-mmu.c.
 *
 * Copyright (c) 2026, IPU4 dev harness.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef HW_MISC_IPU4_MMU_H
#define HW_MISC_IPU4_MMU_H

#include "exec/hwaddr.h"
#include "hw/pci/pci_device.h"
#include "migration/vmstate.h"

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

#define IPU4_PAGE_SIZE_BYTES             4096

/* Latched-state for the MMU register windows + the L1 page-table base
 * pfn the IOVA walker uses. Embedded inside Ipu4State as a single
 * member so the MMU module can be reset / migrated independently. */
typedef struct Ipu4MmuRegs {
    uint32_t is_mmu[IS_MMU_SIZE / 4];
    uint32_t ps_mmu[PS_MMU_SIZE / 4];
    uint32_t is_mmu_l1_pfn;
} Ipu4MmuRegs;

extern const VMStateDescription vmstate_ipu4_mmu;

void ipu4_mmu_reset(Ipu4MmuRegs *r);

/* True if `addr` falls in either MMU register window. */
bool ipu4_mmu_addr_in_window(hwaddr addr);

/* MMIO read/write within the MMU register windows. Both return true
 * when `addr` was handled (caller should not fall through to the
 * unimplemented-access logger); false means the address is outside
 * the MMU windows. */
bool ipu4_mmu_mmio_read(const Ipu4MmuRegs *r, hwaddr addr, uint64_t *val);
bool ipu4_mmu_mmio_write(Ipu4MmuRegs *r, hwaddr addr, uint64_t val);

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
bool ipu4_mmu_iova_to_phys(const Ipu4MmuRegs *r, PCIDevice *pdev,
                           uint32_t iova, hwaddr *out_phys);

/* Read/write `len` bytes from/to the guest-side memory backing IOVA
 * `iova`. Returns false if the IOVA can't be walked or the underlying
 * pci_dma_rw fails. */
bool ipu4_mmu_dma_read_iova(const Ipu4MmuRegs *r, PCIDevice *pdev,
                            uint32_t iova, void *buf, size_t len);
bool ipu4_mmu_dma_write_iova(const Ipu4MmuRegs *r, PCIDevice *pdev,
                             uint32_t iova, const void *buf, size_t len);

/* Step-4 deterministic frame pattern: write `total` bytes starting at
 * `base_iova`, where byte[k] = (k + seq) & 0xff. The IOVA range may
 * span many 4K pages backed by physically discontiguous host pages
 * (vb2_dma_sg memops in kernel/ipu4/ipu6-isys-queue.c:1029), so walk
 * page-by-page and translate each IOVA chunk through the IPU MMU.
 */
void ipu4_mmu_dma_write_iova_pattern(const Ipu4MmuRegs *r, PCIDevice *pdev,
                                     uint32_t base_iova, size_t total,
                                     uint8_t seq);

#endif /* HW_MISC_IPU4_MMU_H */
