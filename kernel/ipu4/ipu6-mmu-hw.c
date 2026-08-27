// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Intel Corporation
 */
#include <asm/barrier.h>

#include <linux/bits.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "ipu6.h"
#include "ipu6-dma.h"
#include "ipu6-mmu.h"
#include "ipu6-platform-regs.h"

#define ISP_PAGE_SHIFT		12
#define ISP_PAGE_SIZE		BIT(ISP_PAGE_SHIFT)
#define ISP_PAGE_MASK		(~(ISP_PAGE_SIZE - 1))

#define ISP_L1PT_SHIFT		22
#define ISP_L1PT_MASK		(~((1U << ISP_L1PT_SHIFT) - 1))

#define ISP_L2PT_SHIFT		12
#define ISP_L2PT_MASK		(~(ISP_L1PT_MASK | (~(ISP_PAGE_MASK))))

#define ISP_L1PT_PTES           1024
#define ISP_L2PT_PTES           1024

#define ISP_PADDR_SHIFT		12

#define REG_TLB_INVALIDATE	0x0000

#define REG_L1_PHYS		0x0004	/* 27-bit pfn */
#define REG_INFO		0x0008

#define TBL_PHYS_ADDR(a)	((phys_addr_t)(a) << ISP_PADDR_SHIFT)

static struct ipu6_mmu_hw ipu6_isys_mmu_hwdata[] = {
	{
		.offset = IPU6_ISYS_IOMMU0_OFFSET,
		.info_bits = IPU6_INFO_REQUEST_DESTINATION_IOSF,
		.nr_l1streams = 16,
		.l1_block_sz = {
			3, 8, 2, 2, 2, 2, 2, 2, 1, 1,
			1, 1, 1, 1, 1, 1
		},
		.nr_l2streams = 16,
		.l2_block_sz = {
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2
		},
		.insert_read_before_invalidate = false,
		.l1_stream_id_reg_offset =
		IPU6_MMU_L1_STREAM_ID_REG_OFFSET,
		.l2_stream_id_reg_offset =
		IPU6_MMU_L2_STREAM_ID_REG_OFFSET,
	},
	{
		.offset = IPU6_ISYS_IOMMU1_OFFSET,
		.info_bits = 0,
		.nr_l1streams = 16,
		.l1_block_sz = {
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 1, 1, 4
		},
		.nr_l2streams = 16,
		.l2_block_sz = {
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2
		},
		.insert_read_before_invalidate = false,
		.l1_stream_id_reg_offset =
		IPU6_MMU_L1_STREAM_ID_REG_OFFSET,
		.l2_stream_id_reg_offset =
		IPU6_MMU_L2_STREAM_ID_REG_OFFSET,
	},
	{
		.offset = IPU6_ISYS_IOMMUI_OFFSET,
		.info_bits = 0,
		.nr_l1streams = 0,
		.nr_l2streams = 0,
		.insert_read_before_invalidate = false,
	},
};

static struct ipu6_mmu_hw ipu6_psys_mmu_hwdata[] = {
	{
		.offset = IPU6_PSYS_IOMMU0_OFFSET,
		.info_bits =
		IPU6_INFO_REQUEST_DESTINATION_IOSF,
		.nr_l1streams = 16,
		.l1_block_sz = {
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2
		},
		.nr_l2streams = 16,
		.l2_block_sz = {
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2
		},
		.insert_read_before_invalidate = false,
		.l1_stream_id_reg_offset =
		IPU6_MMU_L1_STREAM_ID_REG_OFFSET,
		.l2_stream_id_reg_offset =
		IPU6_MMU_L2_STREAM_ID_REG_OFFSET,
	},
	{
		.offset = IPU6_PSYS_IOMMU1_OFFSET,
		.info_bits = 0,
		.nr_l1streams = 32,
		.l1_block_sz = {
			1, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 10,
			5, 4, 14, 6, 4, 14, 6, 4, 8,
			4, 2, 1, 1, 1, 1, 14
		},
		.nr_l2streams = 32,
		.l2_block_sz = {
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2
		},
		.insert_read_before_invalidate = false,
		.l1_stream_id_reg_offset =
		IPU6_MMU_L1_STREAM_ID_REG_OFFSET,
		.l2_stream_id_reg_offset =
		IPU6_PSYS_MMU1W_L2_STREAM_ID_REG_OFFSET,
	},
	{
		.offset = IPU6_PSYS_IOMMU1R_OFFSET,
		.info_bits = 0,
		.nr_l1streams = 16,
		.l1_block_sz = {
			1, 4, 4, 4, 4, 16, 8, 4, 32,
			16, 16, 2, 2, 2, 1, 12
		},
		.nr_l2streams = 16,
		.l2_block_sz = {
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2
		},
		.insert_read_before_invalidate = false,
		.l1_stream_id_reg_offset =
		IPU6_MMU_L1_STREAM_ID_REG_OFFSET,
		.l2_stream_id_reg_offset =
		IPU6_MMU_L2_STREAM_ID_REG_OFFSET,
	},
	{
		.offset = IPU6_PSYS_IOMMUI_OFFSET,
		.info_bits = 0,
		.nr_l1streams = 0,
		.nr_l2streams = 0,
		.insert_read_before_invalidate = false,
	},
};

struct ipu6_mmu_hwdata {
	struct ipu6_mmu_hw *hwdata;
	unsigned int nr_mmus;
};

static const struct ipu6_mmu_hwdata ipu6_mmu_hwdata_lookup[IPU_SUBSYS_NUM] = {
	[IPU_PSYS] = {
		.hwdata = ipu6_psys_mmu_hwdata,
		.nr_mmus = ARRAY_SIZE(ipu6_psys_mmu_hwdata),
	},
	[IPU_ISYS] = {
		.hwdata = ipu6_isys_mmu_hwdata,
		.nr_mmus = ARRAY_SIZE(ipu6_isys_mmu_hwdata),
	},
};

#define MMU0_TLB_INVALIDATE	1
#define MMU1_TLB_INVALIDATE	0xffff

/* ZLW Enable for each stream in L1 MMU AT where i : 0..15 */
#define MMUV2_AT_REG_L1_ZLW_EN_SID(i)		(0x100 + ((i) * 0x20))

/* ZLW 1D mode Enable for each stream in L1 MMU AT where i : 0..15 */
#define MMUV2_AT_REG_L1_ZLW_1DMODE_SID(i)	(0x100 + ((i) * 0x20) + 0x0004)

/* Set ZLW insertion N pages ahead per stream 1D where i : 0..15 */
#define MMUV2_AT_REG_L1_ZLW_INS_N_AHEAD_SID(i)	(0x100 + ((i) * 0x20) + 0x0008)

/* ZLW 2D mode Enable for each stream in L1 MMU AT where i : 0..15 */
#define MMUV2_AT_REG_L1_ZLW_2DMODE_SID(i)	(0x100 + ((i) * 0x20) + 0x0010)

/* ZLW Insertion for each stream in L1 MMU AT where i : 0..15 */
#define MMUV2_AT_REG_L1_ZLW_INSERTION(i)	(0x100 + ((i) * 0x20) + 0x000c)

#define MMUV2_AT_REG_L1_FW_ZLW_FIFO		(0x100 + \
			(IPU4_MMU_MAX_TLB_L1_STREAMS * 0x20) + 0x003c)

/* FW ZLW has prioty - needed for ZLW invalidations */
#define MMUV2_AT_REG_L1_FW_ZLW_PRIO		(0x100 + \
			(IPU4_MMU_MAX_TLB_L1_STREAMS * 0x20))

static struct ipu6_mmu_hw ipu4_isys_mmu_hwdata[] = {
	{
		.offset = IPU4_ISYS_IOMMU0_OFFSET,
		.info_bits = IPU4_INFO_REQUEST_DESTINATION_PRIMARY,
		.nr_l1streams = 0,
		.nr_l2streams = 0,
		.insert_read_before_invalidate = true,
	},
	{
		.offset = IPU4_ISYS_IOMMU1_OFFSET,
		.info_bits = IPU4_INFO_STREAM_ID_SET(0),
		.nr_l1streams = IPU4_MMU_MAX_TLB_L1_STREAMS,
		.l1_block_sz = {
			8, 16, 16, 16, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 8
		},
		.l1_zlw_en = {
			1, 1, 1, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0
		},
		.l1_zlw_1d_mode = {
			0, 1, 1, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0
		},
		.l1_ins_zlw_ahead_pages = {
			0, 3, 3, 3, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0
		},
		.l1_zlw_2d_mode = {
			0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0
		},
		.nr_l2streams = IPU4_MMU_MAX_TLB_L2_STREAMS,
		.l2_block_sz = {
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2
		},
		.insert_read_before_invalidate = false,
		.zlw_invalidate = false,
		.l1_stream_id_reg_offset =
			IPU4_MMU_L1_STREAM_ID_REG_OFFSET,
		.l2_stream_id_reg_offset =
			IPU4_MMU_L2_STREAM_ID_REG_OFFSET,
	},
};

static struct ipu6_mmu_hw ipu4_psys_mmu_hwdata[] = {
	{
		.offset = IPU4_PSYS_IOMMU0_OFFSET,
		.info_bits =
			IPU4_INFO_REQUEST_DESTINATION_PRIMARY,
		.nr_l1streams = 0,
		.nr_l2streams = 0,
		.insert_read_before_invalidate = true,
	},
	{
		.offset = IPU4_PSYS_IOMMU1_OFFSET,
		.info_bits = IPU4_INFO_STREAM_ID_SET(0),
		.nr_l1streams = IPU4_MMU_MAX_TLB_L1_STREAMS,
		.l1_block_sz = {
			0, 0, 0, 0, 10, 8, 10, 8, 0,
			4, 4, 12, 0, 0, 0, 8
		},
		.l1_zlw_en = {
			0, 0, 0, 0, 1, 1, 1, 1, 0, 1,
			1, 1, 0, 0, 0, 0
		},
		.l1_zlw_1d_mode = {
			0, 0, 0, 0, 1, 1, 1, 1, 0,
			1, 1, 1, 0, 0, 0, 0
		},
		.l1_ins_zlw_ahead_pages = {
			0, 0, 0, 0, 3, 3,
			3, 3, 0, 3, 1, 3,
			0, 0, 0, 0
		},
		.l1_zlw_2d_mode = {
			0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0
		},
		.nr_l2streams = IPU4_MMU_MAX_TLB_L2_STREAMS,
		.l2_block_sz = {
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2
		},
		.insert_read_before_invalidate = false,
		.zlw_invalidate = false,
		.l1_stream_id_reg_offset =
			IPU4_MMU_L1_STREAM_ID_REG_OFFSET,
		.l2_stream_id_reg_offset =
			IPU4_MMU_L2_STREAM_ID_REG_OFFSET,
	},
	{
		.offset = IPU4_PSYS_IOMMU1R_OFFSET,
		.info_bits = IPU4_INFO_STREAM_ID_SET(0),
		.nr_l1streams = IPU4_MMU_MAX_TLB_L1_STREAMS,
		.l1_block_sz = {
			0, 0, 0, 0, 0, 0, 0, 0, 8, 0,
			0, 0, 16, 12, 12, 16
		},
		.l1_zlw_en = {
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 1, 1, 1, 1
		},
		.l1_zlw_1d_mode = {
			0, 0, 0, 0, 0, 0, 0, 0, 1,
			0, 0, 0, 0, 1, 1, 1
		},
		.l1_ins_zlw_ahead_pages = {
			0, 0, 0, 0, 0, 0,
			0, 0, 3, 0, 0, 0,
			0, 0, 0, 0
		},
		.l1_zlw_2d_mode = {
			0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 1, 1
		},
		.nr_l2streams = IPU4_MMU_MAX_TLB_L2_STREAMS,
		.l2_block_sz = {
			2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
			2, 2, 2, 2, 2, 2
		},
		.insert_read_before_invalidate = false,
		.zlw_invalidate = false,
		.l1_stream_id_reg_offset =
			IPU4_MMU_L1_STREAM_ID_REG_OFFSET,
		.l2_stream_id_reg_offset =
			IPU4_MMU_L2_STREAM_ID_REG_OFFSET,
	},
};

static const struct ipu6_mmu_hwdata ipu4_mmu_hwdata_lookup[IPU_SUBSYS_NUM] = {
	[IPU_PSYS] = {
		.hwdata = ipu4_psys_mmu_hwdata,
		.nr_mmus = ARRAY_SIZE(ipu4_psys_mmu_hwdata),
	},
	[IPU_ISYS] = {
		.hwdata = ipu4_isys_mmu_hwdata,
		.nr_mmus = ARRAY_SIZE(ipu4_isys_mmu_hwdata),
	},
};

static void zlw_invalidate(struct ipu6_mmu *mmu, struct ipu6_mmu_hw *mmu_hw)
{
	unsigned int retry = 0;
	unsigned int i, j;
	int ret;

	for (i = 0; i < mmu_hw->nr_l1streams; i++) {
		/* We need to invalidate only the zlw enabled stream IDs */
		if (mmu_hw->l1_zlw_en[i]) {
			/*
			 * Maximum 16 blocks per L1 stream
			 * Write trash buffer iova offset to the FW_ZLW
			 * register. This will trigger pre-fetching of next 16
			 * pages from the page table. So we need to increment
			 * iova address by 16 * 4K to trigger the next 16 pages.
			 * Once this loop is completed, the L1 cache will be
			 * filled with trash buffer translation.
			 *
			 * TODO: Instead of maximum 16 blocks, use the allocated
			 * block size
			 */
			for (j = 0; j < mmu_hw->l1_block_sz[i]; j++)
				writel(mmu->iova_trash_page +
					   j * MMUV2_TRASH_L1_BLOCK_OFFSET,
					   mmu_hw->base +
					   MMUV2_AT_REG_L1_ZLW_INSERTION(i));

			/*
			 * Now we need to fill the L2 cache entry. L2 cache
			 * entries will be automatically updated, based on the
			 * L1 entry. The above loop for L1 will update only one
			 * of the two entries in L2 as the L1 is under 4MB
			 * range. To force the other entry in L2 to update, we
			 * just need to trigger another pre-fetch which is
			 * outside the above 4MB range.
			 */
			writel(mmu->iova_trash_page +
				   MMUV2_TRASH_L2_BLOCK_OFFSET,
				   mmu_hw->base +
				   MMUV2_AT_REG_L1_ZLW_INSERTION(0));
		}
	}

	/*
	 * Wait until AT is ready. FIFO read should return 2 when AT is ready.
	 * Retry value of 1000 is just by guess work to avoid the forever loop.
	 */
	do {
		if (retry > 1000) {
			dev_err(mmu->dev, "zlw invalidation failed\n");
			return;
		}
		ret = readl(mmu_hw->base + MMUV2_AT_REG_L1_FW_ZLW_FIFO);
		retry++;
	} while (ret != 2);
}

static void __ipu4_tlb_invalidate(struct ipu6_mmu *mmu)
{
	struct ipu6_mmu_hw *mmu_hw = mmu->ipu6_mmu_hw;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&mmu->ready_lock, flags);
	if (!mmu->ready) {
		spin_unlock_irqrestore(&mmu->ready_lock, flags);
		return;
	}

	for (i = 0; i < mmu->nr_mmus; i++) {
		if (mmu_hw[i].insert_read_before_invalidate)
			readl(mmu_hw[i].base + REG_L1_PHYS);

		/* Normal invalidate or zlw invalidate */
		if (mmu_hw[i].zlw_invalidate) {
			/* trash buffer must be mapped by now, just in case! */
			WARN_ON(!mmu->iova_trash_page);

			zlw_invalidate(mmu, &mmu_hw[i]);
		} else {
			u32 inv;

			if (mmu_hw[i].nr_l1streams == 32)
				inv = 0xffffffff;
			else if (mmu_hw[i].nr_l1streams == 0)
				inv = MMU0_TLB_INVALIDATE;
			else
				inv = MMU1_TLB_INVALIDATE;
			writel(inv, mmu_hw[i].base + REG_TLB_INVALIDATE);
		}
		wmb();
	}
	spin_unlock_irqrestore(&mmu->ready_lock, flags);
}

static int __ipu4_mmu_hw_init(struct ipu6_mmu *mmu)
{
	struct ipu6_mmu_info *mmu_info = mmu->dmap->mmu_info;
	struct ipu6_mmu_hw *mmu_hw = mmu->ipu6_mmu_hw;

	/* Initialise the each MMU HW block */
	for (unsigned int i = 0; i < mmu->nr_mmus; i++) {
		bool zlw_invalidate_in_any = false;
		unsigned int j;
		u16 block_addr;

		/* Write page table address per MMU */
		writel((phys_addr_t)mmu_info->l1_pt_dma,
		       mmu_hw[i].base + REG_L1_PHYS);

		/* Set info bits per MMU */
		writel(mmu_hw[i].info_bits, mmu_hw[i].base + REG_INFO);

		/* Configure MMU TLB stream configuration for L1 */
		for (j = 0, block_addr = 0; j < mmu_hw[i].nr_l1streams;
		     block_addr += mmu_hw[i].l1_block_sz[j], j++) {
			if (block_addr > IPU6_MAX_LI_BLOCK_ADDR) {
				dev_err(mmu->dev, "invalid L1 configuration\n");
				return -EINVAL;
			}

			/* Write block start address for each streams */
			writel(block_addr, mmu_hw[i].base +
			       mmu_hw[i].l1_stream_id_reg_offset + 4 * j);

			/* Enable ZLW for streams based on the init table */
			writel(mmu_hw[i].l1_zlw_en[j],
			       mmu_hw[i].base + MMUV2_AT_REG_L1_ZLW_EN_SID(j));

			/* To track if zlw is enabled in any streams */
			zlw_invalidate_in_any |= mmu_hw[i].l1_zlw_en[j];

			/* Enable ZLW 1D mode for streams from the init table */
			writel(mmu_hw[i].l1_zlw_1d_mode[j],
			       mmu_hw[i].base +
			       MMUV2_AT_REG_L1_ZLW_1DMODE_SID(j));

			/* Set when the ZLW insertion will happen */
			writel(mmu_hw[i].l1_ins_zlw_ahead_pages[j],
			       mmu_hw[i].base +
			       MMUV2_AT_REG_L1_ZLW_INS_N_AHEAD_SID(j));

			/* Set if ZLW 2D mode active for each streams */
			writel(mmu_hw[i].l1_zlw_2d_mode[j],
			       mmu_hw[i].base +
			       MMUV2_AT_REG_L1_ZLW_2DMODE_SID(j));
		}

		if (zlw_invalidate_in_any)
			writel(1, mmu_hw[i].base + MMUV2_AT_REG_L1_FW_ZLW_PRIO);

		/* Configure MMU TLB stream configuration for L2 */
		for (j = 0, block_addr = 0; j < mmu_hw[i].nr_l2streams;
		     block_addr += mmu_hw[i].l2_block_sz[j], j++) {
			if (block_addr > IPU6_MAX_L2_BLOCK_ADDR) {
				dev_err(mmu->dev, "invalid L2 configuration\n");
				return -EINVAL;
			}

			writel(block_addr, mmu_hw[i].base +
			       mmu_hw[i].l2_stream_id_reg_offset + 4 * j);
		}
	}

	return 0;
}

static void __ipu6_tlb_invalidate(struct ipu6_mmu *mmu)
{
	struct ipu6_mmu_hw *mmu_hw = mmu->ipu6_mmu_hw;
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&mmu->ready_lock, flags);
	if (!mmu->ready) {
		spin_unlock_irqrestore(&mmu->ready_lock, flags);
		return;
	}

	for (i = 0; i < mmu->nr_mmus; i++) {
		/*
		 * To avoid the HW bug induced dead lock in some of the IPU6
		 * MMUs on successive invalidate calls, we need to first do a
		 * read to the page table base before writing the invalidate
		 * register. MMUs which need to implement this WA, will have
		 * the insert_read_before_invalidate flags set as true.
		 * Disregard the return value of the read.
		 */
		if (mmu_hw[i].insert_read_before_invalidate)
			readl(mmu_hw[i].base + REG_L1_PHYS);

		writel(0xffffffff, mmu_hw[i].base + REG_TLB_INVALIDATE);
		/*
		 * The TLB invalidation is a "single cycle" (IOMMU clock cycles)
		 * When the actual MMIO write reaches the IPU6 TLB Invalidate
		 * register, wmb() will force the TLB invalidate out if the CPU
		 * attempts to update the IOMMU page table (or sooner).
		 */
		wmb();
	}
	spin_unlock_irqrestore(&mmu->ready_lock, flags);
}

static int __ipu6_mmu_hw_init(struct ipu6_mmu *mmu)
{
	struct ipu6_mmu_info *mmu_info = mmu->dmap->mmu_info;
	struct ipu6_mmu_hw *mmu_hw = mmu->ipu6_mmu_hw;

	/* Initialise the each MMU HW block */
	for (unsigned int i = 0; i < mmu->nr_mmus; i++) {
		unsigned int j;
		u16 block_addr;

		/* Write page table address per MMU */
		writel((phys_addr_t)mmu_info->l1_pt_dma,
		       mmu_hw[i].base + REG_L1_PHYS);

		/* Set info bits per MMU */
		writel(mmu_hw[i].info_bits, mmu_hw[i].base + REG_INFO);

		/* Configure MMU TLB stream configuration for L1 */
		for (j = 0, block_addr = 0; j < mmu_hw[i].nr_l1streams;
		     block_addr += mmu_hw[i].l1_block_sz[j], j++) {
			if (block_addr > IPU6_MAX_LI_BLOCK_ADDR) {
				dev_err(mmu->dev, "invalid L1 configuration\n");
				return -EINVAL;
			}

			/* Write block start address for each streams */
			writel(block_addr, mmu_hw[i].base +
			       mmu_hw[i].l1_stream_id_reg_offset + 4 * j);
		}

		/* Configure MMU TLB stream configuration for L2 */
		for (j = 0, block_addr = 0; j < mmu_hw[i].nr_l2streams;
		     block_addr += mmu_hw[i].l2_block_sz[j], j++) {
			if (block_addr > IPU6_MAX_L2_BLOCK_ADDR) {
				dev_err(mmu->dev, "invalid L2 configuration\n");
				return -EINVAL;
			}

			writel(block_addr, mmu_hw[i].base +
			       mmu_hw[i].l2_stream_id_reg_offset + 4 * j);
		}
	}

	return 0;
}

static int mmu_init_hw_data(const struct ipu6_mmu_hwdata *hwdata_lookup,
			    struct ipu6_mmu *mmu, struct device *dev,
			    void __iomem *base)
{
	const struct ipu6_mmu_hwdata *lookup;
	struct ipu6_mmu_hw *mmu_hw, *src;
	unsigned int i, nr_mmus;

	if (mmu->mmid >= IPU_SUBSYS_NUM)
		return -EINVAL;

	lookup = &hwdata_lookup[mmu->mmid];
	src = lookup->hwdata;
	nr_mmus = lookup->nr_mmus;

	mmu_hw = devm_kcalloc(dev, nr_mmus, sizeof(*mmu_hw), GFP_KERNEL);
	if (!mmu_hw)
		return -ENOMEM;

	for (i = 0; i < nr_mmus; i++) {
		if (src[i].nr_l1streams > IPU6_MMU_MAX_TLB_L1_STREAMS ||
		    src[i].nr_l2streams > IPU6_MMU_MAX_TLB_L2_STREAMS)
			return -EINVAL;

		mmu_hw[i] = src[i];
		mmu_hw[i].base = base + src[i].offset;
	}

	mmu->nr_mmus = nr_mmus;
	mmu->ipu6_mmu_hw = mmu_hw;

	return 0;
}

static int __ipu6_mmu_init_hw_data(struct ipu6_mmu *mmu, struct device *dev,
				   void __iomem *base)
{
	return mmu_init_hw_data(ipu6_mmu_hwdata_lookup, mmu, dev, base);
}

static int __ipu4_mmu_init_hw_data(struct ipu6_mmu *mmu, struct device *dev,
				   void __iomem *base)
{
	return mmu_init_hw_data(ipu4_mmu_hwdata_lookup, mmu, dev, base);
}

const struct ipu6_mmu_hw_ops ipu6_mmu_ops = {
	.init_hw_data = __ipu6_mmu_init_hw_data,
	.hw_init = __ipu6_mmu_hw_init,
	.tlb_invalidate = __ipu6_tlb_invalidate,
};

const struct ipu6_mmu_hw_ops ipu4_mmu_ops = {
	.init_hw_data = __ipu4_mmu_init_hw_data,
	.hw_init = __ipu4_mmu_hw_init,
	.tlb_invalidate = __ipu4_tlb_invalidate,
};
