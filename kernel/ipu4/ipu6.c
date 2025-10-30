// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 - 2023 Intel Corporation
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/pci-ats.h>
#include <linux/pm_runtime.h>
#include <linux/trace.h>
#include <linux/vmalloc.h>

#include "ambu-ipu-bridge.h"
#include "ipu6.h"
#include "ipu6-cpd.h"
#include "ipu6-isys.h"
#include "ipu6-mmu.h"
#include "ipu6-platform-buttress-regs.h"
#include "ipu4-platform-isys-csi2-reg.h"
#include "ipu6-platform-regs.h"

const char *ipu_last_rw_func;
EXPORT_SYMBOL(ipu_last_rw_func);

#define IPU6_PCI_BAR		0

struct ipu6_cell_program {
	u32 magic_number;

	u32 blob_offset;
	u32 blob_size;

	u32 start[3];

	u32 icache_source;
	u32 icache_target;
	u32 icache_size;

	u32 pmem_source;
	u32 pmem_target;
	u32 pmem_size;

	u32 data_source;
	u32 data_target;
	u32 data_size;

	u32 bss_target;
	u32 bss_size;

	u32 cell_id;
	u32 regs_addr;

	u32 cell_pmem_data_bus_address;
	u32 cell_dmem_data_bus_address;
	u32 cell_pmem_control_bus_address;
	u32 cell_dmem_control_bus_address;

	u32 next;
	u32 dummy[2];
};

static unsigned int ipu4_csi_offsets[] = {
	0x64000, 0x65000, 0x66000, 0x67000, 0x6C000, 0x6C800
};

// From iei-4.19.217 ipu4.c
static const struct ipu6_isys_internal_pdata isys_ipdata = {
	.hw_variant = {
	    .offset = IPU4_ISYS_OFFSET,
		.nr_mmus = 2,
		.mmu_hw = {{
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
		},
		.dmem_offset = IPU4_ISYS_DMEM_OFFSET,
		.spc_offset = IPU4_ISYS_SPC_OFFSET,
	},
	.isys_dma_overshoot = IPU4_ISYS_OVERALLOC_MIN,
	.num_parallel_streams = IPU4_STREAM_ID_MAX,
	.csi2.nports = ARRAY_SIZE(ipu4_csi_offsets),
	.csi2.offsets = ipu4_csi_offsets,
	.max_streams = IPU4_ISYS_MAX_STREAMS,
	.max_sram_blocks = IPU4_ISYS_MAX_STREAMS,
	.max_send_queues = IPU4_N_MAX_SEND_QUEUES,

	/* Consider 1 slot per stream since driver is not expected to pipeline
	 * device commands for the same stream
	 */
	.max_devq_size = IPU4_ISYS_MAX_STREAMS,
};

static const struct ipu6_psys_internal_pdata psys_ipdata = {
	.hw_variant = {
		       .offset = IPU4_PSYS_OFFSET,
		       .nr_mmus = 3,
		       .mmu_hw = {
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
			},
		       .dmem_offset = IPU4_PSYS_DMEM_OFFSET,
		       .spc_offset = IPU4_PSYS_SPC_OFFSET,
	},
};

static const struct ipu6_buttress_ctrl ipu6_isys_buttress_ctrl = {
	.ratio = IPU6_IS_FREQ_CTL_DEFAULT_RATIO,
	.qos_floor = IPU6_IS_FREQ_CTL_DEFAULT_QOS_FLOOR_RATIO,
	.freq_ctl = BUTTRESS_REG_IS_FREQ_CTL,
	.pwr_sts_shift = IPU6_BUTTRESS_PWR_STATE_IS_PWR_SHIFT,
	.pwr_sts_mask = IPU6_BUTTRESS_PWR_STATE_IS_PWR_MASK,
	.pwr_sts_on = IPU6_BUTTRESS_PWR_STATE_UP_DONE,
	.pwr_sts_off = IPU6_BUTTRESS_PWR_STATE_DN_DONE,
};

static const struct ipu6_buttress_ctrl ipu6_psys_buttress_ctrl = {
	.ratio = IPU6_PS_FREQ_CTL_DEFAULT_RATIO,
	.qos_floor = IPU6_PS_FREQ_CTL_DEFAULT_QOS_FLOOR_RATIO,
	.freq_ctl = BUTTRESS_REG_PS_FREQ_CTL,
	.pwr_sts_shift = IPU6_BUTTRESS_PWR_STATE_PS_PWR_SHIFT,
	.pwr_sts_mask = IPU6_BUTTRESS_PWR_STATE_PS_PWR_MASK,
	.pwr_sts_on = IPU6_BUTTRESS_PWR_STATE_UP_DONE,
	.pwr_sts_off = IPU6_BUTTRESS_PWR_STATE_DN_DONE,
};

static const struct ipu6_buttress_ctrl ipu4_isys_buttress_ctrl = {
	.ratio = IPU4_IS_FREQ_CTL_DIVISOR,
	.qos_floor = 0,
	.freq_ctl = BUTTRESS_REG_IS_FREQ_CTL,
	.pwr_sts_shift = IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_SHIFT,
	.pwr_sts_mask = IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_MASK,
	.pwr_sts_on = IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_IS_RDY,
	.pwr_sts_off = IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_IDLE,
};

static const struct ipu6_buttress_ctrl ipu4_psys_buttress_ctrl = {
	.ratio = IPU4_PS_FREQ_CTL_DEFAULT_RATIO,
	.qos_floor = IPU4_PS_FREQ_CTL_DEFAULT_RATIO,
	.freq_ctl = BUTTRESS_REG_PS_FREQ_CTL,
	.pwr_sts_shift = IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_SHIFT,
	.pwr_sts_mask = IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_MASK,
	.pwr_sts_on = IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_PS_PWR_UP,
	.pwr_sts_off = IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_IDLE,
};

// List of all registers that the old and new driver reads,
// created from mmiotrace's
static const u32 readable_regs[] = {
	// 0x90000000 offset removed since it is "included" in isp->base
	0x000008,
	0x00000c,
	0x00005c,
	0x000094,
	0x000098,
	0x00009c,
	0x000164,
	0x000168,
	0x000300,
	0x000308,
	0x00030c,
	0x100000,
	0x108008,
	0x108028,
	0x10802c,
	0x108070,
	0x108074,
	0x164000 + CSI2_REG_CSI_RX_ENABLE,
	0x164000 + CSI2_REG_CSI_RX_CONFIG,
	0x164000 + CSI2_REG_CSI2PART_IRQ_STATUS,
	0x164000 + CSI2_REG_CSI2PART_IRQ_ENABLE,
	0x164000 + CSI2_REG_CSIRX_IRQ_STATUS,
	0x164000 + CSI2_REG_CSIRX_IRQ_ENABLE,
	0x164000 + CSI2_REG_CSI2S2M_IRQ_STATUS,
	0x164000 + CSI2_REG_CSI2S2M_IRQ_ENABLE,
	0x164c00 + CSI2_REG_CSI_RX_ENABLE,
	0x164c00 + CSI2_REG_CSI_RX_CONFIG,
	0x164c00 + CSI2_REG_CSI2PART_IRQ_STATUS,
	0x164c00 + CSI2_REG_CSI2PART_IRQ_ENABLE,
	0x164c00 + CSI2_REG_CSIRX_IRQ_STATUS,
	0x164c00 + CSI2_REG_CSIRX_IRQ_ENABLE,
	0x164c00 + CSI2_REG_CSI2S2M_IRQ_STATUS,
	0x164c00 + CSI2_REG_CSI2S2M_IRQ_ENABLE,
	0x17c000,
	0x17c004,
	0x17c008,
	0x17c00c,
	0x17c010,
	0x17c414,
	0x17c418,
	0x1e0004,
	0x408000,
	0x4b0004,
};

void ipu_dump_state(struct ipu6_device *isp, const char *context)
{
	int i;
	static int ipu_dump_state_iteration;
	static DEFINE_MUTEX(mutex);

	mutex_lock(&mutex);
	ipu_dump_state_iteration++;

	mmiotrace_printk("%s in context %d %s begin\n",
			 __func__,
			 ipu_dump_state_iteration,
			 context);
	for (i = 0; i < ARRAY_SIZE(readable_regs); ++i)
		(void)readl(isp->base + readable_regs[i]);

	mmiotrace_printk("%s in context %d %s end\n",
			 __func__,
			 ipu_dump_state_iteration,
			 context);
	mutex_unlock(&mutex);
}
EXPORT_SYMBOL_GPL(ipu_dump_state);

void ipu6_configure_spc(struct ipu6_device *isp,
			const struct ipu6_hw_variants *hw_variant,
			int pkg_dir_idx, void __iomem *base, u64 *pkg_dir,
			dma_addr_t pkg_dir_dma_addr)
{
	void __iomem *dmem_base = base + hw_variant->dmem_offset;
	void __iomem *spc_regs_base = base + hw_variant->spc_offset;
	u32 val;

	val = readl(spc_regs_base + IPU6_PSYS_REG_SPC_STATUS_CTRL);
	val |= IPU6_PSYS_SPC_STATUS_CTRL_ICACHE_INVALIDATE;
	writel(val, spc_regs_base + IPU6_PSYS_REG_SPC_STATUS_CTRL);

	if (isp->secure_mode)
		writel(IPU6_PKG_DIR_IMR_OFFSET, dmem_base);
	else
		WARN(1, "non-secure_,mode not implemented");
}
EXPORT_SYMBOL_NS_GPL(ipu6_configure_spc, INTEL_IPU6);

static int ipu6_isys_check_fwnode_graph(struct fwnode_handle *fwnode)
{
	struct fwnode_handle *endpoint;

	if (IS_ERR_OR_NULL(fwnode))
		return -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (endpoint) {
		fwnode_handle_put(endpoint);
		return 0;
	}

	return ipu6_isys_check_fwnode_graph(fwnode->secondary);
}

static struct ipu6_bus_device *
ipu6_isys_init(struct pci_dev *pdev, struct device *parent,
	       const struct ipu6_buttress_ctrl *ctrl, void __iomem *base,
	       const struct ipu6_isys_internal_pdata *ipdata)
{
	struct fwnode_handle *fwnode = dev_fwnode(&pdev->dev);
	struct ipu6_bus_device *isys_adev;
	struct ipu6_isys_pdata *pdata;
	int ret;

	/* check fwnode at first, fallback into bridge if no fwnode graph */
	ret = ipu6_isys_check_fwnode_graph(fwnode);
	if (ret) {
		if (fwnode && !IS_ERR_OR_NULL(fwnode->secondary)) {
			dev_err(&pdev->dev,
				"fwnode graph has no endpoints connection\n");
			return ERR_PTR(-EINVAL);
		}

		ret = ambu_ipu_bridge_init(&pdev->dev);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_err_probe(&pdev->dev, ret,
					      "IPU6 bridge init failed\n");
			return ERR_PTR(ret);
		}
	}

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->base = base;
	pdata->ipdata = ipdata;

	isys_adev = ipu6_bus_initialize_device(pdev, parent, pdata, ctrl,
					       IPU6_ISYS_NAME);
	if (IS_ERR(isys_adev)) {
		dev_err_probe(&pdev->dev, PTR_ERR(isys_adev),
			      "ipu6_bus_initialize_device isys failed\n");
		kfree(pdata);
		return ERR_CAST(isys_adev);
	}

	isys_adev->mmu = ipu6_mmu_init(&pdev->dev, base, ISYS_MMID,
				       &ipdata->hw_variant);
	if (IS_ERR(isys_adev->mmu)) {
		dev_err_probe(&pdev->dev, PTR_ERR(isys_adev),
			      "ipu6_mmu_init(isys_adev->mmu) failed\n");
		put_device(&isys_adev->auxdev.dev);
		kfree(pdata);
		return ERR_CAST(isys_adev->mmu);
	}

	isys_adev->mmu->dev = &isys_adev->auxdev.dev;

	ret = ipu6_bus_add_device(isys_adev);
	if (ret) {
		kfree(pdata);
		return ERR_PTR(ret);
	}

	return isys_adev;
}

static struct ipu6_bus_device *
ipu6_psys_init(struct pci_dev *pdev, struct device *parent,
	       const struct ipu6_buttress_ctrl *ctrl, void __iomem *base,
	       const struct ipu6_psys_internal_pdata *ipdata)
{
	struct ipu6_bus_device *psys_adev;
	struct ipu6_psys_pdata *pdata;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->base = base;
	pdata->ipdata = ipdata;

	psys_adev = ipu6_bus_initialize_device(pdev, parent, pdata, ctrl,
					       IPU6_PSYS_NAME);
	if (IS_ERR(psys_adev)) {
		dev_err_probe(&pdev->dev, PTR_ERR(psys_adev),
			      "ipu6_bus_initialize_device psys failed\n");
		kfree(pdata);
		return ERR_CAST(psys_adev);
	}

	psys_adev->mmu = ipu6_mmu_init(&pdev->dev, base, PSYS_MMID,
				       &ipdata->hw_variant);
	if (IS_ERR(psys_adev->mmu)) {
		dev_err_probe(&pdev->dev, PTR_ERR(psys_adev),
			      "ipu6_mmu_init(psys_adev->mmu) failed\n");
		put_device(&psys_adev->auxdev.dev);
		kfree(pdata);
		return ERR_CAST(psys_adev->mmu);
	}

	psys_adev->mmu->dev = &psys_adev->auxdev.dev;

	ret = ipu6_bus_add_device(psys_adev);
	if (ret) {
		kfree(pdata);
		return ERR_PTR(ret);
	}

	return psys_adev;
}

static int ipu6_pci_config_setup(struct pci_dev *dev, u8 hw_ver)
{
	int ret;

	/* disable IPU6 PCI ATS on mtl ES2 */
	if (is_ipu6ep_mtl(hw_ver) && boot_cpu_data.x86_stepping == 0x2 &&
	    pci_ats_supported(dev))
		pci_disable_ats(dev);

	/* No PCI msi capability for IPU6EP */
	if (is_ipu6ep(hw_ver) || is_ipu6ep_mtl(hw_ver)) {
		/* likely do nothing as msi not enabled by default */
		pci_disable_msi(dev);
		return 0;
	}

	ret = pci_alloc_irq_vectors(dev, 1, 1, PCI_IRQ_MSI);
	if (ret < 0)
		return dev_err_probe(&dev->dev, ret, "Request msi failed");

	return 0;
}

static void ipu6_configure_vc_mechanism(struct ipu6_device *isp)
{
	u32 val = readl(isp->base + BUTTRESS_REG_BTRS_CTRL);

	if (IPU6_BTRS_ARB_STALL_MODE_VC0 == IPU6_BTRS_ARB_MODE_TYPE_STALL)
		val |= BUTTRESS_REG_BTRS_CTRL_STALL_MODE_VC0;
	else
		val &= ~BUTTRESS_REG_BTRS_CTRL_STALL_MODE_VC0;

	if (IPU6_BTRS_ARB_STALL_MODE_VC1 == IPU6_BTRS_ARB_MODE_TYPE_STALL)
		val |= BUTTRESS_REG_BTRS_CTRL_STALL_MODE_VC1;
	else
		val &= ~BUTTRESS_REG_BTRS_CTRL_STALL_MODE_VC1;

	writel(val, isp->base + BUTTRESS_REG_BTRS_CTRL);
}

static int request_cpd_fw(const struct firmware **firmware_p, const char *name,
			  struct device *device)
{
	const struct firmware *fw;
	struct firmware *dst;
	int ret = 0;

	ret = request_firmware(&fw, name, device);
	if (ret)
		return ret;

	if (is_vmalloc_addr(fw->data)) {
		*firmware_p = fw;
		return 0;
	}

	dst = kzalloc(sizeof(*dst), GFP_KERNEL);
	if (!dst) {
		ret = -ENOMEM;
		goto release_firmware;
	}

	dst->size = fw->size;
	dst->data = vmalloc(fw->size);
	if (!dst->data) {
		kfree(dst);
		ret = -ENOMEM;
		goto release_firmware;
	}

	memcpy((void *)dst->data, fw->data, fw->size);
	*firmware_p = dst;

release_firmware:
	release_firmware(fw);

	return ret;
}

static int ipu6_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	const struct ipu6_buttress_ctrl *isys_ctrl = NULL, *psys_ctrl = NULL;
	void __iomem *isys_base = NULL;
	void __iomem *psys_base = NULL;
	struct ipu6_device *isp;
	phys_addr_t phys;
	int ret;

	trace_printk_init_buffers();
	isp = devm_kzalloc(&pdev->dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	isp->pdev = pdev;
	INIT_LIST_HEAD(&isp->devices);

	ret = pcim_enable_device(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Enable PCI device failed\n");

	phys = pci_resource_start(pdev, IPU6_PCI_BAR);
	dev_dbg(&pdev->dev, "IPU6 PCI bar[%u] = %pa\n", IPU6_PCI_BAR, &phys);

	ret = pcim_iomap_regions(pdev, 1 << IPU6_PCI_BAR, pci_name(pdev));
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to I/O mem remappinp\n");

	isp->base = pcim_iomap_table(pdev)[IPU6_PCI_BAR];
	pci_set_drvdata(pdev, isp);
	pci_set_master(pdev);

	isp->cpd_metadata_cmpnt_size = sizeof(struct ipu6_cpd_metadata_cmpnt);
	switch (id->device) {
	case PCI_DEVICE_ID_INTEL_IPU6:
		isp->hw_ver = IPU6_VER_6;
		isp->cpd_fw_name = IPU6_FIRMWARE_NAME;
		isp->buttress.reg_irq_sts = BUTTRESS_REG_ISR_STATUS;
		break;
	case PCI_DEVICE_ID_INTEL_IPU6SE:
		isp->hw_ver = IPU6_VER_6SE;
		isp->cpd_fw_name = IPU6SE_FIRMWARE_NAME;
		isp->cpd_metadata_cmpnt_size =
			sizeof(struct ipu6se_cpd_metadata_cmpnt);
		isp->buttress.reg_irq_sts = BUTTRESS_REG_ISR_STATUS;
		break;
	case PCI_DEVICE_ID_INTEL_IPU6EP_ADLP:
	case PCI_DEVICE_ID_INTEL_IPU6EP_ADLN:
	case PCI_DEVICE_ID_INTEL_IPU6EP_RPLP:
		isp->hw_ver = IPU6_VER_6EP;
		isp->cpd_fw_name = IPU6EP_FIRMWARE_NAME;
		isp->buttress.reg_irq_sts = BUTTRESS_REG_ISR_STATUS;
		break;
	case PCI_DEVICE_ID_INTEL_IPU6EP_MTL:
		isp->hw_ver = IPU6_VER_6EP_MTL;
		isp->cpd_fw_name = IPU6EPMTL_FIRMWARE_NAME;
		isp->buttress.reg_irq_sts = BUTTRESS_REG_ISR_STATUS;
		break;
	case PCI_DEVICE_ID_INTEL_IPU4:
		isp->hw_ver = IPU4_VER_4;
		isp->cpd_fw_name = IPU4_FIRMWARE_NAME;
		// IPU4 uses same cpd metadata cmpnt as ipu6se
		// (smaller hash size)
		isp->cpd_metadata_cmpnt_size =
			sizeof(struct ipu6se_cpd_metadata_cmpnt);
		isp->buttress.reg_irq_sts = BUTTRESS_REG_ISR_ENABLED_STATUS;
		break;
	default:
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "Unsupported IPU6 device %x\n",
				     id->device);
	}

	isys_base = isp->base + isys_ipdata.hw_variant.offset;
	psys_base = isp->base + psys_ipdata.hw_variant.offset;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(39));
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to set DMA mask\n");

	ret = dma_set_max_seg_size(&pdev->dev, UINT_MAX);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to set max_seg_size\n");

	ret = ipu6_pci_config_setup(pdev, isp->hw_ver);
	if (ret)
		return ret;

	ret = ipu6_buttress_init(isp);
	if (ret)
		return ret;

	ret = request_cpd_fw(&isp->cpd_fw, isp->cpd_fw_name, &pdev->dev);
	if (ret) {
		dev_err_probe(&isp->pdev->dev, ret,
			      "Requesting signed firmware %s failed\n",
			      isp->cpd_fw_name);
		goto buttress_exit;
	}

	ret = ipu6_cpd_validate_cpd_file(isp, isp->cpd_fw->data,
					 isp->cpd_fw->size);
	if (ret) {
		dev_err_probe(&isp->pdev->dev, ret,
			      "Failed to validate cpd\n");
		goto out_ipu6_bus_del_devices;
	}

	isys_ctrl = is_ipu4(isp->hw_ver) ?
			&ipu4_isys_buttress_ctrl : &ipu6_isys_buttress_ctrl;
	isp->isys = ipu6_isys_init(pdev, &pdev->dev, isys_ctrl, isys_base,
				   &isys_ipdata);
	if (IS_ERR(isp->isys)) {
		ret = PTR_ERR(isp->isys);
		goto out_ipu6_bus_del_devices;
	}

	psys_ctrl = is_ipu4(isp->hw_ver) ?
			&ipu4_psys_buttress_ctrl : &ipu6_psys_buttress_ctrl;

	isp->psys = ipu6_psys_init(pdev, &isp->isys->auxdev.dev, psys_ctrl,
				   psys_base, &psys_ipdata);
	if (IS_ERR(isp->psys)) {
		ret = PTR_ERR(isp->psys);
		goto out_ipu_bridge_uninit;
	}

	ret = pm_runtime_resume_and_get(&isp->psys->auxdev.dev);
	if (ret < 0)
		goto out_ipu_bridge_uninit;

	ret = ipu6_mmu_hw_init(isp->psys->mmu);
	if (ret) {
		dev_err_probe(&isp->pdev->dev, ret,
			      "Failed to set MMU hardware\n");
		goto out_ipu_bridge_uninit;
	}

	ret = ipu6_buttress_map_fw_image(isp->psys, isp->cpd_fw,
					 &isp->psys->fw_sgt);
	if (ret) {
		dev_err_probe(&isp->pdev->dev, ret, "failed to map fw image\n");
		goto out_ipu_bridge_uninit;
	}

	ret = ipu6_cpd_create_pkg_dir(isp->psys, isp->cpd_fw->data);
	if (ret) {
		dev_err_probe(&isp->pdev->dev, ret,
			      "failed to create pkg dir\n");
		goto out_ipu_bridge_uninit;
	}

	ret = devm_request_threaded_irq(&pdev->dev, pdev->irq,
					ipu6_buttress_isr,
					ipu6_buttress_isr_threaded,
					IRQF_SHARED, IPU4_NAME, isp);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "Requesting irq failed\n");
		goto out_ipu_bridge_uninit;
	}

	ret = ipu6_buttress_authenticate(isp);
	if (ret) {
		dev_err_probe(&isp->pdev->dev, ret,
			      "FW authentication failed\n");
		goto out_ipu_bridge_uninit;
	}

	ipu6_mmu_hw_cleanup(isp->psys->mmu);
	pm_runtime_put(&isp->psys->auxdev.dev);

	/* Configure the arbitration mechanisms for VC requests */
	ipu6_configure_vc_mechanism(isp);

	// NB: Don't change, currently used as done signal in test
	dev_info(&pdev->dev, "IPU4 PCI driver ready\n");

	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	isp->bus_ready_to_probe = true;

	return 0;

out_ipu_bridge_uninit:
	ambu_ipu_bridge_uninit(&pdev->dev);
out_ipu6_bus_del_devices:
	if (isp->psys) {
		ipu6_cpd_free_pkg_dir(isp->psys);
		ipu6_buttress_unmap_fw_image(isp->psys, &isp->psys->fw_sgt);
	}
	if (!IS_ERR_OR_NULL(isp->psys) && !IS_ERR_OR_NULL(isp->psys->mmu))
		ipu6_mmu_cleanup(isp->psys->mmu);
	if (!IS_ERR_OR_NULL(isp->isys) && !IS_ERR_OR_NULL(isp->isys->mmu))
		ipu6_mmu_cleanup(isp->isys->mmu);
	ipu6_bus_del_devices(pdev);
	release_firmware(isp->cpd_fw);
buttress_exit:
	ipu6_buttress_exit(isp);

	return ret;
}

static void ipu6_pci_remove(struct pci_dev *pdev)
{
	struct ipu6_device *isp = pci_get_drvdata(pdev);
	struct ipu6_mmu *isys_mmu = isp->isys->mmu;
	struct ipu6_mmu *psys_mmu = isp->psys->mmu;

	devm_free_irq(&pdev->dev, pdev->irq, isp);
	ipu6_cpd_free_pkg_dir(isp->psys);

	ipu6_buttress_unmap_fw_image(isp->psys, &isp->psys->fw_sgt);

	ipu6_buttress_exit(isp);

	ipu6_bus_del_devices(pdev);

	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	release_firmware(isp->cpd_fw);

	ipu6_mmu_cleanup(psys_mmu);
	ipu6_mmu_cleanup(isys_mmu);

	ambu_ipu_bridge_uninit(&pdev->dev);
}

static void ipu6_pci_reset_prepare(struct pci_dev *pdev)
{
	struct ipu6_device *isp = pci_get_drvdata(pdev);

	pm_runtime_forbid(&isp->pdev->dev);
}

static void ipu6_pci_reset_done(struct pci_dev *pdev)
{
	struct ipu6_device *isp = pci_get_drvdata(pdev);

	ipu6_buttress_restore(isp);
	if (isp->secure_mode)
		ipu6_buttress_reset_authentication(isp);

	isp->need_ipc_reset = true;
	pm_runtime_allow(&isp->pdev->dev);
}

/*
 * PCI base driver code requires driver to provide these to enable
 * PCI device level PM state transitions (D0<->D3)
 */
static int ipu6_suspend(struct device *dev)
{
	return 0;
}

static int ipu6_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct ipu6_device *isp = pci_get_drvdata(pdev);
	struct ipu6_buttress *b = &isp->buttress;
	int ret;

	/* Configure the arbitration mechanisms for VC requests */
	ipu6_configure_vc_mechanism(isp);

	isp->secure_mode = ipu6_buttress_get_secure_mode(isp);
	dev_info(dev, "IPU6 in %s mode\n",
		 isp->secure_mode ? "secure" : "non-secure");

	ipu6_buttress_restore(isp);

	ret = ipu6_buttress_ipc_reset(isp, &b->cse);
	if (ret)
		dev_err(&isp->pdev->dev, "IPC reset protocol failed!\n");

	ret = pm_runtime_resume_and_get(&isp->psys->auxdev.dev);
	if (ret < 0) {
		dev_err(&isp->psys->auxdev.dev, "Failed to get runtime PM\n");
		return 0;
	}

	ret = ipu6_buttress_authenticate(isp);
	if (ret)
		dev_err(&isp->pdev->dev, "FW authentication failed(%d)\n", ret);

	pm_runtime_put(&isp->psys->auxdev.dev);

	return 0;
}

static int ipu6_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct ipu6_device *isp = pci_get_drvdata(pdev);
	int ret;

	ipu6_configure_vc_mechanism(isp);
	ipu6_buttress_restore(isp);

	if (isp->need_ipc_reset) {
		struct ipu6_buttress *b = &isp->buttress;

		isp->need_ipc_reset = false;
		ret = ipu6_buttress_ipc_reset(isp, &b->cse);
		if (ret)
			dev_err(&isp->pdev->dev, "IPC reset protocol failed\n");
	}

	return 0;
}

static const struct dev_pm_ops ipu6_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(&ipu6_suspend, &ipu6_resume)
	SET_RUNTIME_PM_OPS(&ipu6_suspend, &ipu6_runtime_resume, NULL)
};

static const struct pci_device_id ipu6_pci_tbl[] = {
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_IPU6) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_IPU6SE) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_IPU6EP_ADLP) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_IPU6EP_ADLN) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_IPU6EP_RPLP) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_IPU6EP_MTL) },
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_IPU4)},
	{ }
};
MODULE_DEVICE_TABLE(pci, ipu6_pci_tbl);

static const struct pci_error_handlers pci_err_handlers = {
	.reset_prepare = ipu6_pci_reset_prepare,
	.reset_done = ipu6_pci_reset_done,
};

static struct pci_driver ipu6_pci_driver = {
	.name = IPU4_NAME,
	.id_table = ipu6_pci_tbl,
	.probe = ipu6_pci_probe,
	.remove = ipu6_pci_remove,
	.driver = {
		.pm = pm_ptr(&ipu6_pm_ops),
	},
	.err_handler = &pci_err_handlers,
};

module_pci_driver(ipu6_pci_driver);

MODULE_IMPORT_NS(INTEL_IPU_BRIDGE);
MODULE_AUTHOR("Sakari Ailus <sakari.ailus@linux.intel.com>");
MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_AUTHOR("Qingwu Zhang <qingwu.zhang@intel.com>");
MODULE_AUTHOR("Yunliang Ding <yunliang.ding@intel.com>");
MODULE_AUTHOR("Hongju Wang <hongju.wang@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel IPU4 PCI driver");
