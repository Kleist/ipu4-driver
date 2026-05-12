/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2013 - 2023 Intel Corporation */

#ifndef IPU6_FW_COM_PRIV_H
#define IPU6_FW_COM_PRIV_H

/*
 * Internal layouts for ipu6-fw-com.c. Exposed in a -priv header so the
 * ipu4_fw_com_kunit suite under tools/linux-patches/.../tests/ can pin
 * the syscom ring-buffer mechanics without going through
 * ipu6_fw_com_prepare() (which requires a real DMA backend). Not for
 * use outside the fw_com translation unit and its tests.
 */

#include <linux/types.h>

struct ipu6_bus_device;

/* Shared structure between driver and FW - do not modify */
struct ipu6_fw_sys_queue {
	u64 host_address;
	u32 vied_address;
	u32 size;
	u32 token_size;
	u32 wr_reg;	/* reg number in subsystem's regmem */
	u32 rd_reg;
	u32 _align;
} __packed;

struct ipu6_fw_com_context {
	struct ipu6_bus_device *adev;
	void __iomem *dmem_addr;
	int (*cell_ready)(struct ipu6_bus_device *adev);
	void (*cell_start)(struct ipu6_bus_device *adev);

	void *dma_buffer;
	dma_addr_t dma_addr;
	unsigned int dma_size;

	struct ipu6_fw_sys_queue *input_queue;	/* host to SP */
	struct ipu6_fw_sys_queue *output_queue;	/* SP to host */

	u32 config_vied_addr;

	unsigned int buttress_boot_offset;
	void __iomem *base_addr;
};

#define FW_COM_WR_REG 0
#define FW_COM_RD_REG 4

#endif /* IPU6_FW_COM_PRIV_H */
