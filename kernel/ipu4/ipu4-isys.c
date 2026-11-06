// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013--2024 Intel Corporation
 *
 * IPU4 ISYS hardware: interrupt setup and the CSI-2 receivers, which
 * IPU4 drives directly instead of through a PHY.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/types.h>

#include "ipu4-isys.h"
#include "ipu4-platform-isys-csi2-reg.h"
#include "ipu4-platform-regs.h"
#include "ipu6-bus.h"
#include "ipu6-isys.h"
#include "ipu6-isys-csi2.h"

/* The IPU4 CSI-2 receiver has four virtual channels. */
#define IPU4_CSI2_NR_OF_VC	4

#define DIV_SHIFT	8
#define CSI2_ACCINV	8

static u32 calc_timing(s32 a, s32 b, s64 link_freq, s32 accinv)
{
	return accinv * a + (accinv * b * (500000000 >> DIV_SHIFT)
			     / (s32)(link_freq >> DIV_SHIFT));
}

static int ipu4_isys_csi2_calc_timing(struct ipu6_isys_csi2 *csi2,
				      struct ipu6_isys_csi2_timing *timing)
{
	s64 link_freq;

	link_freq = ipu6_isys_csi2_get_link_freq(csi2);
	if (link_freq < 0)
		return link_freq;

	timing->ctermen = calc_timing(CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_A,
				      CSI2_CSI_RX_DLY_CNT_TERMEN_CLANE_B,
				      link_freq, CSI2_ACCINV);
	timing->csettle = calc_timing(CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_A,
				      CSI2_CSI_RX_DLY_CNT_SETTLE_CLANE_B,
				      link_freq, CSI2_ACCINV);
	timing->dtermen = calc_timing(CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_A,
				      CSI2_CSI_RX_DLY_CNT_TERMEN_DLANE_B,
				      link_freq, CSI2_ACCINV);
	timing->dsettle = calc_timing(CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_A,
				      CSI2_CSI_RX_DLY_CNT_SETTLE_DLANE_B,
				      link_freq, CSI2_ACCINV);

	return 0;
}

void ipu4_isys_register_errors(struct ipu6_isys_csi2 *csi2)
{
	u32 status = readl(csi2->base + CSI2_REG_CSIRX_IRQ_STATUS);

	writel(status, csi2->base + CSI2_REG_CSIRX_IRQ_CLEAR);
	csi2->receiver_errors |= status;
}

void ipu4_isys_csi2_error(struct ipu6_isys_csi2 *csi2)
{
	struct device *dev = &csi2->isys->adev->auxdev.dev;
	/*
	 * Strings corresponding to CSI-2 receiver errors are here.
	 * Corresponding macros are defined in the header file.
	 */
	static const struct ipu_isys_csi2_error {
		const char *error_string;
		bool is_info_only;
	} errors[] = {
		{"Single packet header error corrected", true},
		{"Multiple packet header errors detected", true},
		{"Payload checksum (CRC) error", true},
		{"FIFO overflow", false},
		{"Reserved short packet data type detected", true},
		{"Reserved long packet data type detected", true},
		{"Incomplete long packet detected", false},
		{"Frame sync error", false},
		{"Line sync error", false},
		{"DPHY recoverable synchronization error", true},
		{"DPHY non-recoverable synchronization error", false},
		{"Escape mode error", true},
		{"Escape mode trigger event", true},
		{"Escape mode ultra-low power state for data lane(s)", true},
		{"Escape mode ultra-low power state exit for clock lane", true},
		{"Inter-frame short packet discarded", true},
		{"Inter-frame long packet discarded", true},
	};
	u32 status;
	unsigned int i;

	/* Register errors once more in case of error interrupts are disabled */
	ipu4_isys_register_errors(csi2);
	status = csi2->receiver_errors;
	csi2->receiver_errors = 0;

	for (i = 0; i < ARRAY_SIZE(errors); i++) {
		if (!(status & BIT(i)))
			continue;

		if (errors[i].is_info_only)
			dev_dbg(dev,
				"csi2-%i info: %s\n",
				csi2->port, errors[i].error_string);
		else
			dev_err_ratelimited(dev,
					    "csi2-%i error: %s\n",
					    csi2->port,
					    errors[i].error_string);
	}
}

void ipu4_isys_csi2_stream_disable(struct ipu6_isys_csi2 *csi2)
{
	u32 val;

	ipu4_isys_csi2_error(csi2);

	val = readl(csi2->base + CSI2_REG_CSI_RX_CONFIG);
	val &= ~(CSI2_CSI_RX_CONFIG_DISABLE_BYTE_CLK_GATING |
		 CSI2_CSI_RX_CONFIG_RELEASE_LP11);
	writel(val, csi2->base + CSI2_REG_CSI_RX_CONFIG);

	writel(0, csi2->base + CSI2_REG_CSI_RX_ENABLE);

	/* Disable interrupts */
	writel(0, csi2->base + CSI2_REG_CSI2S2M_IRQ_MASK);
	writel(0, csi2->base + CSI2_REG_CSI2S2M_IRQ_ENABLE);
	writel(0, csi2->base + CSI2_REG_CSI2PART_IRQ_MASK);
	writel(0, csi2->base + CSI2_REG_CSI2PART_IRQ_ENABLE);
}

int ipu4_isys_csi2_stream_enable(struct ipu6_isys_csi2 *csi2)
{
	struct ipu6_isys_csi2_timing timing = { };
	unsigned int nlanes = csi2->nlanes;
	u32 csi2part = 0;
	u32 csi2csirx;
	unsigned int i;
	u32 val;
	int ret;

	ret = ipu4_isys_csi2_calc_timing(csi2, &timing);
	if (ret)
		return ret;

	writel(timing.ctermen,
	       csi2->base + CSI2_REG_CSI_RX_DLY_CNT_TERMEN_CLANE);
	writel(timing.csettle,
	       csi2->base + CSI2_REG_CSI_RX_DLY_CNT_SETTLE_CLANE);

	for (i = 0; i < nlanes; i++) {
		writel(timing.dtermen,
		       csi2->base +
		       CSI2_REG_CSI_RX_DLY_CNT_TERMEN_DLANE(i));
		writel(timing.dsettle,
		       csi2->base +
		       CSI2_REG_CSI_RX_DLY_CNT_SETTLE_DLANE(i));
	}

	val = readl(csi2->base + CSI2_REG_CSI_RX_CONFIG);
	val |= CSI2_CSI_RX_CONFIG_DISABLE_BYTE_CLK_GATING |
	    CSI2_CSI_RX_CONFIG_RELEASE_LP11;
	writel(val, csi2->base + CSI2_REG_CSI_RX_CONFIG);

	writel(nlanes, csi2->base + CSI2_REG_CSI_RX_NOF_ENABLED_LANES);
	writel(CSI2_CSI_RX_ENABLE_ENABLE,
	       csi2->base + CSI2_REG_CSI_RX_ENABLE);

	/* SOF/EOF of VC0-VC3 enabled from CSI2PART register in B0 */
	for (i = 0; i < IPU4_CSI2_NR_OF_VC; i++)
		csi2part |= IPU4_CSI_RX_IRQ_FS_VC(i) | IPU4_CSI_RX_IRQ_FE_VC(i);

	/* Enable csi2 receiver error interrupts */
	csi2csirx = BIT(CSI2_CSIRX_NUM_ERRORS) - 1;
	writel(csi2csirx, csi2->base + CSI2_REG_CSIRX_IRQ_EDGE);
	writel(0, csi2->base + CSI2_REG_CSIRX_IRQ_LEVEL_NOT_PULSE);
	writel(csi2csirx, csi2->base + CSI2_REG_CSIRX_IRQ_CLEAR);
	writel(csi2csirx, csi2->base + CSI2_REG_CSIRX_IRQ_MASK);
	writel(csi2csirx, csi2->base + CSI2_REG_CSIRX_IRQ_ENABLE);

	/* Enable csi2 error and SOF-related irqs */
	writel(csi2part, csi2->base + CSI2_REG_CSI2PART_IRQ_EDGE);
	writel(0, csi2->base + CSI2_REG_CSI2PART_IRQ_LEVEL_NOT_PULSE);
	writel(csi2part, csi2->base + CSI2_REG_CSI2PART_IRQ_CLEAR);
	writel(csi2part, csi2->base + CSI2_REG_CSI2PART_IRQ_MASK);
	writel(csi2part, csi2->base + CSI2_REG_CSI2PART_IRQ_ENABLE);

	return 0;
}

void ipu4_isys_setup_hw(struct ipu6_isys *isys)
{
	void __iomem *base = isys->pdata->base;
	const u8 *thd = isys->pdata->ipdata->hw_variant.cdc_fifo_threshold;
	unsigned int i;
	u32 irqs;

	/* Enable irqs for all MIPI ports */
	irqs = IPU4_ISYS_UNISPART_IRQ_CSI2(0) |
	    IPU4_ISYS_UNISPART_IRQ_CSI2(1) |
	    IPU4_ISYS_UNISPART_IRQ_CSI2(2) |
	    IPU4_ISYS_UNISPART_IRQ_CSI2(3) |
	    IPU4_ISYS_UNISPART_IRQ_CSI2(4) | IPU4_ISYS_UNISPART_IRQ_CSI2(5);

	irqs |= IPU4_ISYS_UNISPART_IRQ_SW;

	writel(irqs, base + IPU4_REG_ISYS_UNISPART_IRQ_EDGE);
	writel(irqs, base + IPU4_REG_ISYS_UNISPART_IRQ_LEVEL_NOT_PULSE);
	writel(irqs, base + IPU4_REG_ISYS_UNISPART_IRQ_CLEAR);
	writel(irqs, base + IPU4_REG_ISYS_UNISPART_IRQ_MASK);
	writel(irqs, base + IPU4_REG_ISYS_UNISPART_IRQ_ENABLE);

	writel(0, base + IPU4_REG_ISYS_UNISPART_SW_IRQ_REG);
	writel(0, base + IPU4_REG_ISYS_UNISPART_SW_IRQ_MUX_REG);

	/* Write CDC FIFO threshold values for isys */
	for (i = 0; i < isys->pdata->ipdata->hw_variant.cdc_fifos; i++)
		writel(thd[i], base + IPU4_REG_ISYS_CDC_THRESHOLD(i));
}
