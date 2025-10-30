/* SPDX-License-Identifier: GPL-2.0-only */
/*
   IPU4 values coming from
   drivers/media/pci/intel/ipu4/ipu-platform-isys-csi2-reg.h
   in linux 4.x driver

   Copyright (C) 2023 Intel Corporation
*/

#ifndef IPU4_PLATFORM_ISYS_CSI2_REG_H
#define IPU4_PLATFORM_ISYS_CSI2_REG_H

/* IRQ-related registers specific to each of the four CSI receivers */
#define CSI2_REG_CSI2PART_IRQ_EDGE			0x400
#define CSI2_REG_CSI2PART_IRQ_MASK			0x404
#define CSI2_REG_CSI2PART_IRQ_STATUS			0x408
#define CSI2_REG_CSI2PART_IRQ_CLEAR			0x40c
#define CSI2_REG_CSI2PART_IRQ_ENABLE			0x410
#define CSI2_REG_CSI2PART_IRQ_LEVEL_NOT_PULSE		0x414
#define CSI2_CSI2PART_IRQ_CSIRX				0x10000

#define CSI2_REG_CSIRX_IRQ_EDGE				0x500
#define CSI2_REG_CSIRX_IRQ_MASK				0x504
#define CSI2_REG_CSIRX_IRQ_STATUS			0x508
#define CSI2_REG_CSIRX_IRQ_CLEAR			0x50c
#define CSI2_REG_CSIRX_IRQ_ENABLE			0x510
#define CSI2_REG_CSIRX_IRQ_LEVEL_NOT_PULSE		0x514
#define CSI2_CSIRX_NUM_ERRORS				17

#define CSI2_REG_CSI2S2M_IRQ_EDGE			0x600
#define CSI2_REG_CSI2S2M_IRQ_MASK			0x604
#define CSI2_REG_CSI2S2M_IRQ_STATUS			0x608
#define CSI2_REG_CSI2S2M_IRQ_CLEAR			0x60c
#define CSI2_REG_CSI2S2M_IRQ_ENABLE			0x610
#define CSI2_REG_CSI2S2M_IRQ_LEVEL_NOT_PULSE		0x614

#define CSI2_IRQ_FS_VC(chn)				(1 << ((chn) * 4))
#define CSI2_IRQ_FE_VC(chn)				(2 << ((chn) * 4))

#define CSI2_REG_CSI_RX_ENABLE				0x00
#define CSI2_CSI_RX_ENABLE_ENABLE			0x01
/* Enabled lanes - 1 */
#define CSI2_REG_CSI_RX_NOF_ENABLED_LANES		0x04
#define CSI2_REG_CSI_RX_CONFIG				0x08
#define CSI2_CSI_RX_CONFIG_RELEASE_LP11			0x1
#define CSI2_CSI_RX_CONFIG_DISABLE_BYTE_CLK_GATING	0x2
#define CSI2_REG_CSI_RX_DLY_CNT_TERMEN_CLANE		0x2c
#define CSI2_REG_CSI_RX_DLY_CNT_SETTLE_CLANE		0x30
/* 0..3 */
#define CSI2_REG_CSI_RX_DLY_CNT_TERMEN_DLANE(n)		(0x34 + (n) * 8)
#define CSI2_REG_CSI_RX_DLY_CNT_SETTLE_DLANE(n)		(0x38 + (n) * 8)

#endif /* IPU6_ISYS_CSI2_REG_H */
