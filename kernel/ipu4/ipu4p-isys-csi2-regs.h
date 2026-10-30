/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef IPU4P_ISYS_CSI2_REGS_H
#define IPU4P_ISYS_CSI2_REGS_H

#include <linux/bits.h>
#include <linux/types.h>

/* ISYS UNISPART and SIP interrupt registers */
#define IPU4P_ISYS_UNISPART_IRQ_EDGE			0x7c000
#define IPU4P_ISYS_UNISPART_IRQ_STATUS			0x7c008
#define IPU4P_ISYS_UNISPART_IRQ_CLEAR			0x7c00c
#define IPU4P_ISYS_UNISPART_SW_IRQ_REG			0x7c414
#define IPU4P_ISYS_UNISPART_SW_IRQ_MUX_REG		0x7c418
#define IPU4P_ISYS_UNISPART_IRQ_SW			BIT(22)
#define IPU4P_ISYS_UNISPART_IRQ_CSI2(port)		((port) ? BIT(4) : BIT(3))
#define IPU4P_ISYS_ISA_ACC_IRQ_CTRL_BASE		0xb0c00
#define IPU4P_ISYS_A_IRQ_CTRL_BASE			0xbe200
#define IPU4P_ISYS_SIP0_IRQ_CTRL_BASE			0x66d00
#define IPU4P_ISYS_SIP1_IRQ_CTRL_BASE			0x6ed00
#define IPU4P_ISYS_IRQ_MASK_OFFSET			0x04
#define IPU4P_ISYS_IRQ_CLEAR_OFFSET			0x0c
#define IPU4P_ISYS_IRQ_ENABLE_OFFSET			0x10
#define IPU4P_ISYS_IRQ_LEVEL_NOT_PULSE_OFFSET		0x14
#define IPU4P_ISYS_SIP0_IRQ_CTRL_STATUS			0x66d08
#define IPU4P_ISYS_SIP0_IRQ_CTRL_CLEAR			0x66d0c
#define IPU4P_ISYS_SIP1_IRQ_CTRL_STATUS			0x6ed08
#define IPU4P_ISYS_SIP1_IRQ_CTRL_CLEAR			0x6ed0c
#define IPU4P_ISYS_SIP0_CSI2_IRQ			BIT(3)
#define IPU4P_ISYS_SIP1_CSI2_IRQ(port)			BIT((port) - 1)
#define IPU4P_ISYS_SIP1_CSI2_IRQ_MASK			GENMASK(6, 0)

/* ISYS per-port CSI interrupt controls */
static inline u32 ipu4p_isys_csi_irq_ctrl_base(unsigned int port)
{
	return port ? (0x6cb00 + 0x800 * (port - 1)) : 0x66300;
}

static inline u32 ipu4p_isys_csi_irq_ctrl0_base(unsigned int port)
{
	return port ? (0x6cc00 + 0x800 * (port - 1)) : 0x66400;
}

#define IPU4P_ISYS_CSI_IRQ_STATUS_OFFSET		0x08
#define IPU4P_ISYS_CSI_IRQ_CLEAR_OFFSET		0x0c
#define IPU4P_ISYS_CSI_IRQ_ACTIVE			BIT(0)
/* CSI-2 frame events occupy bits 16-31 in the shared ctrl0 status. */
#define IPU4P_ISYS_CSI_ERROR_MASK			GENMASK(15, 0)
#define IPU4P_ISYS_CSI_ERROR_BITS			16U

/* IPU4P ISYS GPREG windows */
#define IPU4P_ISYS_GPOFFSET				0x66800
#define IPU4P_ISYS_COMBO_GPOFFSET			0x6e800

/* CSI receiver registers */
#define IPU4P_CSI2_RX_ENABLE				0x00
#define IPU4P_CSI2_RX_ENABLE_ENABLE			BIT(0)
#define IPU4P_CSI2_RX_NOF_ENABLED_LANES			0x04
#define IPU4P_CSI2_RX_CONFIG				0x08
#define IPU4P_CSI2_RX_CONFIG_RELEASE_LP11		BIT(0)
#define IPU4P_CSI2_RX_CONFIG_DISABLE_BYTE_CLK_GATING	BIT(1)
#define IPU4P_CSI2_RX_DLY_CNT_TERMEN_CLANE		0x2c
#define IPU4P_CSI2_RX_DLY_CNT_SETTLE_CLANE		0x30
#define IPU4P_CSI2_RX_DLY_CNT_TERMEN_DLANE(lane)	(0x34 + (lane) * 8)
#define IPU4P_CSI2_RX_DLY_CNT_SETTLE_DLANE(lane)	(0x38 + (lane) * 8)

/* Shifted CSI-2 data-lane timing register offsets. */
#define IPU4P_CSI2_RX_DLY_CNT_SHIFTED_DATA_TICKS(lane)	(0x3c + (lane) * 8)

/* CSI receiver general-purpose registers */
#define IPU4P_CSI2_GPREG_CR_PORT_CONFIG			0x14

#endif /* IPU4P_ISYS_CSI2_REGS_H */
