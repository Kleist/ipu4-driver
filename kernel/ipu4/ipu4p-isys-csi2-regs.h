/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef IPU4P_ISYS_CSI2_REGS_H
#define IPU4P_ISYS_CSI2_REGS_H

#include <linux/bits.h>

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

/* CSI receiver general-purpose registers */
#define IPU4P_CSI2_GPREG_CR_PORT_CONFIG			0x14

#endif /* IPU4P_ISYS_CSI2_REGS_H */
