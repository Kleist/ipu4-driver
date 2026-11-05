// SPDX-License-Identifier: GPL-2.0-only
/* IPU4 device data for ipu6.c: buttress control, registers and pdata. */

#include <linux/kernel.h>

#include "ipu4-pci.h"
#include "ipu4-platform-buttress-regs.h"
#include "ipu4-platform-isys-csi2-reg.h"
#include "ipu4-platform-regs.h"
#include "ipu6.h"
#include "ipu6-buttress.h"
#include "ipu4-fw-isys.h"
#include "ipu6-fw-isys.h"
#include "ipu6-isys.h"
#include "ipu6-platform-buttress-regs.h"

const struct ipu6_buttress_ctrl ipu4_isys_buttress_ctrl = {
	.subsys_id = IPU_ISYS,
	.ratio = IPU4_IS_FREQ_CTL_DIVISOR,
	.qos_floor = 0,
	.freq_ctl = IPU6_BUTTRESS_REG_IS_FREQ_CTL,
	.pwr_sts_shift = IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_SHIFT,
	.pwr_sts_mask = IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_MASK,
	.pwr_sts_on = IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_IS_RDY,
	.pwr_sts_off = IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_IDLE,
};

const struct ipu6_buttress_ctrl ipu4_psys_buttress_ctrl = {
	.subsys_id = IPU_PSYS,
	.ratio = IPU4_PS_FREQ_CTL_DEFAULT_RATIO,
	.qos_floor = IPU4_PS_FREQ_CTL_DEFAULT_RATIO,
	.freq_ctl = IPU6_BUTTRESS_REG_PS_FREQ_CTL,
	.pwr_sts_shift = IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_SHIFT,
	.pwr_sts_mask = IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_MASK,
	.pwr_sts_on = IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_PS_PWR_UP,
	.pwr_sts_off = IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_IDLE,
};

const struct ipu6_buttress_registers ipu4_buttress_regs = {
	/* Registers */
	.irq_status	= BUTTRESS_REG_ISR_ENABLED_STATUS,
	.irq_clear	= BUTTRESS_REG_ISR_CLEAR,
	.irq_enable	= BUTTRESS_REG_ISR_ENABLE,
	.pwr_status	= BUTTRESS_REG_PWR_STATE,
	.security_ctl	= BUTTRESS_REG_SECURITY_CTL,
	.fw_reset_ctl	= BUTTRESS_REG_FW_RESET_CTL,
	.fabric_cmd	= BUTTRESS_REG_FABRIC_CMD,
	.tsw_ctl	= BUTTRESS_REG_TSW_CTL,
	.tsc_lo		= BUTTRESS_REG_TSC_LO,
	.wdt		= BUTTRESS_REG_WDT,
	.btrs_ctrl	= BUTTRESS_REG_BTRS_CTRL,
	.csr_in		= BUTTRESS_REG_CSE2IUCSR,
	.csr_out	= BUTTRESS_REG_IU2CSECSR,
	.db0_in		= BUTTRESS_REG_CSE2IUDB0,
	.db0_out	= BUTTRESS_REG_IU2CSEDB0,
	.data0_in	= BUTTRESS_REG_CSE2IUDATA0,
	.data0_out	= BUTTRESS_REG_IU2CSEDATA0,
	.sku_id		= BUTTRESS_REG_SKU,

	/* Bitmasks */
	.irq_is		= BUTTRESS_ISR_IS_IRQ,
	.irq_ps		= BUTTRESS_ISR_PS_IRQ,
	.irq_all	= BUTTRESS_IRQS,
	.irq_events	= BUTTRESS_EVENT,
	.irq_cse_ipc	= BUTTRESS_ISR_IPC_FROM_CSE_IS_WAITING,
	.irq_exec_done	= BUTTRESS_ISR_IPC_EXEC_DONE_BY_CSE,
	.irq_sai	= BUTTRESS_ISR_SAI_VIOLATION,
};

/* IPU4 sets every field: ipu6.c's IPU6 defaults do not apply to it. */
void ipu4_internal_pdata_init(struct ipu6_isys_internal_pdata *isys,
			      struct ipu6_psys_internal_pdata *psys)
{
	*isys = (struct ipu6_isys_internal_pdata) {
		.hw_variant = {
			.offset = IPU4_ISYS_OFFSET,
			.dmem_offset = IPU4_ISYS_DMEM_OFFSET,
			.spc_offset = IPU4_ISYS_SPC_OFFSET,
		},
		.isys_dma_overshoot = IPU6_ISYS_OVERALLOC_MIN,
		.num_parallel_streams = IPU4_STREAM_ID_MAX,
		.csi2.nports = ARRAY_SIZE(ipu4_csi_offsets),
		.max_streams = IPU4_ISYS_MAX_STREAMS,
		.max_sram_blocks = IPU4_ISYS_MAX_STREAMS,
		.max_send_queues = IPU4_N_MAX_SEND_QUEUES,
		/* One slot per stream: device commands are not pipelined */
		.max_devq_size = IPU4_ISYS_MAX_STREAMS,
	};
	*psys = (struct ipu6_psys_internal_pdata) {
		.hw_variant = {
			.offset = IPU4_PSYS_OFFSET,
			.dmem_offset = IPU4_PSYS_DMEM_OFFSET,
			.spc_offset = IPU4_PSYS_SPC_OFFSET,
		},
	};
}
