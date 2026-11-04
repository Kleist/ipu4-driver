/* SPDX-License-Identifier: GPL-2.0-only */
/* IPU4 device data for ipu6.c: buttress control, registers and pdata. */

#ifndef IPU4_PCI_H
#define IPU4_PCI_H

struct ipu6_buttress_ctrl;
struct ipu6_buttress_registers;
struct ipu6_isys_internal_pdata;
struct ipu6_psys_internal_pdata;

/* Not static, so the buttress KUnit suite can pin their fields. */
extern const struct ipu6_buttress_ctrl ipu4_isys_buttress_ctrl;
extern const struct ipu6_buttress_ctrl ipu4_psys_buttress_ctrl;
extern const struct ipu6_buttress_registers ipu4_buttress_regs;

void ipu4_internal_pdata_init(struct ipu6_isys_internal_pdata *isys,
			      struct ipu6_psys_internal_pdata *psys);

#endif /* IPU4_PCI_H */
