/* SPDX-License-Identifier: GPL-2.0-only */
/* IPU4 ISYS hardware: interrupt setup and the CSI-2 receivers. */

#ifndef IPU4_ISYS_H
#define IPU4_ISYS_H

struct ipu6_isys;
struct ipu6_isys_csi2;

void ipu4_isys_setup_hw(struct ipu6_isys *isys);
void ipu4_isys_register_errors(struct ipu6_isys_csi2 *csi2);
void ipu4_isys_csi2_error(struct ipu6_isys_csi2 *csi2);
void ipu4_isys_csi2_stream_disable(struct ipu6_isys_csi2 *csi2);
int ipu4_isys_csi2_stream_enable(struct ipu6_isys_csi2 *csi2);

#endif /* IPU4_ISYS_H */
