/* SPDX-License-Identifier: GPL-2.0-only */
/* Author: Claude for the IPU4 QEMU dev harness. */

#ifndef __IPU4_VIRT_SENSOR_H
#define __IPU4_VIRT_SENSOR_H

#include <linux/pci.h>

int ipu4_virt_sensor_install(struct pci_dev *pdev);
void ipu4_virt_sensor_remove(struct pci_dev *pdev);

#endif
