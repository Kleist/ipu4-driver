# SPDX-License-Identifier: GPL-2.0-only
#
# In-tree Makefile for drivers/media/pci/intel/ipu4/.
# GCOV is enabled unconditionally: the coverage reports from full-VM
# STREAMON runs drive the register-fuzzing loop for hw/misc/ipu4.c.

GCOV_PROFILE := y

intel-ipu4-y := \
	ipu6.o \
	ipu6-bus.o \
	ipu6-dma.o \
	ipu6-mmu.o \
	ipu6-buttress.o \
	ipu6-cpd.o \
	ipu6-fw-com.o

intel-ipu4-$(CONFIG_VIDEO_IPU4_VIRT_SENSOR) += virt-sensor.o

obj-$(CONFIG_VIDEO_INTEL_IPU4) += intel-ipu4.o

# Phy sources (ipu6-isys-{mcd,jsl,dwc}-phy.c) reference IPU6-only register
# names that don't exist in the IPU4 register headers; intel-ipu4-isys
# leaves their function-pointer slots unresolved at modpost time and the
# real-hardware path will need IPU4-specific phy code wired in later.
intel-ipu4-isys-y := \
	ipu6-isys.o \
	ipu6-isys-csi2.o \
	ipu6-fw-isys.o \
	ipu6-isys-video.o \
	ipu6-isys-queue.o \
	ipu6-isys-subdev.o

obj-$(CONFIG_VIDEO_INTEL_IPU4) += intel-ipu4-isys.o

obj-$(CONFIG_VIDEO_INTEL_IPU4) += ambu-ipu-bridge.o

obj-$(CONFIG_VIDEO_INTEL_IPU4) += tests/
