# SPDX-License-Identifier: GPL-2.0-only
#
# In-tree Makefile for drivers/media/pci/intel/ipu4/.
# Built via `make M=drivers/media/pci/intel/ipu4`; does not depend on the
# parent Makefile descending into this directory.
#
# GCOV is enabled unconditionally: the coverage reports from full-VM
# STREAMON runs drive the register-fuzzing loop for hw/misc/ipu4.c.

# -Werror is supplied globally by CONFIG_WERROR in the in-tree build.
GCOV_PROFILE := y

intel-ipu4-objs				+= ipu6.o \
					ipu6-bus.o \
					ipu6-dma.o \
					ipu6-mmu.o \
					ipu6-buttress.o \
					ipu6-cpd.o \
					ipu6-fw-com.o

obj-m		+= intel-ipu4.o

intel-ipu4-isys-objs			+= ipu6-isys.o \
					ipu6-isys-csi2.o \
					ipu6-fw-isys.o \
					ipu6-isys-video.o \
					ipu6-isys-queue.o \
					ipu6-isys-subdev.o

obj-m		+= intel-ipu4-isys.o

obj-m		+= ambu-ipu-bridge.o
