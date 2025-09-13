# SPDX-License-Identifier: GPL-2.0-only
ccflags-y = -Werror

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

HDR := ${intel-ipu4-objs:.o=.h} ${intel-ipu4-isys-objs:.o=.h} ambu-ipu-bridge.h
SRC := ${intel-ipu4-objs:.o=.c} ${intel-ipu4-isys-objs:.o=.c} ambu-ipu-bridge.c

all:
	$(MAKE) -j $(nproc) -C ${KERNEL_SRC} M=$$PWD

modules_install:
	$(MAKE) -C ${KERNEL_SRC} M=$$PWD INSTALL_MOD_PATH=${INSTALL_MOD_PATH} modules_install

checkpatch:
	${KERNEL_SRC}/scripts/checkpatch.pl \
		--no-tree \
		--quiet \
		--file \
		--ignore EMBEDDED_FILENAME,LINUX_VERSION_CODE,NEW_TYPEDEFS,BRACES,MSLEEP,POINTER_LOCATION \
		--max-line-length=100 \
		--color=always \
		--showfile \
		${SRC} \
		${HDR}

clean:
	rm -rf *.o *.ko *.mod.c *.symvers *.order .*.cmd .tmp_versions *.mod

