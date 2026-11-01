# SPDX-License-Identifier: GPL-2.0-only
#
# Virtual CSI-2 sensor for the ipu6-flavor QEMU harness. Built out of tree
# against the pinned kernel by tools/virt-sensor/build.sh:
#
#   make -C tools/linux-ipu6 M=$PWD/tools/virt-sensor MO=<out> modules

obj-m += virt-sensor.o

# Linux 7.4 passes the subdev client info to set_fmt; the version code
# still reads 7.3 in the merge window, so detect it from the header.
ifneq ($(shell grep -s 'struct v4l2_subdev_client_info {' $(srctree)/include/media/v4l2-subdev.h),)
ccflags-y += -DVIRT_SENSOR_SUBDEV_CLIENT_INFO
endif
