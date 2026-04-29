#!/usr/bin/env bash
# Build the IPU4 driver in-tree in tools/linux/ and produce intel-ipu4.ko.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
LINUX_DIR="$ROOT/tools/linux"

if [[ ! -d "$LINUX_DIR" ]]; then
	echo "tools/linux/ missing; run tools/bootstrap.sh first" >&2
	exit 1
fi

cd "$LINUX_DIR"

CONFIG="${IPU4_KCONFIG:-$ROOT/tools/rootfs/kernel.config}"
if [[ -f "$CONFIG" ]]; then
	cp "$CONFIG" .config
else
	# Only synthesise a fresh .config from defconfig when none exists.
	# On a cache-restored tree we already have one — running defconfig
	# would reset symbols the scripts/config block below depends on
	# being already-set, and would invalidate ~every cached .o. The
	# scripts/config + olddefconfig pass that follows is idempotent
	# and reapplies the canonical settings on top of either path.
	if [[ ! -f .config ]]; then
		make defconfig
	fi
	# MEDIA_SUPPORT must be =y (not =m) so MEDIA_CONTROLLER — a bool that
	# depends on it — can itself be =y. Without MEDIA_CONTROLLER the
	# driver cannot see struct v4l2_subdev::entity (ipu6-isys.c:114) or
	# struct video_device::entity, and the build dies with type errors.
	#
	# MEDIA_SUPPORT_FILTER must be =n. When it is =y (the defconfig
	# default on x86_64), the "Media core support" menu is `visible if
	# !MEDIA_SUPPORT_FILTER`, which strips the prompts from VIDEO_DEV
	# and MEDIA_CONTROLLER. olddefconfig then re-evaluates those symbols
	# via their default expressions (MEDIA_CAMERA_SUPPORT || ...), which
	# are themselves gated by the filter, so --enable MEDIA_CONTROLLER
	# silently reverts to =n and the driver no longer sees
	# struct v4l2_subdev::entity.
	# VIDEO_DEV / VIDEOBUF2_* are built =y (not =m) so the IPU4 module
	# resolves its kernel-side v4l2 symbols without a separate insmod
	# dance in the guest initramfs. MEDIA_SUPPORT_FILTER=n keeps
	# MEDIA_CAMERA_SUPPORT auto-enabled (see the MEDIA_CONTROLLER note).
	./scripts/config \
		--enable MEDIA_SUPPORT \
		--disable MEDIA_SUPPORT_FILTER \
		--enable MEDIA_PCI_SUPPORT \
		--enable MEDIA_CONTROLLER \
		--enable VIDEO_V4L2_SUBDEV_API \
		--enable VIDEO_DEV \
		--enable VIDEOBUF2_V4L2 \
		--enable VIDEOBUF2_DMA_SG \
		--enable VIDEOBUF2_DMA_CONTIG \
		--enable FTRACE \
		--enable MMIOTRACE \
		--enable DYNAMIC_DEBUG \
		--enable GCOV_KERNEL \
		--enable KUNIT \
		--disable VIDEO_INTEL_IPU6 \
		--module VIDEO_INTEL_IPU4 \
		--enable  VIDEO_IPU4_VIRT_SENSOR \
		--disable VIDEO_INTEL_IPU4_KUNIT_TESTS
	# KUnit tests are built only in the kunit kernel built by kunit.py
	# (.kunitconfig has CONFIG_VIDEO_INTEL_IPU4_KUNIT_TESTS=y, with the
	# driver also =y, so internal symbols resolve at link time). In the
	# streamon-smoke kernel built here the driver is =m, and the new
	# suites (ipu4_format_helpers_kunit, ipu4_video_accessors_kunit)
	# reference non-exported helpers — building them as separate
	# modules would fail modpost.

	# Trim the defconfig to what the QEMU test VM actually exercises.
	# tools/run-vm.sh boots with -nographic -no-reboot, no -drive, no
	# -netdev, and no -usb; the only devices in the guest are the
	# emulated IPU4 PCI device, the 8250 serial console, and a 9p-
	# over-virtio share. Every subsystem disabled below is dead code at
	# both compile and initcall time. Debug surfaces (FTRACE/MMIOTRACE/
	# DYNAMIC_DEBUG/KUNIT/GCOV_KERNEL/KALLSYMS/PRINTK/DEBUG_FS) stay
	# enabled by the block above and the defconfig defaults.
	#
	# Keep these on the path explicitly so the disables below — which
	# can deselect intermediate symbols — don't drop them via
	# olddefconfig: NET (9p transport), 9P_FS / NET_9P / NET_9P_VIRTIO
	# (the share itself), VIRTIO_PCI (9p transport), SERIAL_8250
	# (console), DEVTMPFS (init mounts /dev), DEBUG_FS (dyndbg control
	# file), BLK_DEV_INITRD (initramfs).
	./scripts/config \
		--enable NET \
		--enable NET_9P \
		--enable NET_9P_VIRTIO \
		--enable 9P_FS \
		--enable VIRTIO \
		--enable VIRTIO_PCI \
		--enable VIRTIO_MENU \
		--enable PCI \
		--enable PCI_MSI \
		--enable ACPI \
		--enable SERIAL_8250 \
		--enable SERIAL_8250_CONSOLE \
		--enable DEVTMPFS \
		--enable DEVTMPFS_MOUNT \
		--enable TMPFS \
		--enable PROC_FS \
		--enable SYSFS \
		--enable DEBUG_FS \
		--enable BLK_DEV_INITRD \
		--enable RD_GZIP \
		--enable PRINTK \
		--enable KALLSYMS

	# No sound / USB / HID / input / graphics: -nographic guest, no
	# keyboard or mouse passed through, no USB controller wired up.
	./scripts/config \
		--disable SOUND \
		--disable USB_SUPPORT \
		--disable HID_SUPPORT \
		--disable INPUT_KEYBOARD \
		--disable INPUT_MOUSE \
		--disable INPUT_JOYSTICK \
		--disable INPUT_TABLET \
		--disable INPUT_TOUCHSCREEN \
		--disable INPUT_MISC \
		--disable AGP \
		--disable DRM \
		--disable FB \
		--disable BACKLIGHT_CLASS_DEVICE \
		--disable VGA_CONSOLE \
		--disable FRAMEBUFFER_CONSOLE \
		--disable LOGO

	# No NIC, wireless, or NFC: 9p-virtio is the only network-facing
	# transport and it doesn't use the IP stack. INET / IPV6 /
	# NETFILTER / bridging are all dead weight here.
	./scripts/config \
		--disable BT \
		--disable RFKILL \
		--disable WIRELESS \
		--disable WLAN \
		--disable WAN \
		--disable NFC \
		--disable ETHERNET \
		--disable PHYLIB \
		--disable MDIO_DEVICE \
		--disable PPP \
		--disable SLIP \
		--disable USB_NET_DRIVERS \
		--disable INET \
		--disable IPV6 \
		--disable NETFILTER \
		--disable BRIDGE \
		--disable VLAN_8021Q \
		--disable NET_SCHED \
		--disable XFRM_USER \
		--disable PACKET_DIAG \
		--disable UNIX_DIAG

	# No disk: rootfs is an in-memory cpio, /mnt/tests is 9p. No SCSI,
	# ATA, MD/DM, NVMe, MMC, MTD, or PCMCIA stack needed.
	./scripts/config \
		--disable ATA \
		--disable SCSI \
		--disable MD \
		--disable BCACHE \
		--disable BLK_DEV_DM \
		--disable NVME_CORE \
		--disable MMC \
		--disable MTD \
		--disable PCMCIA \
		--disable RAPIDIO

	# Only initramfs + 9p are mounted by the guest inits. Drop every
	# on-disk filesystem driver — none are reachable.
	./scripts/config \
		--disable EXT2_FS \
		--disable EXT3_FS \
		--disable EXT4_FS \
		--disable BTRFS_FS \
		--disable XFS_FS \
		--disable F2FS_FS \
		--disable REISERFS_FS \
		--disable JFS_FS \
		--disable NTFS3_FS \
		--disable OVERLAY_FS \
		--disable FUSE_FS \
		--disable NFS_FS \
		--disable NFSD \
		--disable CIFS \
		--disable AFS_FS \
		--disable CEPH_FS

	# Misc subsystems with no presence in our QEMU topology. The VM
	# never sleeps (panic=-1 oops=panic, -no-reboot), doesn't host
	# nested guests (VIRTUALIZATION), and has no real-hardware
	# monitoring, EDAC, watchdog, IPMI, or thermal sensors to talk to.
	./scripts/config \
		--disable SUSPEND \
		--disable HIBERNATION \
		--disable VIRTUALIZATION \
		--disable HWMON \
		--disable IIO \
		--disable INFINIBAND \
		--disable EDAC \
		--disable WATCHDOG \
		--disable IPMI_HANDLER \
		--disable POWER_SUPPLY \
		--disable THERMAL \
		--disable RAS \
		--disable SAMPLES \
		--disable PARPORT \
		--disable AUXDISPLAY \
		--disable SECURITY_SELINUX \
		--disable SECURITY_SMACK \
		--disable SECURITY_TOMOYO \
		--disable SECURITY_APPARMOR \
		--disable IMA \
		--disable EVM \
		--disable AUDIT
fi
make olddefconfig

# Allow callers (e.g. tools/build-kernel.sh) to use this script purely
# for its canonical config block, skipping the driver M= build below.
# Without this, build-kernel.sh would skip build.sh whenever a cached
# .config existed and a config-affecting change to this script wouldn't
# take effect on cache-hit CI runs.
if [[ "${IPU4_BUILD_CONFIG_ONLY:-0}" == "1" ]]; then
	exit 0
fi

JOBS="$(nproc)"
make -j"$JOBS" modules_prepare

# Build the driver as an external module. KBUILD_MODPOST_WARN downgrades
# "undefined symbol" from an error to a warning: vmlinux is not built
# here, so Module.symvers is empty and every kernel-exported symbol the
# driver uses is technically unresolved. Those symbols are resolved at
# insmod time inside the test VM once M2's full kernel build lands.
make -j"$JOBS" KBUILD_MODPOST_WARN=1 M=drivers/media/pci/intel/ipu4

KO="$LINUX_DIR/drivers/media/pci/intel/ipu4/intel-ipu4.ko"
if [[ ! -f "$KO" ]]; then
	echo "build produced no $KO" >&2
	exit 2
fi
echo ">>> built $KO"
modinfo "$KO" | sed -n '1,10p'
