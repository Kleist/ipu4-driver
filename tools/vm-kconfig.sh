#!/usr/bin/env bash
# Apply the QEMU-test-VM kconfig trim to the kernel tree in the current
# directory (via scripts/config). The caller runs `make olddefconfig`
# afterwards. Shared by tools/build.sh (fork flavor) and
# tools/ipu6/build.sh (ipu6 flavor) so both guest kernels boot under the
# same tools/run-vm.sh topology.
set -euo pipefail

if [[ ! -x scripts/config ]]; then
	echo "${0##*/}: run from the top of a kernel tree" >&2
	exit 1
fi

# Trim the defconfig to what the QEMU test VM actually exercises.
# tools/run-vm.sh boots with -nographic -no-reboot, no -drive, no
# -netdev, and no -usb; the only devices in the guest are the
# emulated IPU4 PCI device, the 8250 serial console, and a 9p-
# over-virtio share. Every subsystem disabled below is dead code at
# both compile and initcall time. Debug surfaces (FTRACE/MMIOTRACE/
# DYNAMIC_DEBUG/KUNIT/GCOV_KERNEL/KALLSYMS/PRINTK/DEBUG_FS) stay
# enabled by the caller's own config block (fork: tools/build.sh)
# and the defconfig defaults.
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
