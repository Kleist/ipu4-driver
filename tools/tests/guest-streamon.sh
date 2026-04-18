#!/bin/sh
# Runs inside the guest VM as /init (or chained from init).
# Loads the driver, streams frames, prints result on the serial console.
set -e

mount -t proc none /proc
mount -t sysfs none /sys
mount -t debugfs none /sys/kernel/debug 2>/dev/null || true
mount -t 9p -o trans=virtio modules /mnt/modules
mount -t 9p -o trans=virtio tests   /mnt/tests

insmod /mnt/modules/intel-ipu4.ko
insmod /mnt/modules/intel-ipu4-isys.ko 2>/dev/null || true

# Give the driver a moment to enumerate.
sleep 1

if ! ls /dev/video* >/dev/null 2>&1; then
	echo "E2E: FAIL (no /dev/video* nodes)"
	echo "--- dmesg tail ---"
	dmesg | tail -40
	poweroff -f
fi

# Capture 5 frames of 800x800 RGB888 into /tmp/frame-%d.raw.
yavta -c5 -n4 -f RGB24 -s 800x800 -I -F/tmp/frame-#.raw /dev/video0

for i in 0 1 2 3 4; do
	sha256sum "/tmp/frame-$i.raw" || true
done

# Archive gcov for the e2e harvester.
if [ -d /sys/kernel/debug/gcov ]; then
	tar -cf /mnt/tests/out/gcov.tar -C /sys/kernel/debug/gcov . 2>/dev/null || true
fi

echo "E2E: PASS"
poweroff -f
