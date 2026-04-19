#!/usr/bin/env bash
# Build a minimal initramfs that can load the IPU4 modules and probe the
# emulated PCI device. Output: tools/rootfs/out/rootfs.cpio.gz.
#
# Uses busybox-static from the host (apt: busybox-static) + the kernel
# tree's gen_init_cpio to produce a cpio archive that includes /dev
# device nodes without needing root on the host.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
LINUX_DIR="$ROOT/tools/linux"
OUT="$HERE/out"
mkdir -p "$OUT"

BUSYBOX="$(command -v busybox)"
if [[ -z "$BUSYBOX" ]]; then
	echo "busybox not installed (apt install busybox-static)" >&2
	exit 1
fi

GEN="$LINUX_DIR/usr/gen_init_cpio"
if [[ ! -x "$GEN" ]]; then
	echo "missing $GEN; run tools/build-kernel.sh first" >&2
	exit 2
fi

DRV_DIR="$LINUX_DIR/drivers/media/pci/intel/ipu4"

LIST="$(mktemp)"
trap 'rm -f "$LIST"' EXIT

# Directory skeleton + busybox + /init.
cat > "$LIST" <<EOF
dir  /bin                0755 0 0
dir  /sbin               0755 0 0
dir  /proc               0755 0 0
dir  /sys                0755 0 0
dir  /dev                0755 0 0
dir  /lib                0755 0 0
dir  /lib/modules        0755 0 0
dir  /mnt                0755 0 0
dir  /mnt/tests          0755 0 0
file /bin/busybox        $BUSYBOX 0755 0 0
file /init               $HERE/init 0755 0 0
nod  /dev/console        0622 0 0 c 5 1
nod  /dev/null           0666 0 0 c 1 3
nod  /dev/zero           0666 0 0 c 1 5
EOF

# Busybox applet symlinks.
for applet in sh mount umount insmod rmmod modprobe lsmod dmesg echo cat ls \
	ln sleep poweroff reboot mkdir grep sed awk head tail find; do
	echo "slink /bin/$applet      busybox 0777 0 0" >> "$LIST"
done

# Driver modules (if build.sh has run).
if compgen -G "$DRV_DIR/*.ko" > /dev/null; then
	for ko in "$DRV_DIR"/*.ko; do
		echo "file /lib/modules/$(basename "$ko")  $ko 0644 0 0" >> "$LIST"
	done
fi

"$GEN" "$LIST" | gzip -9 > "$OUT/rootfs.cpio.gz"
echo ">>> built $OUT/rootfs.cpio.gz ($(stat -c%s "$OUT/rootfs.cpio.gz") bytes)"
