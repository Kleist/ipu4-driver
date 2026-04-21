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

# Guest /init to embed. Defaults to the M2 vm-smoke init; callers pass a
# name ("probe-ok", "streamon", ...) to pick a different one from
# tools/rootfs/init.<name>.
INIT_NAME="${IPU4_INIT:-vm-smoke}"
INIT_SRC="$HERE/init.$INIT_NAME"
if [[ ! -f "$INIT_SRC" ]]; then
	echo "no guest init at $INIT_SRC" >&2
	exit 3
fi

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

# Compile the streamon-attempt helper statically against the host's
# libc/headers. Used only by tools/tests/streamon-smoke.sh.
STREAMON_SRC="$ROOT/tools/tests/streamon.c"
STREAMON_BIN="$OUT/streamon"
if [[ -f "$STREAMON_SRC" ]] && \
   { [[ ! -x "$STREAMON_BIN" ]] || [[ "$STREAMON_SRC" -nt "$STREAMON_BIN" ]]; }; then
	gcc -static -O2 -Wall -o "$STREAMON_BIN" "$STREAMON_SRC"
fi

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
file /init               $INIT_SRC 0755 0 0
nod  /dev/console        0622 0 0 c 5 1
nod  /dev/null           0666 0 0 c 1 3
nod  /dev/zero           0666 0 0 c 1 5
EOF

# Busybox applet symlinks.
for applet in sh mount umount insmod rmmod modprobe lsmod dmesg echo cat ls \
	ln sleep poweroff reboot mkdir grep sed awk head tail find tar; do
	echo "slink /bin/$applet      busybox 0777 0 0" >> "$LIST"
done

# Videobuf2 dependencies of intel-ipu4-isys.ko. videodev is built-in
# (CONFIG_VIDEO_DEV=y), but vb2 helpers land as modules.
VB2_DIR="$LINUX_DIR/drivers/media/common/videobuf2"
for vb2 in videobuf2-common videobuf2-memops videobuf2-v4l2 videobuf2-dma-sg; do
	ko="$VB2_DIR/$vb2.ko"
	if [[ -f "$ko" ]]; then
		echo "file /lib/modules/$vb2.ko  $ko 0644 0 0" >> "$LIST"
	fi
done

# Driver modules (if build.sh has run).
if compgen -G "$DRV_DIR/*.ko" > /dev/null; then
	for ko in "$DRV_DIR"/*.ko; do
		[[ "$(basename "$ko")" == *_kunit.ko ]] && continue
		echo "file /lib/modules/$(basename "$ko")  $ko 0644 0 0" >> "$LIST"
	done
fi

# CPD firmware blob. Generated on-demand by tools/firmware/gen-cpd.py;
# minimal content that passes ipu6_cpd_validate_cpd_file() but stops
# probe later at pkg_dir creation (see STATUS.md for the M4 roadmap).
FW_DIR="$OUT/firmware"
mkdir -p "$FW_DIR"
FW_BLOB="$FW_DIR/ipu4_cpd_b0.bin"
if [[ ! -f "$FW_BLOB" || "$ROOT/tools/firmware/gen-cpd.py" -nt "$FW_BLOB" ]]; then
	python3 "$ROOT/tools/firmware/gen-cpd.py" "$FW_BLOB"
fi
echo "dir  /lib/firmware           0755 0 0" >> "$LIST"
echo "file /lib/firmware/ipu4_cpd_b0.bin  $FW_BLOB 0644 0 0" >> "$LIST"

if [[ -x "$STREAMON_BIN" ]]; then
	echo "file /bin/streamon  $STREAMON_BIN 0755 0 0" >> "$LIST"
fi

"$GEN" "$LIST" | gzip -9 > "$OUT/rootfs.cpio.gz"
echo ">>> built $OUT/rootfs.cpio.gz ($(stat -c%s "$OUT/rootfs.cpio.gz") bytes)"
