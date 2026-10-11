#!/usr/bin/env bash
# Build a minimal initramfs that can load the IPU4 modules and probe the
# emulated PCI device. Output: tools/rootfs/out/rootfs.cpio.gz.
#
# Uses busybox-static from the host (apt: busybox-static) + the kernel
# tree's gen_init_cpio to produce a cpio archive that includes /dev
# device nodes without needing root on the host.
#
# IPU4_FLAVOR=ipu6 takes the kernel modules and gen_init_cpio from
# tools/linux-ipu6/ (driver: drivers/media/pci/intel/ipu6), adds the
# harness's tools/virt-sensor/out/virt-sensor.ko when built, stages the
# synthesized CPD firmware as /lib/firmware/intel/ipu/ipu4_fw.bin, and writes
# tools/rootfs/out-ipu6/rootfs.cpio.gz, so the two flavors' images
# never overwrite each other. IPU4_FLAVOR=out-of-tree takes the modules
# from tools/linux-ipu4-oot/ and kernel/ipu4/ (tools/out-of-tree/build.sh)
# and writes tools/rootfs/out-out-of-tree/.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
EXTRA_KOS=()
FW_EXTRA_NAMES=()
case "${IPU4_FLAVOR:-fork}" in
fork)
	LINUX_DIR="$ROOT/tools/linux"
	DRV_DIR="$LINUX_DIR/drivers/media/pci/intel/ipu4"
	OUT="$HERE/out"
	# Firmware path the fork driver requests (relative to /lib/firmware).
	FW_NAME="ipu4_cpd_b0.bin"
	;;
ipu6)
	LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu6}"
	DRV_DIR="$LINUX_DIR/drivers/media/pci/intel/ipu6"
	OUT="$HERE/out-ipu6"
	# Harness-only virtual sensor (tools/virt-sensor/build.sh).
	EXTRA_KOS=("$ROOT/tools/virt-sensor/out/virt-sensor.ko")
	# IPU4_FIRMWARE_NAME in patches/ipu6-ipu4/.
	FW_NAME="intel/ipu/ipu4_fw.bin"
	# Same blob under the name upstream requests for the IPU6 the ipu6
	# QEMU variant presents (IPU6_FIRMWARE_NAME, 8086:9a19); it passes
	# ipu6_cpd_validate_cpd_file() there too. Same directory as FW_NAME.
	# Likewise for the ipu4p variant's IPU4P (8086:8a19).
	FW_EXTRA_NAMES=("intel/ipu/ipu6_fw.bin" "intel/ipu/ipu4p_fw.bin")
	;;
out-of-tree)
	LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu4-oot}"
	DRV_DIR="$ROOT/kernel/ipu4"
	OUT="$HERE/out-out-of-tree"
	# IPU4_FIRMWARE_NAME in kernel/ipu4/ipu6.h.
	FW_NAME="ipu4_cpd_b0.bin"
	;;
*) echo "unknown IPU4_FLAVOR '${IPU4_FLAVOR}' (expected fork, ipu6 or out-of-tree)" >&2; exit 1 ;;
esac
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

# Default to the streamon-smoke build's gen_init_cpio; honour an
# explicit override (kunit.sh passes the kunit build's copy when only
# .kunit/ has been built, e.g. on the build-and-kunit CI leg that
# doesn't run build-kernel.sh).
GEN="${IPU4_GEN_INIT_CPIO:-$LINUX_DIR/usr/gen_init_cpio}"
if [[ ! -x "$GEN" ]]; then
	echo "missing $GEN; run tools/build-kernel.sh first" >&2
	exit 2
fi

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

# Snippets of the init, if it has any (tools/rootfs/init.<name>.d/, e.g.
# the ipu6 probe tests' checks), as /init.d/.
if [[ -d "$INIT_SRC.d" ]]; then
	echo "dir  /init.d             0755 0 0" >> "$LIST"
	for f in "$INIT_SRC.d"/*.sh; do
		[[ -e "$f" ]] || continue
		echo "file /init.d/$(basename "$f")  $f 0644 0 0" >> "$LIST"
	done
fi

# The flavor's driver names for the ladder's inits (ipu-driver.<flavor>.conf).
DRIVER_CONF="$HERE/ipu-driver.${IPU4_FLAVOR:-fork}.conf"
if [[ -f "$DRIVER_CONF" ]]; then
	echo "dir  /etc              0755 0 0" >> "$LIST"
	echo "file /etc/ipu-driver.conf  $DRIVER_CONF 0644 0 0" >> "$LIST"
fi

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

# v4l2 core helpers that some 6.12.y point releases split out of the
# previously built-in videodev module. On v6.12.0 the symbols are
# inlined and these .ko files don't exist, making the loop a no-op;
# on a tip where v4l2-async / v4l2-fwnode are separate modules,
# intel-ipu4-isys fails insmod with "Unknown symbol
# __v4l2_async_register_subdev" / "Unknown symbol
# v4l2_fwnode_endpoint_parse" unless the modules are staged here and
# loaded before intel-ipu4-isys. v4l2-fwnode depends on v4l2-async, so
# the load order in init.* matters.
V4L2_DIR="$LINUX_DIR/drivers/media/v4l2-core"
for v4l in v4l2-async v4l2-fwnode; do
	ko="$V4L2_DIR/$v4l.ko"
	if [[ -f "$ko" ]]; then
		echo "file /lib/modules/$v4l.ko  $ko 0644 0 0" >> "$LIST"
	fi
done

# Driver modules (if build.sh has run).
if compgen -G "$DRV_DIR/*.ko" > /dev/null; then
	for ko in "$DRV_DIR"/*.ko; do
		[[ "$(basename "$ko")" == *_kunit.ko ]] && continue
		echo "file /lib/modules/$(basename "$ko")  $ko 0644 0 0" >> "$LIST"
	done
fi

for ko in "${EXTRA_KOS[@]}"; do
	if [[ -f "$ko" ]]; then
		echo "file /lib/modules/$(basename "$ko")  $ko 0644 0 0" >> "$LIST"
	fi
done

# CPD firmware blob. Generated on-demand by tools/firmware/gen-cpd.py;
# minimal content that passes ipu6_cpd_validate_cpd_file() but stops
# probe later at pkg_dir creation (see STATUS.md for the M4 roadmap).
# Both flavors get the same blob, under the name their driver requests.
FW_DIR="$OUT/firmware"
mkdir -p "$FW_DIR"
FW_BLOB="$FW_DIR/ipu4_cpd_b0.bin"
if [[ ! -f "$FW_BLOB" || "$ROOT/tools/firmware/gen-cpd.py" -nt "$FW_BLOB" ]]; then
	python3 "$ROOT/tools/firmware/gen-cpd.py" "$FW_BLOB"
fi
fw_dir=/lib/firmware
echo "dir  $fw_dir           0755 0 0" >> "$LIST"
IFS=/ read -ra fw_parts <<< "$(dirname "$FW_NAME")"
for part in "${fw_parts[@]}"; do
	[[ "$part" == . ]] && continue
	fw_dir="$fw_dir/$part"
	echo "dir  $fw_dir           0755 0 0" >> "$LIST"
done
echo "file /lib/firmware/$FW_NAME  $FW_BLOB 0644 0 0" >> "$LIST"
for name in "${FW_EXTRA_NAMES[@]}"; do
	echo "file /lib/firmware/$name  $FW_BLOB 0644 0 0" >> "$LIST"
done

if [[ -x "$STREAMON_BIN" ]]; then
	echo "file /bin/streamon  $STREAMON_BIN 0755 0 0" >> "$LIST"
fi

"$GEN" "$LIST" | gzip -9 > "$OUT/rootfs.cpio.gz"
echo ">>> built $OUT/rootfs.cpio.gz ($(stat -c%s "$OUT/rootfs.cpio.gz") bytes)"
