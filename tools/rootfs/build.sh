#!/usr/bin/env bash
# Build a minimal initramfs that can load intel-ipu4.ko and run
# tools/tests/guest-streamon.sh. Output: tools/rootfs/out/{bzImage,rootfs.cpio.gz}.
#
# Implementation placeholder. Options under consideration:
#   - buildroot with br2-external pointing at tools/rootfs/br2/
#   - alpine minirootfs + mkinitramfs
#   - dracut on an existing host distro
# The choice is made once M2 is ready to be run.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="$HERE/out"
mkdir -p "$OUT"

echo "tools/rootfs/build.sh: not yet implemented" >&2
echo "create: $OUT/bzImage (guest kernel)"         >&2
echo "create: $OUT/rootfs.cpio.gz (initramfs)"      >&2
exit 2
