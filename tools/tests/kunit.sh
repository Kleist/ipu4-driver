#!/usr/bin/env bash
# Tier-1 tests: run KUnit suites under qemu-kvm via kunit.py.
# Target: <1 second wall clock for the five IPU4 unit suites.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
LINUX_DIR="$ROOT/tools/linux"

if [[ ! -d "$LINUX_DIR" ]]; then
	echo "tools/linux/ missing; run tools/bootstrap.sh first" >&2
	exit 1
fi

cd "$LINUX_DIR"

KUNITCFG="drivers/media/pci/intel/ipu4/tests/.kunitconfig"
if [[ ! -f "$KUNITCFG" ]]; then
	echo "missing $KUNITCFG (was bootstrap.sh run?)" >&2
	exit 2
fi

# kunit.py uses O=.kunit and refuses to run if the source tree is dirty
# from a prior in-tree make (e.g. tools/build.sh leaving .o files behind).
# Clean once before invoking; mrproper is a no-op on an already-clean tree.
make -s mrproper

exec ./tools/testing/kunit/kunit.py run \
	--arch=x86_64 \
	--kunitconfig=drivers/media/pci/intel/ipu4/tests \
	--jobs="$(nproc)" \
	'ipu4_*'
