# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Out-of-tree Linux kernel driver for Intel IPU4, forked from the upstream IPU6 driver (`drivers/media/pci/intel/ipu6/`, upstreamed in 6.10). Tested against stable kernels 6.6.111 and 6.12.47. The driver does not work on any public hardware as-is — `ambu_ipu_bridge_*` calls must be replaced before the device probes.

There are two parallel layouts for the driver right now (see STATUS.md for the migration plan):

1. **`kernel/ipu4/` — the source of truth.** Out-of-tree module with its own `Makefile`. Edit driver `.c`/`.h` here.
2. **`tools/linux/drivers/media/pci/intel/ipu4/` — a seeded copy.** `tools/bootstrap.sh` clones upstream Linux at `v6.12` into `tools/linux/` and copies `kernel/ipu4/*.[ch]` into the in-tree path. This copy is regenerated and is *not* committed. Do not edit files under `tools/linux/` directly; edit `kernel/ipu4/` and re-bootstrap.

## The upstream-IPU6 discipline

Backports from upstream should be performed with `git cherry-pick -x` and fixing any conflicts. If the code doesn't work directly for IPU4 it should be handled with

- `#ifdef IPU6` (the `IPU6` macro is never defined — it's a "this is the upstream version, leave alone" marker), This can both be used for functions that are not needed, and function bodies where the behaviour should be different on IPU4 **or**
- a new function named `ipu4_*` parallel to the `ipu6_*` original.

Do not restructure or "clean up" IPU6 code. Do not rename `ipu6_*` symbols. Keep diffs minimal and localized.

`kernel/ipu4/ipu4-compat.h` holds `LINUX_VERSION_CODE`-gated shims for 6.6 / 6.10 / 6.11 kernel API changes. Add new shims here rather than `#ifdef`-ing callers.

## Common commands

Out-of-tree build against an external kernel tree:

```bash
make -C kernel/ipu4 KERNEL_SRC=/path/to/linux            # build intel-ipu4.ko + intel-ipu4-isys.ko
make -C kernel/ipu4 KERNEL_SRC=/path/to/linux checkpatch # upstream checkpatch, --strict, 80col
make -C kernel/ipu4 clean
```

In-tree build via the QEMU harness (preferred for local iteration):

```bash
tools/bootstrap.sh          # clones tools/linux/ @ v6.12 and tools/qemu/ @ v9.1.0, seeds patches + driver
tools/build.sh              # configures kconfig and builds intel-ipu4.ko in tools/linux/
tools/tests/kunit.sh        # Tier 1: KUnit suites (ipu4_mmu, ipu4_format, ipu4_bayer) under qemu-kvm
tools/tests/e2e.sh          # Tier 2: boot VM, STREAMON, frame-hash check (needs tools/rootfs/build.sh — not yet implemented)
tools/rebase.sh             # rebase tools/linux/ onto linux-6.12.y and re-run tiers
```

`tools/build.sh` uses `KBUILD_MODPOST_WARN=1`: undefined-symbol errors are downgraded to warnings because vmlinux is not built here. Those symbols resolve at `insmod` time inside the guest VM.

The bootstrap script is idempotent. Re-run after pulling to re-apply patches from `tools/{linux,qemu}-patches/`. Override the upstream URLs/tags via env vars `IPU4_LINUX_URL`, `IPU4_LINUX_TAG`, `IPU4_QEMU_URL`, `IPU4_QEMU_TAG`.

## CI

- `.github/workflows/pr.yml` — bootstrap + build + kunit on every PR.
- `.github/workflows/main.yml` — same, on push to `main`/`master`. Coverage and e2e jobs are staged but not yet wired.
- `.github/workflows/rebase.yml` — weekly cron that rebases `tools/linux/` onto `linux-6.12.y`.

## Known in-progress work

See `STATUS.md` for milestone state. Key "not done yet" items that affect what tests actually run:

- `drivers/media/pci/intel/Kconfig` is not yet patched to expose `CONFIG_VIDEO_IPU4`, so `tools/tests/kunit.sh` prints `kunit: skipping` and exits 0 rather than failing. CI is green on purpose until that wiring lands.
- `tools/rootfs/build.sh` is a stub — `tools/tests/e2e.sh` and `tools/run-vm.sh` can't run end-to-end until it produces `bzImage` + `rootfs.cpio.gz`.
- QEMU's `hw/misc/ipu4.c` register behavior is labeled `guess` in `tools/notes/registers.md` — no real hardware captures exist yet.

## Legacy hardware trace scripts

`trace_ipu4.sh`, `trace_functions.sh`, `split_trace.sh`, `postprocess_trace.py` are mmiotrace helpers meant to run on real target hardware (require `kernel/configs/mmiotrace.config` and `ambu-tc358748`). They're kept as reference for porting to other devices; they won't run in the QEMU harness.
