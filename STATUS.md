# IPU4 dev/test harness — status

This file tracks the state of the QEMU-based dev/test environment described
in `/root/.claude/plans/i-want-a-setup-sunny-moler.md`.

## Layout

```
tools/
  bootstrap.sh           fork Linux + QEMU, apply patches, seed driver sources
  build.sh               build the driver in-tree in tools/linux/
  run-vm.sh              boot test VM with the emulated IPU4 device
  rebase.sh              weekly rebase onto linux-6.12.y, run tiered tests
  tests/
    kunit.sh             Tier 1: KUnit suites under qemu-kvm (~500 ms)
    e2e.sh               Tier 2: VM + STREAMON + 5 frames (~10 s)
    guest-streamon.sh    runs inside the guest: insmod, yavta, harvest gcov
    frames.sha256        expected dequeued-buffer hashes (populated at M4)
  coverage/
    collect.sh           gcov tar -> lcov --extract -> genhtml
  trace/
    diff.py              diff two mmiotrace captures across fuzz iterations
  notes/
    registers.md         register-behavior log for hw/misc/ipu4.c
  linux-patches/         files mirrored into tools/linux/ on bootstrap
    drivers/media/pci/intel/ipu4/
      Kconfig Makefile     in-tree Kconfig + Makefile for the driver
      tests/
        Kconfig Makefile .kunitconfig
        ipu4_format_kunit.c  ipu4_bayer_kunit.c  ipu4_mmu_kunit.c
  qemu-patches/          files mirrored into tools/qemu/ on bootstrap
    hw/misc/ipu4.c
  rootfs/
    build.sh             initramfs builder (not yet implemented)
.github/workflows/
  pr.yml                 KUnit + build
  main.yml               KUnit + e2e + coverage
  rebase.yml             weekly rebase cron
```

## Milestone state

- **M0 — in-tree migration:** done. `tools/bootstrap.sh` clones Linux
  at `v6.12`, copies an in-tree `Makefile` and `Kconfig` to
  `drivers/media/pci/intel/ipu4/`, seeds the driver sources from
  `kernel/ipu4/`, and idempotently appends a `source` line to
  `drivers/media/pci/intel/Kconfig` and an `obj-$(CONFIG_VIDEO_INTEL_IPU4)
  += ipu4/` line to the parent `Makefile`. The driver builds via
  `make M=drivers/media/pci/intel/ipu4`. `ipu4-compat.h` is kept in
  place: on v6.12 its only active macro is unused, but removing it
  would require editing four `#include` sites, deferred to a later
  upstreaming pass. The `kernel/ipu4/` tree remains the source of
  truth in this repo until M2 is green.

- **M1 — KUnit tier:** done. `ipu4_format_kunit.c`,
  `ipu4_bayer_kunit.c`, `ipu4_mmu_kunit.c` build and run as KUnit
  modules under `qemu-system-x86_64`. 12 tests pass in ~1.2 s of test
  execution (the surrounding kernel build dominates wall time).
  `ipu6_mmu_pgsize()` was un-staticed and declared in `ipu6-mmu.h` so
  the test can reference it. `ipu4_ring_kunit.c` and
  `ipu4_queue_kunit.c` are still skipped because those call paths go
  through `readl`/`writel` and list helpers; adding them means
  introducing small MMIO fakes.

- **M2 — QEMU skeleton:** `hw/misc/ipu4.c` in place with the buttress
  register subset the probe path touches first. Guest rootfs builder is
  a stub.

- **M3/M4/M5 — fuzzing loops + e2e + coverage:** tooling written,
  expected to be iterated on the first live boot.

- **M6/M7/M8 — rebase cadence + 6.18/mainline:** cron workflow in
  place; 6.18 and mainline jobs not yet added — they wait on M5 being
  green on 6.12.

## Running the harness

```bash
tools/bootstrap.sh            # one-time: forks Linux + QEMU, seeds patches
tools/build.sh                # build intel-ipu4.ko
tools/tests/kunit.sh          # tier 1, should be <1 s once booted
tools/rootfs/build.sh         # TODO: produce out/bzImage + out/rootfs.cpio.gz
tools/tests/e2e.sh            # tier 2, boots VM and streams
tools/coverage/collect.sh     # HTML report in tools/coverage/html/
```

The forked Linux and QEMU URLs are configurable via `IPU4_LINUX_URL` and
`IPU4_QEMU_URL`; see `tools/bootstrap.sh` for all environment hooks.

## What is intentionally not done here

- No real hardware captures exist, so register behavior in
  `hw/misc/ipu4.c` is labeled `guess` in the notes. The M3 loop is the
  plan for upgrading those guesses.
- `kernel/ipu4/` is not deleted yet. That happens after the first
  green e2e run on 6.12, so a revert of the in-tree layout remains
  trivial until then.
- `ipu4-compat.h` is kept rather than removed by `bootstrap.sh`.
