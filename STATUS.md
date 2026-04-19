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
  pr.yml                 KUnit + build (PR gate)
  main.yml               reserved: e2e + coverage when they land
  vm-smoke.yml           boot VM + assert 0x8086:0x5a88 (M2)
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

- **M2 — QEMU skeleton + VM boot:** done. `tools/bootstrap.sh` now
  clones a forked QEMU and wires `hw/misc/ipu4.c` into the
  `hw/misc/Kconfig` + `hw/misc/meson.build` via idempotent appends.
  `tools/build-qemu.sh` builds `qemu-system-x86_64` with the IPU4
  device registered; `tools/build-kernel.sh` builds `bzImage` + module
  set from the same config as `build.sh`; `tools/rootfs/build.sh`
  produces a busybox initramfs via the kernel's `gen_init_cpio`;
  `tools/run-vm.sh` boots the whole thing with `-device ipu4`.
  `tools/tests/vm-smoke.sh` asserts the guest enumerates
  `0x8086:0x5a88` on its PCI bus and prints `VM_SMOKE: PASS`. A
  dedicated `.github/workflows/vm-smoke.yml` runs this on merges to
  the integration branch and on demand (not on PRs — too slow).

- **M3 — probe progresses to firmware load:** done. Progress-graded
  smoke test (`probe-smoke.sh`) reports the furthest reached marker
  and passes against a configurable `IPU4_PROBE_REQUIRED` checkpoint.

- **M4 — firmware synthesized, probe reaches bridge init:** done as a
  checkpoint. `tools/firmware/gen-cpd.py` generates a minimal CPD blob
  that satisfies `ipu6_cpd_validate_cpd_file()`; `rootfs/build.sh`
  drops it at `/lib/firmware/ipu4_cpd_b0.bin`. Probe now runs to
  `ambu_ipu_bridge_init()` and stops at the I2C adapter lookup
  (adapters 0 and 3 don't exist under QEMU). `probe-smoke.sh`
  default `IPU4_PROBE_REQUIRED` is raised to `probe:fw_valid`; the
  actual reached marker is `probe:bridge`.

- **M4.5 — virt-sensor bridge bypass:** done.
  `kernel/ipu4/virt-sensor.c` installs a software-node graph on
  `pdev->dev.fwnode->secondary` and registers a v4l2_subdev
  advertising `MEDIA_BUS_FMT_RGB888_1X24` at 800×800, replacing
  the `ambu_ipu_bridge_init()` call under
  `CONFIG_VIDEO_IPU4_VIRT_SENSOR=y`.

- **M5a — probe completes, /dev/video* appears:** done. The M4.5
  panic turned out to be a `BTRS_PWR_STATE_IS_PWR_RDY` shift
  off-by-one in `hw/misc/ipu4.c` (19 → 20) — probe was racing the
  power-up poll, not crashing on MMU. Fixing the constant so
  `PWR_STATE` reports all IPU4 power islands ready
  (`bits 13:12=HH_DONE, 23:20=IS_RDY, 28:24=PS_PWR_UP, 1:0=PWR_RDY`)
  unblocked the whole probe path: `ipu6_psys_init()` completes,
  isys notifier binds the virt-sensor, and the v4l2 core registers
  23 `/dev/video*` nodes. `probe-smoke` default `IPU4_PROBE_REQUIRED`
  is raised to `probe:video_node`. One cosmetic `"Change power
  status timeout"` line is logged by the power-down poll in
  `ipu6_buttress_power(on=false)` — the always-ready constant never
  reads 0 so the poll times out, but probe ignores it and continues.

- **M5b — streaming + coverage:** next piece. MMU page-table walks
  in `hw/misc/ipu4.c` so real DMA allocations land correctly,
  syscom ring state + doorbell, a QEMUTimer-driven frame generator,
  the virt-sensor `.s_stream` hook that produces deterministic
  frames, and the yavta SHA-256 e2e test in `tools/tests/e2e.sh`.

- **M6/M7/M8 — rebase cadence + 6.18/mainline:** cron workflow in
  place; 6.18 and mainline jobs not yet added — they wait on M5 being
  green on 6.12.

## Running the harness

```bash
tools/bootstrap.sh            # one-time: forks Linux + QEMU, seeds patches
tools/build.sh                # build intel-ipu4.ko (driver-only, fast)
tools/tests/kunit.sh          # tier 1, ~1 s of test execution
tools/build-qemu.sh           # build qemu-system-x86_64 with ipu4 device
tools/build-kernel.sh         # full guest kernel + modules (for VM runs)
tools/rootfs/build.sh         # busybox initramfs via gen_init_cpio
tools/tests/vm-smoke.sh       # M2: boot VM, assert 0x8086:0x5a88 enumerates
tools/tests/e2e.sh            # tier 2, boots VM and streams (M4 target)
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
