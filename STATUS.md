# IPU4 dev/test harness — status

This file tracks the state of the QEMU-based dev/test environment for the
IPU4 driver: an in-tree fork of Linux at `v6.12` plus a fork of QEMU at
`v9.1.0` carrying our `hw/misc/ipu4.c` device model, joined by a tiered
test stack that runs entirely in software (no real silicon required).

For the project's overall posture (out-of-tree driver, upstream-IPU6
discipline, prerequisites for a fresh checkout) see `CLAUDE.md`. For the
ongoing upstream-tracking workflow see `tools/upstream/`.

## Layout

```
tools/
  bootstrap.sh           fork Linux + QEMU, apply patches, seed driver sources
  build.sh               build the driver as an external module in tools/linux/
  build-qemu.sh          build qemu-system-x86_64 with the IPU4 device wired in
  build-kernel.sh        build bzImage + modules from the same .config as build.sh
  run-vm.sh              boot test VM with the emulated IPU4 device
  rebase.sh              weekly rebase onto linux-6.12.y, run tiered tests
  tests/
    kunit.sh             Tier 1: ipu4_format + ipu4_bayer KUnit suites via kunit.py
    streamon-smoke.sh    Tier 2 (live): full v4l2 capture API walk to STREAM:pattern_ok
    mmiotrace.sh         re-run streamon-smoke under mmiotrace, capture qemu.trace
    compare-mmio.sh      diff captured trace against silicon's data/trace.txt
    probe-smoke.sh       progress-graded probe smoke (driver loads, devices appear)
    vm-smoke.sh          boot VM, assert 0x8086:0x5a88 enumerates
    pytest.sh            pytest + 90% coverage gate for the trace tools
    e2e.sh               Tier 2 placeholder (pending SHA frame check)
    streamon.c           static v4l2 client embedded as /bin/streamon in the rootfs
  rootfs/
    build.sh             busybox + IPU4 module initramfs via gen_init_cpio
    init.{vm-smoke,probe-ok,streamon,streamon-mmiotrace,mmiotrace}
                         per-test guest /init scripts
  firmware/
    gen-cpd.py           synthesise the minimal CPD blob for ipu6_cpd_validate
  trace/
    compare.py           silicon-vs-QEMU mmiotrace diff
  coverage/
    collect.sh           gcov tarball -> lcov --extract -> genhtml
  upstream/
    diff.sh              regenerate tools/notes/upstream-diff/ divergence report
    watch.sh             cron-driven new-IPU6-commit detector + cherry-pick triage PR
    render-pr-body.sh    formats the watcher's NDJSON output into a PR body
    _lib.sh              shared file mapping + IPU4_LOCAL_ONLY skip list
  notes/
    registers.md         source of truth for every implemented hw/misc/ipu4.c handler
    upstream-divergence.md  static audit of kernel/ipu4/ vs upstream IPU6
    ifdef-ipu6-audit.md  per-site classification of every #ifdef IPU6 hunk
    upstream-watch-state.json  rolling state for tools/upstream/watch.sh
  linux-patches/         files mirrored into tools/linux/ by bootstrap.sh
    drivers/media/pci/intel/ipu4/
      Kconfig Makefile     in-tree Kconfig + Makefile for the driver
      tests/
        Kconfig Makefile .kunitconfig
        ipu4_format_kunit.c  ipu4_bayer_kunit.c
  qemu-patches/          files mirrored into tools/qemu/ by bootstrap.sh
    hw/misc/ipu4.c
.github/
  actions/setup-harness/action.yml   shared apt + pip install (canonical prereq list)
  workflows/
    build-and-kunit.yml  reusable: apt setup + bootstrap + build + kunit
    pr.yml               PR gate (calls build-and-kunit.yml)
    main.yml             same on push to main/master
    vm-smoke.yml         full VM: probe-smoke + streamon-smoke + mmiotrace + compare-mmio
    rebase.yml           weekly rebase onto linux-6.12.y
    upstream-watch.yml   daily IPU6-cherry-pick triage cron
data/trace.txt           silicon's mmiotrace capture; baseline for compare-mmio
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

- **M1 — KUnit tier:** done. `ipu4_format_kunit.c` and
  `ipu4_bayer_kunit.c` build and run as KUnit modules under
  `qemu-system-x86_64` via the kernel's `tools/testing/kunit/kunit.py`.
  The `ipu4_mmu_kunit.c` suite was retired when upstream's MMU
  map/unmap optimisation (Linux v6.12-era backport) inlined
  `ipu6_mmu_pgsize()` into the lower `l2_*` helpers, removing the
  symbol the test was wired to. `ipu4_ring_kunit.c` and
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

- **M5b — streaming-smoke baseline:** done as a checkpoint.
  `tools/tests/streamon.c` is a tiny static v4l2 client that walks
  the capture API one ioctl at a time and prints a `STREAM:STEP`
  marker on each. `tools/rootfs/build.sh` compiles it (`gcc -static`)
  and embeds it as `/bin/streamon` in the initramfs.
  `tools/tests/streamon-smoke.sh` boots the VM with
  `init.streamon`, walks the markers, and grades against
  `IPU4_STREAM_REQUIRED` (default `STREAM:reqbufs`).

  Current furthest reached: `STREAM:qbuf` for both buffers — all of
  open / QUERYCAP / S_FMT / REQBUFS / QUERYBUF / QBUF succeed.
  STREAMON still returns `ENOLINK`, but media-graph state is now
  explicitly bootstrapped: `streamon.c` locates the media entity
  wrapping `/dev/video0` (by v4l2 minor with name fallback) and
  enables the one link that matters — `Intel IPU4 CSI2 0:1 →
  sink=<video-entity>:0`. The virt-sensor → CSI2 link was already
  enabled + immutable. Default `IPU4_STREAM_REQUIRED` tightened
  from `STREAM:reqbufs` to `STREAM:qbuf`.

  STREAMON still fails because the IPU6 driver uses the streams-API
  (`VIDIOC_SUBDEV_S_ROUTING`) — dynamic links alone don't satisfy
  `media_pipeline_start()` without a route table entry on the CSI2
  subdev. Next M5b PR: either populate routes from the guest init
  (post-QBUF, pre-STREAMON) or have the driver auto-route on
  sensor bind.

- **M5b-3 — CSI2 active route auto-installed on sensor bind:** done
  as infrastructure. `isys_install_virt_sensor_route()` in
  `kernel/ipu4/ipu6-isys.c` runs on the `isys_notifier_bound`
  callback when `CONFIG_VIDEO_IPU4_VIRT_SENSOR=y` and calls
  `v4l2_subdev_set_routing_with_fmt()` on the CSI2 subdev with
  `sink=0/0 → source=1/0, 800x800 RGB888` (matching the
  virt-sensor's reported format). Dmesg now shows
  `virt-sensor: installed active route on Intel IPU4 CSI2 0`.

  STREAMON still returns `ENOLINK` — `ipu6_isys_setup_video()`
  fails one layer deeper at
  `media_pad_remote_pad_unique(&av->pad)`. The Capture-side pad
  doesn't see the CSI2→Capture link as enabled even after
  `MEDIA_IOC_SETUP_LINK`, so the pipeline walker can't find the
  unique remote. Route-install is still needed infrastructure;
  the remaining barrier is separate (investigating whether the
  link needs enabling via a different mechanism, or whether the
  Capture entity's `link_validate` needs to accept dynamic links
  differently). `IPU4_STREAM_REQUIRED` remains `STREAM:qbuf`.

- **M6 — weekly rebase cron:** done. `.github/workflows/rebase.yml`
  runs every Monday 06:00 UTC (and on `workflow_dispatch`).
  `tools/rebase.sh` wipes `tools/linux/`, re-bootstraps pointing at
  `linux-6.12.y` on the stable-tree mirror (a moving branch, not
  the pinned `v6.12` tag), rebuilds, and runs the full smoke tier
  we have today — `kunit.sh`, `vm-smoke.sh`, `probe-smoke.sh`,
  `streamon-smoke.sh`. The defunct `e2e.sh` reference is dropped
  until M5c provides real frame delivery. On failure, the
  workflow uploads serial logs and `.config` as artifacts.

- **M5c-4 — gcov harvest + lcov HTML:** done. `init.streamon`
  rounds each `.gcda` in `/sys/kernel/debug/gcov/` through `cat`
  into a regular-file scratch tree (debugfs reports size=0 so a
  plain `tar` under-captures), tars it onto the 9p share, and
  the host's `tools/coverage/collect.sh` overlays each `.gcda`
  onto the build tree before running `lcov --capture` +
  `genhtml`. The report is filtered to
  `*/drivers/media/pci/intel/ipu4/*` so v4l2-core lines don't
  dilute the driver picture.

  `vm-smoke.yml` installs `lcov`, runs `collect.sh` after
  `streamon-smoke.sh`, and uploads the HTML tree as
  `coverage-html-<run-id>`. Current baseline:
  **33.1% lines / 40.7% functions** of the IPU4 driver covered
  by one streamon-smoke run. The number moves visibly when the
  smoke tier exercises new paths — future PRs can watch it.

- **M5c-3 — deterministic frame pattern + verifier:** done.
  `buf_queue_virt` now fills each plane with `byte[k] = (k +
  sequence) & 0xff` and increments a sequence counter per frame.
  `tools/tests/streamon.c` mmaps the queued buffers, and after
  `VIDIOC_DQBUF` checks the pattern at a handful of offsets
  (0, 1, 127, 128, 255, 256, 4095, 65535) against the per-frame
  expected byte. Mismatch reports the offset, got/want, and
  sequence so a regression in either the kernel fill or the DMA
  mapping shows up immediately.

  `streamon-smoke.sh` default `IPU4_STREAM_REQUIRED` raised to
  `STREAM:pattern_ok`. First DQBUF reports
  `STREAM:pattern_ok seq=0 bytes=1945600`.

  Not yet: frame-rate pacing (buffers still complete synchronously
  from `buf_queue`), multi-frame sequence verification (the test
  reads just one DQBUF), and the full-buffer SHA check — follow-up
  commits.

- **M5c-2 — software streaming path, STREAMON and DQBUF green:**
  done. `kernel/ipu4/ipu6-isys-queue.c` now has
  `start_streaming_virt`, `stop_streaming_virt`, and
  `buf_queue_virt` gated by
  `#if IS_ENABLED(CONFIG_VIDEO_IPU4_VIRT_SENSOR)`. The main
  `start_streaming` / `stop_streaming` / `buf_queue` ops
  early-dispatch to the virt variants.

  The virt path:
  - runs `ipu6_isys_setup_video()` (brings up the media pipeline
    and allocates `av->stream`),
  - increments `stream->nr_streaming` and sets `streaming = 1`
    so the stop path has matching state to tear down,
  - returns every queued buffer immediately from `buf_queue_virt`
    with `VB2_BUF_STATE_DONE`, `bytesused = plane length`, and
    `timestamp = ktime_get_ns()` — vb2's already-allocated
    DMA-coherent pages hold zeros,
  - on stop, releases the stream ref and flushes any stragglers
    with `VB2_BUF_STATE_ERROR`.

  `streamon-smoke.sh` now reaches `STREAM:dqbuf` — the full
  capture-API walk:
  open / QUERYCAP / S_FMT / REQBUFS / QUERYBUF / QBUF / **STREAMON /
  DQBUF**. `STREAM:done bytes=1948032` on success. Default
  `IPU4_STREAM_REQUIRED` raised to `STREAM:dqbuf`.

  The M5c-1 `-EOPNOTSUPP` guard in `ipu6_fw_isys_open()` stays as
  defensive — the new code path never calls it, but leaving the
  guard in place means a stray call (e.g. the restart-streams
  recovery branch) still fails cleanly instead of trying firmware
  that isn't backed.

  Not in this PR:
  - Deterministic frame content — buffers are zero-filled.
    A follow-up commit will either fill a pattern in kernel
    (simple) or push data from the QEMU device model via an
    MMU walker (closer to real hardware, bigger).
  - Frame timing — buffers complete synchronously from buf_queue;
    nothing emulates a sensor's frame-rate cadence yet.

- **M5c-1 — graceful STREAMON short-circuit when there's no
  firmware:** done. `ipu6_configure_spc()` in the M5b-5 run
  kernel-panicked because the stub CPD blob produces zeroed
  pkg_dir entries, which yields a wild pointer in
  `ipu6_pkg_dir_configure_spc()` at
  `kernel/ipu4/ipu6.c:435`.

  Gating `ipu6_fw_isys_open()` with
  `#if IS_ENABLED(CONFIG_VIDEO_IPU4_VIRT_SENSOR)` to return
  `-EOPNOTSUPP` up front bypasses the whole SPC / firmware /
  syscom path during STREAMON. Guest sees
  `STREAM:fail step=streamon errno=95 (Operation not supported)`
  and the VM shuts down cleanly — no more kernel panic.

  Actual frame delivery needs a virt-sensor-direct path: the
  virt-sensor's `.s_stream()` arms a timer, the QEMU device model
  emits a frame into the buffer DMA address, and an M5c vb2
  shim returns it to userspace without touching the firmware.
  That's the next commit (or set of commits — MMU + timer + vb2
  bypass).

- **M5b-5 — streamon uses BGR24 so link_validate passes:** done.
  `ipu6_isys_pfmts[]` has no entry for `V4L2_PIX_FMT_RGB24`; only
  `V4L2_PIX_FMT_BGR24` maps to `MEDIA_BUS_FMT_RGB888_1X24` — the
  mbus code the virt-sensor / CSI2 subdev advertise. Asking for
  `RGB24` from `streamon.c` silently fell back to the first table
  entry (`SBGGR12`), and `link_validate()` rejected the pipeline
  with `-EPIPE` on format mismatch.

  Switching the fourcc to `BGR24` (`'BGR3'`) makes
  `ipu6_isys_setup_video()` pass: `remote_pad` lookup, route walk,
  `media_pipeline_start()` and `link_validate()` all succeed, and
  the driver proceeds into `ipu6_configure_spc()` — where it
  kernel-panics on a null-pointer dereference at
  `CR2: ffa000008084e048` (firmware control structures not backed
  by the QEMU model's MMU). `streamon-smoke.sh` now detects and
  reports "kernel panic mid-STREAMON" explicitly; the next
  milestone is the first real QEMU MMU / syscom work.

- **M5b-4 — fix video-entity lookup so the right link gets
  enabled:** done. Root cause of the M5b-3 `ENOLINK` was a
  one-line bug in `tools/tests/streamon.c`:
  `find_video_entity()` started the `MEDIA_IOC_ENUM_ENTITIES`
  walk at `id=1 | FLAG_NEXT`, which returns entities with id > 1
  — skipping the first video entity (id=1, "Intel IPU4 ISYS
  Capture 0", v4l2 minor=0) that actually backs `/dev/video0`.
  The fallback-by-name was hard-coded to "Capture 1" for the
  same reason.

  Starting the walk at `id=0 | FLAG_NEXT` and computing the
  fallback name from `want_minor` (`"Intel IPU4 ISYS Capture
  %u"`) finds entity id=1 correctly. STREAMON now reaches
  deeper: `remote_pad` lookup succeeds, the pipeline walker
  proceeds past `media_pipeline_start()`, and the ioctl returns
  `-EPIPE` ("Broken pipe") from the subdev `link_validate` or
  per-pad format propagation. That's the next M5b barrier —
  investigating whether the virt-sensor + CSI2 subdev formats
  are advertised consistently for streams-API validation.
  `IPU4_STREAM_REQUIRED` stays at `STREAM:qbuf`.

- **M5c — frame delivery + e2e:** after media-link enable lands.
  MMU page-table walks in `hw/misc/ipu4.c`, syscom ring state +
  doorbell, a QEMUTimer-driven frame generator, the virt-sensor
  `.s_stream` hook that produces deterministic frames, and the
  yavta SHA-256 e2e test in `tools/tests/e2e.sh`.

- **M6/M7/M8 — rebase cadence + 6.18/mainline:** cron workflow in
  place; 6.18 and mainline jobs not yet added — they wait on M5 being
  green on 6.12.

## Running the harness

Prereqs: install the apt + pip packages listed in `CLAUDE.md` ("Local
prerequisites"). They match the canonical set in
`.github/actions/setup-harness/action.yml`.

```bash
tools/bootstrap.sh            # one-time: clones Linux@v6.12 + QEMU@v9.1.0, seeds patches + driver
tools/build.sh                # build intel-ipu4{,-isys}.ko + ambu-ipu-bridge.ko (fast)
tools/tests/kunit.sh          # Tier 1: ipu4_format + ipu4_bayer KUnit suites (~1 s)
tools/build-qemu.sh           # build qemu-system-x86_64 with the ipu4 device
tools/build-kernel.sh         # full guest kernel + modules (for VM runs)
tools/rootfs/build.sh         # busybox initramfs via gen_init_cpio

# Tier 2 (live): boots the VM end-to-end. ~1 min on warm caches.
IPU4_ACCEL=tcg tools/tests/streamon-smoke.sh    # walk v4l2 capture API to STREAM:pattern_ok
IPU4_ACCEL=tcg tools/tests/mmiotrace.sh         # rerun under mmiotrace, capture qemu.trace
tools/tests/compare-mmio.sh                     # diff against silicon's data/trace.txt

# Optional:
tools/coverage/collect.sh     # HTML report in tools/coverage/html/ (after streamon-smoke)
tools/upstream/diff.sh        # regenerate tools/notes/upstream-diff/ vs upstream IPU6
```

The forked Linux and QEMU URLs/tags are configurable via `IPU4_LINUX_URL`,
`IPU4_LINUX_TAG`, `IPU4_QEMU_URL`, `IPU4_QEMU_TAG`; see `tools/bootstrap.sh`
for all environment hooks.

## What is intentionally not done here

- No real hardware captures exist, so register behavior in
  `hw/misc/ipu4.c` is labeled `guess` in the notes. The M3 loop is the
  plan for upgrading those guesses.
- `kernel/ipu4/` is not deleted yet. That happens after the first
  green e2e run on 6.12, so a revert of the in-tree layout remains
  trivial until then.
- `ipu4-compat.h` is kept rather than removed by `bootstrap.sh`.
