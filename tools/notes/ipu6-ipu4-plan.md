# Plan — IPU4 support on top of the upstream IPU6 driver (Ruslan's IPU4P series)

Status: **proposed** (PR 1 of the series below). Each later PR updates the
"State" column in the PR table.

## Goal

Upstream-quality patches that add Intel IPU4 (Broxton / Apollo Lake,
`8086:5a88`) to the mainline `drivers/media/pci/intel/ipu6/` driver, on top
of Ruslan Bay's IPU4P (Ice Lake, `8086:8a19`) series, verified against this
repo's QEMU IPU4 model on every PR.

## Base

| What | Value |
|---|---|
| Tree | `https://github.com/ruslanbay/linux.git` |
| Ref | `340985596` (`ipu4p-upstream`, "media: ipu6: Enable IPU4P PCI device") |
| Upstream base | v7.3-rc4 + Antti Laakso's IPU7-in-ipu6 series ([lore][antti]) + Ruslan's 11 IPU4P patches ([lore][ruslan]) |

The pin is the SHA referenced from Ruslan's lore mail. His later
`work/ipu4p-upstream-series-rebuild-*` branches (e.g. the 64-bit firmware-ABI
alignment fix) are picked up by an explicit pin-bump PR, not automatically.

[antti]: https://lore.kernel.org/all/20260907113004.2489993-1-sakari.ailus@linux.intel.com/
[ruslan]: https://lore.kernel.org/all/aedlbUANNY6QvZ_mrVqU9aH15j76MKgYxxwkpmayED7tFEDztNQN8A-vDuLUIRpb9At6QpBp4gg9RAOA6gvD-LpCOUpFm8aVDJ5IK87tpPg=@proton.me/

## Layout and conventions

- Kernel changes are kept as `git format-patch` output in
  `patches/ipu6-ipu4/NNNN-*.patch`. The harness clones the pinned tree and
  applies them with `git am`, so the files in the repo are exactly what
  would be mailed to linux-media.
- The new flow runs **alongside** the existing `kernel/ipu4/` fork flow,
  selected with `IPU4_FLAVOR=ipu6` (default stays `fork`). Existing CI,
  pins and tests keep running unchanged.
- The pin lives in `tools/ipu6/pin` (one line to bump). The tree is
  fetched into `tools/linux-ipu6/` (gitignored, never `tools/linux/`) by
  `tools/ipu6/bootstrap.sh`, which resets to the pin and re-applies the
  series on every run; `tools/ipu6/build.sh` builds the ipu6 modules.
- Harness-only code never goes into `patches/`. The virt-sensor becomes a
  standalone test module that attaches a software-node graph to the PCI
  device before `intel-ipu6` loads.
- Patch style follows Ruslan's series: `IS_IPU4()`, `ipu4_*` helpers next to
  `ipu4p_*`, `checkpatch.pl --strict` clean, every patch builds on its own.
- Every change goes in through a PR from a short-lived branch into `main`.

## Working rules

- **Every step starts by re-reading this plan** (on the tip of the branch
  it builds on) and **ends by updating it** in the same PR: the "State"
  column, any change to the PR → test mapping, and new facts in
  "Learnings" below. A PR that learned nothing says so in its
  description.
- **Steps are pipelined, PRs are not.** As soon as a step's PR is up,
  the next step starts on a local branch on top of it while the PR is in
  review. That branch is pushed and its PR opened only once the previous
  PR has merged, rebased onto the new `main` (and re-tested). So the
  track has at most one open PR (see `CLAUDE.md`), and review feedback
  is folded into the local branch on its rebase.
- **Stay in this track's files.** Other tracks (QEMU variants) run in
  parallel with their own plans; see "Tracks and file ownership" below
  for which files each owns and how shared files are changed.

## Decisions

| # | Decision |
|---|---|
| 1 | Pin the lore SHA `340985596`. |
| 2 | Firmware file `intel/ipu/ipu4_fw.bin` (matches `ipu4p_fw.bin` naming). |
| 3 | Keep the fork's CSI-2 frame-start/frame-end interrupts (useful for debugging). Firmware SOF/EOF responses stay the source of V4L2 events, so the CSI-2 path must not emit them a second time; it handles the interrupt and logs at `dev_dbg`. May be revisited before posting upstream. |
| 4 | One small PR per step, branch → `main`, CI (including the QEMU tests) runs on each. |

## IPU4 vs IPU4P — what the patches must cover

The two share the ISYS firmware ABI (byte-identical stream config, frame
buffer set, response and token structs) and the ISYS/PSYS/MMU/DMEM/SPC base
offsets. Ruslan's `ipu4p_fw_isys_ops` are reused for IPU4. Differences
(source: `kernel/ipu4/` and the QEMU model, which is derived from the
silicon trace in `data/trace.txt`):

| Area | IPU4P (Ruslan) | IPU4 (Broxton) |
|---|---|---|
| PCI ID | `8a19` | `5a88` |
| Buttress PWR_STATE FSM | IS 24:20 = `0xc`, PS 29:25 = `0x10` | IS 23:20 = `0xa`, PS 28:24 = `0xf` |
| FREQ_CTL ratio field | IS ratio at bit 1; PS at bit 8 + override bit 7 | ratio bits 7:0, QoS floor bits 15:8 |
| SysCom queue-pointer base register (DMEM) | 8 | 6 |
| MMU L1 stream block sizes | IPU4P tables | IPU4 tables (`kernel/ipu4/ipu6.c`) |
| UNISPART software IRQ | BIT(22) | BIT(30) |
| CSI-2 port IRQ in UNISPART | SIP0/SIP1 scheme | `0x8 << port` (0–3), `0x800 << (port - 4)` (4–5) |
| CSI-2 ports | 8, IRQ controllers in separate windows | 6 at `0x64000 0x65000 0x66000 0x67000 0x6c000 0x6c800`; IRQ trios at port base + `0x400/0x500/0x600` |
| CSI-2 receiver registers | `IPU4P_CSI2_RX_*` | same layout (reused) |
| PHY sideband setup | required | none |
| CSI-2 port ↔ firmware source | `{3, 6..12}` | identity |
| CSI-RX error bits | 16 | 17 (adds "inter-frame long packet discarded") |
| Buttress IRQ status register | `BUTTRESS_REG_ISR_STATUS` (`ipu6_buttress_regs`) | `BUTTRESS_REG_ISR_ENABLED_STATUS` (`0x94`, fork: `isp->buttress.reg_irq_sts`) |

## Testing approach

A new CI workflow, `vm-ipu6.yml`, runs on every PR: bootstrap the pinned
tree, `git am` the series, build kernel + QEMU, boot the VM and run a
**test ladder**. Which tests are gated is controlled by a checked-in list,
`tools/tests/ipu6/enabled-tests`, one test name per line. The workflow runs
every test but only fails on the enabled ones, so later tests are visible as
"expected fail" before they are switched on. **It starts with every test
disabled; each PR enables (at least) one more.**

Upstream wants the PCI ID table entry last, so the series always ends with
`media: ipu6: Enable IPU4 PCI device`. The series carries that patch from
PR 8a on (new patches go in before it), so the driver binds `5a88` from its
PCI table inside `insmod`, exactly as on a real system; there is no
harness-only binding. (PRs 5–8 used a `new_id` write instead.)

The runner is `tools/tests/ipu6/ladder.sh` (`--list` prints the names).
Each test ends PASS, FAIL, NOT-IMPLEMENTED (not written yet) or SKIP (a test
it depends on, run in the same invocation, did not pass); an enabled test
must PASS, and an unknown name in `enabled-tests` fails the job.

Planned tests (names are what goes into `enabled-tests`). Big tests are
split, or gated by the existing graded markers (`IPU4_PROBE_REQUIRED`,
`IPU4_STREAM_REQUIRED`), so every test is small enough to turn on alone:

| Test | Passes when |
|---|---|
| `series-applies` | `git am` of `patches/ipu6-ipu4/` succeeds on the pin |
| `build` | `intel-ipu6*.ko` build cleanly (W=1) |
| `checkpatch` | `checkpatch.pl --strict` clean on every patch |
| `vm-boot` | guest boots and enumerates `8086:5a88` |
| `module-load` | `intel-ipu6` + `intel-ipu6-isys` + virt-sensor load without oops |
| `probe-identify` | probe (from the PCI table, at `insmod`) identifies IPU4 |
| `probe-buttress` | buttress init + ISYS/PSYS power-up succeed |
| `probe-mmu` | the PSYS MMU init at probe writes the IPU4 stream tables: the guest's writes to INFO and the L1/L2 stream start registers of the three PSYS MMUs (BAR + `0x4b0000`) equal, in order, the ones in `data/trace.txt` (QEMU write log, `tools/trace/check_writes.py`) |
| `isys-irq` | a forced ISYS runtime resume (probe init, `power/control`) succeeds and writes what silicon's second ISYS resume writes (`data/trace.txt` lines 487–599): ISYS MMU stream tables, no LTR/DID (`0x68`), one TSC sync request (`0x88`), the UNISPART interrupt setup (`0x17c000`, `0x40001878`) |
| `probe-fw` | firmware (`ipu4_fw.bin`, synthesized) is mapped and authenticated (`ipu6_buttress_authenticate()`) (passes under the model since PR 7, like `probe-complete`; its step, PR 10, was folded into PR 11) |
| `probe-complete` | ISYS probe finishes, `/dev/video*` and media graph appear (passes under the model since PR 7, so it gates nothing in the IPU4 sequence; kept for the IPU6 variant, see "Parallel track") |
| `fw-open` | syscom open + `STREAM_OPEN_DONE` round trip: on the stream boot (`stream-boot.sh`, `init.ipu6-stream`), the driver gets `STREAM_OPEN_DONE` for stream 0 and completes the open, and the boot's ISYS DMEM writes start with silicon's from the firmware open to that response (`data/trace.txt` lines 600–622, `check_writes.py --prefix`) |
| `stream-on` | `streamon-smoke` reaches `STREAM:streamon` |
| `stream-pattern` | `streamon-smoke` reaches `STREAM:pattern_ok` |
| `csi2-sof-irq` | CSI-2 FS IRQ observed on port 0 |

## PR sequence

| PR | Content | Test(s) enabled | State |
|---|---|---|---|
| 1 | This plan | — | merged (#100) |
| 2 | `IPU4_FLAVOR=ipu6` bootstrap/build + empty `patches/ipu6-ipu4/` + `vm-ipu6.yml` ladder runner, all tests disabled | — | merged (#101) |
| 3 | Enable static tests on the empty series | `series-applies`, `build`, `checkpatch` | merged (#102) |
| 4 | Guest kernel config + rootfs for the ipu6 flavor | `vm-boot` | merged (#103) |
| 5 | Standalone virt-sensor module, init loads `intel-ipu6*` | `module-load` | merged (#113) |
| 6 | `media: ipu6: Identify IPU4 devices` (helper, PCI ID, firmware name, probe case; no table entry) | `probe-identify` | merged (#115) |
| 7 | `media: ipu6: Add IPU4 buttress support` + harness stages a synthesized `intel/ipu/ipu4_fw.bin` (see PR 6 learnings) | `probe-buttress` | merged (#117) |
| 8 | `media: ipu6: Add IPU4 MMU support` (MMU tables + the IPU4P subsystem offsets they hang off) + harness logs the probe boot's MMIO writes | `probe-mmu` | merged (#121) |
| 8a | Carry `media: ipu6: Enable IPU4 PCI device` as the always-last patch; drop the harness's `new_id` binding (was PR 14) | — (`probe-identify` now binds natively; `native-bind` dropped) | merged (#122) |
| 9 | `media: ipu6: Support the IPU4 TSC sync handshake` + `media: ipu6: Add IPU4 ISYS and CSI-2 register definitions` + `media: ipu6: Add IPU4 ISYS interrupt handling` (wire-up in ISYS, 6 CSI-2 ports); probe init forces an ISYS runtime resume (was PR 10, swapped with the pdata step) | `isys-irq` (was `probe-complete`) | merged (#125) |
| 10 | ~~Internal pdata (`media: ipu6: Add IPU4 platform data`)~~; the firmware name landed in PR 6, the ISYS/PSYS/SPC/DMEM offsets in PR 8, the CSI-2 port count in PR 9, the stream limits in PR 11 | — (was `probe-fw`) | folded into PR 11 |
| 11 | `media: ipu6: Add the IPU4 ISYS stream limits` (the rest of the pdata, old PR 10) + `media: ipu6: Support the IPU4 SysCom queue layout`; before it, a shared-core PR adds the stream boot (`stream-boot.sh`, `init.ipu6-stream`, `streamon` subdev format) | `fw-open` | in review |
| 12 | `media: ipu6: Add IPU4 CSI-2 receiver support` | `stream-on`, `stream-pattern` | |
| 13 | CSI-2 frame interrupts (decision 3) | `csi2-sof-irq` | |
| 14 | ~~`media: ipu6: Enable IPU4 PCI device`~~ | — | moved to PR 8a |
| 15 | Cover letter draft + `get_maintainer.pl` output | — | |
| Pages | Dashboard section "IPU4 port (upstream ipu6)": `ladder.sh` writes `results.json`, `vm-ipu6` uploads it per variant, Pages shows a test × variant matrix | — | merged (#109) |

The patch-to-test mapping follows probe order and may shift by one when
the real probe path is run; if a patch can't turn a test green on its own,
the PR is split rather than enabling nothing.

## Tracks and file ownership

Three tracks work on the ipu6 flavor in parallel, each with its own plan
and at most one open PR at a time, from `main` (`CLAUDE.md`):

| Track | Plan | Owns |
|---|---|---|
| IPU4 port (this plan) | `tools/notes/ipu6-ipu4-plan.md` | `patches/ipu6-ipu4/`, `tools/tests/ipu6/enabled-tests` |
| IPU6 variant | [`tools/notes/ipu6-variant-plan.md`](ipu6-variant-plan.md) | `tools/tests/ipu6/enabled-tests.ipu6` |
| IPU4P variant | [`tools/notes/ipu4p-variant-plan.md`](ipu4p-variant-plan.md) | `tools/tests/ipu6/enabled-tests.ipu4p` |

Goal of the variant tracks: run the same ladder against the other variants
of the shared driver, so the IPU4 patches are shown not to break them. They
run interleaved with this plan's sequence and never block it.

To keep the tracks from conflicting:

- **One file per test.** A ladder test is `tools/tests/ipu6/tests.d/NNN-<test>.sh`
  and its guest-side probe checks are
  `tools/rootfs/init.ipu6-probe.d/NNN-<test>.{bound,unloaded}.sh` (READMEs
  there). A track adding a test adds files; it does not edit a shared list.
  A test belongs to the track that added it; another track adapting it to
  its variant does so with a `$IPU4_VARIANT` / `$want` case in that file,
  in a PR of its own.
- **Shared core changes go first, alone.** `ladder.sh`, `probe-boot.sh`,
  `stream-boot.sh`, the `init.ipu6-*` frames, `tools/tests/streamon.c`,
  `tools/run-vm.sh`, `tools/rootfs/build.sh` and `vm-ipu6.yml` are
  shared. A change to them is its own commit, ahead of the feature
  commits that need it; the feature PR may carry it, but the commit
  must pass CI on its own and hold no feature changes.
- **Own plan only.** A track's PR edits its own plan file; facts another
  track needs go in its PR description (and the owner records them).
- **QEMU model.** `tools/qemu-patches/hw/misc/ipu4.c` is shared by every
  variant. Variant-specific register behaviour goes behind the model's
  `variant` switch in functions of its own (`ipu4p_*`, `ipu6v_*`), added
  in their own hunks, so two variant tracks rarely touch the same lines.

## Learnings

Facts found while executing the plan that later steps depend on. Newest
last; each entry names the PR that found it.

- (PR 1) The pinned tree also carries Ruslan's downstream IPU4/IPU4P
  driver (`CONFIG_VIDEO_INTEL_IPU`, `drivers/media/pci/intel/ipu4/`,
  `ipu-*.c`). It binds `5a88`/`8a19` too, so the harness config must
  keep it disabled.
- (PR 1) `lore.kernel.org` and `git.kernel.org` are not reachable from the
  CI/agent sandbox; GitHub is. Fetch kernel trees from GitHub mirrors.
- (PR 1) The QEMU model hard-codes the syscom queue layout with the queue
  pointer base at DMEM register 6 (msg-send cursors at DMEM regs 10, 12, …),
  matching IPU4 silicon. A driver that uses IPU4P's base (8) on IPU4 gets
  no firmware responses from the model.
- (PR 1) The model never answers the firmware proxy queue. Ruslan's
  `set_iwake_register` sends proxy tokens and waits up to 100 ms for a
  reply, so expect timeouts/errors there until the model or the driver
  path is adjusted.
- (PR 2) GitHub serves the pin by SHA: `git fetch --depth 1 <url>
  <full-sha>` works although the SHA is not on the default branch.
  The shallow tree is ~300 MB of `.git`; the fetch took ~45 s here.
- (PR 2) The `build` test is `defconfig` + `modules_prepare` +
  `make M=drivers/media/pci/intel/ipu6 W=1`: ~22 s cold on 4 cores, so no
  full kernel build is needed until the VM tests (PR 4). The pinned
  ipu6 directory is W=1-clean with gcc 13.3 (zero warnings), so the test
  treats any compiler warning as a failure. `modules_prepare` needs
  `libelf-dev` (objtool); `setup-harness` already installs it.
- (PR 2) Ruslan's `drivers/media/pci/intel/Makefile` adds
  `subdir-ccflags-y` warning suppressions (unused-parameter,
  uninitialized, int-conversion, enum-conversion, ...) for his downstream
  driver, and they also apply to `ipu6/` in a full-tree build. An `M=`
  build does not read that Makefile, so it shows the warnings upstream
  would see. Keep the `build` test on `M=` (or check warnings in a way
  that bypasses those flags) when PR 4 adds a full kernel build.
- (PR 2) QEMU is still bootstrapped by `tools/bootstrap.sh` together with
  the fork's `tools/linux/` clone, and `tools/build-kernel.sh` /
  `tools/rootfs/build.sh` hard-code `tools/linux/`. PR 4 needs a
  QEMU-only bootstrap path and ipu6-flavor variants of those two
  scripts. Note `tools/build-kernel.sh` calls `tools/build.sh`, which
  now dispatches on `IPU4_FLAVOR`.
- (Pages PR) The ladder's machine-readable output is
  `tools/tests/out/ipu6-ladder/results.json` (schema in
  `tools/dashboard/README.md`), written by `write_results_json` at the
  very end of `ladder.sh`. New tests need no change for it: anything in
  `TESTS` is recorded, unimplemented ones as `NOT-IMPLEMENTED`. Keep
  that function and its call as the last thing before `exit`.
- (Pages PR) `workflow_run` only fires for workflows that exist on the
  default branch, and `gh run list --workflow=vm-ipu6.yml --branch=main`
  returns nothing (no error) until `vm-ipu6` has run there, so the
  dashboard shows "not run yet" until PR 2 is merged and has run once
  on `main`.
- (PR 3) With the empty series, `series-applies` and `build` are real
  gates (bootstrap resets to the pin; `build` recompiles every ipu6
  object at W=1), but `checkpatch` passes vacuously ("no patches") until
  PR 6 adds the first kernel patch. Verified with throw-away patches:
  a style/no-`Signed-off-by` patch fails `checkpatch`, a patch adding a
  function without a prototype fails `build`, and a non-applying patch
  fails `series-applies` (and `build` is SKIPped). The pin's `defconfig`
  has `CONFIG_WERROR=y`, so W=1 warnings in ipu6 are already hard errors
  on top of the test's log scan.
- (PR 3) `build` only builds the tip of the series. The "every patch
  builds on its own" rule in `patches/ipu6-ipu4/README.md` is not
  checked yet; add a per-patch build (e.g. `git rebase -x`) once the
  series has more than one patch.
- (PR 4) The fork's VM scripts take the flavor as a parameter instead of
  getting ipu6 copies: `IPU4_FLAVOR=ipu6` makes `tools/build-kernel.sh`,
  `tools/rootfs/build.sh`, `tools/run-vm.sh` and `tools/tests/vm-smoke.sh`
  use `tools/linux-ipu6/`, the ipu6 driver directory,
  `tools/rootfs/out-ipu6/` and `tools/tests/out/vm-smoke-ipu6.serial`.
  The default (fork) paths are unchanged. QEMU comes from
  `tools/bootstrap.sh --qemu-only`, which skips `tools/linux/`; the CI
  QEMU cache key is the same as in `vm-smoke-reusable.yml`, so either
  workflow reuses the other's build.
- (PR 4) The QEMU-guest kconfig trim moved from `tools/build.sh` into
  `tools/vm-kconfig.sh`, which both flavors call. `tools/ipu6/build.sh`
  applies it too, so the `build` test (M=) and the guest kernel share one
  `.config` and neither invalidates the other's objects. The trimmed
  config keeps the ipu6 directory W=1-clean.
- (PR 4) `CONFIG_WERROR` is **off** for the ipu6 guest kernel (the pin's
  defconfig has it on). The full v7.3-rc4 tree builds without warnings
  with gcc 13.3 today, but with `-Werror` a runner gcc bump that adds a
  warning in any unrelated subsystem would fail every VM test. The
  `build` test's log scan is the warning gate for ipu6; re-checked with
  a throw-away patch (missing prototype) → `build` FAIL without
  `-Werror`.
- (PR 4) The full build compiles ipu6 with the parent Makefile's
  `subdir-ccflags` (see PR 2), and the `build` test's `M=` clean +
  rebuild then compiles it again with upstream flags. The two builds
  overwrite each other's ipu6 objects, which costs a few seconds per
  run; the modules packed into the initramfs come from the full build.
  The initramfs already contains `intel-ipu6*.ko` and the vb2 modules
  (VIDEOBUF2_* end up `=m` because only `VIDEO_INTEL_IPU6=m` selects
  them), so PR 5 only has to load them.
- (PR 4) Timings on 4 cores (TCG): cold `vm-boot` 334 s, nearly all of it
  the bzImage + modules build; the TCG boot reaches `/init` at ~1.6 s and
  the whole VM run (rootfs rebuild + boot + poweroff) is ~4 s. Warm
  ladder (all four enabled tests): 33 s. QEMU from `--qemu-only`: clone
  + wiring 4 s, cold build 2.5 min. Cold CI is therefore ~15 min plus
  apt and cache transfer; `vm-ipu6.yml` timeout is 45 min (same as
  `vm-smoke-reusable.yml`).
- (PR 4) The build tree is ~2.7 GB on disk (≈1.5 GB checkout, 290 MB
  `.git`, rest build output; `DEBUG_INFO_NONE` in defconfig keeps it
  small). An interrupted earlier fetch had left a 2.7 GB
  `tmp_pack_*` in `tools/linux-ipu6/.git` locally; delete it if a local
  tree looks oversized (CI trees are fresh fetches).
- (PR 5) The virt-sensor is `tools/virt-sensor/virt-sensor.c`, built out
  of tree by `tools/virt-sensor/build.sh` (`make -C tools/linux-ipu6
  M=tools/virt-sensor MO=tools/virt-sensor/out W=1`, a few seconds,
  W=1- and `checkpatch --strict`-clean). On load it finds `8086:5a88`
  (`device=`/`vendor=` module parameters for other variants), builds the
  ipu-bridge-shaped software-node graph (`INT343E` node, `port@0` /
  `endpoint@0` with `data-lanes`, sensor endpoint with `bus-type`,
  `data-lanes`, `link-frequencies`), attaches it with
  `set_secondary_fwnode()`, registers an `ipu4-virt-sensor` platform
  device carrying the sensor node and a state-based v4l2 subdev on it
  (800x800 `RGB888_1X24`, `V4L2_CID_LINK_FREQ`, `enable_streams` no-op).
  It refuses to load when the IPU is already bound or already has a
  graph, and unloads cleanly (`set_secondary_fwnode(dev, NULL)`). In
  QEMU the IPU (00:04.0) has an ACPI primary fwnode, so the graph is a
  secondary, exactly as with ipu-bridge.
- (PR 5) Upstream ipu6 calls `ipu_bridge_init()` unconditionally in
  `ipu6_isys_init()`, but needs no change for the virt-sensor: the
  harness config has `CONFIG_IPU_BRIDGE` off, where `ipu_bridge_init()`
  is the constant 0, and with it on, `ipu_bridge_check_fwnode_graph()`
  returns early because the graph exists. `isys_notifier_init()` walks
  `dev_fwnode(pci dev)` ports 0..7 with `fwnode_graph_get_endpoint_by_id()`,
  which sees secondaries.
- (PR 5) `RGB888_1X24` is in upstream `csi2_supported_codes` and in the
  ISYS video format table (`V4L2_PIX_FMT_BGR24`, CSI-2 DT `0x24`), so the
  fork's format is kept.
- (PR 5) The fork's `isys_install_virt_sensor_route()` (M5b-3) is not
  needed as a route: upstream's `ipu6_isys_subdev_init_state()` already
  installs the active 0/0 → 1/0 route on each CSI-2 subdev. What the
  fork really installs is the **format**: upstream's default is
  4096x3072 `SGRBG10_1X10`, which fails link validation against the
  800x800 RGB888 sensor. The stream tests should set the CSI-2 sink (and
  downstream) format from userspace (`VIDIOC_SUBDEV_S_FMT` in the
  streamon helper) instead of a driver hook.
- (PR 5) `module-load` boots `tools/rootfs/init.ipu6-modules`: insmod
  vb2-common/memops/v4l2/dma-sg, `virt-sensor`, `intel-ipu6`,
  `intel-ipu6-isys` (`MODLOAD:ok|fail <name>`), assert `5a88` unbound and
  the sensor device present, rmmod in reverse, scan dmesg for
  `Oops|BUG:|WARNING:|Call Trace`, print `MODLOAD: PASS|FAIL`. Serial log:
  `tools/tests/out/ipu6-module-load.serial`. v4l2-async/fwnode are
  built in (`=y`) on this config. Out-of-tree load taints the kernel
  (`O`); harmless.
- (PR 5) Binding by `echo 8086 5a88 > .../drivers/intel-ipu6/new_id`
  today reaches probe and fails in the device switch:
  `intel-ipu6 0000:00:04.0: error -ENODEV: Unsupported IPU6 device 5a88`
  (the `new_id` write itself returns 0). That line is what PR 6's
  `probe-identify` turns into an IPU4 identification.
- (PR 5) Timings (4 cores, TCG, warm): `module-load` alone 16 s
  standalone (includes a no-op `build-kernel.sh`), 6 s inside the full
  ladder; whole warm ladder 39 s. Gate checked: an init skipping
  `videobuf2-common` and a truncated `virt-sensor.ko` both end in
  `MODLOAD: FAIL`.
- (PR 6) The first kernel patch is
  `0001-media-ipu6-Identify-IPU4-devices.patch`: `IPU_VERSION_4`
  (`BIT(7)`, "APL") + `IS_IPU4()`, `PCI_DEVICE_ID_INTEL_IPU4 0x5a88` in
  `include/media/ipu6-pci-table.h` (not in `ipu6_pci_tbl`),
  `IPU4_FIRMWARE_NAME "intel/ipu/ipu4_fw.bin"`, and the probe case
  (`hw_ver`, `cpd_fw_name`, `ipu6se_cpd_metadata_cmpnt` size — the fork
  uses the IPU6SE layout for IPU4 too, as Ruslan does for IPU4P). One
  patch, not Ruslan's helper/probe split: his helper patch only adds the
  bit, and his probe case arrives with the buttress patch; ours needs the
  probe case for the `probe-identify` test, and the whole patch is 10
  added lines. The subject says "Identify IPU4 devices" because it does
  more than add the helper.
- (PR 6) The probe case leaves `isp->buttress.regs` and the
  `isys_ctrl`/`psys_ctrl` at the IPU6 defaults set before the switch (the
  commit message says so); PR 7 points them at IPU4 tables. IPU4 also
  needs its own `ipu6_buttress_registers`: the fork reads IRQ status from
  `BUTTRESS_REG_ISR_ENABLED_STATUS` (`0x94`), see the new table row.
- (PR 6) `hw_ver` is a `u8`; `IPU_VERSION_4` takes its last free bit.
  Another variant would have to widen it.
- (PR 6) `probe-identify` boots `tools/rootfs/init.ipu6-probe` (module
  load as in `init.ipu6-modules`, then `new_id`, unbind if bound, rmmod,
  oops scan; `PROBE:` markers). Identification = no `Unsupported IPU6
  device 5a88` and a firmware request for `intel/ipu/ipu4_fw.bin` from
  `0000:00:04.0`: the loader logs the name when the file is missing
  ("Direct firmware load ... failed") and, with
  `drivers/base/firmware_loader/main.c` dyndbg (enabled by the init),
  when it loads it ("Loading firmware from ..."), so the marker stays
  valid once later PRs ship the file. It needs probe to get through
  `ipu6_buttress_init()`, which it does. The later probe tests should
  reuse this init and add markers. busybox `insmod` ignores
  `<module>.dyndbg=` from the kernel command line, so the init passes
  `dyndbg=+p` itself (the `module-load` init gets no driver debug).
- (PR 6) How far probe gets now: PCI BAR, `ipu6_buttress_init()` passes
  on the IPU6 defaults under the model ("IPU in secure mode", "IPU6 touch
  0x0 mask 0x0", "IPC reset done"), then the first failure is
  `request_firmware()`: `error -ENOENT: Requesting signed firmware
  intel/ipu/ipu4_fw.bin failed` (the ipu6 rootfs only has the fork's
  `ipu4_cpd_b0.bin`). No oops. Experiment with the fork's
  `tools/firmware/gen-cpd.py` blob staged as `intel/ipu/ipu4_fw.bin`:
  the CPD validates with the IPU6SE component size ("FW version: 0"),
  ISYS and PSYS MMU domains initialise, then ISYS/PSYS power-up fails
  twice with `Change power status timeout with 0xa02003` and probe ends
  with `-EBUSY` (-16), no oops. `0xa02003` has the IPU4 IS FSM value `0xa`
  in bits 23:20, while the IPU6 control polls bits 4:3 for `0x3`.
- (PR 6) Probe order is buttress init → `request_firmware()` + CPD
  validation → ISYS/PSYS bus devices (MMU sw init) → PSYS runtime resume
  (buttress power-up) → MMU hw init → map firmware → authenticate. So
  `probe-buttress` (PR 7) can only pass once the rootfs carries a CPD
  file that validates; PR 7 has to stage the synthesized blob as
  `/lib/firmware/intel/ipu/ipu4_fw.bin` (harness change in
  `tools/rootfs/build.sh` for the ipu6 flavor). `probe-fw` is redefined
  above to mean mapping + authentication, which is where probe goes after
  the MMU.
- (PR 6) Gate check: with the patch removed, `probe-identify` FAILs with
  `PROBE:identify fail (Unsupported IPU6 device 5a88)`. `checkpatch` now
  checks a real patch (0 errors/warnings/checks, `--strict`); `build` is
  W=1-clean with it. Timings (4 cores, TCG, warm): full ladder 64 s
  (`vm-boot` 37 s incl. the kernel rebuild for the new patch,
  `probe-identify` 5 s); `series-applies` + `probe-identify` alone 38 s.
- (PR 7) Second kernel patch `0002-media-ipu6-Add-IPU4-buttress-support.patch`
  (0001 kept byte-identical; 0002 generated with `format-patch -N
  --start-number 2 HEAD~1`). Values checked against the silicon trace
  (`data/trace.txt`, BAR `0x90000000`): power-up writes IS_FREQ_CTL
  `0x80000004` and PS_FREQ_CTL `0x80001212`, PWR_STATE then reads
  `0xa72433` (IS FSM 23:20 = `0xa`) and `0xfa72333` (PS FSM 28:24 =
  `0xf`); power-down writes 0. So the FREQ_CTL layout is IPU6's
  (ratio 7:0, QoS floor 15:8; IS ratio 4 / floor 0, PS ratio 0x12 / floor
  0x12) **without** the ICCMAX level (19:16) that `__ipu6_power()` always
  sets. The patch therefore reuses `__ipu6_power()` with an
  `if (!IS_IPU4(isp))` around the ICCMAX bit instead of adding an
  `__ipu4_power()` (Ruslan's IPU4P needs one for its shifted ratio
  fields; IPU4 does not). Also in the patch: IPU4 `ipu6_buttress_ctrl`
  pair, an `ipu4_buttress_regs` table (copy of `ipu6_buttress_regs` with
  `irq_status = BUTTRESS_REG_ISR_ENABLED_STATUS`, like IPU7P5's copy for
  one differing field; the trace only ever reads 0x94), bootloader status
  at PSYS + 0x8000 via `(IS_IPU4(isp) || IS_IPU4P(isp))` reusing
  `IPU4P_BOOTLOADER_STATUS_OFFSET`, and the probe case points at them.
  One patch: the register-set part is a table plus one line, too small to
  split.
- (PR 7) The QEMU model already matches: `IS/PS_FREQ_CTL` BIT(31) latches
  per-island power, PWR_STATE composes `0xa << 20` / `0xf << 24`, ICCMAX
  is ignored, 0x90 and 0x94 read the same. No model change needed. The
  model pre-sets AUTH_DONE in SECURITY_CTL, so `ipu6_buttress_authenticate()`
  returns early and the bootloader-offset change is not exercised by any
  test (silicon-only, per the fork and the trace).
- (PR 7) Deferred buttress differences (not on the probe path, all
  ISYS-stream time): TSC sync polls `BUTTRESS_PWR_STATE_HH_STATUS_MASK`
  = GENMASK(12, 11), but IPU4 reports the handshake in bits 13:12 (fork
  `IPU4_BUTTRESS_PWR_STATE_HH_STATUS_MASK`; silicon `0xa72433` has DONE=2
  in 13:12 and 0 in 12:11; the model sets `2 << 12`). It runs in
  `isys_runtime_pm_resume()`, i.e. at the first stream-on, so it will
  show up as "Start tsc sync timeout" at `fw-open`/`stream-on`; it needs
  an IPU4 case (a follow-up buttress patch or with the ISYS patches).
  TSC tick conversion uses `b->ref_clk` from BTRS_CTRL; silicon reads
  BTRS_CTRL `0x10` (REF_CLK_IND 0 → 24.0 MHz) while the fork hard-codes
  19.2 MHz for IPU4 — timestamps only, decide with the stream tests.
  The SECURITY_TOUCH / CAMERA_MASK debug reads that Ruslan skips for
  IPU4P are kept for IPU4 (the fork reads them too; the model returns 0).
- (PR 7) Harness: `tools/rootfs/build.sh` stages the `gen-cpd.py` blob
  under the name each flavor requests (`FW_NAME`): fork
  `/lib/firmware/ipu4_cpd_b0.bin` (unchanged), ipu6
  `/lib/firmware/intel/ipu/ipu4_fw.bin`. `probe-identify` keeps passing
  with the file present (dyndbg "Loading firmware from ..." marker).
- (PR 7) Once power-up works, the PSYS MMU init maps the 8 MB trash
  buffer page by page with `dev_dbg` per page (~8000 lines), which pushes
  the start of probe out of the kernel log buffer and broke the
  `identify` check. `init.ipu6-probe` now turns off `ipu6-mmu.c` debug
  after insmod, except `format "iova trash buffer"`, and dumps every
  `intel_ipu6`/`intel-ipu6` line (aux devices too), not just the PCI
  device's.
- (PR 7) `probe-buttress` = `PROBE:buttress ok` + `PROBE:pwr-errors
  none`: the `intel_ipu6.psys.*: iova trash buffer for MMUID` line
  (printed by the PSYS MMU init right after the PSYS runtime resume,
  which powers up ISYS — its parent — first), both aux devices
  `runtime_status` = suspended after probe (power-down through the FSM
  worked), and no "Change power status timeout" / "power down failed!"
  from bind to rmmod. The probe tests now share one VM boot per ladder
  run (`boot_probe_vm`, stamp file because tests run in a subshell).
  Gate check: without 0002, `probe-buttress` FAILs with
  `PROBE:buttress fail (Change power status timeout)` (`0xa02003`, twice)
  while `probe-identify` still passes. Timings (4 cores, TCG, warm):
  full ladder 68 s (`vm-boot` 38 s incl. the kernel rebuild for the new
  patch, `probe-identify` 6 s incl. the boot, `probe-buttress` 0 s on the
  shared boot).
- (PR 7) **How far probe gets now: all the way.** Under the model,
  `ipu6_pci_probe()` completes ("IPU0-v0[5a88] hardware version 128"; SKU
  reads 0): PSYS MMU hw init passes with the IPU6 MMU defaults, the
  firmware is mapped, authentication short-circuits on the model's
  AUTH_DONE, and the ISYS driver probes too (virt-sensor bound on port 0,
  `/dev/media0`, 32 `/dev/video*`, subdevs). Unbind/rmmod are clean, no
  oops. So the model does not distinguish the IPU6 from the IPU4 MMU
  layout at probe: `probe-mmu`, `probe-fw` and `probe-complete` as
  defined above would already pass with PR 7 and cannot gate PR 8/9/10.
  The first IPU4-specific failure is expected at `fw-open` (ISYS runtime
  resume: ISYS MMU hw init, TSC sync HH mask, SysCom queue base 8 vs 6).
  PR 8 has to redefine `probe-mmu` on something the model can observe —
  e.g. the MMU L1/L2 stream block-size registers written in
  `ipu6_mmu_hw_init()` checked against the IPU4 values (model-side
  check or an mmiotrace compare), or a model change that rejects wrong
  values (separate PR) — or the mapping should be reordered.
- (PR 8) Third kernel patch `0003-media-ipu6-Add-IPU4-MMU-support.patch`
  (0001/0002 byte-identical; `format-patch -N --start-number 3 HEAD~1`).
  The pin keeps the MMU tables in `ipu6-mmu-hw.c`, selected per variant
  through `mmu->ops` in `ipu6_mmu_init()`, so they are not platform
  data. IPU4 reuses everything of IPU4P's except the L1 block sizes:
  `ipu4p_mmu_ops` (`IS_IPU4() || IS_IPU4P()` in `ipu6_mmu_init()`),
  whose `__ipu4p_mmu_init_hw_data()` now picks an
  `ipu4_mmu_hwdata_lookup` for IPU4. The IPU4P MMU offsets, info bit
  (`BIT(4)` on the MMU0s), stream counts (16/16) and MMU0
  read-before-invalidate are the fork's values too. L1 block sizes
  (fork `kernel/ipu4/ipu6.c`): ISYS MMU1 `8,16,16,16,0×11,8`; PSYS MMU1
  `0,0,0,0,10,8,10,8,0,4,4,12,0,0,0,8`; PSYS MMU1R
  `0×8,8,0,0,0,16,12,12,16`; L2 all 2. The start addresses they give
  are exactly the silicon trace's (PSYS at `data/trace.txt` lines
  55–254, ISYS at 269–369 and again at 489–589).
- (PR 8) The MMU windows are addressed from the subsystem base
  (`hw_variant.offset`), which IPU4 had at 0 (IPU6 unified layout), so
  the IPU6 tables went to BAR + `0x1b0000`. The patch therefore also
  sets the IPU4P subsystem layout for IPU4 in
  `ipu6_internal_pdata_init()` (ISYS `0x100000`, PSYS `0x400000`, SPC 0,
  DMEM `0x8000`; = fork `ipu4-platform-regs.h`) in its own
  `if (IS_IPU4(isp))` block, which PR 9 extends with the rest of the
  internal pdata (CSI-2 ports, stream/SRAM/queue limits). SPC/DMEM are
  only used at ISYS stream open, so they are not visible at probe. With
  the PSYS base right, `ipu6_buttress_authenticate()` now reads the
  bootloader status at the real BAR + `0x408000` (still short-circuited
  by the model's AUTH_DONE).
- (PR 8) Not ported, on purpose: the reference driver's address
  translator ZLW setup (silicon writes `ZLW_EN/1D/N/2D` per L1 stream
  at MMU1 + `0x100 + 0x20 * i` and `FW_ZLW_PRIO` at MMU1 + `0x300`;
  upstream ipu6 keeps the `l1_zlw_*` fields but never writes them, and
  IPU4P does not either), and its TLB invalidate value (silicon writes
  `0x1` to MMU0 and `0xffff` to MMU1, upstream `0xffffffff` to all).
  Both only matter on hardware (prefetch, the extra bits); revisit if
  real IPU4 hardware shows MMU faults or throughput problems.
- (PR 8) How `probe-mmu` sees the MMU: `tools/run-vm.sh` takes
  `IPU4_MMIO_WRITE_LOG=<file>` and adds `-D <file> -trace
  memory_region_ops_write` (QEMU core trace event, log backend, built
  into the harness QEMU; absolute address, value and region name,
  `ipu4-bar0` for the IPU). It is host-side, so it needs no
  `CONFIG_MMIOTRACE` in the ipu6 guest, does not slow the guest down and
  rides on the shared probe boot: ~15 MB raw (150k writes, mostly serial
  and VGA), filtered to the ~160 IPU BAR writes in
  `tools/tests/out/ipu6-probe.mmio-writes` (uploaded by `vm-ipu6`).
  `tools/trace/check_writes.py` (unit tests in
  `tests/test_check_writes.py`) compares the in-order writes of two
  captures in given BAR-offset ranges; it reads mmiotrace and the QEMU
  log alike (BAR from the `MAP` line, or the address modulo 16 MiB), so
  it also works on the fork's mmiotrace captures. The test compares MMU
  base + `0x08..0x8c` of the three PSYS MMUs: 67 writes. A guest
  mmiotrace (the fork's `init.mmiotrace` path) was the alternative; it
  needs a kernel option, a slower traced boot and a flavor-aware init,
  and gives no more information about writes.
- (PR 8) Only the PSYS MMU is initialised at probe; the ISYS MMU init
  runs in `isys_runtime_pm_resume()` (first video open). `probe-mmu`
  therefore checks the PSYS tables; the ISYS ones (ranges
  `0x1e0008-0x1e0090 0x1e0108-0x1e0190`) should be added to a test once
  a ladder boot opens a stream (`fw-open`). Silicon initialises the ISYS
  MMU twice (two resumes), so compare against a guest that resumes as
  often or cut the expected side to the first pass. The ISYS MMU0 write
  to BAR + `0x1e0004` is also what the model latches as the L1
  page-table base for its IOVA walker, so `fw-open` needed this patch
  anyway. Forcing an ISYS resume at probe through `power/control` was
  rejected: the resume runs the TSC sync, which times out on the IPU4 HH
  status bits (PR 7 learnings) and leaves the device in a runtime error
  state.
- (PR 8) Gate check: without 0003, `probe-mmu` FAILs with
  `expected 67 writes, got 0` (the tables went to `0x1b0000`); with two
  PSYS MMU1 block sizes swapped in 0003 it FAILs on the one start
  address between them (`+0x4b0128 = 0x1a`, expected `0x1c`); with 0003
  all 67 writes match. Probe still completes under the model (32 video
  nodes, clean unbind/rmmod, no oops). Timings (4 cores, TCG, warm):
  full ladder ~60 s (`vm-boot` 33 s incl. the kernel rebuild,
  `probe-identify` 5 s incl. the traced boot, `probe-buttress` and
  `probe-mmu` 0 s on the shared boot); the trace event adds no
  measurable boot time.
- (PR 8) For PR 9: none of the remaining internal pdata is used on the
  PCI probe path (only the PSYS MMU and authentication are), so
  `probe-fw` as defined passes before PR 9. What PR 9 changes visibly is
  the ISYS probe (6 CSI-2 ports instead of IPU6's 4: CSI-2 subdevs and
  the video node count), so PR 9 likely needs `probe-fw` redefined on
  that, or PR 9 and PR 10 swapped in the mapping.
- (PR 8a) The PCI ID patch `0004-media-ipu6-Enable-IPU4-PCI-device.patch`
  is one line in `include/media/ipu6-pci-table.h`, modelled on Ruslan's
  `media: ipu6: Enable IPU4P PCI device` (the pin's last commit). It is
  always the last patch: a new patch is committed in `tools/linux-ipu6`
  and moved before it (`git rebase -i` there), then the series is
  regenerated with `format-patch`, which renumbers the PCI ID patch. The
  rule is in `patches/ipu6-ipu4/README.md`.
- (PR 8a) With the table entry, `intel-ipu6` probes inside its `insmod`.
  So the `ipu6-mmu.c` dynamic-debug settings that keep the trash-buffer
  mapping out of the log can no longer be written to the control file
  after `insmod`; `init.ipu6-probe` passes them as the module's
  `dyndbg="+p;file ipu6-mmu.c -p;format iova\040trash\040buffer +p"`
  parameter (busybox `insmod` joins its arguments with spaces; `\040` is
  dynamic debug's escape for a space in a format). `init.ipu6-modules`
  reports the binding instead of requiring the device to stay unbound.
- (PR 8a) Gate check: without 0004, `probe-identify` FAILs (`no request
  for intel/ipu/ipu4_fw.bin`, `PROBE:bound no`) and the later probe tests
  SKIP. `native-bind` is gone from the ladder: every probe test now binds
  natively.
- (PR 9) **PR 9 and PR 10 swapped** (user decision): the ISYS register
  and interrupt step came before the internal pdata step because the
  pdata has no effect the model can see at probe (PR 8 learnings), while
  the ISYS setup can be checked against silicon once a ladder boot
  resumes the ISYS. Earlier entries that say "PR 9" for the pdata mean
  what is now PR 10. The CSI-2 port count (`csi2.nports = 6`) went into
  PR 9 with the port table it sizes, as Ruslan's IPU4P ISR patch did;
  PR 10 keeps the stream/SRAM/queue limits and needs its own observable
  test (the 48 video nodes are already PR 9's).
- (PR 9) The ISYS interrupt setup runs only in `isys_runtime_pm_resume()`
  (after the ISYS MMU init and the TSC sync), so the probe init now (snippet `085-isys-irq.bound.sh`)
  forces one ISYS runtime resume after the buttress checks: `echo on` /
  `echo auto` to the ISYS aux device's `power/control`, then
  `PROBE:isys-resume ok|fail (TSC sync|runtime_status <x>|not suspended
  again)`. Opening a video node does not resume the ISYS (only
  `ipu6_isys_fw_open()` at stream start does), so `power/control` is the
  smallest trigger. The resume goes through the bus PM domain (buttress
  power-up) and the ISYS driver's callback exactly as at stream start.
- (PR 9) The PR 8 note that forcing an ISYS resume fails is fixed by its
  own kernel patch, `0004-media-ipu6-Support-the-IPU4-TSC-sync-handshake`:
  `ipu6_buttress_send_tsc_request()` polls PWR_STATE bits 13:12 on IPU4
  (`IPU4_BUTTRESS_PWR_STATE_HH_STATUS_MASK`, fork value; silicon reads
  `0xa71433` = in progress, then `0xa72433` = done, `data/trace.txt`
  lines 591–592). `FIELD_GET()` needs a constant mask, so the patch uses
  `field_get()` (`<linux/bitfield.h>`, runtime masks) instead. The
  model ties HH_DONE (`2 << 12`) high, so it cannot tell whether the
  driver waits; the test only sees that the resume succeeds and that
  there is one `0x88` request instead of ten (retries).
- (PR 9) Kernel patches `0005-media-ipu6-Add-IPU4-ISYS-and-CSI-2-register-definiti`
  and `0006-media-ipu6-Add-IPU4-ISYS-interrupt-handling` (PCI ID patch
  now 0007). Values (fork `kernel/ipu4/ipu4-platform-regs.h`,
  `ipu4-platform-isys-csi2-reg.h`; silicon lines 593–599, 664–668,
  672–675, 765–766): UNISPART SW BIT(30), port bits `0x8 << port` /
  `0x800 << (port - 4)` (6 ports + SW = `0x40001878`, written to
  EDGE, LEVEL_NOT_PULSE, CLEAR, MASK, ENABLE in that order, then 0 to
  SW_IRQ_REG and SW_IRQ_MUX_REG), port bases `0x64000 0x65000 0x66000
  0x67000 0x6c000 0x6c800`, per-port CSI2PART/CSIRX/CSI2S2M interrupt
  controllers at port + `0x400/0x500/0x600` (CSI2PART bit 16 = CSIRX
  summary, FS/FE per VC at bits `4*vc` / `4*vc+1`; silicon's ISR reads
  CSI2PART status `0x1`/`0x2` and CSIRX status `0x4000`). The IPU4 defines
  live in `ipu4p-isys-csi2-regs.h` (UNISPART register offsets, CSI-RX
  receiver layout and the 17 `IPU4P_CSIRX_*` error bits are IPU4P's): a
  new header file makes checkpatch warn "does MAINTAINERS need
  updating?", which fails the `checkpatch` test. The ISR mirrors
  `ipu4p_isys_isr()` without the SIP controllers; the setup writes no
  CDC thresholds and no LTR/DID (silicon never writes `0x68`), so
  `set_iwake_ltrdid()` is skipped at suspend as for IPU4P. IPU4 now uses
  `ipu4p_fw_isys_ops` (its own `ipu4_isys_auxdrv_data`), since the ISR
  hands responses to `fw_ops->isr_one`; PR 11's SysCom work is on that
  path. Port → firmware source stays identity (the non-IPU4P default).
  The error table reuses `ipu4p_csi2_errors` plus a 17th entry ("Inter-frame
  long packet discarded"); IPU4P still prints only its 16. Not in this
  step: CSI-2 receiver enable/disable and PHY (`stream_enable` still takes
  the IPU6 path; PR 12) and the CSI-2 frame interrupts (decision 3,
  PR 13).
- (PR 9) `isys-irq` compares the probe boot's writes with silicon's
  **second** ISYS resume only (`check_writes.py --expected-lines
  487-599`, new option): silicon initialises the ISYS MMU twice (lines
  269–369 without an interrupt setup, then 489–589), and its ISR rewrites
  UNISPART CLEAR and SW_IRQ_REG on every firmware event from line 617
  on. Ranges: ISYS MMU0/MMU1 `+0x08..+0x8c` (the ISYS half of the PR 8
  to-do), `0x68`, `0x88`, `0x17c000-0x17c420`; 42 writes. The guest
  makes each of them once per resume and the probe boot resumes once,
  so the guest side needs no window. A later test that streams in the
  same boot would add ISR writes to the UNISPART range.
- (PR 9) Gate check: without 0005/0006 `isys-irq` FAILs (`expected 42
  writes, got 37`: no UNISPART writes, the IPU6 setup went to BAR +
  `0x37c000` and `set_iwake_ltrdid()` wrote `0x68` twice); without 0004
  (but with 0005/0006) on `PROBE:isys-resume fail (TSC sync)` and nine
  extra `0x88` writes; with `csi2.nports` left at IPU6's 4 on
  `0x40000078` instead of `0x40001878`. With the series it passes and the
  ISYS registers six CSI-2 ports (48 video nodes, was 32). Probe, unbind
  and rmmod stay clean. Timings (4 cores, TCG, warm): full ladder ~65 s
  (`vm-boot` 34 s incl. the kernel rebuild), `isys-irq` 0 s on the shared
  boot; the forced resume adds well under a second to it.
- (PR 11) **PR 10 folded into PR 11** (user decision): the remaining
  internal pdata has no effect before the ISYS firmware is opened, so it
  went into this step as its own patch, `0007-media-ipu6-Add-the-IPU4-ISYS-stream-limits`,
  before `0008-media-ipu6-Support-the-IPU4-SysCom-queue-layout` (PCI ID
  patch now 0009). IPU4 had the IPU6 limits (12 parallel streams = 12
  message send queues, 16-entry device send queue); it now takes IPU4P's
  (`IPU4P_ISYS_MAX_STREAMS`, `IPU4P_NOF_SRAM_BLOCKS_MAX`,
  `IPU4P_DEV_SEND_QUEUE_SIZE`, all 8), which are the fork's
  (`IPU4_STREAM_ID_MAX`, `IPU4_ISYS_MAX_STREAMS`). Silicon confirms the
  8 message queues: the message receive queue's pointers are DMEM
  registers 28/29 (`0x108070/0x108074`, `data/trace.txt` lines
  619–622) = 6 + 2 × (1 proxy + 1 device + 8 message send queues + 1
  proxy receive queue). Left at the values IPU4P inherits from IPU6, on
  purpose: the SRAM granularity/size (iwake watermark only; the fork
  leaves them 0), `max_send_queues` and the sensor types (IPU6 firmware
  ABI only), `csi2.irq_mask` (IPU4 uses the IPU4P error table).
- (PR 11) The SysCom patch: `ipu6_fw_com_prepare()` starts the queue
  pointers at DMEM register 6 (`IPU4_SYSCOM_QPR_BASE_REG`, next to
  Ruslan's `ipu4p_regmem_id`) for a DMEM SysCom on IPU4 (`IS_IPU4(adev->isp)`),
  8 on IPU4P. Registers 0–5 are the same on both (fork
  `kernel/ipu4/ipu6-fw-com.c`; silicon lines 602–607: pkg dir `0x40`
  at reg 0, state/command UNINIT at regs 2/3, config IOVA at reg 1,
  then SPC start). Silicon line 613 writes the stream 0 send queue's
  write pointer at reg 10 (`0x108028`) = 6 + 2 × 2.
- (PR 11) **Stream boot** (shared-core PR before this step): the probe
  boot cannot stream, because `probe-mmu` and `isys-irq` compare all of
  its ISYS writes with silicon and a stream adds a second ISYS resume
  and ISR writes. `tools/rootfs/init.ipu6-stream` loads the modules as
  the probe init does, finds "Intel IPU6 CSI2 0" and "Intel IPU6 ISYS
  Capture 0" in `/sys/class/video4linux/*/name` and runs
  `streamon <video> /dev/media0 <subdev>`; the new optional subdev
  argument sets the CSI-2 sink pad to 800x800 `RGB888_1X24`
  (`VIDIOC_SUBDEV_S_FMT`, propagated to the source pads by the routing),
  as the PR 5 learnings asked. `boot_stream_vm` (`tools/tests/ipu6/stream-boot.sh`)
  runs it once per ladder run: serial log `tools/tests/out/ipu6-stream.serial`,
  IPU BAR writes `tools/tests/out/ipu6-stream.mmio-writes`. Markers
  `STREAMVM:ok|fail|oops|DONE` plus streamon's `SUBDEV:`/`MEDIA:`/`STREAM:`.
  `stream_log_ok` does not require `STREAMVM:oops none`; each stream test
  decides (see below). The boot adds ~7 s to the ladder.
- (PR 11) `fw-open` (dependency `isys-irq`; `probe-complete` stays a
  NOT-IMPLEMENTED placeholder) passes on `FW resp 00 STREAM_OPEN_DONE,
  stream 0` + `start stream: open complete` (isys dyndbg) and on the
  DMEM write check: ranges `0x108000-0x108004 0x108008-0x108018
  0x108020-0x108100` against silicon lines 600–622 with the new
  `check_writes.py --prefix` (the guest's first 5 writes there must be
  silicon's; it streams differently afterwards). Left out: SPC
  STATUS_CTRL (`0x100000`; the driver ORs bits into what it reads and the
  model reads `0x20` where silicon reads `0xa0`, so the guest writes
  `0x1020` instead of `0x10a0`), the config IOVA (`0x108004`) and the
  proxy send queue (`0x108018/0x10801c`, see below). Gate check: without
  0008 the model sees the open on the register pair it maps to stream 1,
  answers for stream 1 ("stream of stream_handle 1 is unused") and the
  driver times out ("stream open time out"); without 0007 the open works
  but the receive queue's read pointer is written at `0x108094` (12
  message queues) instead of `0x108074`, so the write check FAILs. The
  model alone does not see the queue count (it reads the receive queue's
  registers from the queue descriptor).
- (PR 11) Seen on the stream boot, for later steps:
  - Before the stream open, `update_watermark_setting()` sends two iwake
    proxy tokens (`ipu4p_fw_isys_set_iwake_register`, GDA threshold and
    enable) on the proxy queue; the model never answers the proxy queue
    (PR 1 learnings), so each times out after ~180 ms ("write 2 failed
    -110", "write 1 failed -110"), and it writes LTR/DID (`ltr: value
    112 ...`) and REG_PKGC_PMON_CFG. Silicon's trace has no proxy queue
    writes at all, and the fork does no iwake on IPU4. Non-fatal; decide
    with `stream-on` (skip iwake on IPU4, as the fork, or answer the
    proxy in the model).
  - After the open, STREAM_START_AND_CAPTURE, a frame (SOF,
    PIN_DATA_READY) and the flush/close round trips all work, but the
    CSI-2 receiver enable takes the IPU6 path ("PHY0 powerup ack
    timeout", `stream on Intel IPU6 CSI2 0 failed with -110`), so
    VIDIOC_STREAMON fails with ETIMEDOUT. The failure path then trips
    vb2's `WARN_ON` in `vb2_start_streaming()` (videobuf2-core.c:1821:
    a buffer handed to the firmware is not returned), so the stream boot
    reports `STREAMVM:oops found`. That is PR 12's (`stream-on`), and
    the warning on the error path is worth a look then (possibly
    upstream).
  - The probe boot still never opens the firmware, so its
    `isys-irq`/`probe-mmu` write checks are unchanged.
