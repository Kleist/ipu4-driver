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
  column, any change to the PR → rung mapping, and new facts in
  "Learnings" below. A PR that learned nothing says so in its
  description.
- **Steps are pipelined.** As soon as a step's PR is up, the next step
  starts on a branch stacked on it while the previous PR is in review.
  The stacked PR targets the previous step's branch and is retargeted
  to `main` once that merges. Review feedback on an earlier PR is merged
  forward into the stacked branches.

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
**test ladder**. Which rungs are gated is controlled by a checked-in list,
`tools/tests/ipu6/enabled-tests`, one test name per line. The workflow runs
every rung but only fails on the enabled ones, so later rungs are visible as
"expected fail" before they are switched on. **It starts with every rung
disabled; each PR enables (at least) one more.**

Because upstream wants the PCI ID table entry last, the probe-level rungs
bind the driver with `echo 8086 5a88 > /sys/bus/pci/drivers/intel-ipu6/new_id`
(harness-only). The first kernel patch adds the `PCI_DEVICE_ID_INTEL_IPU4`
case in probe (without a table entry), so the driver identifies IPU4 when
bound this way. The final kernel patch adds the table entry and the ladder
switches to native binding.

The runner is `tools/tests/ipu6/ladder.sh` (`--list` prints the names).
Each rung ends PASS, FAIL, NOT-IMPLEMENTED (no test yet) or SKIP (a rung
it depends on, run in the same invocation, did not pass); an enabled rung
must PASS, and an unknown name in `enabled-tests` fails the job.

Planned rungs (names are what goes into `enabled-tests`). Big tests are
split, or gated by the existing graded markers (`IPU4_PROBE_REQUIRED`,
`IPU4_STREAM_REQUIRED`), so every rung is small enough to turn on alone:

| Rung | Passes when |
|---|---|
| `series-applies` | `git am` of `patches/ipu6-ipu4/` succeeds on the pin |
| `build` | `intel-ipu6*.ko` build cleanly (W=1) |
| `checkpatch` | `checkpatch.pl --strict` clean on every patch |
| `vm-boot` | guest boots and enumerates `8086:5a88` |
| `module-load` | `intel-ipu6` + `intel-ipu6-isys` + virt-sensor load without oops |
| `probe-identify` | `new_id` bind reaches probe and identifies IPU4 |
| `probe-buttress` | buttress init + ISYS/PSYS power-up succeed |
| `probe-mmu` | MMU hardware init succeeds |
| `probe-fw` | firmware (`ipu4_fw.bin`, synthesized) is mapped and authenticated (`ipu6_buttress_authenticate()`) |
| `probe-complete` | ISYS probe finishes, `/dev/video*` and media graph appear |
| `fw-open` | syscom open + `STREAM_OPEN_DONE` round trip |
| `stream-on` | `streamon-smoke` reaches `STREAM:streamon` |
| `stream-pattern` | `streamon-smoke` reaches `STREAM:pattern_ok` |
| `csi2-sof-irq` | CSI-2 FS IRQ observed on port 0 |
| `native-bind` | driver binds `5a88` from the PCI table (no `new_id`) |

## PR sequence

| PR | Content | Rung(s) enabled | State |
|---|---|---|---|
| 1 | This plan | — | merged (#100) |
| 2 | `IPU4_FLAVOR=ipu6` bootstrap/build + empty `patches/ipu6-ipu4/` + `vm-ipu6.yml` ladder runner, all rungs disabled | — | in review (#101) |
| 3 | Enable static rungs on the empty series | `series-applies`, `build`, `checkpatch` | in review (#102) |
| 4 | Guest kernel config + rootfs for the ipu6 flavor | `vm-boot` | in review (#103) |
| 5 | Standalone virt-sensor module, init loads `intel-ipu6*` | `module-load` | in review (#104) |
| 6 | `media: ipu6: Identify IPU4 devices` (helper, PCI ID, firmware name, probe case; no table entry) | `probe-identify` | in review |
| 7 | `media: ipu6: Add IPU4 buttress support` + harness stages a synthesized `intel/ipu/ipu4_fw.bin` (see PR 6 learnings) | `probe-buttress` | |
| 8 | `media: ipu6: Add IPU4 MMU support` | `probe-mmu` | |
| 9 | Internal pdata (`media: ipu6: Add IPU4 platform data`); the firmware name landed in PR 6 | `probe-fw` | |
| 10 | `media: ipu6: Add IPU4 ISYS and CSI-2 register definitions` + `media: ipu6: Add IPU4 ISYS interrupt handling` + wire-up in ISYS | `probe-complete` | |
| 11 | `media: ipu6: Support the IPU4 SysCom queue layout` | `fw-open` | |
| 12 | `media: ipu6: Add IPU4 CSI-2 receiver support` | `stream-on`, `stream-pattern` | |
| 13 | CSI-2 frame interrupts (decision 3) | `csi2-sof-irq` | |
| 14 | `media: ipu6: Enable IPU4 PCI device` | `native-bind` | |
| 15 | Cover letter draft + `get_maintainer.pl` output | — | |

The patch-to-rung mapping follows probe order and may shift by one when
the real probe path is run; if a patch can't turn a rung green on its own,
the PR is split rather than enabling nothing.

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
- (PR 2) The `build` rung is `defconfig` + `modules_prepare` +
  `make M=drivers/media/pci/intel/ipu6 W=1`: ~22 s cold on 4 cores, so no
  full kernel build is needed until the VM rungs (PR 4). The pinned
  ipu6 directory is W=1-clean with gcc 13.3 (zero warnings), so the rung
  treats any compiler warning as a failure. `modules_prepare` needs
  `libelf-dev` (objtool); `setup-harness` already installs it.
- (PR 2) Ruslan's `drivers/media/pci/intel/Makefile` adds
  `subdir-ccflags-y` warning suppressions (unused-parameter,
  uninitialized, int-conversion, enum-conversion, ...) for his downstream
  driver, and they also apply to `ipu6/` in a full-tree build. An `M=`
  build does not read that Makefile, so it shows the warnings upstream
  would see. Keep the `build` rung on `M=` (or check warnings in a way
  that bypasses those flags) when PR 4 adds a full kernel build.
- (PR 2) QEMU is still bootstrapped by `tools/bootstrap.sh` together with
  the fork's `tools/linux/` clone, and `tools/build-kernel.sh` /
  `tools/rootfs/build.sh` hard-code `tools/linux/`. PR 4 needs a
  QEMU-only bootstrap path and ipu6-flavor variants of those two
  scripts. Note `tools/build-kernel.sh` calls `tools/build.sh`, which
  now dispatches on `IPU4_FLAVOR`.
- (PR 3) With the empty series, `series-applies` and `build` are real
  gates (bootstrap resets to the pin; `build` recompiles every ipu6
  object at W=1), but `checkpatch` passes vacuously ("no patches") until
  PR 6 adds the first kernel patch. Verified with throw-away patches:
  a style/no-`Signed-off-by` patch fails `checkpatch`, a patch adding a
  function without a prototype fails `build`, and a non-applying patch
  fails `series-applies` (and `build` is SKIPped). The pin's `defconfig`
  has `CONFIG_WERROR=y`, so W=1 warnings in ipu6 are already hard errors
  on top of the rung's log scan.
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
  applies it too, so the `build` rung (M=) and the guest kernel share one
  `.config` and neither invalidates the other's objects. The trimmed
  config keeps the ipu6 directory W=1-clean.
- (PR 4) `CONFIG_WERROR` is **off** for the ipu6 guest kernel (the pin's
  defconfig has it on). The full v7.3-rc4 tree builds without warnings
  with gcc 13.3 today, but with `-Werror` a runner gcc bump that adds a
  warning in any unrelated subsystem would fail every VM rung. The
  `build` rung's log scan is the warning gate for ipu6; re-checked with
  a throw-away patch (missing prototype) → `build` FAIL without
  `-Werror`.
- (PR 4) The full build compiles ipu6 with the parent Makefile's
  `subdir-ccflags` (see PR 2), and the `build` rung's `M=` clean +
  rebuild then compiles it again with upstream flags. The two builds
  overwrite each other's ipu6 objects, which costs a few seconds per
  run; the modules packed into the initramfs come from the full build.
  The initramfs already contains `intel-ipu6*.ko` and the vb2 modules
  (VIDEOBUF2_* end up `=m` because only `VIDEO_INTEL_IPU6=m` selects
  them), so PR 5 only has to load them.
- (PR 4) Timings on 4 cores (TCG): cold `vm-boot` 334 s, nearly all of it
  the bzImage + modules build; the TCG boot reaches `/init` at ~1.6 s and
  the whole VM run (rootfs rebuild + boot + poweroff) is ~4 s. Warm
  ladder (all four enabled rungs): 33 s. QEMU from `--qemu-only`: clone
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
  800x800 RGB888 sensor. The stream rungs should set the CSI-2 sink (and
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
  probe case for the `probe-identify` rung, and the whole patch is 10
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
  `ipu6_buttress_init()`, which it does. The later probe rungs should
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

## Parallel track — QEMU coverage for IPU4P and IPU6

Goal: run the same ladder against IPU4P and IPU6 so the IPU4 patches are
shown not to break the other variants of the shared driver.

- **Mechanism:** a `variant=` property on the QEMU device
  (`-device ipu4,variant=ipu4|ipu4p|ipu6`) that switches PCI ID and the
  per-variant register behaviour; a matrix job in `vm-ipu6.yml`, with one
  `enabled-tests` list per variant.
- **IPU4P — feasible, moderate.** Same firmware ABI and base offsets, so the
  syscom/frame-delivery emulation is reused. Needs: PWR_STATE FSM values,
  UNISPART SW bit 22, SIP0/SIP1 CSI IRQ windows, queue-pointer base register 8,
  8 CSI-2 ports, PHY sideband registers as latches.
- **IPU6 — feasible for probe-level rungs, streaming is a larger project.**
  IPU6 keeps syscom state in buttress boot-parameter registers, uses a
  different firmware ABI, CSE authentication/secure mode and different MMU
  and CSI-2 layouts. Plan: `vm-boot` → `probe-complete` first; streaming
  only if it turns out to be worth it.
- **Caveat:** there is no silicon trace for either variant here, so their
  models are inferred from the driver, which risks testing the driver
  against itself. An mmiotrace from a Surface Pro 7 (Ruslan /
  linux-surface) would give IPU4P the same footing IPU4 has with
  `data/trace.txt`.
- Runs as separate PRs interleaved with the main sequence; it never
  blocks it.
