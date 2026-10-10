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
  forward into the stacked branches. At most 3 unmerged PRs may be
  stacked (see `CLAUDE.md`); when the stack is full, the next step waits
  for the bottom PR to merge.

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
| `probe-fw` | firmware (`ipu4_fw.bin`, synthesized) validates and loads |
| `probe-complete` | ISYS probe finishes, `/dev/video*` and media graph appear |
| `fw-open` | syscom open + `STREAM_OPEN_DONE` round trip |
| `stream-on` | `streamon-smoke` reaches `STREAM:streamon` |
| `stream-pattern` | `streamon-smoke` reaches `STREAM:pattern_ok` |
| `csi2-sof-irq` | CSI-2 FS IRQ observed on port 0 |
| `native-bind` | driver binds `5a88` from the PCI table (no `new_id`) |

## PR sequence

| PR | Content | Rung(s) enabled | State |
|---|---|---|---|
| 1 | This plan | — | in review (#100) |
| 2 | `IPU4_FLAVOR=ipu6` bootstrap/build + empty `patches/ipu6-ipu4/` + `vm-ipu6.yml` ladder runner, all rungs disabled | — | in review |
| 3 | Enable static rungs on the empty series | `series-applies`, `build`, `checkpatch` | |
| 4 | Guest kernel config + rootfs for the ipu6 flavor | `vm-boot` | |
| 5 | Standalone virt-sensor module, init loads `intel-ipu6*` | `module-load` | |
| 6 | `media: ipu6: Add helper to identify IPU4` (+ probe case, no table entry) | `probe-identify` | |
| 7 | `media: ipu6: Add IPU4 buttress support` | `probe-buttress` | |
| 8 | `media: ipu6: Add IPU4 MMU support` | `probe-mmu` | |
| 9 | Firmware name + internal pdata (`media: ipu6: Add IPU4 platform data`) | `probe-fw` | |
| 10 | `media: ipu6: Add IPU4 ISYS and CSI-2 register definitions` + `media: ipu6: Add IPU4 ISYS interrupt handling` + wire-up in ISYS | `probe-complete` | |
| 11 | `media: ipu6: Support the IPU4 SysCom queue layout` | `fw-open` | |
| 12 | `media: ipu6: Add IPU4 CSI-2 receiver support` | `stream-on`, `stream-pattern` | |
| 13 | CSI-2 frame interrupts (decision 3) | `csi2-sof-irq` | |
| 14 | `media: ipu6: Enable IPU4 PCI device` | `native-bind` | |
| 15 | Cover letter draft + `get_maintainer.pl` output | — | |
| Pages | Dashboard section "IPU4 port (upstream ipu6)": `ladder.sh` writes `results.json`, `vm-ipu6` uploads it per variant, Pages shows a rung × variant matrix | — | in review (stacked on PR 2) |

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
- (Pages PR) The ladder's machine-readable output is
  `tools/tests/out/ipu6-ladder/results.json` (schema in
  `tools/dashboard/README.md`), written by `write_results_json` at the
  very end of `ladder.sh`. New rungs need no change for it: anything in
  `RUNGS` is recorded, unimplemented ones as `NOT-IMPLEMENTED`. Keep
  that function and its call as the last thing before `exit`.
- (Pages PR) `workflow_run` only fires for workflows that exist on the
  default branch, and `gh run list --workflow=vm-ipu6.yml --branch=main`
  returns nothing (no error) until `vm-ipu6` has run there, so the
  dashboard shows "not run yet" until PR 2 is merged and has run once
  on `main`.

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
- **Dashboard hook (Pages PR):** the Pages "IPU4 port" matrix already
  has `ipu4`, `ipu4p` and `ipu6` columns. A variant lights up once its
  matrix leg runs the ladder with `IPU4_VARIANT=<variant>` (recorded in
  `results.json`) and uploads it as `ipu6-ladder-results-<variant>` —
  the upload step already names the artifact `${{ matrix.variant ||
  'ipu4' }}`, so adding `strategy.matrix.variant` and passing
  `IPU4_VARIANT` / `IPU4_IPU6_ENABLED_TESTS` (per-variant list) to the
  ladder step is all that is needed. Pages picks up every
  `ipu6-ladder-results-*` artifact of the run.
