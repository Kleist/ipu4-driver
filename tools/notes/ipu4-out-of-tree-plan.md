# Plan — the out-of-tree module on the test ladder

Owned by the **IPU4 track** (the out-of-tree baseline). The IPU4 port's plan is
[`ipu6-ipu4-plan.md`](ipu6-ipu4-plan.md); "Tracks and file ownership" there
says which files each track owns and how shared files are changed. Every
step of this track starts by re-reading this file and ends by updating it
(State column, Learnings) in the same PR. It edits no other track's plan.

## Goal

Run `kernel/ipu4/`, the source of truth for the fork driver, through the
same ladder as the other drivers, built the way a user builds it: as an
out-of-tree module with its own Makefile against a stock kernel. It is the
unmodified IPU4 baseline: IPU4 tests and model changes are developed
against it first, then gated on the upstreamable `ipu4-patches` driver
(the IPU4 port). Until now only the fork harness tested it (seeded
in-tree into `tools/linux/`, `vm-smoke`).

## Base and harness

- **Driver:** `ipu4-out-of-tree` (`IPU4_DRIVER`, `IPU4_FLAVOR=out-of-tree`),
  run under the `ipu4` QEMU model variant (leg `ipu4-out-of-tree / ipu4`).
- **Kernel:** stock v7.3-rc4 (`tools/out-of-tree/pin`, GitHub mirror of
  torvalds/linux), the base of the other ladder pins, fetched by
  `tools/ipu6/bootstrap.sh` with no series into `tools/linux-ipu4-oot/`.
- **Build:** `tools/out-of-tree/build.sh`: kconfig pass shared with the
  guest kernel, `modules_prepare`, then the `M=kernel/ipu4` build its
  Makefile's `all` target runs, with the built-in virt sensor
  (`CONFIG_VIDEO_IPU4_VIRT_SENSOR`, as the fork harness builds it).
  `intel-ipu4.ko`, `intel-ipu4-isys.ko` and `ambu-ipu-bridge.ko` land in
  `kernel/ipu4/`.
- **Gating:** `tools/tests/ipu6/enabled-tests.ipu4-out-of-tree.ipu4`, owned
  by this track. The driver is known to work, so every implemented test is
  gated; a failure points at the test, the harness or the model.
- **CI:** the `vm-ipu6.yml` leg `ipu4-out-of-tree / ipu4`, tree cache
  `linux-ipu4-oot`; `kernel/ipu4/*` changes select it (`tools/ci/changes.sh`).

## What differs from the ipu6 legs

`tools/rootfs/ipu-driver.<flavor>.conf` names the driver for the guest
inits and the host checks:

| Area | ipu6 legs | out-of-tree |
|---|---|---|
| Modules | `intel-ipu6`, `intel-ipu6-isys` | `intel-ipu4`, `intel-ipu4-isys` |
| Auxiliary devices | `intel_ipu6.isys.*`, `intel_ipu6.psys.*` | `intel_ipu4.*` |
| Entities | `Intel IPU6 ...` | `Intel IPU4 ...` |
| Sensor | `tools/virt-sensor/` module | built into `intel-ipu4` |
| Firmware | `intel/ipu/ipu4_fw.bin` | `ipu4_cpd_b0.bin` |

Two tests check the out-of-tree driver differently (a `$FLAVOR` case):
`isys-irq` compares with silicon from line 269, since the driver, like
silicon, also initialises the ISYS MMU at probe; `csi2-sof-irq` looks for
the handler's CSI2PART CLEAR of the frame start bit, since it logs
nothing.

## PR sequence

| PR | Content | `enabled-tests.ipu4-out-of-tree.ipu4` gains | State |
|---|---|---|---|
| O1 | Six ladder legs (baselines + `ipu4-patches`), `IPU4_FLAVOR=out-of-tree` | `series-applies`, `build`, `vm-boot` | merged (#145) |
| O2 | Driver names per flavor for the guest inits; built-in virt sensor; tests adapted to the driver | every implemented test | in review |
| O3 | Fix the W=1 warnings in `kernel/ipu4/`; `build` at W=1 like the ipu6 legs | — | |

## Learnings

- (O1) `kernel/ipu4/` does not build at W=1: its Makefile's `-Werror` turns
  `-Wunused-const-variable` (`dphy_rx_errors`, `ipu4_csi_offsets`) into
  errors. The plain build is clean, so `build` runs at the module's own
  level until O2. (On v6.12, the first base, `-Wformat-truncation` in
  `ambu-ipu-bridge.c` failed too; v7.3-rc4 no longer warns there.)
- (O1) The Makefile's `all` target passes `-j $(nproc)` (an undefined make
  variable, so a bare `-j`), and a parallel outer make then prints "warning:
  -j0 forced in submake", which the `build` test's warning scan would catch;
  the ladder calls the same `M=` build directly.
- (O1) `make checkpatch` with v7.3-rc4's checkpatch: 0 errors, 11 warnings,
  30 checks (`--strict`), mostly in IPU6 code (v6.12's: 4 warnings).
- (O1) `vm-boot` passes: the trimmed v7.3-rc4 guest kernel boots under the
  IPU4 model and sees `8086:5a88`. `module-load` boots too (the harness's
  `tools/virt-sensor/` builds on v7.3-rc4, not on v6.12) and fails on the
  guest init loading the `intel-ipu6` modules (O4).
- (O2) With the harness's `tools/virt-sensor/` module, probe uses its
  fwnode graph and never calls `ambu_ipu_bridge_init()`, but remove still
  calls `ambu_ipu_bridge_uninit()`, which dereferences the bridge that was
  never set up (NULL pointer oops at `rmmod intel_ipu4`). On hardware the
  bridge always initialises, so the ladder uses the built-in virt sensor,
  whose install and remove pair up.
- (O2) The stock config has nothing selecting vb2, so the in-tree
  `VIDEO_INTEL_IPU6` is built as a module for its dependencies; the guest
  never loads it.
- (O2) `checkpatch` checks a series; like the other baselines this driver
  has none, so it passes with nothing to check. `make checkpatch` on the
  whole module (11 warnings, 30 checks) is not a ladder test.
- (O2) The driver's probe and stream writes equal silicon's in every
  compared range: PSYS MMU (`probe-mmu`), the ISYS MMU at probe and resume
  plus UNISPART (`isys-irq`, 76 writes), SysCom, CSI-2 receiver and
  CSI2PART (`fw-open`, `stream-on`, `csi2-sof-irq`).
