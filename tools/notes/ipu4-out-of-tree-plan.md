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

## Convergence onto mainline + IPU4P + IPU4 patches

Goal: `kernel/ipu4/<f>` equals `drivers/media/pci/intel/ipu6/<f>` of the
**target tree** — `tools/linux-ipu6/` as `tools/ipu6/bootstrap.sh` builds
it: the mainline pin (all of the 7.4 merge window's IPU6 commits) +
`patches/ipu6-ipu4p/` (Ruslan's RFC v2) + `patches/ipu6-ipu4/` — for every
file, except a small isolated permanent delta: the Ambu features (silent
reset, `ambu-ipu-bridge`, extra logging), compat shims for kernels
6.12–7.3 confined to `ipu4-compat.h`, and the out-of-tree build glue
(`Makefile`, `intel-ipu4*` module names).

Decided: switch to upstream's per-CSI2 video topology; kernel floor 6.12;
carry and build every target file (IPU6 PHYs, `ipu7-*.c`, IPU8,
`ipu4p-*.c`); lazy STREAMON and the 500 ms firmware timeout converge to
upstream.

Steps run back to back: while a step's PR is in review, the next one is
prepared on a local branch; as soon as the PR merges, that branch is
rebased onto the new `main`, re-checked, pushed and its PR opened without
waiting to be asked (still one open PR at a time, as `CLAUDE.md` says).

Upstream commits and series patches come in with `git am --3way
--directory=kernel/ipu4`, keeping authorship and `(cherry picked from …)`,
with an `[IPU4: …]` / `[out-of-tree: …]` note only where an edit was
needed. Each commit builds warning-free on 6.12, 6.18 and 7.3-rc4 and
keeps KUnit, `vm-smoke` and this leg green; the MMIO-vs-silicon tests are
the behaviour guard.

| PR | Content | State |
|---|---|---|
| O4 | Divergence history on the dashboard: `tools/upstream/divergence_history.py`, one point per release since v6.6 and up to 10 driver commits in between | merged (#210) |
| O4b | Divergence against the target tree (not just mainline) + allow-list of the permanent delta (`tools/out-of-tree/delta.txt`); "unexplained divergence" as the progress number, with a per-file worklist | merged (#212) |
| O5 | Bugs and stale code: double `kfree(pdata)`, lost `out_ipu6_rpm_put`, `ERR_PTR` psys in the probe cleanup, `TBL_PHYS_ADDR(…;)`, `ISP_L2PT_PTES`, duplicate defines, copyright years, older-upstream leftovers | merged (#213) |
| O6 | Compat into `ipu4-compat.h` with floor 6.12: string namespaces, `kzalloc_obj`; drop 6.6/6.10-gated code | merged (#214) |
| O7 | Harness out of the driver: `tools/virt-sensor/` + `streamon.c` s_fmt instead of the built-in sensor and route install | merged (#216) |
| O7b | Drop the empty `pkg_dir` entry skip in `ipu6_pkg_dir_configure_spc()` | in review |
| O8 | `ambu-ipu-bridge` as a pre-probe module that installs the fwnode graph (no hook in `ipu6.c`) | |
| O9 | Upstream video topology (per-CSI2 video nodes, upstream setup/prepare signatures) | |
| O10 | Streaming behaviour: drop lazy STREAMON, firmware timeout back to 2 s | |
| O11 | Build every file: `ipu7-*.c`, IPU6 PHYs; drop the IPU7 `#ifdef IPU6` guards | |
| O12–O14 | Upstream commits 46–88: streams/watermark rework, RPM rework (silent reset redesigned on runtime PM, in its own file), fixes + IPU8 | |
| O15 | Ruslan's IPU4P v2 (`patches/ipu6-ipu4p/`) on top, IPU4 code still in place | |
| O16 | IPU4 identity + buttress → series 0002, 0003, 0005 | |
| O17 | MMU → series 0004 (check dropped ZLW writes against silicon) | |
| O18 | Firmware layer → series 0008 + `ipu4p-fw-isys.[ch]` (drops the IPU4 ABI copy) | |
| O19 | ISR, CSI-2, iWake → series 0006, 0007, 0009, 0010 (`csi2-sof-irq` enabled for `ipu4-patches` first) | |
| O20 | PCI enable (series 0011), leftovers, KUnit exports; only the allow-listed delta remains | |
| O21 | Stay converged: `watch.sh` with real 3-way merges; IPU4P refreshes re-applied the same way | |

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
- (O4) The repository's root is GitHub's "Initial commit" (2025-12-03); the
  driver's 2023–2025 history was rebased onto it. Ceiling commit dates over
  all ancestors would lift the whole import to 2025-12-03, so the history
  chart ceils over driver-touching ancestors only (path-simplified history).
- (O4) Fetching the ~150 mainline tags since v6.6 at depth 1 without blobs,
  then the ipu6 blobs, takes about a minute and 44 MB; full tag history
  would be 2.1 GB.
- (O4b) Starting point against the target tree (mainline `cbbddb6b` + 12
  IPU4P + 2 fixes + 10 IPU4 patches): 11262 lines of unexplained
  divergence over 49 files — ~5000 in target files not carried yet
  (`ipu7-*.c`, `ipu4p-*`), ~4600 in the ISYS core (`ipu6-isys*.c`,
  `ipu6-fw-isys.*`, `ipu6-mmu-hw.c`), ~550 in files the target lacks
  (`virt-sensor.*`, `ipu4-platform-*.h`).
- (O5) Target distance 11262 → 11189. Two "stale" differences are not
  cosmetic and wait for later steps: `ipu6_isys_stream_start()` holds
  `stream_mutex` only around `set_streaming` (upstream: `guard()` for the
  whole function), tied to lazy STREAMON (O10); the `ipu6-fw-com.c` style
  differences go with its DMEM layout rewrite (O18).
- (O6) The IPU6/IPU7 PCI IDs stay in `ipu6.h`: `<media/ipu6-pci-table.h>`
  also defines `ipu6_pci_tbl`, which would bind IPU6 devices and clashes
  with the IPU4-only table; the IPU4 series adds IPU4 to that header,
  which an out-of-tree module can't. The namespace shim covers 6.13 too
  (the old one started at 6.14, so 6.13 did not build). Target distance
  11189 → 11057. The typed allocation helpers are backported to 6.12.y
  and 6.18.y (CI's pins), so the shim tests `#ifndef kzalloc_obj`, not
  the version; build against the pins (GitHub mirror `gregkh/linux`),
  not plain v6.12.
- (O7) With the graph supplied by `tools/virt-sensor/`, probe skips
  `ambu_ipu_bridge_init()`, so uninit must tolerate that (the O2 oops);
  the CSI-2 subdev's active state already has the default route, so
  `streamon`'s subdev s_fmt replaces the bind-time route install. The
  sensor builds from a copy per kernel tree (6.12 has no `MO=`). Target
  distance 11057 → 10577.
- (O7b) The skip was dead code: the stub firmware's package directory
  entries are valid, and the MMIO trace is unchanged without it. "IPU4
  PCI driver ready" stays, as Ambu logging: an external test uses it as
  its done signal. Target distance 10577 → 10560.
