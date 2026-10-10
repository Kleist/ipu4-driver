# Plan — QEMU coverage for the IPU6 variant

Owned by the **IPU6-variant track**. The IPU4 port's plan is
[`ipu6-ipu4-plan.md`](ipu6-ipu4-plan.md), the IPU4P variant's
[`ipu4p-variant-plan.md`](ipu4p-variant-plan.md); see "Tracks and file
ownership" in the IPU4 port's plan. Every step of this track re-reads this
file and updates it (State column, Learnings) in the same PR, and edits no
other track's plan.

(Moved here from the IPU4 port's plan, section "Parallel track", unchanged
apart from the IPU4P paragraph, which is now its own plan.)


Goal: run the same ladder against IPU4P and IPU6 so the IPU4 patches are
shown not to break the other variants of the shared driver.

- **Mechanism:** a `variant=` property on the QEMU device
  (`-device ipu4,variant=ipu4|ipu4p|ipu6`) that switches PCI ID and the
  per-variant register behaviour; a matrix job in `vm-ipu6.yml`, with one
  `enabled-tests` list per variant.
- **IPU6 — feasible for probe-level tests, streaming is a larger project.**
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
- **Base.** The ipu6 variant runs on unpatched mainline v7.3-rc4
  (`tools/ipu6/pin.upstream`), not the IPU4 pin + series: the IPU6
  emulation is developed against the upstream driver as released.
- **PRs.** One PR per ladder test for the variant, top of the table
  first, at most 3 stacked (`CLAUDE.md`). The stack starts from `main`
  and is independent of the main sequence's stack.

  | PR | Content | `enabled-tests.<variant>` gains | State |
  |---|---|---|---|
  | V1 | `vm-ipu6` matrix (`ipu4`, `ipu6`), `IPU4_VARIANT` plumbing in `ladder.sh` / `run-vm.sh`, ipu6 base = unpatched mainline v7.3-rc4 | ipu6: `series-applies`, `build`, `checkpatch` | merged (#118) |
  | V2 | QEMU `variant=ipu6` property (IPU6 PCI ID), variant-aware `vm-boot` | ipu6: `vm-boot` | merged (#119) |
  | V3 | variant-aware `module-load` | ipu6: `module-load` | in review (#120) |

- **Dashboard hook (Pages PR):** the Pages "IPU4 port" matrix already
  has `ipu4`, `ipu4p` and `ipu6` columns. A variant lights up once its
  matrix leg runs the ladder with `IPU4_VARIANT=<variant>` (recorded in
  `results.json`) and uploads it as `ipu6-ladder-results-<variant>` —
  the upload step already names the artifact `${{ matrix.variant ||
  'ipu4' }}`, so adding `strategy.matrix.variant` and passing
  `IPU4_VARIANT` / `IPU4_IPU6_ENABLED_TESTS` (per-variant list) to the
  ladder step is all that is needed. Pages picks up every
  `ipu6-ladder-results-*` artifact of the run.
- (V1) The ipu6 variant is developed against **unpatched mainline
  v7.3-rc4** (`tools/ipu6/pin.upstream`, GitHub mirror of torvalds/linux,
  the IPU4 pin's base): neither Ruslan's IPU4P/IPU7 patches nor
  `patches/ipu6-ipu4/`, so the model is checked against the upstream
  driver as released. `ladder.sh` maps `IPU4_VARIANT` to a base and
  exports it: ipu4 (and later ipu4p) → `tools/ipu6/pin` + the series in
  `tools/linux-ipu6/`; ipu6 → `pin.upstream`, no series, in
  `tools/linux-ipu6-upstream/` (a second ~300 MB shallow fetch plus its
  own kernel build; separate trees so switching variants locally does
  not rebuild everything). `tools/ipu6/bootstrap.sh` takes the base via
  `IPU4_IPU6_PIN` / `IPU4_IPU6_PATCH_DIR` (empty = no series); every
  other script already honoured `IPU4_IPU6_LINUX_DIR`.
- (V1) Mainline has no `CONFIG_VIDEO_INTEL_IPU` (Ruslan's downstream
  driver), so `tools/ipu6/build.sh` checks that it is not `=y`/`=m`
  rather than that it is explicitly unset. `VIDEO_INTEL_IPU6_IPU7` is
  also absent there; `olddefconfig` drops it.
- (V1) On the ipu6 base `series-applies` is the bare fetch and
  `checkpatch` passes vacuously ("no series"); both are gated anyway so
  a broken fetch fails the leg. `IPU4_VARIANT` selects `enabled-tests`
  (ipu4) or `enabled-tests.<variant>`; a missing file gates nothing.
  `run-vm.sh` passes `-device ipu4,variant=<v>` for non-ipu4 variants
  only, so the fork flow's QEMU command line is unchanged; until V2 adds
  the property, QEMU rejects it and the ipu6 leg's VM tests FAIL
  (ungated) instead of silently booting the IPU4 model.
- (V1) CI: matrix `include` carries each variant's tree directory and pin
  file for the tree cache (named after the directory, so the ipu4 key
  is unchanged); the QEMU cache is shared. The dashboard cards show each
  variant's base and series count, since the columns no longer share a
  base. Adding the matrix renames the check from `ladder` to
  `ladder (ipu4)` / `ladder (ipu6)`; update branch protection if it
  requires `ladder` by name.
- (V2) The model's `variant` property (`-device ipu4,variant=ipu6`) only
  changes the PCI device ID to `9a19` (Tiger Lake, `IPU_VERSION_6`) via
  `pci_config_set_device_id()` at realize; registers stay IPU4's. An
  unknown variant fails realize. `run-vm.sh` maps the variant to its ID
  and passes it to the guest as `ipu_pci_id=` (the kernel hands unknown
  parameters to `/init`); `init.vm-smoke` reads `/proc/cmdline` and
  defaults to `0x5a88`, so the fork flow is unaffected.
- (V3) On the ipu6 variant (mainline v7.3-rc4), `insmod intel-ipu6`
  binds `9a19` from the ID table and probe gets through
  `ipu6_pci_config_setup()` and `ipu6_buttress_init()` on the IPU4
  register model, then fails cleanly with `-ENOENT` on
  `intel/ipu/ipu6_fw.bin` (no oops); the device ends unbound. The IPU4
  pin's tree behaves the same. So the next ipu6 test that needs new
  model work is the firmware (a synthesized `ipu6_fw.bin` CPD), not the
  buttress.
- (V3) `init.ipu6-modules` reads `ipu_pci_id=` like `init.vm-smoke`,
  loads `virt-sensor device=<id>` and looks for that ID. Binding is only
  reported (since PR 8a, for every variant); the probe-* tests check it.
