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
  per-variant register behaviour. This track develops against its
  baseline driver `ipu6` (unpatched mainline), leg `ipu6 / ipu6` in
  `vm-ipu6.yml`, and owns `enabled-tests.ipu6.ipu6`, which the V steps
  below fill. The upstreamable driver
  under this variant is the IPU4 port's `enabled-tests.ipu4-patches.ipu6`.
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
  first. One open PR at a time for this track, never stacked
  (`CLAUDE.md`): the next step is prepared on a local branch while the
  current PR is in review, and pushed and opened only after it merges,
  rebased onto the new `main`. Independent of the other tracks' PRs.

  | PR | Content | `enabled-tests.<variant>` gains | State |
  |---|---|---|---|
  | V1 | `vm-ipu6` matrix (`ipu4`, `ipu6`), `IPU4_VARIANT` plumbing in `ladder.sh` / `run-vm.sh`, ipu6 base = unpatched mainline v7.3-rc4 | ipu6: `series-applies`, `build`, `checkpatch` | merged (#118) |
  | V2 | QEMU `variant=ipu6` property (IPU6 PCI ID), variant-aware `vm-boot` | ipu6: `vm-boot` | merged (#119) |
  | V3 | variant-aware `module-load` | ipu6: `module-load` | merged (#120) |
  | V4 | `probe-identify` for ipu6: virt-sensor gets the variant's ID (core PR #127), `060-probe-identify.bound.sh` expects the variant's firmware | ipu6: `probe-identify` | merged (#123) |
  | V5a | QEMU `ipu6v_pwr_state()`: IPU6 PWR_STATE layout for the ipu6 variant (no test yet; prerequisite for staging `ipu6_fw.bin`) | — | merged (#128) |
  | core | stage the synthesized CPD as `intel/ipu/ipu6_fw.bin` in the ipu6 initramfs (`tools/rootfs/build.sh`) | — | merged (#132) |
  | V5b | gate `probe-buttress` for ipu6 | ipu6: `probe-buttress` | merged (#133) |
  | V6 | QEMU: TSC-sync HH status at IPU6's bits 12:11 in `ipu6v_pwr_state()`; new variant-neutral `isys-resume` test | ipu6: `isys-resume` | merged (#136) |
  | V7 | `probe-mmu` / `isys-irq` for ipu6: expected writes computed from the upstream driver's IPU6 tables (`tools/tests/ipu6/upstream_writes.py`) instead of the IPU4 silicon trace | ipu6: `probe-mmu`, `isys-irq` | merged (#143) |
  | V8 | QEMU `ipu6v_mmio_*()`: IPU6 ISYS firmware interface (SPC 0x210000, DMEM SysCom 0x200000, config/state in buttress boot params, MMU0 L1 0x2e0004, UNISPART SW IRQ BIT(22)), iwake proxy answers, IPU6 48-byte responses; ipu6 `fw-open` vs upstream writes | ipu6: `fw-open` | merged (#146) |
  | V9 | QEMU: MCD PHY power handshake (`ipu6v_phy_ctl`); virt-sensor presents 2 lanes on 9a19; ipu6 `stream-on` vs upstream CSI-2/PHY writes | ipu6: `stream-on` | merged (#151) |
  | V10 | gate `stream-pattern` (variant-neutral; passes since V9) | ipu6: `stream-pattern` | merged (#153) |
  | V11 | QEMU: CSI-2 port 0 error IRQ (corrected header error per frame); ipu6 `csi2-sof-irq` redefined as error-IRQ coverage + sync IRQs stay masked | ipu6: `csi2-sof-irq` | in review |

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
- (V4) Since #126 the probe checks are per-test snippets: the
  identify check is `init.ipu6-probe.d/060-probe-identify.bound.sh`,
  which maps the frame's `$want` to the firmware file
  `ipu6_pci_probe()` requests for it (`ipu4_fw.bin` for `5a88`,
  `ipu6_fw.bin` for `9a19`). The frame change it needs (virt-sensor
  loaded with `device=$want`, `ipu_pci_id=` read before the module
  loop) went in first as its own core PR (#127). On mainline the ipu6
  variant passes `probe-identify` with no model change. Its
  `probe-buttress` fails: probe stops at the missing firmware before the
  PSYS runtime resume the buttress check looks for, so the next ipu6
  test needs a synthesized `ipu6_fw.bin` (an IPU6 CPD that
  `ipu6_cpd_validate_cpd_file()` accepts), not register work.
- (V5a) With `intel/ipu/ipu6_fw.bin` staged (the synthesized CPD that
  `tools/firmware/gen-cpd.py` already makes for IPU4 passes IPU6's
  `ipu6_cpd_validate_cpd_file()` too), mainline probe on the ipu6
  variant goes on to the buttress power-up and polls PWR_STATE for
  IPU6's 2-bit island status (IS [4:3], PS [7:6], UP_DONE = 3). The
  IPU4 model's FSM word (`0x0fa02003`) never matches, so probe fails
  with `Change power status timeout with 0xa02003`. `ipu6v_pwr_state()`
  answers in IPU6's layout (`0x20db` with both islands up), and probe
  then completes: `IPU0-v0[9a19] hardware version 1`, 64 video nodes,
  ISYS/PSYS powered up and suspended again (`PROBE:buttress ok`). The
  SKU register reads 0, hence "IPU0-v0".
- (V5a) **Upstream bug seen on the way:** without `ipu6v_pwr_state()`,
  the failed power-up sends `ipu6_pci_probe()` down its error path,
  which WARNs in `ipu6_dma_free()` (ipu6-dma.c:255) and then oopses in
  `ipu6_dma_unmap_sg()` called from `ipu6_buttress_unmap_fw_image()`.
  Mainline v7.3-rc4 as released; worth a report or fix upstream. In the
  harness `oops=panic` kills the guest, so staging `ipu6_fw.bin` must
  not land before `ipu6v_pwr_state()` or the gated ipu6 `module-load`
  breaks. Order: V5a (model) → core PR staging `ipu6_fw.bin` in
  `tools/rootfs/build.sh` → V5b gating `probe-buttress`.
- (V5a) Next after `probe-buttress` on ipu6: the first ISYS runtime
  resume logs `Start tsc sync timeout` ten times and `TSC sync failed`
  (IPU6's TSC sync handshake, BUTTRESS_REG_TSW_CTL / HH status, is not
  modelled for the variant). `probe-mmu` as written compares PSYS MMU
  writes with the IPU4 silicon trace, so it needs an ipu6 counterpart
  (or stays IPU4-only) rather than model work.
- (V5b) With V5a and #132, the ipu6 variant passes `probe-buttress`:
  probe powers ISYS/PSYS up and down through `ipu6v_pwr_state()`, and
  probe completes (`hardware version 1`, 64 video nodes). GitHub once
  refused to open #132 on #128's branch ("PullRequest.base invalid")
  although the branch existed; #132 went up as a draft on `main` until
  #128 merged. Stacking #133 on #132's branch worked, so that was not a
  standing rule.
- (V6) After `probe-buttress`, the first ISYS runtime resume on the ipu6
  variant timed out in `ipu6_buttress_start_tsc_sync()`: IPU6 reads the
  TSC-sync handshake status (`BUTTRESS_PWR_STATE_HH_STATUS_MASK`) at
  PWR_STATE bits 12:11, IPU4 at 13:12, and `ipu6v_pwr_state()` had
  copied IPU4's HH_DONE. With DONE (2) at bits 12:11 the resume passes
  (`PROBE:isys-resume ok`). `isys-irq` cannot gate this for ipu6 (it
  compares the IPU4 resume's writes with the silicon trace), so V6 adds
  `tests.d/082-isys-resume.sh`, which only checks the guest's
  `PROBE:isys-resume` marker; it also passes on ipu4 (ungated there).
- (V7) No IPU6 silicon trace exists, and the purpose of the variant is
  test coverage of the upstream driver, so the ipu6 `probe-mmu` and
  `isys-irq` compare the guest's writes with what the upstream driver
  should write, computed from its own tables in
  `tools/tests/ipu6/upstream_writes.py` (MMU `l1/l2_block_sz` and
  `info_bits`, 8 CSI-2 ports on 9a19, `ISYS_UNISPART_IRQS`, CDC
  thresholds, LTR/DID fabric control). Both tests stay one file each and
  branch on `$IPU4_VARIANT`; the ranges come from the same script. They
  passed without model changes (129 PSYS MMU writes at probe, 85 ISYS
  resume + suspend writes). Next: the firmware tests (`probe-fw`
  onwards), which need the IPU6 firmware interface in the model
  (`ipu6v_*`).
- (V8) The IPU6 ISYS firmware ABI matches IPU4's except the addresses,
  the SysCom registers (config/state in the buttress boot parameters,
  queue cursors from DMEM register 4), the firmware IRQ bit and the
  response token (48 bytes, no IPU4-only fields). Upstream also waits
  for answers to its iwake proxy writes, which the IPU4 model never
  gave. With those, one frame is delivered (STREAM_OPEN_DONE through
  PIN_DATA_READY); `stream-on` then fails at the CSI-2 PHY power-up
  ("PHY0 powerup ack timeout"), the next step.
- (V9) Upstream's MCD PHY rejects a 4-lane sensor on driver port 0
  (`ipu6_isys_driver_port_to_phy_port()`: 4 lanes only on PHY ports 0
  and 2, and driver port 0 is PHY port 1), so the virt-sensor presents 2
  lanes on 9a19. With the PHY power/ready handshake modelled, the frame
  streams end to end (`STREAM:pattern_ok`). `upstream_writes.py` reads
  the PHY tables from the pinned upstream source instead of copying
  them.
- (V10) `csi2-sof-irq` does not apply to upstream IPU6 as written:
  `ipu6_isys_csi2_set_stream()` enables the CSI-2 sync interrupts but
  writes their MASK as 0 ("Using event from firmware instead of irq"),
  so frame starts reach the driver only as the firmware's FRAME_SOF
  responses, which `stream-on` already exercises.
- (V11) Decided with the user: for ipu6, `csi2-sof-irq` covers the CSI-2
  error IRQ instead. The model reports a corrected packet header error on
  port 0 with each frame, so `isys_isr()` → `ipu6_isys_csi2_isr()` →
  `ipu6_isys_register_errors()` and `ipu6_isys_csi2_error()` run; the test
  also checks the sync MASK is only written 0. Both CLEAR registers are
  now written by the ISR at IRQ-dependent times, so `stream-on` leaves
  them out.
