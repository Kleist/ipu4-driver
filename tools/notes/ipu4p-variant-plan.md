# Plan — QEMU coverage for the IPU4P variant

Owned by the **IPU4P-variant track**. The IPU4 port's plan is
[`ipu6-ipu4-plan.md`](ipu6-ipu4-plan.md) and the IPU6 variant's is
[`ipu6-variant-plan.md`](ipu6-variant-plan.md). "Tracks and file ownership"
in the IPU4 port's plan says which files each track owns and how shared
files are changed. Every step of this track starts by re-reading this file
and ends by updating it (State column, Learnings) in the same PR. It edits
no other track's plan.

## Goal

Run the ipu6 test ladder against an IPU4P (Ice Lake, `8086:8a19`) model in
QEMU. Tests are developed against the unmodified baseline, Ruslan Bay's
IPU4P series without the IPU4 patches; the same tests then run on the
upstreamable driver with the series applied (the IPU4 port's gate). This
shows two things:

- Ruslan Bay's IPU4P support works under the model.
- The IPU4 patches (`patches/ipu6-ipu4/`) don't regress it.

## Base and harness

- **Kernel:** the baseline driver `ipu4p`: the mainline pin
  `tools/ipu6/pin` (bumped weekly) plus `patches/ipu6-ipu7/` (Antti's
  IPU7 series) and `patches/ipu6-ipu4p/` (Ruslan's IPU4P series), exported from his
  `ipu4p-upstream` branch), in `tools/linux-ipu6-ruslan/`
  (`IPU4_DRIVER=ipu4p`). The patched tree (`IPU4_DRIVER=ipu4-patches
  IPU4_VARIANT=ipu4p`) is gated by the IPU4 port in
  `enabled-tests.ipu4-patches.ipu4p`, which gains a test once it passes
  on both.
- **QEMU:** a `variant=ipu4p` value of the model's `variant` property
  (`-device ipu4,variant=ipu4p`). It sets PCI ID `8a19` and, step by step,
  the IPU4P register behaviour below. Variant-specific behaviour goes in
  `ipu4p_*` functions behind the variant switch, in their own hunks of
  `tools/qemu-patches/hw/misc/ipu4.c`.
- **Gating:** `tools/tests/ipu6/enabled-tests.ipu4p.ipu4p`, owned by this
  track.
- **CI:** the `vm-ipu6.yml` leg `ipu4p / ipu4p`, tree cache
  `linux-ipu6-ruslan`; the QEMU cache is shared.
- **Dashboard:** the `ipu4p/ipu4p` column of the "IPU4 port" matrix.

## What the model must do differently

Taken from the IPU4 port's comparison table:

| Area | IPU4 (model today) | IPU4P (to add) |
|---|---|---|
| PCI ID | `5a88` | `8a19` |
| Buttress PWR_STATE FSM | IS 23:20 = `0xa`, PS 28:24 = `0xf` | IS 24:20 = `0xc`, PS 29:25 = `0x10` |
| FREQ_CTL | ratio 7:0, QoS floor 15:8 | IS ratio at bit 1; PS ratio at bit 8, override bit 7 |
| SysCom queue-pointer base register (DMEM) | 6 | 8 |
| UNISPART software IRQ | BIT(30) | BIT(22) |
| CSI-2 IRQ routing | `0x8 << port`, `0x800 << (port - 4)`; trios at port base + `0x400/0x500/0x600` | SIP0/SIP1 IRQ windows |
| CSI-2 ports | 6 | 8; port ↔ firmware source `{3, 6..12}` |
| PHY sideband | none | sideband registers (latches are enough) |
| Firmware file | `intel/ipu/ipu4_fw.bin` | `intel/ipu/ipu4p_fw.bin` (synthesized like IPU4's, `tools/firmware/gen-cpd.py`) |

The ISYS firmware ABI and the ISYS/PSYS/MMU/DMEM/SPC base offsets are the
same as IPU4's. So the syscom and frame-delivery emulation is reused as is.

**Caveat: there is no IPU4P silicon trace.** The model is inferred from
Ruslan's driver, so the tests risk checking the driver against itself.
Silicon-trace checks like the IPU4 port's `probe-mmu`/`isys-irq` are not
possible. An mmiotrace from a Surface Pro 7 (Ruslan / linux-surface) would
give IPU4P the footing IPU4 has with `data/trace.txt`. Until then, the tests
check driver behaviour the model can observe: probe paths, power FSM
handshakes, firmware round trips and frames.

## PR sequence

One PR per ladder test for the variant, top of the table first. One open
PR at a time, from `main` (`CLAUDE.md`); the next step waits on a local
branch until it merges. Independent of the other tracks. A step that needs a shared-core change (`ladder.sh`, `run-vm.sh`,
the `init.ipu6-*` frames, `vm-ipu6.yml`) puts that change in its own
commit, first in the step's PR.

| PR | Content | `enabled-tests.ipu4p.ipu4p` gains | State |
|---|---|---|---|
| P1 | `vm-ipu6.yml` matrix leg `ipu4p` and `enabled-tests.ipu4p` with the static tests | `series-applies`, `build`, `checkpatch` | merged (#139) |
| P2 | QEMU `variant=ipu4p` (PCI ID `8a19`); `run-vm.sh` maps the variant to `ipu_pci_id=` | `vm-boot`, `module-load` | merged (#142) |
| P3 | `module-load` for the variant (virt-sensor on the variant's device, as the IPU6 variant's V3 does) | `module-load` | folded into P2 |
| P4 | Stage `intel/ipu/ipu4p_fw.bin`; `probe-identify` expects that file for `8a19` | `probe-identify` | merged (#144) |
| P4b | Baseline leg `ipu4p / ipu4p` (Ruslan's tree without the series); P1–P4 move to it, `ipu4-patches / ipu4p` keeps them for the series | `series-applies`, `build`, `checkpatch`, `vm-boot`, `module-load`, `probe-identify` | merged (with the ladder's six legs) |
| P5 | Model: IPU4P PWR_STATE FSM and FREQ_CTL | `probe-buttress` | merged (#147) |
| P5b | Ruslan's code as a patch series on the mainline pin (`patches/ipu6-ipu7/`, `patches/ipu6-ipu4p/`), pin bumped weekly | — | merged (#148) |
| P6 | ISYS binds on IPU4P: virt-sensor on firmware port 3; the forced ISYS resume then runs with ISYS bound | `isys-resume` | in review |
| P7 | Model: SysCom queue base register 8, UNISPART SW IRQ bit 22, SIP0/SIP1 IRQ windows | `fw-open` | |
| P8 | Model: 8 CSI-2 ports, port ↔ source map, PHY sideband latches | `stream-on`, `stream-pattern` | |

## Open questions

- **P6 test:** without a trace, what can show the IPU4P ISYS setup is right?
  The best candidate is the model raising the firmware IRQ only on the bit
  the driver unmasked. Then `fw-open` fails if UNISPART is wrong.
- **Silicon trace:** is a Surface Pro 7 mmiotrace obtainable? With it, P5+
  could gate on silicon-trace checks like the IPU4 port.

## Learnings

- (P1) The ipu4p leg reuses the ipu4 leg's `matrix.include` fields
  (`tools/linux-ipu6`, `tools/ipu6/pin`, cache `linux-ipu6`), so both legs
  restore the same tree cache. Whichever leg finishes first saves it; the
  other's save is a no-op. `ladder.sh` already mapped `ipu4p` to that tree
  and `enabled-tests.ipu4p`, and the dashboard already had the column, so
  P1 needed no shared-script change beyond the matrix leg.
- (P1) Until P2 the VM tests fail on this leg (`run-vm.sh` rejects
  `IPU4_VARIANT=ipu4p` and the model has no `variant=ipu4p`). They are
  not gated, so the leg stays green.
- (P2) `module-load` passed as soon as the model presented `8a19`, with no
  test change: `init.ipu6-modules` already reads `ipu_pci_id=`, and
  `intel-ipu6` binds `8a19` on load (Ruslan's driver lists it), then
  fails probe on the missing `intel/ipu/ipu4p_fw.bin`. Whether probe
  succeeds is the probe-* tests' job, so P3 was folded into P2.
- (P2) First full-ladder run on ipu4p (TCG): `probe-identify` fails only
  because `ipu4p_fw.bin` isn't staged in the rootfs yet (P4); everything
  after it is SKIP.
- (P4) The synthesized IPU4 blob loads and validates as `ipu4p_fw.bin`
  unchanged. Probe then stops at the power-up: `Change power status
  timeout with 0xa02003`, the model still reporting IPU4's PWR_STATE
  layout (P5).
- (P4) Ruslan's driver has its own `ipu4p-fw-isys.c` (separate
  `ipu4p_fw_isys_*` send/proxy tokens), so "the ISYS firmware ABI is the
  same as IPU4's" above needs checking before P7; the model's syscom
  emulation may need IPU4P-specific token sizes.
- (P4b) BASELINE_LEARNING
- (P5) FREQ_CTL needed no model change: IPU4P still powers an island on
  with START (BIT(31)), which the model already tracks; only the ratio
  and override bits move. PWR_STATE got `ipu4pv_pwr_state()`.
- (P5) Past power-up, `intel-ipu6-isys` fails probe with `invalid csi2
  port 0`: virt-sensor attaches to firmware port 0, which IPU4P's port
  map `{3, 6..12}` doesn't have. Needs a per-variant virt-sensor port
  (P8, or earlier if a resume test needs ISYS bound).
- (P5) `isys-resume` passes on ipu4p even with ISYS unbound, so it is
  not gated here; revisit once ISYS binds.
- (P5b) Ruslan's pin was 116 commits on v7.3-rc4: 60 for his downstream
  `intel/ipu4` driver, which the ipu6 flavor never builds, then Antti's 45
  IPU7 patches and his 11 IPU4P patches. The 56 `ipu6/` patches
  cherry-pick cleanly onto v7.3-rc4 and give a byte-identical `ipu6/` and
  `staging/media/ipu7/`, so they alone became `patches/ipu6-ipu7/` (45)
  and `patches/ipu6-ipu4p/` (11).
- (P6) With virt-sensor on firmware port 3 (IPU4P's CSI-2 port 0),
  `intel-ipu6-isys` binds (64 video nodes) and the forced resume passes
  with no model change: UNISPART bit 22 and the SIP IRQ windows only
  matter once the firmware raises interrupts, so they moved to P7.
