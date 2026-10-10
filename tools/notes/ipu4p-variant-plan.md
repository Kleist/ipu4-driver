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
QEMU. This shows two things:

- Ruslan Bay's IPU4P support in the shared driver still works with the IPU4
  series (`patches/ipu6-ipu4/`) applied on top.
- The IPU4 patches don't regress it.

## Base and harness

- **Kernel:** the same tree as the IPU4 port, `tools/ipu6/pin` (Ruslan's
  `ipu4p-upstream`) plus `patches/ipu6-ipu4/`, in `tools/linux-ipu6/`.
  `ladder.sh` already maps `IPU4_VARIANT=ipu4p` to it. Unlike the IPU6
  variant, which runs on unpatched mainline, the IPU4P variant runs on the
  tree that carries both IPU4P and IPU4. That is the combination upstream
  would get.
- **QEMU:** a `variant=ipu4p` value of the model's `variant` property
  (`-device ipu4,variant=ipu4p`). It sets PCI ID `8a19` and, step by step,
  the IPU4P register behaviour below. Variant-specific behaviour goes in
  `ipu4p_*` functions behind the variant switch, in their own hunks of
  `tools/qemu-patches/hw/misc/ipu4.c`.
- **Gating:** `tools/tests/ipu6/enabled-tests.ipu4p`, owned by this track.
- **CI:** a third `vm-ipu6.yml` matrix leg (`variant: ipu4p`). It shares the
  ipu4 leg's tree cache (same directory, same pin) and the QEMU cache.
- **Dashboard:** the "IPU4 port" matrix already has an `ipu4p` column. It
  lights up once the leg uploads `ipu6-ladder-results-ipu4p`.

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

One PR per ladder test for the variant, top of the table first. At most
3 PRs are stacked (`CLAUDE.md`), and the stack is independent of the other
tracks. A step that needs a shared-core change (`ladder.sh`, `run-vm.sh`,
the `init.ipu6-*` frames, `vm-ipu6.yml`) puts that change in its own small
PR first.

| PR | Content | `enabled-tests.ipu4p` gains | State |
|---|---|---|---|
| P1 | `vm-ipu6.yml` matrix leg `ipu4p` and `enabled-tests.ipu4p` with the static tests | `series-applies`, `build`, `checkpatch` | |
| P2 | QEMU `variant=ipu4p` (PCI ID `8a19`); `run-vm.sh` maps the variant to `ipu_pci_id=` | `vm-boot` | |
| P3 | `module-load` for the variant (virt-sensor on the variant's device, as the IPU6 variant's V3 does) | `module-load` | |
| P4 | Stage `intel/ipu/ipu4p_fw.bin`; `probe-identify` expects that file for `8a19` | `probe-identify` | |
| P5 | Model: IPU4P PWR_STATE FSM and FREQ_CTL | `probe-buttress` | |
| P6 | ISYS runtime resume on IPU4P: UNISPART SW IRQ bit 22, SIP0/SIP1 windows. Needs the IPU4 port's forced ISYS resume (its PR 9) | an IPU4P resume test (driver-observable, not trace-based) | |
| P7 | Model: SysCom queue base register 8 | `fw-open` | |
| P8 | Model: 8 CSI-2 ports, port ↔ source map, PHY sideband latches | `stream-on`, `stream-pattern` | |

## Open questions

- **P6 test:** without a trace, what can show the IPU4P ISYS setup is right?
  The best candidate is the model raising the firmware IRQ only on the bit
  the driver unmasked. Then `fw-open` fails if UNISPART is wrong.
- **Silicon trace:** is a Surface Pro 7 mmiotrace obtainable? With it, P5+
  could gate on silicon-trace checks like the IPU4 port.

## Learnings

(None yet.)
