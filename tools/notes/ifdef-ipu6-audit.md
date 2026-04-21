# `#ifdef IPU6` / `#ifndef IPU6` hunk audit

Step 3 of the upstream-prep roadmap in
`tools/notes/upstream-divergence.md`. The `IPU6` macro is never
defined — per `CLAUDE.md`, these markers tag the upstream behaviour
as "leave alone, IPU4 does something different here." To land IPU4
in mainline we need to turn each one into either:

- **(a)** a `is_ipu4(hw_ver)` runtime discriminator (real pipeline
  / ISR divergence),
- **(b)** a resolved bug report (we suspect upstream is wrong and
  we're right, or vice versa), or
- **(c)** a conditional on an IPU4-only feature / register (mostly
  header constants and struct fields unused by the IPU4 firmware
  ABI).

35 sites; 2 of them (b), 15 (a), 18 (c).

## Per-site tagging

| File | Line | Form | Tag | What the IPU6 branch does |
|---|---:|---|---|---|
| `isys-csi2.c` | 207 | `#ifdef IPU6` | (a) | Error ISR: register / clear CSI2 error bits |
| `isys-csi2.c` | 311 | `#ifdef IPU6` | (a) | Stream enable/disable IRQ setup |
| `isys-queue.c` | 784 | `#ifdef IPU6` (disabled) | (a) | Stream watermark configuration on queue setup |
| `isys-queue.c` | 810 | `#ifdef IPU6` (disabled) | (a) | Stream watermark update on stream-start error |
| `isys-queue.c` | 944 | `#ifdef IPU6` (disabled) | (a) | Stream watermark update on stream stop |
| `fw-isys.h` | 198 | `#ifdef IPU6` | (c) | MIPI virtual channel enum (16 VCs vs 4 on IPU4) |
| `fw-isys.h` | 272 | `#ifdef IPU6` (disabled) | (c) | Capture mode + sensor mode enums (N/A on IPU4) |
| `fw-isys.h` | 710 | `#ifdef IPU6` | (c) | FW response struct (IPU6 firmware ABI only) |
| `isys-video.c` | 550 | `#ifdef IPU6` (disabled) | (c) | MIPI decompression + capture-mode fields |
| `isys-video.c` | 557 | `#ifdef IPU6` (disabled) | (c) | Crop first/last-lines field |
| `isys-video.c` | 577 | `#ifdef IPU6` (disabled) | (c) | Output pin timestamp + remapping + sensor-type fields |
| `isys-video.c` | 619 | `#ifdef IPU6` (disabled) | (c) | Stream config sensor-mode field |
| `isys-video.c` | 826 | `#ifdef IPU6` (disabled) | (a) | Watermark configure function |
| `isys.c` | 272 | `#ifdef IPU6` | (a) | Per-port IRQ setup (dynamic offsets vs hard-coded) |
| `isys.c` | 334 | `#ifdef IPU6` | (a) | CSI2 sync-IRQ status read in ISR |
| `isys.c` | 355 | `#ifdef IPU6` | **(b)** | `csi2_sof_event_by_stream` call — *"is this a bug in upstream? 2 x sof_event send?"* |
| `isys.c` | 366 | `#ifdef IPU6` | **(b)** | `csi2_eof_event_by_stream` call — *"is this a bug in upstream? 2 x eof_event send?"* |
| `isys.c` | 428 | `#ifndef IPU6` | (a) | ISR dispatch: IPU4 → `ipu4_isys_isr`, IPU6 → main handler |
| `isys.c` | 491 | `#ifdef IPU6` | (a) | iWake / LTR watermark setup functions |
| `isys.c` | 836 | `#ifdef IPU6` | (c) | `ipu_bridge_instantiate_vcm` (no VCM on IPU4) |
| `isys.c` | 1034 | `#ifdef IPU6` | (a) | iWake LTR/DID on runtime resume |
| `isys.c` | 1060 | `#ifdef IPU6` | (a) | iWake LTR/DID on runtime suspend |
| `isys.c` | 1234 | `#ifdef IPU6` (disabled) | (c) | Sensor-type initialization |
| `isys.c` | 1275 | `#ifdef IPU6` (disabled) | (a) | Watermark init + PHY `set_power` wiring |
| `isys.c` | 1341 | `#ifdef IPU6` (disabled) | (a) | Watermark cleanup |
| `isys.c` | 1489 | `#ifdef IPU6` (disabled) | (a) | EOF event dispatch in FW response handler |
| `isys-csi2.h` | 20 | `#ifdef IPU6` | (c) | `NR_OF_CSI2_VC` (16 vs 4) |
| `isys-csi2.h` | 28 | `#ifdef IPU6` | (c) | `NR_OF_CSI2_SRC_PADS` (8 vs 4) |
| `ipu6.c` | 23 | `#ifdef IPU6` | (c) | Upstream `<media/ipu-bridge.h>` include |
| `ipu6.c` | 78 | `#ifdef IPU6` | (c) | `isys_ipdata` struct definition |
| `ipu6.c` | 493 | `#ifdef IPU6` | (c) | ISYS pdata initialization |
| `ipu6.c` | 577 | `#ifndef IPU6` | (a) | ISYS init: IPU4 fwnode-graph check before bridge |
| `ipu6.c` | 585 | `#ifdef IPU6` | (a) | IPU6: `ipu_bridge_init()`; IPU4: fwnode-graph fallback |
| `ipu6.c` | 907 | `#ifdef IPU6` | (c) | SKU version string in boot-banner printk |
| `platform-isys-csi2-reg.h` | 11 | `#ifdef IPU6` | (c) | Per-port CSI register base addresses + IRQ mappings |

## (b) sites — suspected upstream bugs

Both are on the CSI2 FS/FE interrupt paths in
`kernel/ipu4/ipu6-isys.c`:

- **`isys.c:355`** — `#ifdef IPU6 // TBD - is this a bug in upstream? 2 x sof_event send?`
  calls `ipu6_isys_csi2_sof_event_by_stream(stream)` from the
  ISR. IPU4 branch does nothing. The concern: a later FW response
  path (line ~1489 on IPU6) also fires an SOF — so upstream may
  be double-dispatching.
- **`isys.c:366`** — `#ifdef IPU6 // TBD - is this a bug in upstream? 2 x eof_event send?`
  calls `ipu6_isys_csi2_eof_event_by_stream(stream)` from the
  ISR. Same double-dispatch concern on EOF.

**Needed:** trace upstream IPU6 on real hardware (or in the QEMU
harness with a real firmware) to confirm whether userspace sees
one or two `V4L2_EVENT_FRAME_SYNC` per frame. If two, file with
the media maintainer. If one, our fork is drifting and should
re-enable the ISR path.

## (a) clusters — the big upstream patches

The 15 (a) sites group into five thematic clusters. Each cluster
is one logical patch in the series we'd send to LKML:

1. **Watermark / iWake / LTR subsystem.** Sites: `isys-queue.c`
   × 3, `isys-video.c:826`, `isys.c:491 + 1034 + 1060 + 1275 +
   1341`. Entire feature absent on IPU4 hardware. Patch shape:
   new `ipu6_has_iwake(hw_ver)` predicate, each current `#ifdef
   IPU6` block becomes `if (ipu6_has_iwake(hw_ver))`.
2. **CSI2 ISR handling.** Sites: `isys-csi2.c:207 + 311`,
   `isys.c:272 + 334`. IPU4 wires `ipu4_isys_isr` instead of the
   IPU6 handler (`isys.c:428`). Patch shape: runtime dispatch at
   `request_irq` time based on `hw_ver`.
3. **Bridge init.** Sites: `ipu6.c:577 + 585`. IPU4 needs a
   fwnode-graph probe before falling back to `ipu_bridge_init()`.
   Patch shape: extend upstream's `ipu_bridge_init` to try
   fwnode first, or add a IPU4-specific sensor probe.
4. **EOF-response dispatch.** Site: `isys.c:1489`. IPU4 skips
   `FRAME_EOF` processing from the FW response. Tied to (b) —
   resolve (b) first.
5. **(nothing else freestanding.)** All other (a) sites fall into
   one of the above four.

## (c) sites — conditionalize on a runtime flag or compile-time Kconfig

18 sites, mostly header constants and struct fields unused by the
IPU4 firmware ABI. These are the easiest to land: a single
`is_ipu4(hw_ver)` runtime check (for the sizes / counts) or a
mainline `CONFIG_VIDEO_INTEL_IPU4` Kconfig symbol that compiles
the IPU6-only fields out. No behaviour risk.

Notable sub-group: **array sizes** at `isys-csi2.h:20 + 28` and
`fw-isys.h:198`. IPU4 uses 4 VCs / 4 source pads, IPU6 uses 16 /
8. Upstream has no precedent for runtime-sized CSI2 arrays, so
these likely become two struct layouts + a pointer-of-struct
dispatch in the driver. A real patch candidate but small.

## What this audit does NOT do

- Check that each (a) site still has a working IPU4 branch. (Some
  of the "disabled for IPU4" blocks could hide bugs — e.g. if the
  IPU4 FW actually does populate `capture_mode` and we're
  dropping it on the floor. Not investigated.)
- Verify the (c) struct-field drops are safe under the IPU4
  firmware ABI. We ship a stub firmware in the harness so we've
  never exercised the real ABI.
- Produce actual patches. The tagging is the input to patching,
  not the output.

## Regenerating this audit

```sh
grep -rnE '^#\s*(ifdef|ifndef)\s+IPU6\b' kernel/ipu4/ \
  | grep -v '_H$' | sort
```

gives the 35-site list. Each site needs ~25 lines of surrounding
context to tag; the tags above were produced by reading every one.
