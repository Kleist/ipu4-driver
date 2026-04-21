# Upstream divergence audit — `kernel/ipu4/` vs `drivers/media/pci/intel/ipu6/` at v6.12

End goal (per `CLAUDE.md`): land IPU4 support in mainline, either as
the moral equivalent of a new `drivers/media/pci/intel/ipu4/` or as
IPU6-subsumes-IPU4 patches on top of the existing IPU6 driver.
Everything added for the QEMU harness is scaffolding that must not
bleed into the upstream patch series.

This note is the first inventory of what we carry on top of upstream
IPU6 at Linux `v6.12`. It drives the cleanup sequence; it is NOT a
change list for submission yet.

## File-level divergence

Files the fork adds, with disposition for upstream:

| Path                                 | Lines         | Disposition |
|--------------------------------------|---------------|-------------|
| `ambu-ipu-bridge.c` / `.h`           | ~330 + ~100   | **Drop.** Downstream I2C bridge. Replaced at build time by `virt-sensor.c` under `CONFIG_VIDEO_IPU4_VIRT_SENSOR`; no upstream path. Targets a private `ambu,tc358748` quirk only. |
| `virt-sensor.c` / `.h`               | ~320 + ~20    | **Harness-only.** Never leaves this repo. Keep in-tree here; flag at the top of the file. |
| `ipu4-compat.h`                      | ~60           | **Conditional drop.** On `v6.12` only the `v4l2_subdev_get_pad_format` macro is live, and no source calls it. Remove it plus the four `#include "ipu4-compat.h"` lines in `ipu6-isys-{csi2,subdev,video}.c` / `ipu6-buttress.c`. Zero-line cleanup for upstream if done on a recent-kernel-only branch. |
| `ipu4-platform-buttress-regs.h`      | 400+          | **Upstream.** Genuine IPU4 register definitions. Already structured alongside `ipu6-platform-buttress-regs.h` — same pattern mainline uses for IPU6SE. |
| `ipu4-platform-isys-csi2-reg.h`      | ~200          | **Upstream.** Same story. |
| `ipu4-platform-regs.h`               | ~150          | **Upstream.** Same story. |

Files the fork is missing (upstream has them, we don't):

| Path | Why missing |
|---|---|
| `Kconfig` | Shipped instead via `tools/linux-patches/drivers/media/pci/intel/ipu4/Kconfig` with its own symbol `VIDEO_INTEL_IPU4`. Reasonable; a real upstream submission would ship the same file under the driver directory. |

## Hunk-type inventory

Total diff magnitude (`diff | wc -l`, shared files only):

| File | Total | Added | Removed |
|---|---:|---:|---:|
| `ipu6-isys-queue.c` | 683 | 395 | 39 |
| `ipu6-isys.c`       | 621 | 236 | 68 |
| `ipu6.c`            | 571 | 295 | 38 |
| `ipu6-isys-video.c` | 493 | 129 | 47 |
| `ipu6-dma.c`        | 394 |  98 | 108 |
| `ipu6-fw-isys.h`    | 389 | 280 |  9 |
| `ipu6-fw-isys.c`    | 279 | 172 | 13 |
| `ipu6-buttress.c`   | 265 |  55 | 29 |
| `ipu6-mmu.c`        | 255 | 147 | 15 |
| `ipu6-isys-csi2.c`  | 228 | 157 |  6 |
| `ipu6-fw-com.c`     | 202 |  42 | 46 |

Everything smaller than that is under 80 diff lines and usually a
header add.

Hunks fall into five categories:

### 1. `CONFIG_VIDEO_IPU4_VIRT_SENSOR` guards (harness-only)

14 hunks across 5 files (`ipu6.c` × 3, `ipu6-isys.c` × 2,
`ipu6-isys-queue.c` × 7, `ipu6-fw-isys.c` × 1, `virt-sensor.c` × 1).
Every one compiles out when the symbol is `n` — they're the M4.5
/ M5b-3 / M5c-1 / M5c-2 hooks.

**Disposition for upstream: none of these go.** The
`ipu6-isys-queue.c` ones in particular replace the firmware
streaming path with software completion; upstream IPU6 needs its
firmware path intact.

**Action:** when we start the upstream branch, `grep -v
CONFIG_VIDEO_IPU4_VIRT_SENSOR` on each file should produce the
"production" variant. Current code is close to that discipline but
a mechanical audit is needed — `M5b-3`'s
`isys_install_virt_sensor_route` helper sits outside the `#if`
block and is dead code when the symbol is off (OK, just
gratuitous). Move it *inside* the guard to keep the production
view pristine.

### 2. `#ifdef IPU6` / `#ifndef IPU6` (IPU4-vs-IPU6 behavioural fork)

The `IPU6` macro is never defined. The discipline per CLAUDE.md is:
these mark "upstream version, leave alone" branches. 40 such hunks
across 10 files, concentrated in `ipu6-isys.c` (13) and `ipu6.c`
(6).

**Disposition for upstream:** each one is a potential patch. The
question per-hunk is "does IPU4 actually need the behaviour
change, or is the fork drifting?" Three sub-categories:

- **(a) IPU4 runs a different pipeline.** e.g.
  `ipu6-isys.c:272` (CSI2 ISR handling), `:428` (resets),
  `:1234`-range (watermark). Needs a runtime discriminator
  (`is_ipu4(hw_ver)` already exists in `ipu6.c`) rather than a
  compile-time macro, then land as clean patches.
- **(b) Suspected upstream bug, marked locally.** e.g.
  `ipu6-isys.c:355` *"TBD - is this a bug in upstream? 2 x
  sof_event send?"*. Needs a proper debug and either a bug
  report or a confirmation we're the ones drifting.
- **(c) IPU4-only feature.** Mostly in headers (`fw-isys.h`
  stream-id / pin-count macros).

**Action:** walk every `#ifdef IPU6` hunk, tag with `(a|b|c)`, and
produce a pre-upstream cleanup list. Biggest payoff per hour of
work.

### 3. `ipu4-compat.h` uses (kernel-version shims)

5 files include it. Live content on `v6.12`:
- 6.11 macro: only needed for `<6.11` → dead on 6.12.
- 6.10 else-branch: macro `v4l2_subdev_get_pad_format` for
  `>= 6.10`; no call site.
- 6.6.103 macro: only needed for `<6.6.103` → dead on 6.12.

**Disposition for upstream: zero.** The upstream patch series
targets a single kernel; no compat layer belongs there.

**Action:** separate upstream-prep branch gets `ipu4-compat.h`
deleted and its four include sites removed. ~15 lines total, no
behaviour change on v6.12+.

### 4. `ambu_ipu_bridge_*` references (downstream bridge)

Used from `ipu6.c` (probe / teardown) and `virt-sensor.c` (the
wrapper).

**Disposition:** drop for upstream. In the non-virt-sensor build,
a real sensor plus `ipu_bridge_init()` from the core (what upstream
IPU6 uses) fills the role. `ambu-ipu-bridge` is a downstream-only
substitute that the harness replaces with `virt-sensor`.

**Action:** make `ipu6.c`'s choice a single two-way dispatch:
upstream `ipu_bridge_init()` when IPU_BRIDGE is selected,
`ipu4_virt_sensor_install()` when harness is selected, error
otherwise. Retire the ambu path entirely from the upstream-prep
branch.

### 5. Bare differences (unconditional drift)

The residual — hunks not guarded by anything, not tagged by
`#ifdef IPU6`, not touching `ipu4-compat.h` or ambu. These are the
hardest category. Sampled examples:

- `ipu6-dma.c`: cosmetic renames `__dma_alloc_buffer` →
  `__alloc_buffer`, etc. CLAUDE.md says *"Do not rename ipu6_*
  symbols"*; these are static helpers so technically fine, but
  they still grow the diff.
- `ipu6-fw-isys.h` +280 / -9: large additive block of
  `IPU4_*` constants (stream ids, pin counts, message queue
  sizes). Genuinely IPU4-specific; belongs upstream if IPU6
  header is extended.
- `ipu6-fw-com.c` +42 / -46: looks like a real restructure of
  the FW com helpers. Needs a real review against upstream to
  decide if this is IPU4-necessary or unwarranted drift.

**Action:** largest risk pool. Per-file review with a reviewer
who remembers the IPU6 upstreaming. Each bare hunk gets labeled
one of:

- **cherry-pick from upstream** — we're behind; pull it forward.
- **IPU4 behaviour change** — needs a runtime branch or a patch.
- **drift, revert for upstream** — gratuitous diff.

## Pre-upstream cleanup roadmap

In rough priority order (smallest / highest-leverage first):

1. **Move `isys_install_virt_sensor_route` inside the
   `CONFIG_VIDEO_IPU4_VIRT_SENSOR` guard** in `ipu6-isys.c` so the
   production file is bit-for-bit cleaner. Trivial.
2. **Delete `ipu4-compat.h`** on an upstream-prep branch (single
   commit, no behaviour change on 6.12). Small.
3. **Walk the 40 `#ifdef IPU6` hunks**, tag each (a/b/c), file bug
   reports for the (b)s. Medium.
4. **Replace `#ifdef IPU6` (a) hunks with `is_ipu4(hw_ver)`
   runtime checks.** Medium-large; each is a real patch
   candidate.
5. **Bare-diff audit** on `ipu6-dma.c`, `ipu6-fw-com.c`,
   `ipu6-fw-isys.h` and friends; cherry-pick vs. revert vs.
   upstream-patch. Large.
6. **Retire `ambu-ipu-bridge`** in the upstream-prep branch and
   make `ipu6.c` dispatch upstream `ipu_bridge_init` / harness
   `virt-sensor`.
7. **Produce a split patch series:** core IPU6 improvements we
   have landed (steps 3–5's cherry-picks), then the IPU4-only
   additions. That's the unit of submission to LKML.

## What this note does NOT claim

- Coverage of hunk contents — only magnitudes and categories.
- A timeline. Each step is real work and depends on review
  bandwidth.
- That the harness shims ever get removed — they stay here; the
  point is that they are isolable.

## Regenerating this note

The audit is reproducible from a fresh bootstrap:

```sh
tools/bootstrap.sh
for f in $(ls kernel/ipu4/ | grep -E '\.(c|h)$' | sort); do
    u=tools/linux/drivers/media/pci/intel/ipu6/"$f"
    [ -f "$u" ] || { echo "ADDED: $f"; continue; }
    l=$(diff "$u" kernel/ipu4/"$f" | wc -l)
    printf '%5d %s\n' "$l" "$f"
done | sort -rn
```
