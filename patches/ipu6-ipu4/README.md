# IPU4 support for the upstream ipu6 driver — patch series

This directory holds the kernel patches that add Intel IPU4 (Broxton /
Apollo Lake, `8086:5a88`) to `drivers/media/pci/intel/ipu6/`. The plan,
the base tree and the PR sequence are in
[`tools/notes/ipu6-ipu4-plan.md`](../../tools/notes/ipu6-ipu4-plan.md).

## Format

- Plain `git format-patch` output, one file per commit, named
  `NNNN-<subject>.patch` so that the shell glob order is the series order.
- Applied with `git am` on top of the pinned tree (`tools/ipu6/pin`) and
  the base series (`patches/ipu6-ipu7/`, `patches/ipu6-ipu4p/`) by
  `tools/ipu6/bootstrap.sh` (or `IPU4_FLAVOR=ipu6 tools/bootstrap.sh`).
  Every bootstrap resets the tree to the pin first, so the files here are
  the only source of truth.
- Upstream quality: each patch is what would be mailed to linux-media —
  `media: ipu6: ...` subject, commit message, `Signed-off-by`,
  `scripts/checkpatch.pl --strict` clean, and the tree builds after every
  patch.
- No harness-only code. Test helpers (virt-sensor, QEMU model changes)
  live under `tools/`, never here.
- The patch that adds `8086:5a88` to the PCI ID table
  (`media: ipu6: Enable IPU4 PCI device`) is always the **last** patch,
  as upstream wants the device enabled only once its support is in. The
  series carries it from the start so the tests bind the device the
  normal way; a new patch goes in before it, and regenerating the series
  renumbers it.

## Workflow

```bash
tools/ipu6/bootstrap.sh                   # tools/linux-ipu6 = pin + this series
# ... commit changes in tools/linux-ipu6 ...
git -C tools/linux-ipu6 format-patch -o "$PWD/patches/ipu6-ipu4" series-base
tools/tests/ipu6/ladder.sh                # series-applies, build, checkpatch, ...
```

Regenerating the whole series with `format-patch` keeps the numbering
contiguous; delete the old files first when a patch is dropped or renamed.
