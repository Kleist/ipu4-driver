# Intel IPU4 Kernel Driver

Project state at a glance: <https://kleist.github.io/ipu4-driver/> — ladder results, code coverage, MMIO divergence and CI status.

A Linux driver for the Intel IPU4 (PCI `8086:5a88`), based on the upstream IPU6 driver (`drivers/media/pci/intel/ipu6/`, mainline since 6.10). The goal is upstream IPU4 support; there is no timeline.

The driver does not work on any publicly available hardware as-is: at minimum the `ambu_ipu_bridge_*` calls must be replaced.

## Two forms of the driver

- **Out-of-tree module** (`kernel/ipu4/`): the IPU6 driver with IPU4 differences behind `#ifdef IPU6` (never defined; marks the upstream code) or in parallel `ipu4_*` functions. Builds `intel-ipu4.ko` and `intel-ipu4-isys.ko` against 6.6 – 7.3 via the shims in `ipu4-compat.h`. Known to work on silicon.
- **Upstreamable patch series** (`patches/ipu6-ipu4/`): IPU4 support added to the upstream `ipu6` driver as mailable patches. They apply on mainline (`tools/ipu6/pin`, v7.3-rc4, bumped weekly) after two series not yet in mainline: the IPU7 series (`patches/ipu6-ipu7/`) and Ruslan Bay's IPU4P series (`patches/ipu6-ipu4p/`, from [ruslanbay/linux `ipu4p-upstream`](https://github.com/ruslanbay/linux/tree/ipu4p-upstream)). Work in progress.

## Testing

Everything runs in software: a patched QEMU (`tools/qemu-patches/`) emulates the IPU, a virtual sensor (`tools/virt-sensor/`) feeds it a test pattern, and register traffic is compared against a trace from real silicon (`data/trace.txt`).

### Test ladder (`vm-ipu6.yml`)

`tools/tests/ipu6/ladder.sh` runs a fixed sequence of tests, each depending on the previous: series applies, build, checkpatch, VM boot, module load, probe (identify, buttress power-up, MMU, ISYS resume/IRQ), firmware open, stream on, captured pattern, CSI-2 frame-start IRQ. One file per test in `tools/tests/ipu6/tests.d/`.

Each run is one *leg*: a driver under a QEMU model *variant* (`ipu4`, `ipu4p`, `ipu6`):

| Driver (`IPU4_DRIVER`) | Tree | Variants run |
|---|---|---|
| `ipu4-out-of-tree` | `kernel/ipu4/` built against v7.3-rc4 | `ipu4` |
| `ipu4p` | mainline + IPU7 + IPU4P series | `ipu4p` |
| `ipu6` | unmodified mainline | `ipu6` |
| `ipu4-patches` (default) | the `ipu4p` tree + `patches/ipu6-ipu4/` | `ipu4`, `ipu4p`, `ipu6` |

The first three are baselines that tests and QEMU models are developed against; `ipu4-out-of-tree` is known to work, so a failure there points at the test or model. `ipu4-patches` must eventually pass everywhere its baselines do. Every test runs on every leg, but a leg only fails on tests listed in its `tools/tests/ipu6/enabled-tests.<driver>.<variant>`.

```bash
tools/tests/ipu6/ladder.sh                                    # default leg
IPU4_DRIVER=ipu6 IPU4_VARIANT=ipu6 tools/tests/ipu6/ladder.sh  # another leg
```

Plans: `tools/notes/ipu6-ipu4-plan.md` (the series), `ipu4-out-of-tree-plan.md`, `ipu4p-variant-plan.md`, `ipu6-variant-plan.md`.

### Out-of-tree harness (`ci.yml`, `vm-smoke.yml`)

For `kernel/ipu4/` across stable kernels (`.github/kernel-pins/`: 6.12, 6.18, 7.3):

- **Build + KUnit** on every pin: `tools/bootstrap.sh && tools/build.sh && tools/tests/kunit.sh`.
- **VM smoke** (6.12 per PR, others weekly): boots the VM, walks the V4L2 capture API end to end (`streamon-smoke.sh`), records an mmiotrace and diffs it against silicon (`compare-mmio.sh`).
- **pytest** (`tests/`) for the harness's own Python tooling.

See [CLAUDE.md](CLAUDE.md) for prerequisites, all commands and CI details, and [STATUS.md](STATUS.md) for milestones.

## Upstream sync

- `tools/upstream/diff.sh` reports the divergence of `kernel/ipu4/` from upstream IPU6.
- `tools/upstream/watch.sh` runs weekly in CI and, when there is something new, opens a PR with cherry-picks of new upstream IPU6 commits.

## Hardware trace scripts

`trace_ipu4.sh`, `trace_functions.sh`, `split_trace.sh` and `postprocess_trace.py` capture and annotate mmiotraces on real hardware (kernel built with `kernel/configs/mmiotrace.config`). They won't run as-is elsewhere; treat them as a reference.

## History

Derived from the upstream IPU6 driver and the IPU4 driver in Clear Linux ([linux-iot-lts2018](https://github.com/clearlinux-pkgs/linux-iot-lts2018)). IPU4 support was first hacked onto IPU6 on 6.6 ([lore](https://lore.kernel.org/all/e136389011517dbc65b30f6bf0b1a9c49ab4e599.camel@gmail.com/), [branch](https://github.com/Kleist/linux/tree/kleist-v6.6-ipu4-hacks-1)), then turned into this out-of-tree module.

## License

GPL-2.0-only. See [LICENSE](LICENSE).
