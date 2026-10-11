# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Out-of-tree Linux kernel driver for Intel IPU4, forked from the upstream IPU6 driver (`drivers/media/pci/intel/ipu6/`, upstreamed in 6.10). Tested against stable kernels 6.6.111 and 6.12.47, and v7.3-rc4 (the base of the upstream-ipu6 pins). The driver does not work on any public hardware as-is — `ambu_ipu_bridge_*` calls must be replaced before the device probes.

There are two parallel layouts for the driver right now (see STATUS.md for the migration plan):

1. **`kernel/ipu4/` — the source of truth.** Out-of-tree module with its own `Makefile`. Edit driver `.c`/`.h` here.
2. **`tools/linux/drivers/media/pci/intel/ipu4/` — a seeded copy.** `tools/bootstrap.sh` clones upstream Linux at `v6.12` into `tools/linux/` and copies `kernel/ipu4/*.[ch]` into the in-tree path. This copy is regenerated and is *not* committed. Do not edit files under `tools/linux/` directly; edit `kernel/ipu4/` and re-bootstrap.

## The upstream-IPU6 discipline

Backports from upstream should be performed with `git cherry-pick -x` and fixing any conflicts. If the code doesn't work directly for IPU4 it should be handled with

- `#ifdef IPU6` (the `IPU6` macro is never defined — it's a "this is the upstream version, leave alone" marker), This can both be used for functions that are not needed, and function bodies where the behaviour should be different on IPU4 **or**
- a new function named `ipu4_*` parallel to the `ipu6_*` original.

Do not restructure or "clean up" IPU6 code. Do not rename `ipu6_*` symbols. Keep diffs minimal and localized.

`kernel/ipu4/ipu4-compat.h` holds `LINUX_VERSION_CODE`-gated shims for 6.6 / 6.10 / 6.11 / 6.14 / 6.18 kernel API changes. The cascade is upper-bound-open, so kernels newer than the highest shim (currently 6.18) ride on the no-shim path — v7.3-rc4 falls through cleanly. Add new shims here rather than `#ifdef`-ing callers.

## Commits and PRs

- Make small, logical commits: one self-contained change per commit, with a message that explains why.
- Every commit must pass CI on its own, not just the tip of the PR. Don't leave "fix previous commit" commits in a PR; fold them into the commit they fix before review.
- The same applies to kernel patches in `patches/`: each patch in the series must build and pass the checks on its own.
- Changes go through PRs from short-lived branches into `main`; never commit directly to `main`.
- Be brief. Commit messages: a subject line plus at most a few lines on why; no restating the diff. PR descriptions: a few bullets, no headings or test logs unless asked. Code comments: only where the code isn't self-explanatory, one or two lines, no history or investigation notes.
- One open PR at a time per line of work, never stacked. Prepare the next step on a local branch while the current PR is in review, but push it and open its PR only after the current one has merged (rebased onto the new `main`); review feedback then never has to be carried forward through a stack.

## Local prerequisites

Everything CI needs to build the harness lives in `.github/actions/setup-harness/action.yml`. To get the same setup on a fresh machine (Ubuntu 24.04 / Debian-equivalent):

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential bc bison flex libelf-dev libssl-dev kmod \
  python3 python3-pip lcov qemu-system-x86 \
  ninja-build pkg-config libglib2.0-dev libpixman-1-dev \
  meson python3-venv busybox-static cpio gzip
pip3 install --user pytest pytest-cov   # optional, only for tools/tests/pytest.sh
```

The first list matches the `build-and-kunit` workflow; the second is the `vm-smoke` extras (QEMU build deps + initramfs builder). `qemu-system-x86` is the stock QEMU binary used by the kernel's `tools/testing/kunit/kunit.py`; the full-VM tests run against our own IPU4-patched QEMU built by `tools/build-qemu.sh`. After this list, every command in this file (and in `STATUS.md`) just works.

## Upstream sync tooling

Two helpers under `tools/upstream/`, both relying on the same file mapping (`drivers/media/pci/intel/ipu6/<f>` ↔ `kernel/ipu4/<f>`, IPU4-only files in `IPU4_LOCAL_ONLY` skipped):

- `tools/upstream/diff.sh` — regenerates `tools/notes/upstream-diff/{summary.md,per-file/<f>.diff}` showing the current divergence between `kernel/ipu4/` and upstream IPU6 at the pinned tag. Read this when picking the next divergence chunk to remove. Output is gitignored — re-run on demand. Honours `IPU4_LINUX_TAG` for one-off comparisons against a different upstream pin.
- `tools/upstream/watch.sh` — driven by `.github/workflows/upstream-watch.yml` (daily 04:00 UTC). Detects new IPU6 commits on `linux-6.12.y` (stable) and Linus `master` (deduped against 6.12.y by patch-id), tries `git am` of each onto `kernel/ipu4/`, and opens a PR on `claude/upstream-watch/<date>` listing applied / conflict / n-a commits. State is kept in `tools/notes/upstream-watch-state.json` and rolls forward in the last commit on the bot branch. Run locally with `IPU4_UPSTREAM_WATCH_DRY_RUN=1` to inspect the would-be PR without pushing.

## ipu6 flavor (upstream ipu6 driver + IPU4 patch series)

A second, parallel flow adds IPU4 to the upstream `drivers/media/pci/intel/ipu6/` driver as mailable patches. It never touches `tools/linux/` or `kernel/ipu4/`. Three tracks work on it in parallel, each with its own plan (re-read it before, and update it at the end of, every step) and its own line of PRs: the IPU4 port (`tools/notes/ipu6-ipu4-plan.md`, which also holds the test table), the IPU6 QEMU variant (`tools/notes/ipu6-variant-plan.md`) and the IPU4P QEMU variant (`tools/notes/ipu4p-variant-plan.md`):

```bash
tools/ipu6/bootstrap.sh       # tools/linux-ipu6/ = pin (tools/ipu6/pin) + git am patches/ipu6-ipu4/*.patch
tools/ipu6/build.sh           # intel-ipu6*.ko (W=1, M= build on modules_prepare)
tools/tests/ipu6/ladder.sh    # every test; fails only on tests listed in tools/tests/ipu6/enabled-tests.<driver>.<variant>
```

`IPU4_FLAVOR=ipu6 tools/bootstrap.sh` / `tools/build.sh` dispatch to the same scripts (default `fork`). Kernel changes go in `patches/ipu6-ipu4/` as `git format-patch` output (see its README); CI is `.github/workflows/vm-ipu6.yml`.

The ladder runs six legs, each a driver under a QEMU model variant, `IPU4_DRIVER=<driver> IPU4_VARIANT=<variant> tools/tests/ipu6/ladder.sh` (table in `ladder.sh`). The drivers are three unmodified baselines, which tests are developed against — `ipu4-out-of-tree` (IPU4: `kernel/ipu4/` built as an out-of-tree module against stock v7.3-rc4, the base of the other pins, `tools/out-of-tree/pin`, `IPU4_FLAVOR=out-of-tree`, plan `tools/notes/ipu4-out-of-tree-plan.md`), `ipu4p` (IPU4P: Ruslan's tree, `tools/ipu6/pin`, no series) and `ipu6` (IPU6: mainline) — and the upstreamable `ipu4-patches` (Ruslan's tree + `patches/ipu6-ipu4/`, the default). Each IPU (variant `ipu4`, `ipu4p`, `ipu6`) runs against its own baseline and against `ipu4-patches`. The guest inits and host checks take the driver's module, device and node names from `tools/rootfs/ipu-driver.<flavor>.conf`. `ipu4-out-of-tree` is known to work, so it gates every implemented test: a failure there points at the test, harness or model.

To keep the tracks out of each other's way ("Tracks and file ownership" in the IPU4 port's plan): every ladder test is its own file, `tools/tests/ipu6/tests.d/NNN-<test>.sh`, with its guest-side probe checks in `tools/rootfs/init.ipu6-probe.d/NNN-<test>.{bound,unloaded}.sh` (READMEs in both); a new test adds files, it never edits a shared list. A track edits only its own plan and its own `enabled-tests[.<variant>]`. Changes to the shared core (`ladder.sh`, `probe-boot.sh`, `stream-boot.sh`, the `init.ipu6-*` frames, `tools/tests/streamon.c`, `run-vm.sh`, `rootfs/build.sh`, `vm-ipu6.yml`) go in their own commit, ahead of the feature commits that need them; the PR may carry both, but each commit must pass CI on its own.

## Common commands

Out-of-tree build against an external kernel tree:

```bash
make -C kernel/ipu4 KERNEL_SRC=/path/to/linux            # build intel-ipu4.ko + intel-ipu4-isys.ko
make -C kernel/ipu4 KERNEL_SRC=/path/to/linux checkpatch # upstream checkpatch, --strict, 80col
make -C kernel/ipu4 clean
```

In-tree build via the QEMU harness (preferred for local iteration):

```bash
tools/bootstrap.sh          # clones tools/linux/ @ v6.12 and tools/qemu/ @ v9.1.0, seeds patches + driver
tools/build.sh              # configures kconfig and builds intel-ipu4.ko in tools/linux/
tools/tests/kunit.sh        # Tier 1: KUnit suites (ipu4_format, ipu4_bayer) under qemu-kvm via kunit.py
```

Full-VM test loop (the same set CI's `vm-smoke` runs — fast on warm caches, ~5 min, dramatically shorter feedback than push-and-wait CI):

```bash
tools/bootstrap.sh && tools/build-qemu.sh && tools/build-kernel.sh && tools/rootfs/build.sh
IPU4_ACCEL=tcg tools/tests/streamon-smoke.sh    # boot VM, walk v4l2 capture API end-to-end
IPU4_ACCEL=tcg tools/tests/mmiotrace.sh          # rerun under mmiotrace, capture qemu.trace
tools/tests/compare-mmio.sh                       # diff against silicon's data/trace.txt
```

`tools/build.sh` uses `KBUILD_MODPOST_WARN=1`: undefined-symbol errors are downgraded to warnings because vmlinux is not built here. Those symbols resolve at `insmod` time inside the guest VM.

The bootstrap script is idempotent. Re-run after pulling to re-apply patches from `tools/{linux,qemu}-patches/`. Override the upstream URLs/tags via env vars `IPU4_LINUX_URL`, `IPU4_LINUX_TAG`, `IPU4_QEMU_URL`, `IPU4_QEMU_TAG`.

## CI

- **PR job selection.** `ci.yml`, `vm-smoke.yml` and `vm-ipu6.yml` each start with the reusable `.github/workflows/changes.yml`, which runs `tools/ci/changes.sh` over the PR's diff and emits one flag per area (`kunit`, `pytest`, `vm_smoke`, `ipu6`). Jobs whose area wasn't touched are skipped *inside* the job (reusable-workflow `run` input, per-step `if:` on the vm-ipu6 matrix) so every check keeps its name and reports success-by-skip; docs-only PRs run nothing heavy. Push to `main`/`master` and `workflow_dispatch` always run everything. When a job starts reading a new file, add it to the path table in `tools/ci/changes.sh` (and a case to `tests/test_ci_changes.py`) — an unlisted path selects nothing.
- `.github/workflows/ci.yml` — bootstrap + build + kunit on every PR and on push to `main`/`master`. A `pins` setup job reads `.github/kernel-pins/` and emits a matrix-shaped output; `build-and-kunit` then fans across `fail-fast: false` so per-PR CI is deterministic. The 6.18 leg is expected red until the corresponding compat shims land in `kernel/ipu4/ipu4-compat.h`. The 7.3 leg (v7.3-rc4) builds clean on the existing shim cascade.
- `.github/workflows/build-and-kunit.yml` — reusable workflow (`workflow_call`) that owns the actual checkout → setup-harness → pytest → bootstrap → build → kunit pipeline. Inputs: `linux-url`, `linux-ref`, `display-name`.
- `.github/workflows/bump-kernel-pins.yml` — Monday 05:30 UTC cron that resolves the latest stable point release for each kernel track and rewrites the matching `.github/kernel-pins/<key>.json` (one file per track; `ci.yml`, `vm-smoke.yml`, and `vm-smoke-weekly.yml` all glob that directory at workflow start). Opens **one PR per track** via `peter-evans/create-pull-request` when a value changed. The matrix is derived from the directory itself by a `pins` setup job, so adding a track is a one-file PR (drop a new `<key>.json` under `.github/kernel-pins/`). One file per track also means parallel bot PRs touching different tracks never conflict on each other on rebase. The script that does the actual rewrite is `tools/bump-kernel-pins.sh` (accepts `--key <track>`). Note: bot-opened PRs don't fire CI under the default `GITHUB_TOKEN`; set a `BUMP_PAT` secret to lift that, or close-and-reopen the PR by hand.
- `.github/actions/setup-harness/action.yml` — composite action shared by every workflow. Owns the apt-package list and the optional pytest pip install (`install-pip: "true"`).
- `.github/workflows/vm-smoke.yml` — full-VM boot + probe-smoke + streamon-smoke + mmiotrace + `compare-mmio` divergence report on every PR and on push to `main`/`master`. Pinned to the **6.12 leg only** (read from `.github/kernel-pins/`, the same source `ci.yml` uses); 6.18 and 7.0 run weekly via `vm-smoke-weekly.yml`. A thin `workflow_call` caller — the body lives in `vm-smoke-reusable.yml`.
- `.github/workflows/vm-smoke-weekly.yml` — Sundays 07:00 UTC + `workflow_dispatch`. Matrix caller covering every non-6.12 track in `.github/kernel-pins/` (today: 6.18 + 7.3).
- `.github/workflows/vm-smoke-reusable.yml` — reusable workflow (`workflow_call`) shared by `vm-smoke.yml` and `vm-smoke-weekly.yml`. Inputs: `linux-url`, `linux-ref`, `display-name`. The Linux build cache is namespaced by `display-name` so the 6.12 and 6.18 callers don't trample each other; the QEMU cache (independent of the kernel ref) is shared. Failure artifacts (`vm-smoke-failure-<display-name>-*`) include the serial logs; the coverage report (`mmio-trace-coverage-vm-smoke-<display-name>-*`) is published unconditionally.
- `.github/workflows/vm-ipu6.yml` — the test ladder (`tools/tests/ipu6/ladder.sh`) on every PR and on push to `main`/`master`, six matrix legs: each QEMU variant under its baseline driver (`ipu4-out-of-tree / ipu4`, `ipu4p / ipu4p`, `ipu6 / ipu6`) and under `ipu4-patches`, checks `ladder (<driver> / <variant>)`; each leg is gated only by its list (`tools/tests/ipu6/enabled-tests.<driver>.<variant>`). See "ipu6 flavor" above.
- `.github/workflows/pages.yml` — renders the dashboard (`tools/dashboard/`, build steps in the `.github/actions/build-dashboard` composite) and deploys it to GitHub Pages on push to `main`/`master`, `vm-smoke` completion, a daily cron and `workflow_dispatch`. Each deploy also publishes open PRs' previews from the `pages-previews` branch under `/pr-preview/pr-<N>/`.
- `.github/workflows/pages-preview.yml` — on every same-repo PR push, renders the dashboard from the PR head, parks it on `pages-previews` via `tools/dashboard/previews.sh`, dispatches `pages.yml` on the default branch, and comments the preview URL; closing the PR removes it. See `tools/dashboard/README.md`.
- `.github/workflows/upstream-watch.yml` — daily cron that surfaces new upstream IPU6 commits as cherry-pick PRs (see "Upstream sync tooling" above).

## QEMU device-model workflow

The QEMU IPU4 device model (`tools/qemu-patches/hw/misc/ipu4.c`) is the part of the harness most likely to need changes. A few project-specific things that aren't obvious:

- **The mmiotrace coverage report drives the worklist.** `tools/tests/out/compare-mmio/report.txt` (also published as `mmio-trace-coverage-vm-smoke-*` in CI) classifies silicon-vs-QEMU divergence into `unimplemented` (silicon touches an address we don't handle) and `value_mismatch` (we handle it but values differ). Each model change targets a specific row; after merge the row should drop or reclassify. `tools/notes/registers.md` is the source of truth for what every implemented handler does — add a row when you add a handler.
- **`fprintf(stderr, …)` is the only QEMU debug output that survives CI.** `tools/run-vm.sh` runs QEMU without `-d unimp`, so `qemu_log_mask(LOG_UNIMP, …)` calls go to `/dev/null`. To make a diagnostic visible in the failure-artifact serial log, use `fprintf(stderr, …)`. Convert to `qemu_log_mask(LOG_UNIMP, …)` once a behaviour is stable; PRs shouldn't merge with `fprintf` left behind.
- **Driver-provided "DMA addresses" are IPU IOVAs, not host-physical.** To `pci_dma_read` / `pci_dma_write` against them, walk the IPU MMU page tables: L1 PT pfn latched from `BAR+0x1e0004` (sub-block 0 of the ISYS MMU window), 22-bit L1 idx, 10-bit L2 idx, 12-bit page offset; both PT levels store 27-bit pfns. The walker is `ipu4_iova_to_phys()` in `tools/qemu-patches/hw/misc/ipu4.c`. Multi-page accesses must walk per-page (`vb2_dma_sg_memops` makes buffers physically discontiguous).
- **Multi-PR rollout discipline against `qemu-kunit-setup`.** Each milestone is a single squashed commit against `origin/qemu-kunit-setup`. After a parent PR merges, rebase the next branch onto the new merged tip — the merge SHA differs from the local pre-merge SHA — with `git rebase --onto origin/qemu-kunit-setup <old-base>` and force-push. Each step must (a) leave the workflow it touches green in vm-smoke, and (b) shrink the divergence report visibly, so a regression at any step is local and undo-able.
- **VMState versioning is loose pre-release.** The `qemu-kunit-setup` branch isn't tagged. Bumping `version_id` and removing fields is fine in development; back-compat shims aren't needed until a release branch lands.

## Known in-progress work

See `STATUS.md` for milestone state. Key things to know about what tests actually run:

- `tools/tests/kunit.sh` runs `ipu4_format` and `ipu4_bayer` KUnit suites via the kernel's `tools/testing/kunit/kunit.py` (no separate VM boot needed; kunit.py runs them under qemu-kvm). The parent `drivers/media/pci/intel/Kconfig` is patched idempotently by `tools/bootstrap.sh`, so `CONFIG_VIDEO_INTEL_IPU4=y` is wired.
- `tools/tests/e2e.sh` is a tier-2 placeholder — the live full-VM path is `tools/tests/streamon-smoke.sh` (gated by `IPU4_STREAM_REQUIRED`, default `STREAM:pattern_ok` end-to-end) plus `tools/tests/mmiotrace.sh` and `tools/tests/compare-mmio.sh`.
- `tools/notes/registers.md` rows are no longer mostly `guess` — most behaviour is `inferred` against silicon's `data/trace.txt`. The remaining `value_mismatch` rows (PWR_STATE FSM transitional values, TSC clock skew, SPC_STATUS_CTRL silicon-specific bits, ring-cursor counts) are intrinsic divergence, not missing handlers.

## Legacy hardware trace scripts

`trace_ipu4.sh`, `trace_functions.sh`, `split_trace.sh`, `postprocess_trace.py` are mmiotrace helpers meant to run on real target hardware (require `kernel/configs/mmiotrace.config` and `ambu-tc358748`). They're kept as reference for porting to other devices; they won't run in the QEMU harness.
