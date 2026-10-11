# tools/dashboard/

Renders a single-page GitHub Pages site with the project's current
state: the test ladder per leg, the supported kernel tracks with their
latest build + KUnit result, latest CI runs and, collapsed, the
out-of-tree harness's coverage, MMIO divergence, register coverage and
upstream divergence.

Deployed by `.github/workflows/pages.yml` via `actions/deploy-pages`
on every push to `main`, every `vm-smoke` / `vm-ipu6` completion, and
a daily 06:30 UTC cron. Coverage and MMIO data come from the latest
*successful* `vm-smoke` run on the default branch; the test
ladder comes from the latest *completed* `vm-ipu6` run there that has
`ipu6-ladder-results-*` artifacts, whatever its conclusion (a failing
gated test is status worth showing). PR previews look at the PR's own
branch first for both and fall back to the default branch. Both
downloads live in `.github/actions/build-dashboard/`.

## How history works

`actions/deploy-pages` is stateless — there's no persistent branch.
The dashboard's only piece of carried-forward state is
`data/history.json`, and the workflow re-reads it from the previously
published Pages URL (`https://<owner>.github.io/<repo>/data/history.json`)
before each rebuild. First-time deploys (404) start with an empty
history.

`build.sh` reads `$OUT_DIR/data/history.json` (if present), appends a
new datapoint with the current SHA / coverage % / MMIO divergence
counts, and writes it back into the new artifact. Multiple triggers
on the same SHA (push + vm-smoke completion + cron) refresh the same
point rather than appending duplicates. History is capped at 365
points (one year of daily refreshes plus per-push events).

## PR previews

`.github/workflows/pages-preview.yml` renders the dashboard from each
same-repo PR's head and publishes it to a temporary area,
`https://<owner>.github.io/<repo>/pr-preview/pr-<N>/`, with a sticky
PR comment linking to it. Coverage / MMIO sections come from the PR
branch's newest successful `vm-smoke` run (the default branch's if
the PR has none yet).

`deploy-pages` replaces the whole site on every deploy and the
`github-pages` environment only accepts the default branch, so the
PR job can't deploy itself. Instead `previews.sh publish` parks the
rendered site on the `pages-previews` branch (one `pr-<N>/` directory
per PR, a single force-pushed orphan commit, so no history piles up)
and dispatches `pages.yml` on the default branch. Every production
deploy runs `previews.sh merge`, which copies that branch into
`site/pr-preview/`, skipping PRs that are no longer open. Closing a
PR runs `previews.sh remove` and redeploys. Fork PRs are skipped
(their token can't push).

## One-time Pages setup

In repo Settings → Pages, set **Source: GitHub Actions**. No `gh-pages`
branch is involved; `pages-previews` is only a parking spot that
`pages.yml` reads from, not a Pages source.

## Inputs

The dashboard is a contract over files already produced elsewhere:

| Input | Producer | Required? |
|---|---|---|
| `tools/coverage/summary.json` | `tools/coverage/collect.sh` | optional (renders "no data" when missing) |
| `tools/coverage/html/` | `tools/coverage/collect.sh` (genhtml) | optional |
| `tools/tests/out/compare-mmio/report.json` | `tools/tests/compare-mmio.sh` | optional |
| `tools/tests/out/compare-mmio/report.txt` | same | optional |
| `tools/tests/out/streamon.summary` | `tools/tests/streamon-smoke.sh` | optional |
| `tools/tests/out/ipu6-ladder-results/**/results.json` (`$IPU6_LADDER_DIR`) | `tools/tests/ipu6/ladder.sh`, via the `ipu6-ladder-results-<leg>` artifacts of `vm-ipu6` | optional (matrix renders "not run yet") |
| `tools/tests/out/ipu6-ladder/results.json` | a local `tools/tests/ipu6/ladder.sh` run | optional |
| `tools/tests/ipu6/ladder.sh --list` | committed | optional (row names before any results exist) |
| `tools/notes/registers.md` | committed | yes |
| `tools/notes/upstream-watch-state.json` | `tools/upstream/watch.sh` (committed) | yes |
| `tools/notes/upstream-diff/summary.md` | `tools/upstream/diff.sh` (regenerated each pages run) | optional |
| `.github/kernel-pins/*.json` | committed (`bump-kernel-pins.yml`) | optional |
| GitHub Actions API | `gh api` at build time | optional |

## Test ladder section (`results.json`)

`tools/tests/ipu6/ladder.sh` writes `tools/tests/out/ipu6-ladder/results.json`
at the end of every run (schema 1):

```jsonc
{
  "schema": 1,
  "leg": "ipu4-patches/ipu4",     // "<driver>/<variant>"
  "driver": "ipu4-patches",       // $IPU4_DRIVER: ipu4-out-of-tree | ipu4p | ipu6 | ipu4-patches
  "variant": "ipu4",              // $IPU4_VARIANT (QEMU model): ipu4 | ipu4p | ipu6
  "generated_at": "2026-10-10T12:14:40Z",
  "commit_sha": "…", "run_id": "…", "run_url": "…",   // null outside CI
  "pin": {"url": "https://github.com/ruslanbay/linux.git", "sha": "…"},
  "series": {"count": 1, "patches": [{"file": "0001-….patch", "subject": "media: ipu6: …"}]},
  "enabled_tests": "tools/tests/ipu6/enabled-tests.ipu4-patches.ipu4",
  "exit_status": 0,
  "tests": [
    {"name": "build", "depends_on": "series-applies", "enabled": true,
     "result": "PASS", "duration_s": 27}
  ]
}
```

`result` is one of `PASS`, `FAIL`, `SKIP`, `NOT-IMPLEMENTED` or `NOT-RUN`
(test not selected on the command line); `enabled` means the test is
listed in the leg's `enabled-tests.<driver>.<variant>` and gates CI.
`parse_ladder.py` keeps the newest file per `leg` and always shows one
column per leg (each IPU under its baseline and under ipu4-patches,
six), so a leg without data reads "not run yet"; the pin and series block describes `ipu4-patches/ipu4`. Files
without `leg` (written before legs existed) count by their `variant`:
`ipu4` as `ipu4-patches/ipu4`, `ipu6` as `ipu6/ipu6`.

## Local render

```bash
# (Optional) populate fresh inputs:
tools/upstream/diff.sh

# Render to /tmp/site/. No vm-smoke artifacts → coverage / mmio
# sections render as "no data yet" placeholders. A local
# tools/tests/ipu6/ladder.sh run fills its leg's column of the IPU4
# port section; IPU6_LADDER_DIR=<dir> points at downloaded
# ipu6-ladder-results-* artifacts instead.
OUT_DIR=/tmp/site tools/dashboard/build.sh
xdg-open /tmp/site/index.html
```

The script tolerates missing inputs so a checkout without a recent
`vm-smoke` run still produces a viewable page.

## File map

| File | Role |
|---|---|
| `build.sh` | Top-level orchestrator. Calls the Python modules in order. |
| `previews.sh` | PR previews on the `pages-previews` branch: publish / remove / merge / comment. |
| `build_payload.py` | Gathers all inputs into a single JSON dict for the renderer. |
| `render.py` | Jinja2 → `index.html`, copies CSS, writes `data/history.json`. |
| `parse_registers.py` | `tools/notes/registers.md` → `{rows, counts}`. Tested. |
| `parse_kernels.py` | `.github/kernel-pins/` + latest `ci` run jobs → kernel tracks with status. Tested. |
| `parse_ladder.py` | ipu6 ladder `results.json` files → test × variant matrix. Tested. |
| `templates/index.html.j2` | Single page. Embeds Chart.js from CDN; no JS build step. |
| `templates/style.css` | Dark-mode CSS, no framework. |

## Adding a new section

1. Decide whether the section consumes a committed file (cheap, no
   contract change) or a CI artifact (touch
   `.github/actions/build-dashboard/action.yml` to download it).
2. Add a parser in `tools/dashboard/` if needed. Keep parsers
   stateless and unit-testable — see `parse_registers.py` for the
   shape.
3. Wire the parser into `build_payload.py`. Add the corresponding key
   to the `payload` dict.
4. Add a `<section class="card">` block in `templates/index.html.j2`.
5. If the section has a trend, include the relevant fields in the
   `_append_history()` datapoint and add a `lineCfg(...)` call in the
   inline JS at the bottom of the template.
