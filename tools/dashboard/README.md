# tools/dashboard/

Renders a single-page GitHub Pages site with the project's "current
state": code coverage, MMIO divergence, register coverage, upstream
divergence + watcher state, latest CI runs, and STATUS.md milestones.

Deployed by `.github/workflows/pages.yml` on every push to `main` and
on every successful `vm-smoke` completion (so the coverage / mmio
sections refresh as soon as the artifacts they consume become
available). The site source is the `gh-pages` branch.

## Inputs

The dashboard is a contract over files already produced elsewhere:

| Input | Producer | Required? |
|---|---|---|
| `tools/coverage/summary.json` | `tools/coverage/collect.sh` | optional (renders "no data" when missing) |
| `tools/coverage/html/` | `tools/coverage/collect.sh` (genhtml) | optional |
| `tools/tests/out/compare-mmio/report.json` | `tools/tests/compare-mmio.sh` | optional |
| `tools/tests/out/compare-mmio/report.txt` | same | optional |
| `tools/tests/out/streamon.summary` | `tools/tests/streamon-smoke.sh` | optional |
| `tools/notes/registers.md` | committed | yes |
| `tools/notes/upstream-watch-state.json` | `tools/upstream/watch.sh` (committed) | yes |
| `tools/notes/upstream-diff/summary.md` | `tools/upstream/diff.sh` (regenerated each pages run) | optional |
| `STATUS.md` | committed | yes |
| GitHub Actions API | `gh api` at build time | optional |

## Local render

```bash
# (Optional) populate fresh inputs:
tools/upstream/diff.sh

# Render to /tmp/site/. No vm-smoke artifacts → coverage / mmio
# sections render as "no data yet" placeholders.
OUT_DIR=/tmp/site tools/dashboard/build.sh
xdg-open /tmp/site/index.html
```

The script tolerates missing inputs so a checkout without a recent
`vm-smoke` run still produces a viewable page.

## How history works

`build.sh` reads `$OUT_DIR/data/history.json` (if present), appends a
new datapoint with the current SHA / coverage % / MMIO divergence
counts, and writes it back. The pages workflow checks out the
`gh-pages` branch into `site/` before running `build.sh`, so each
deployment carries forward the prior history. Multiple triggers on the
same SHA (push + vm-smoke completion + cron) refresh the same point
rather than appending duplicates.

History is capped at 365 points (one year of daily refreshes plus
per-push events).

## File map

| File | Role |
|---|---|
| `build.sh` | Top-level orchestrator. Calls the Python modules in order. |
| `build_payload.py` | Gathers all inputs into a single JSON dict for the renderer. |
| `render.py` | Jinja2 → `index.html`, copies CSS, writes `data/history.json`. |
| `parse_registers.py` | `tools/notes/registers.md` → `{rows, counts}`. Tested. |
| `parse_status.py` | `STATUS.md` → milestone list with state classification. Tested. |
| `templates/index.html.j2` | Single page. Embeds Chart.js from CDN; no JS build step. |
| `templates/style.css` | Dark-mode CSS, no framework. |

## Adding a new section

1. Decide whether the section consumes a committed file (cheap, no
   contract change) or a CI artifact (touch `pages.yml` to download
   it).
2. Add a parser in `tools/dashboard/` if needed. Keep parsers
   stateless and unit-testable — see `parse_registers.py` for the
   shape.
3. Wire the parser into `build_payload.py`. Add the corresponding key
   to the `payload` dict.
4. Add a `<section class="card">` block in `templates/index.html.j2`.
5. If the section has a trend, include the relevant fields in the
   `_append_history()` datapoint and add a `lineCfg(...)` call in the
   inline JS at the bottom of the template.
