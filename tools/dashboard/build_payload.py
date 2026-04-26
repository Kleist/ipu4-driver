"""Assemble the JSON payload that render.py consumes.

Pulls together every input the dashboard needs and writes a single
JSON document to stdout. Inputs that don't exist degrade silently to
``null`` so first-time bring-up doesn't fail before vm-smoke has ever
published an artifact. Also responsible for appending a new datapoint
to the trend history (read from ``$OUT_DIR/data/history.json`` if it
exists).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
import subprocess
import sys
from pathlib import Path

# Make the parsers importable when this script lives next to them.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import parse_registers  # noqa: E402
import parse_status  # noqa: E402


HISTORY_CAP = 365  # one year of daily refreshes + per-push events


def _read_json(path: Path):
    if not path.is_file():
        return None
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        return None


def _commit_sha(root: Path) -> str | None:
    try:
        out = subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            stderr=subprocess.DEVNULL,
        )
        return out.decode().strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def _read_streamon_marker(path: Path) -> str | None:
    if not path.is_file():
        return None
    text = path.read_text().strip()
    return text or None


_DIFF_TOTAL_RE = re.compile(
    r"^\|\s*\*\*total\*\*\s*\|\s*\*\*(?P<hunks>\d+)\*\*\s*\|"
    r"\s*\*\*(?P<plus>\d+)\*\*\s*\|\s*\*\*(?P<minus>\d+)\*\*",
    re.MULTILINE,
)
_DIFF_PIN_RE = re.compile(r"^Upstream pin:\s*`([^`]+)`", re.MULTILINE)


def _parse_upstream_diff(summary_md: Path) -> dict | None:
    if not summary_md.is_file():
        return None
    text = summary_md.read_text()
    pin = _DIFF_PIN_RE.search(text)
    total = _DIFF_TOTAL_RE.search(text)
    if not total:
        return None
    files_differ = sum(
        1
        for line in text.splitlines()
        if line.startswith("| `") and "_identical_" not in line
    )
    return {
        "upstream_pin": pin.group(1) if pin else "unknown",
        "files_differ": files_differ,
        "hunks": int(total.group("hunks")),
        "plus": int(total.group("plus")),
        "minus": int(total.group("minus")),
    }


def _gh_workflows(repo: str) -> list[dict]:
    """Best-effort: query GitHub for the latest run of every workflow.

    Returns an empty list if `gh` is unavailable or unauthenticated —
    the dashboard renders an "no runs found" placeholder in that case.
    Each workflow shows the most recent run regardless of conclusion;
    failed runs are visually distinguished in the template.
    """
    if not _have("gh"):
        return []
    workflows = _gh_json(
        ["api", f"repos/{repo}/actions/workflows", "--jq", ".workflows"]
    ) or []
    out = []
    for wf in workflows:
        # Skip disabled / state==active filter — surface everything that
        # currently has at least one run.
        runs = _gh_json([
            "api",
            f"repos/{repo}/actions/workflows/{wf['id']}/runs?per_page=1",
            "--jq", ".workflow_runs",
        ]) or []
        if not runs:
            continue
        run = runs[0]
        out.append({
            "name": wf["name"],
            "url": run.get("html_url"),
            "conclusion": run.get("conclusion") or run.get("status"),
            "created_at": run.get("created_at"),
            "relative": _relative_time(run.get("created_at")),
            "duration": _duration(run.get("created_at"), run.get("updated_at")),
        })
    return out


def _have(prog: str) -> bool:
    from shutil import which
    return which(prog) is not None


def _gh_json(args: list[str]):
    try:
        out = subprocess.check_output(["gh", *args], stderr=subprocess.DEVNULL)
        return json.loads(out)
    except (subprocess.CalledProcessError, json.JSONDecodeError, FileNotFoundError):
        return None


def _relative_time(ts: str | None) -> str | None:
    if not ts:
        return None
    try:
        when = _dt.datetime.fromisoformat(ts.replace("Z", "+00:00"))
    except ValueError:
        return None
    delta = _dt.datetime.now(_dt.timezone.utc) - when
    if delta.days >= 1:
        return f"{delta.days}d ago"
    hours = delta.seconds // 3600
    if hours:
        return f"{hours}h ago"
    minutes = max(1, delta.seconds // 60)
    return f"{minutes}m ago"


def _duration(start: str | None, end: str | None) -> str | None:
    if not (start and end):
        return None
    try:
        a = _dt.datetime.fromisoformat(start.replace("Z", "+00:00"))
        b = _dt.datetime.fromisoformat(end.replace("Z", "+00:00"))
    except ValueError:
        return None
    secs = int((b - a).total_seconds())
    if secs < 0:
        return None
    if secs < 60:
        return f"{secs}s"
    return f"{secs // 60}m{secs % 60:02d}s"


def _append_history(out_dir: Path, payload: dict, vm_smoke_run_id: str | None) -> dict:
    history_path = out_dir / "data" / "history.json"
    history = _read_json(history_path) or {"schema": 1, "points": []}
    points = history.get("points") or []

    cov = (payload.get("coverage") or {}).get("summary") or {}
    mmio = (payload.get("mmio") or {}).get("report") or {}
    regs = payload.get("registers") or {}
    diff = (payload.get("upstream") or {}).get("diff") or {}

    new_point = {
        "ts": payload["generated_at"],
        "sha": payload.get("commit_sha"),
        "vm_smoke_run_id": vm_smoke_run_id,
        "coverage": {
            "lines_pct": ((cov.get("lines") or {}).get("pct")),
            "functions_pct": ((cov.get("functions") or {}).get("pct")),
        },
        "mmio": {
            "unimplemented": len(mmio.get("unimplemented") or []) if mmio else None,
            "value_mismatch": len(mmio.get("value_mismatches") or []) if mmio else None,
            "extra_in_qemu": len(mmio.get("extra_in_qemu") or []) if mmio else None,
        },
        "registers": (regs.get("counts") or {}),
        "upstream_diff": {
            "files_differ": diff.get("files_differ"),
            "hunks": diff.get("hunks"),
        },
    }

    # Idempotency: a single commit can trigger pages.yml multiple times
    # (push, vm-smoke completion, daily cron). Refresh the existing
    # entry for that SHA instead of accumulating duplicates.
    if points and points[-1].get("sha") and points[-1]["sha"] == new_point["sha"]:
        points[-1] = new_point
    else:
        points.append(new_point)

    points = points[-HISTORY_CAP:]
    history["points"] = points
    history["schema"] = 1
    return history


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--root", required=True, help="repo root")
    p.add_argument("--out-dir", required=True, help="dashboard output dir (for prior history)")
    p.add_argument("--cov-dir", required=True, help="tools/coverage/ dir")
    p.add_argument("--mmio-dir", required=True, help="tools/tests/out/compare-mmio/ dir")
    p.add_argument("--streamon-summary", required=True, help="streamon.summary file")
    p.add_argument("--repo", required=True, help="GitHub owner/name slug")
    p.add_argument("--vm-smoke-run-id", default=None)
    args = p.parse_args(argv)

    root = Path(args.root)
    out_dir = Path(args.out_dir)
    cov_dir = Path(args.cov_dir)
    mmio_dir = Path(args.mmio_dir)

    coverage_summary = _read_json(cov_dir / "summary.json")
    mmio_report = _read_json(mmio_dir / "report.json")

    registers_md = root / "tools" / "notes" / "registers.md"
    registers = parse_registers.parse(registers_md.read_text()) if registers_md.is_file() else {"rows": [], "counts": {"doc": 0, "inferred": 0, "guess": 0}, "total": 0}

    status_md = root / "STATUS.md"
    milestones = parse_status.parse(status_md.read_text()) if status_md.is_file() else {"milestones": [], "counts": {}, "total": 0}

    upstream_diff = _parse_upstream_diff(root / "tools" / "notes" / "upstream-diff" / "summary.md")
    upstream_watch = _read_json(root / "tools" / "notes" / "upstream-watch-state.json") or {
        "last_seen": {}, "processed_patch_ids": [],
    }

    payload = {
        "generated_at": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "commit_sha": _commit_sha(root),
        "repo": args.repo,
        "streamon_marker": _read_streamon_marker(Path(args.streamon_summary)),
        "ci": {"workflows": _gh_workflows(args.repo)},
        "coverage": {"summary": coverage_summary},
        "mmio": {"report": mmio_report},
        "registers": registers,
        "milestones": milestones,
        "upstream": {"diff": upstream_diff, "watch": upstream_watch},
    }
    payload["history"] = _append_history(out_dir, payload, args.vm_smoke_run_id)

    json.dump(payload, sys.stdout, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
