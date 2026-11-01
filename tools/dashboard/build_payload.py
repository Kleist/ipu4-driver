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
import parse_ladder  # noqa: E402
import parse_kernels  # noqa: E402
import parse_registers  # noqa: E402


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


def _ci_jobs(repo: str) -> tuple[list[dict] | None, str | None]:
    """Jobs and URL of the latest completed ``ci`` run on the default
    branch, or ``(None, None)`` without ``gh``."""
    if not _have("gh"):
        return None, None
    runs = _gh_json([
        "api",
        f"repos/{repo}/actions/workflows/ci.yml/runs?event=push&status=completed&per_page=1",
        "--jq", ".workflow_runs",
    ]) or []
    if not runs:
        return None, None
    run = runs[0]
    jobs = _gh_json([
        "api", f"repos/{repo}/actions/runs/{run['id']}/jobs?per_page=100",
        "--jq", ".jobs",
    ])
    return jobs, run.get("html_url")


def _kernels(root: Path, repo: str) -> dict:
    jobs, run_url = _ci_jobs(repo)
    pins = parse_kernels.load_pins(root / ".github" / "kernel-pins")
    return {
        "tracks": parse_kernels.build(pins, jobs),
        "run_url": run_url,
        "workflow_url": f"https://github.com/{repo}/actions/workflows/ci.yml",
    }


def _have(prog: str) -> bool:
    from shutil import which
    return which(prog) is not None


def _gh_json(args: list[str]):
    try:
        out = subprocess.check_output(["gh", *args], stderr=subprocess.DEVNULL)
        return json.loads(out)
    except (subprocess.CalledProcessError, json.JSONDecodeError, FileNotFoundError):
        return None


def _ladder_tests(root: Path) -> list[str]:
    """Test names from ``ladder.sh --list``, so the IPU4-port matrix
    has its rows even before any vm-ipu6 run published results."""
    ladder = root / "tools" / "tests" / "ipu6" / "ladder.sh"
    if not ladder.is_file():
        return []
    try:
        out = subprocess.check_output(
            ["bash", str(ladder), "--list"], stderr=subprocess.DEVNULL, timeout=30,
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError):
        return []
    return [line.strip() for line in out.decode().splitlines() if line.strip()]


def _ipu6_port(root: Path, repo: str, ladder_dirs: list[str], run_id: str | None,
               ladder_cov_dir: str) -> dict:
    port = parse_ladder.build(
        parse_ladder.load_results(ladder_dirs), _ladder_tests(root),
    )
    primary = port["primary"] or {}
    if run_id:
        run_url = f"https://github.com/{repo}/actions/runs/{run_id}"
    else:
        run_url = primary.get("run_url")
    port["run_url"] = run_url
    port["workflow_url"] = f"https://github.com/{repo}/actions/workflows/vm-ipu6.yml"
    port["plan_url"] = f"https://github.com/{repo}/blob/main/tools/notes/ipu6-ipu4-plan.md"
    # build.sh publishes each leg's lcov HTML (ipu6-ladder-coverage-*
    # artifacts) at ladder-coverage/<driver>-<variant>/.
    for leg in port["legs"]:
        slug = leg["name"].replace("/", "-")
        if (Path(ladder_cov_dir) / slug / "html" / "index.html").is_file():
            leg["coverage_url"] = f"ladder-coverage/{slug}/"
    return port


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
        # Unexplained divergence from the convergence target tree.
        "target_divergence": (((payload.get("upstream") or {}).get("history") or {})
                              .get("target") or {}).get("divergence"),
        # Driver line coverage per ladder leg ("<driver>/<variant>": pct).
        "ladder_coverage": {
            leg["name"]: ((leg.get("coverage") or {}).get("lines") or {}).get("pct")
            for leg in (payload.get("ipu6_port") or {}).get("legs") or []
            if leg.get("coverage")
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
    p.add_argument("--repo", required=True, help="GitHub owner/name slug")
    p.add_argument("--vm-smoke-run-id", default=None)
    p.add_argument("--ipu6-ladder-dir", action="append", default=[],
                   help="dir searched for ladder results*.json (repeatable)")
    p.add_argument("--vm-ipu6-run-id", default=None)
    p.add_argument("--ipu6-ladder-coverage-dir", default="",
                   help="dir of per-leg ladder coverage reports (<driver>-<variant>/)")
    args = p.parse_args(argv)

    root = Path(args.root)
    out_dir = Path(args.out_dir)
    cov_dir = Path(args.cov_dir)
    mmio_dir = Path(args.mmio_dir)

    coverage_summary = _read_json(cov_dir / "summary.json")
    mmio_report = _read_json(mmio_dir / "report.json")

    registers_md = root / "tools" / "notes" / "registers.md"
    registers = parse_registers.parse(registers_md.read_text()) if registers_md.is_file() else {"rows": [], "counts": {"doc": 0, "inferred": 0, "guess": 0}, "total": 0}

    upstream_diff = _parse_upstream_diff(root / "tools" / "notes" / "upstream-diff" / "summary.md")
    # tools/upstream/divergence_history.py: divergence per release tag and
    # sampled driver commit, for the trend chart.
    upstream_history = _read_json(root / "tools" / "notes" / "upstream-diff" / "history.json")
    upstream_watch = _read_json(root / "tools" / "notes" / "upstream-watch-state.json") or {
        "last_seen": {}, "processed_patch_ids": [],
    }

    payload = {
        "generated_at": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "commit_sha": _commit_sha(root),
        "repo": args.repo,
        "coverage": {"summary": coverage_summary},
        "mmio": {"report": mmio_report},
        "registers": registers,
        "kernels": _kernels(root, args.repo),
        "upstream": {"diff": upstream_diff, "watch": upstream_watch,
                     "history": upstream_history},
        "ipu6_port": _ipu6_port(root, args.repo, args.ipu6_ladder_dir, args.vm_ipu6_run_id,
                                args.ipu6_ladder_coverage_dir),
    }
    payload["history"] = _append_history(out_dir, payload, args.vm_smoke_run_id)

    json.dump(payload, sys.stdout, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
