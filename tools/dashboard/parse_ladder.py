"""Collect ipu6 test-ladder results into a rung x variant matrix.

``tools/tests/ipu6/ladder.sh`` writes one ``results.json`` per run
(schema 1, see tools/dashboard/README.md). The vm-ipu6 workflow uploads
it as ``ipu6-ladder-results-<variant>``; pages.yml downloads every such
artifact of the latest vm-ipu6 run into one directory. This module
finds those files, keeps the newest per variant and lays them out for
the "IPU4 port (upstream ipu6)" section of the dashboard.

Variants in ``VARIANTS`` always get a column, so a variant the parallel
track has not wired up yet renders as "not run yet" instead of being
hidden. Unknown variants found in the data are appended after them.
"""
from __future__ import annotations

import json
from pathlib import Path

# Column order: IPU4 (the port), then the variants of the shared driver
# it must not break (plan "Parallel track").
VARIANTS = ("ipu4", "ipu4p", "ipu6")

RESULTS = ("PASS", "FAIL", "SKIP", "NOT-IMPLEMENTED", "NOT-RUN")


def load_results(dirs) -> dict[str, dict]:
    """Return ``{variant: results}`` from every results*.json under *dirs*.

    Missing directories and unreadable / malformed files are ignored.
    When a variant appears more than once, the newest ``generated_at``
    wins (ISO-8601 UTC strings sort chronologically).
    """
    by_variant: dict[str, dict] = {}
    for d in dirs:
        d = Path(d)
        if not d.is_dir():
            continue
        for path in sorted(d.rglob("results*.json")):
            try:
                data = json.loads(path.read_text())
            except (OSError, json.JSONDecodeError):
                continue
            if not isinstance(data, dict) or not isinstance(data.get("rungs"), list):
                continue
            variant = data.get("variant") or "ipu4"
            prev = by_variant.get(variant)
            if prev is None or (data.get("generated_at") or "") > (prev.get("generated_at") or ""):
                by_variant[variant] = data
    return by_variant


def _rung_order(by_variant: dict[str, dict], fallback: list[str]) -> list[str]:
    """Ladder order: the ipu4 run first, then rungs only other variants
    (or ``ladder.sh --list``) know about, appended in their order."""
    order: list[str] = []
    sources = [by_variant[v] for v in VARIANTS if v in by_variant]
    sources += [r for v, r in by_variant.items() if v not in VARIANTS]
    for res in sources:
        for rung in res["rungs"]:
            name = rung.get("name")
            if name and name not in order:
                order.append(name)
    for name in fallback:
        if name not in order:
            order.append(name)
    return order


def _summary(res: dict) -> dict:
    rungs = res["rungs"]
    enabled = [r for r in rungs if r.get("enabled")]
    enabled_failing = [r["name"] for r in enabled if r.get("result") != "PASS"]
    return {
        "total": len(rungs),
        "passing": sum(1 for r in rungs if r.get("result") == "PASS"),
        "enabled": len(enabled),
        "enabled_passing": len(enabled) - len(enabled_failing),
        "enabled_failing": enabled_failing,
        "ok": not enabled_failing,
    }


def build(by_variant: dict[str, dict], fallback_rungs: list[str] | None = None) -> dict:
    """Shape the per-variant results for the template.

    Returns ``{variants, rungs, primary, has_data}`` where ``variants``
    is a list of ``{name, has_data, summary, generated_at, run_url,
    commit_sha}``, ``rungs`` a list of ``{name, cells}`` with one cell
    per variant (``None`` when that variant has no data for the rung),
    and ``primary`` the results whose pin and series the section shows
    (ipu4 when present).
    """
    names = list(VARIANTS) + [v for v in by_variant if v not in VARIANTS]
    variants = []
    for v in names:
        res = by_variant.get(v)
        variants.append({
            "name": v,
            "has_data": res is not None,
            "summary": _summary(res) if res else None,
            "generated_at": (res or {}).get("generated_at"),
            "run_url": (res or {}).get("run_url"),
            "commit_sha": (res or {}).get("commit_sha"),
        })

    rungs = []
    for name in _rung_order(by_variant, list(fallback_rungs or [])):
        cells = []
        for v in names:
            res = by_variant.get(v)
            if res is None:
                cells.append(None)
                continue
            rung = next((r for r in res["rungs"] if r.get("name") == name), None)
            if rung is None:
                cells.append(None)
                continue
            result = rung.get("result") or "NOT-RUN"
            cells.append({
                "result": result,
                "css": result.lower() if result in RESULTS else "unknown",
                "enabled": bool(rung.get("enabled")),
                "duration_s": rung.get("duration_s"),
            })
        rungs.append({"name": name, "cells": cells})

    primary = next(
        (by_variant[v] for v in names if v in by_variant), None
    )
    return {
        "variants": variants,
        "rungs": rungs,
        "primary": primary,
        "has_data": bool(by_variant),
    }
