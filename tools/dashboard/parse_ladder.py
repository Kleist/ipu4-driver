"""Collect test-ladder results into a test x leg matrix.

``tools/tests/ipu6/ladder.sh`` writes one ``results.json`` per run
(schema 1, see tools/dashboard/README.md). The vm-ipu6 workflow uploads
it as ``ipu6-ladder-results-<leg>``; pages.yml downloads every such
artifact of the latest vm-ipu6 run into one directory. This module
finds those files, keeps the newest per leg and lays them out for
the "Test ladder" section of the dashboard.

Legs in ``LEGS`` always get a column, so a leg its track has not wired
up yet renders as "not run yet" instead of being hidden. Unknown legs
found in the data are appended after them.
"""
from __future__ import annotations

import json
from pathlib import Path

# A leg is "<driver>/<variant>": each IPU under its unmodified baseline,
# then under the upstreamable ipu4-patches (plan "Tracks and file
# ownership").
VARIANTS = ("ipu4", "ipu4p", "ipu6")
BASELINES = {"ipu4": "ipu4-out-of-tree", "ipu4p": "ipu4p", "ipu6": "ipu6"}
LEGS = tuple(f"{BASELINES[v]}/{v}" for v in VARIANTS) + tuple(
    f"ipu4-patches/{v}" for v in VARIANTS)
# The section's pin and series block describes this leg's run.
PRIMARY = "ipu4-patches/ipu4"

# results.json written before legs existed only name the QEMU variant.
_OLD_VARIANT_LEG = {"ipu4": "ipu4-patches/ipu4", "ipu6": "ipu6/ipu6"}

# The not-yet-mainline series the ipu4p and ipu4-patches trees apply on
# the mainline pin before patches/ipu6-ipu4/.
BASE_SERIES = {
    "patches/ipu6-ipu7": (
        "IPU7 series", "lore",
        "https://lore.kernel.org/all/20260907113004.2489993-1-sakari.ailus@linux.intel.com/"),
    "patches/ipu6-ipu4p": (
        "IPU4P series", "ruslanbay/linux ipu4p-upstream",
        "https://github.com/ruslanbay/linux/tree/ipu4p-upstream"),
}

RESULTS = ("PASS", "FAIL", "SKIP", "NOT-IMPLEMENTED", "NOT-RUN")


def load_results(dirs) -> dict[str, dict]:
    """Return ``{leg: results}`` from every results*.json under *dirs*.

    Missing directories and unreadable / malformed files are ignored.
    When a leg appears more than once, the newest ``generated_at``
    wins (ISO-8601 UTC strings sort chronologically).
    """
    by_leg: dict[str, dict] = {}
    for d in dirs:
        d = Path(d)
        if not d.is_dir():
            continue
        for path in sorted(d.rglob("results*.json")):
            try:
                data = json.loads(path.read_text())
            except (OSError, json.JSONDecodeError):
                continue
            # results.json written before the rename called them "rungs".
            if isinstance(data, dict) and "tests" not in data and "rungs" in data:
                data["tests"] = data.pop("rungs")
            if not isinstance(data, dict) or not isinstance(data.get("tests"), list):
                continue
            leg = data.get("leg")
            if not leg:
                variant = data.get("variant") or "ipu4"
                leg = _OLD_VARIANT_LEG.get(variant, variant)
            prev = by_leg.get(leg)
            if prev is None or (data.get("generated_at") or "") > (prev.get("generated_at") or ""):
                by_leg[leg] = data
    return by_leg


def _test_order(by_leg: dict[str, dict], fallback: list[str]) -> list[str]:
    """Ladder order: the first leg's run first, then tests only other legs
    (or ``ladder.sh --list``) know about, appended in their order."""
    order: list[str] = []
    sources = [by_leg[v] for v in LEGS if v in by_leg]
    sources += [r for v, r in by_leg.items() if v not in LEGS]
    for res in sources:
        for test in res["tests"]:
            name = test.get("name")
            if name and name not in order:
                order.append(name)
    for name in fallback:
        if name not in order:
            order.append(name)
    return order


def _summary(res: dict) -> dict:
    tests = res["tests"]
    enabled = [r for r in tests if r.get("enabled")]
    enabled_failing = [r["name"] for r in enabled if r.get("result") != "PASS"]
    return {
        "total": len(tests),
        "passing": sum(1 for r in tests if r.get("result") == "PASS"),
        "enabled": len(enabled),
        "enabled_passing": len(enabled) - len(enabled_failing),
        "enabled_failing": enabled_failing,
        "ok": not enabled_failing,
    }


def _base_series(res: dict) -> list[dict]:
    out = []
    for b in res.get("base_series") or []:
        label, source, url = BASE_SERIES.get(b.get("dir"), (b.get("dir"), None, None))
        out.append({**b, "label": label, "source": source, "url": url})
    return out


def build(by_leg: dict[str, dict], fallback_tests: list[str] | None = None) -> dict:
    """Shape the per-leg results for the template.

    Returns ``{legs, tests, primary, has_data}`` where ``legs`` is a
    list of ``{name, has_data, summary, generated_at, run_url,
    commit_sha, pin, series_count, base_series}``, ``tests`` a list of ``{name,
    cells}`` with one cell per leg (``None`` when that leg has no data
    for the test), and ``primary`` the results whose pin and series the
    section shows (``PRIMARY`` when present, else the first leg with data).
    """
    names = list(LEGS) + [v for v in by_leg if v not in LEGS]
    legs = []
    for v in names:
        res = by_leg.get(v)
        legs.append({
            "name": v,
            "has_data": res is not None,
            "summary": _summary(res) if res else None,
            "generated_at": (res or {}).get("generated_at"),
            "run_url": (res or {}).get("run_url"),
            "commit_sha": (res or {}).get("commit_sha"),
            # Legs run on different bases (ipu6 on unpatched mainline,
            # ipu4-out-of-tree on stock v7.3-rc4), so each card names its own.
            "pin": (res or {}).get("pin"),
            "series_count": ((res or {}).get("series") or {}).get("count"),
            "base_series": _base_series(res or {}),
        })

    tests = []
    for name in _test_order(by_leg, list(fallback_tests or [])):
        cells = []
        for v in names:
            res = by_leg.get(v)
            if res is None:
                cells.append(None)
                continue
            test = next((r for r in res["tests"] if r.get("name") == name), None)
            if test is None:
                cells.append(None)
                continue
            result = test.get("result") or "NOT-RUN"
            cells.append({
                "result": result,
                "css": result.lower() if result in RESULTS else "unknown",
                "enabled": bool(test.get("enabled")),
                "duration_s": test.get("duration_s"),
            })
        tests.append({"name": name, "cells": cells})

    primary = by_leg.get(PRIMARY) or next(
        (by_leg[v] for v in names if v in by_leg), None
    )
    if primary is not None:
        primary = {**primary, "base_series": _base_series(primary)}
    return {
        "legs": legs,
        "tests": tests,
        "primary": primary,
        "has_data": bool(by_leg),
    }
