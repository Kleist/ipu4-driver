"""Supported kernel tracks for the dashboard.

One track per ``.github/kernel-pins/<name>.json`` (``{"url", "ref"}``),
the matrix ``ci.yml`` builds ``kernel/ipu4/`` against. Each track's
status is its ``build-and-kunit (<name>)`` jobs in the latest ``ci`` run.
"""
from __future__ import annotations

import json
from pathlib import Path

_BAD = frozenset({"failure", "cancelled", "timed_out", "startup_failure"})


def _version_key(name: str):
    return tuple(int(p) if p.isdigit() else 0 for p in name.split("."))


def load_pins(pin_dir) -> list[dict]:
    """``[{name, ref, url}]`` from *pin_dir*, oldest track first.

    Unreadable or malformed files are skipped.
    """
    pins = []
    d = Path(pin_dir)
    if not d.is_dir():
        return pins
    for path in d.glob("*.json"):
        try:
            data = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        if not isinstance(data, dict) or not data.get("ref"):
            continue
        pins.append({"name": path.stem, "ref": data["ref"], "url": data.get("url")})
    return sorted(pins, key=lambda p: _version_key(p["name"]))


def track_status(jobs: list[dict], name: str) -> str | None:
    """Fold the ``build-and-kunit (<name>)`` jobs into one conclusion.

    ``None`` when the run has no such job; ``in_progress`` while any
    job has no conclusion yet.
    """
    prefix = f"build-and-kunit ({name})"
    mine = [j for j in jobs if (j.get("name") or "").startswith(prefix)]
    if not mine:
        return None
    conclusions = [j.get("conclusion") for j in mine]
    if any(c in _BAD for c in conclusions):
        return "failure"
    if any(c is None for c in conclusions):
        return "in_progress"
    if all(c == "skipped" for c in conclusions):
        return "skipped"
    return "success"


def build(pins: list[dict], jobs: list[dict] | None) -> list[dict]:
    """One ``{name, ref, url, status}`` row per pinned track."""
    return [{**p, "status": track_status(jobs or [], p["name"])} for p in pins]
