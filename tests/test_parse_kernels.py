"""Unit tests for tools/dashboard/parse_kernels.py.

The dashboard's "Supported kernels" section lists one row per
``.github/kernel-pins/*.json`` with its ``build-and-kunit`` status from
the latest ``ci`` run.
"""
from __future__ import annotations

import json
import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "tools" / "dashboard"))
import parse_kernels  # noqa: E402


def _pin(d, name, ref, url="https://example.org/linux.git"):
    (d / f"{name}.json").write_text(json.dumps({"url": url, "ref": ref}))


def test_load_pins_sorts_by_version(tmp_path):
    _pin(tmp_path, "7.3", "v7.3-rc4")
    _pin(tmp_path, "6.18", "v6.18.55")
    _pin(tmp_path, "6.12", "v6.12.112")
    names = [p["name"] for p in parse_kernels.load_pins(tmp_path)]
    assert names == ["6.12", "6.18", "7.3"]


def test_load_pins_skips_bad_files(tmp_path):
    _pin(tmp_path, "6.12", "v6.12.112")
    (tmp_path / "broken.json").write_text("{")
    (tmp_path / "noref.json").write_text("{}")
    (tmp_path / "list.json").write_text("[]")
    pins = parse_kernels.load_pins(tmp_path)
    assert pins == [{"name": "6.12", "ref": "v6.12.112", "url": "https://example.org/linux.git"}]


def test_load_pins_missing_dir(tmp_path):
    assert parse_kernels.load_pins(tmp_path / "nope") == []


def test_load_pins_real_repo():
    pins = parse_kernels.load_pins(_REPO_ROOT / ".github" / "kernel-pins")
    assert pins and all(p["ref"].startswith("v") for p in pins)


def _job(name, conclusion):
    return {"name": name, "conclusion": conclusion}


def test_track_status():
    jobs = [
        _job("build-and-kunit (6.12) / build-and-kunit", "success"),
        _job("build-and-kunit (6.18) / build-and-kunit", "failure"),
        _job("build-and-kunit (7.3) / build-and-kunit", None),
        _job("build-and-kunit (6.6) / build-and-kunit", "skipped"),
        _job("pytest", "failure"),
    ]
    st = parse_kernels.track_status
    assert st(jobs, "6.12") == "success"
    assert st(jobs, "6.18") == "failure"
    assert st(jobs, "7.3") == "in_progress"
    assert st(jobs, "6.6") == "skipped"
    assert st(jobs, "5.15") is None
    # "6.1" must not match the 6.12 / 6.18 jobs.
    assert st(jobs, "6.1") is None


def test_build_without_ci_data():
    rows = parse_kernels.build([{"name": "6.12", "ref": "v6.12.112", "url": None}], None)
    assert rows == [{"name": "6.12", "ref": "v6.12.112", "url": None, "status": None}]
