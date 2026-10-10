"""Unit tests for tools/dashboard/parse_ladder.py.

The parser reads the ``results.json`` files that
``tools/tests/ipu6/ladder.sh`` writes (one per QEMU variant) and lays
them out as the test x variant matrix of the dashboard's "IPU4 port"
section. Tests pin the schema fields the dashboard relies on and the
"degrade to not run yet" behaviour.
"""
from __future__ import annotations

import json
import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "tools" / "dashboard"))
import parse_ladder  # noqa: E402


def _results(variant="ipu4", generated_at="2026-10-10T12:00:00Z", tests=None, **extra):
    data = {
        "schema": 1,
        "variant": variant,
        "generated_at": generated_at,
        "commit_sha": "0123456789abcdef",
        "run_id": "42",
        "run_url": "https://github.com/o/r/actions/runs/42",
        "pin": {"url": "https://github.com/ruslanbay/linux.git", "sha": "340985596f38"},
        "series": {"count": 1, "patches": [{"file": "0001-x.patch", "subject": "media: ipu6: x"}]},
        "enabled_tests": "tools/tests/ipu6/enabled-tests",
        "exit_status": 0,
        "tests": tests if tests is not None else [
            {"name": "series-applies", "depends_on": None, "enabled": True, "result": "PASS", "duration_s": 40},
            {"name": "build", "depends_on": "series-applies", "enabled": True, "result": "FAIL", "duration_s": 20},
            {"name": "vm-boot", "depends_on": "build", "enabled": False, "result": "SKIP", "duration_s": None},
            {"name": "native-bind", "depends_on": "vm-boot", "enabled": False, "result": "NOT-IMPLEMENTED", "duration_s": None},
        ],
    }
    data.update(extra)
    return data


def _write(path: pathlib.Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(data if isinstance(data, str) else json.dumps(data))


def test_load_finds_results_in_artifact_subdirs(tmp_path):
    _write(tmp_path / "ipu6-ladder-results-ipu4" / "results.json", _results())
    _write(tmp_path / "ipu6-ladder-results-ipu4p" / "results.json", _results("ipu4p"))
    out = parse_ladder.load_results([tmp_path])
    assert set(out) == {"ipu4", "ipu4p"}


def test_load_ignores_missing_dirs_and_bad_files(tmp_path):
    _write(tmp_path / "a" / "results.json", "{not json")
    _write(tmp_path / "b" / "results.json", json.dumps([1, 2]))
    _write(tmp_path / "c" / "results.json", json.dumps({"variant": "ipu4"}))
    assert parse_ladder.load_results([tmp_path, tmp_path / "missing"]) == {}


def test_load_keeps_newest_run_per_variant(tmp_path):
    _write(tmp_path / "old" / "results.json", _results(generated_at="2026-10-01T00:00:00Z", run_id="1"))
    _write(tmp_path / "new" / "results.json", _results(generated_at="2026-10-09T00:00:00Z", run_id="2"))
    out = parse_ladder.load_results([tmp_path])
    assert out["ipu4"]["run_id"] == "2"


def test_missing_variant_defaults_to_ipu4(tmp_path):
    data = _results()
    del data["variant"]
    _write(tmp_path / "results.json", data)
    assert list(parse_ladder.load_results([tmp_path])) == ["ipu4"]


def test_build_without_data_lists_fallback_tests_as_not_run():
    out = parse_ladder.build({}, ["series-applies", "build"])
    assert out["has_data"] is False
    assert out["primary"] is None
    assert [v["name"] for v in out["variants"]] == ["ipu4", "ipu4p", "ipu6"]
    assert all(not v["has_data"] and v["summary"] is None for v in out["variants"])
    assert [r["name"] for r in out["tests"]] == ["series-applies", "build"]
    assert all(c is None for r in out["tests"] for c in r["cells"])


def test_build_without_data_or_fallback_has_no_rows():
    out = parse_ladder.build({})
    assert out["tests"] == []


def test_build_summary_counts_and_gating():
    out = parse_ladder.build({"ipu4": _results()})
    ipu4 = out["variants"][0]
    assert ipu4["has_data"]
    assert ipu4["run_url"].endswith("/runs/42")
    s = ipu4["summary"]
    assert s == {
        "total": 4,
        "passing": 1,
        "enabled": 2,
        "enabled_passing": 1,
        "enabled_failing": ["build"],
        "ok": False,
    }
    assert out["primary"]["variant"] == "ipu4"


def test_build_all_gated_pass_is_ok():
    tests = [{"name": "series-applies", "enabled": True, "result": "PASS"},
             {"name": "build", "enabled": False, "result": "FAIL"}]
    s = parse_ladder.build({"ipu4": _results(tests=tests)})["variants"][0]["summary"]
    assert s["ok"] is True
    assert s["enabled_passing"] == 1


def test_build_cells_per_variant():
    out = parse_ladder.build({"ipu4": _results()})
    rows = {r["name"]: r["cells"] for r in out["tests"]}
    ipu4_build, ipu4p_build, ipu6_build = rows["build"]
    assert ipu4_build == {"result": "FAIL", "css": "fail", "enabled": True, "duration_s": 20}
    assert ipu4p_build is None and ipu6_build is None
    assert rows["native-bind"][0]["css"] == "not-implemented"


def test_build_unknown_result_and_missing_test_in_variant():
    ipu6 = _results("ipu6", tests=[
        {"name": "vm-boot", "enabled": True, "result": "WEIRD"},
        {"name": "ipu6-only", "enabled": False},
    ])
    out = parse_ladder.build({"ipu4": _results(), "ipu6": ipu6}, ["vm-boot", "extra"])
    names = [r["name"] for r in out["tests"]]
    # ipu4 order first, then tests only ipu6 has, then --list leftovers.
    assert names == ["series-applies", "build", "vm-boot", "native-bind", "ipu6-only", "extra"]
    rows = {r["name"]: r["cells"] for r in out["tests"]}
    assert rows["vm-boot"][2]["css"] == "unknown"
    assert rows["ipu6-only"][2]["result"] == "NOT-RUN"
    assert rows["ipu6-only"][0] is None  # ipu4 has data but not this test
    assert rows["extra"] == [None, None, None]


def test_build_appends_unknown_variants_and_primary_falls_back():
    out = parse_ladder.build({"ipu7": _results("ipu7")})
    assert [v["name"] for v in out["variants"]] == ["ipu4", "ipu4p", "ipu6", "ipu7"]
    assert out["primary"]["variant"] == "ipu7"
    assert [len(r["cells"]) for r in out["tests"]] == [4, 4, 4, 4]


def test_real_ladder_output_shape(tmp_path):
    """The fields ladder.sh's write_results_json emits round-trip."""
    data = _results(tests=[{"name": "checkpatch", "depends_on": None,
                            "enabled": False, "result": "NOT-RUN",
                            "duration_s": None}])
    _write(tmp_path / "results.json", data)
    out = parse_ladder.build(parse_ladder.load_results([tmp_path]))
    cell = out["tests"][0]["cells"][0]
    assert cell == {"result": "NOT-RUN", "css": "not-run", "enabled": False, "duration_s": None}


def test_load_results_reads_old_rungs_key(tmp_path):
    data = _results()
    data["rungs"] = data.pop("tests")
    (tmp_path / "results.json").write_text(json.dumps(data))
    out = parse_ladder.load_results([tmp_path])
    assert [r["name"] for r in out["ipu4"]["tests"]] == [
        r["name"] for r in _results()["tests"]]


def test_build_variants_carry_their_own_base():
    upstream = _results(
        variant="ipu6",
        pin={"url": "https://github.com/torvalds/linux.git", "sha": "93f51579e7df"},
        series={"count": 0, "patches": []},
    )
    out = parse_ladder.build({"ipu4": _results(), "ipu6": upstream})
    by_name = {v["name"]: v for v in out["variants"]}
    assert by_name["ipu4"]["pin"]["url"] == "https://github.com/ruslanbay/linux.git"
    assert by_name["ipu4"]["series_count"] == 1
    assert by_name["ipu6"]["pin"]["sha"] == "93f51579e7df"
    assert by_name["ipu6"]["series_count"] == 0
    assert by_name["ipu4p"]["pin"] is None and by_name["ipu4p"]["series_count"] is None
    # The section's pin/series block still describes the ipu4 run.
    assert out["primary"]["variant"] == "ipu4"
