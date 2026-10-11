"""Unit tests for tools/dashboard/parse_ladder.py.

The parser reads the ``results.json`` files that
``tools/tests/ipu6/ladder.sh`` writes (one per leg) and lays
them out as the test x leg matrix of the dashboard's "IPU4 port"
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


def _results(leg="ipu4-patches/ipu4", generated_at="2026-10-10T12:00:00Z", tests=None, **extra):
    data = {
        "schema": 1,
        "leg": leg,
        "driver": leg.split("/")[0],
        "variant": leg.split("/")[-1],
        "generated_at": generated_at,
        "commit_sha": "0123456789abcdef",
        "run_id": "42",
        "run_url": "https://github.com/o/r/actions/runs/42",
        "pin": {"url": "https://github.com/ruslanbay/linux.git", "sha": "340985596f38"},
        "series": {"count": 1, "patches": [{"file": "0001-x.patch", "subject": "media: ipu6: x"}]},
        "enabled_tests": "tools/tests/ipu6/enabled-tests.ipu4-patches.ipu4",
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
    _write(tmp_path / "ipu6-ladder-results-ipu4-patches" / "results.json", _results())
    _write(tmp_path / "ipu6-ladder-results-ipu4p-ipu4p" / "results.json", _results("ipu4p/ipu4p"))
    out = parse_ladder.load_results([tmp_path])
    assert set(out) == {"ipu4-patches/ipu4", "ipu4p/ipu4p"}


def test_load_ignores_missing_dirs_and_bad_files(tmp_path):
    _write(tmp_path / "a" / "results.json", "{not json")
    _write(tmp_path / "b" / "results.json", json.dumps([1, 2]))
    _write(tmp_path / "c" / "results.json", json.dumps({"variant": "ipu4"}))
    assert parse_ladder.load_results([tmp_path, tmp_path / "missing"]) == {}


def test_load_keeps_newest_run_per_variant(tmp_path):
    _write(tmp_path / "old" / "results.json", _results(generated_at="2026-10-01T00:00:00Z", run_id="1"))
    _write(tmp_path / "new" / "results.json", _results(generated_at="2026-10-09T00:00:00Z", run_id="2"))
    out = parse_ladder.load_results([tmp_path])
    assert out["ipu4-patches/ipu4"]["run_id"] == "2"


def test_missing_leg_and_variant_defaults_to_ipu4_patches(tmp_path):
    data = _results()
    del data["leg"], data["variant"]
    _write(tmp_path / "results.json", data)
    assert list(parse_ladder.load_results([tmp_path])) == ["ipu4-patches/ipu4"]


def test_results_without_leg_map_their_variant(tmp_path):
    for leg in ("ipu4-patches/ipu4", "ipu6/ipu6"):
        data = _results(leg)
        del data["leg"]
        _write(tmp_path / leg / "results.json", data)
    assert set(parse_ladder.load_results([tmp_path])) == {"ipu4-patches/ipu4", "ipu6/ipu6"}


def test_build_without_data_lists_fallback_tests_as_not_run():
    out = parse_ladder.build({}, ["series-applies", "build"])
    assert out["has_data"] is False
    assert out["primary"] is None
    assert [v["name"] for v in out["legs"]] == list(parse_ladder.LEGS)
    assert list(parse_ladder.LEGS) == [
        "ipu4-out-of-tree/ipu4", "ipu4p/ipu4p", "ipu6/ipu6",
        "ipu4-patches/ipu4", "ipu4-patches/ipu4p", "ipu4-patches/ipu6"]
    assert all(not v["has_data"] and v["summary"] is None for v in out["legs"])
    assert [r["name"] for r in out["tests"]] == ["series-applies", "build"]
    assert all(c is None for r in out["tests"] for c in r["cells"])


def test_build_without_data_or_fallback_has_no_rows():
    out = parse_ladder.build({})
    assert out["tests"] == []


def test_build_summary_counts_and_gating():
    out = parse_ladder.build({"ipu4-patches/ipu4": _results()})
    ipu4 = next(v for v in out["legs"] if v["name"] == "ipu4-patches/ipu4")
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
    assert out["primary"]["leg"] == "ipu4-patches/ipu4"


def test_build_all_gated_pass_is_ok():
    tests = [{"name": "series-applies", "enabled": True, "result": "PASS"},
             {"name": "build", "enabled": False, "result": "FAIL"}]
    legs = parse_ladder.build({"ipu4-patches/ipu4": _results(tests=tests)})["legs"]
    s = next(v for v in legs if v["has_data"])["summary"]
    assert s["ok"] is True
    assert s["enabled_passing"] == 1


def test_build_cells_per_leg():
    out = parse_ladder.build({"ipu4-patches/ipu4": _results()})
    rows = {r["name"]: r["cells"] for r in out["tests"]}
    cells = dict(zip(parse_ladder.LEGS, rows["build"]))
    assert cells.pop("ipu4-patches/ipu4") == {"result": "FAIL", "css": "fail", "enabled": True, "duration_s": 20}
    assert all(c is None for c in cells.values())
    assert rows["native-bind"][parse_ladder.LEGS.index("ipu4-patches/ipu4")]["css"] == "not-implemented"


def test_build_unknown_result_and_missing_test_in_leg():
    ipu6 = _results("ipu6/ipu6", tests=[
        {"name": "vm-boot", "enabled": True, "result": "WEIRD"},
        {"name": "ipu6-only", "enabled": False},
    ])
    out = parse_ladder.build({"ipu4-patches/ipu4": _results(), "ipu6/ipu6": ipu6}, ["vm-boot", "extra"])
    names = [r["name"] for r in out["tests"]]
    # Tests in leg order: ipu6/ipu6 comes before ipu4-patches/ipu4, then
    # --list leftovers.
    assert names == ["vm-boot", "ipu6-only", "series-applies", "build", "native-bind", "extra"]
    rows = {r["name"]: dict(zip(parse_ladder.LEGS, r["cells"])) for r in out["tests"]}
    assert rows["vm-boot"]["ipu6/ipu6"]["css"] == "unknown"
    assert rows["ipu6-only"]["ipu6/ipu6"]["result"] == "NOT-RUN"
    assert rows["ipu6-only"]["ipu4-patches/ipu4"] is None  # has data, not this test
    assert all(c is None for c in rows["extra"].values())


def test_build_appends_unknown_legs_and_primary_falls_back():
    out = parse_ladder.build({"ipu7/ipu7": _results("ipu7/ipu7")})
    assert [v["name"] for v in out["legs"]] == list(parse_ladder.LEGS) + ["ipu7/ipu7"]
    assert out["primary"]["leg"] == "ipu7/ipu7"
    assert [len(r["cells"]) for r in out["tests"]] == [7, 7, 7, 7]


def test_real_ladder_output_shape(tmp_path):
    """The fields ladder.sh's write_results_json emits round-trip."""
    data = _results(tests=[{"name": "checkpatch", "depends_on": None,
                            "enabled": False, "result": "NOT-RUN",
                            "duration_s": None}])
    _write(tmp_path / "results.json", data)
    out = parse_ladder.build(parse_ladder.load_results([tmp_path]))
    cell = out["tests"][0]["cells"][parse_ladder.LEGS.index("ipu4-patches/ipu4")]
    assert cell == {"result": "NOT-RUN", "css": "not-run", "enabled": False, "duration_s": None}


def test_load_results_reads_old_rungs_key(tmp_path):
    data = _results()
    data["rungs"] = data.pop("tests")
    (tmp_path / "results.json").write_text(json.dumps(data))
    out = parse_ladder.load_results([tmp_path])
    assert [r["name"] for r in out["ipu4-patches/ipu4"]["tests"]] == [
        r["name"] for r in _results()["tests"]]


def test_build_legs_carry_their_own_base():
    upstream = _results(
        leg="ipu6/ipu6",
        pin={"url": "https://github.com/torvalds/linux.git", "sha": "93f51579e7df"},
        series={"count": 0, "patches": []},
    )
    out = parse_ladder.build({"ipu4-patches/ipu4": _results(), "ipu6/ipu6": upstream})
    by_name = {v["name"]: v for v in out["legs"]}
    assert by_name["ipu4-patches/ipu4"]["pin"]["url"] == "https://github.com/ruslanbay/linux.git"
    assert by_name["ipu4-patches/ipu4"]["series_count"] == 1
    assert by_name["ipu6/ipu6"]["pin"]["sha"] == "93f51579e7df"
    assert by_name["ipu6/ipu6"]["series_count"] == 0
    assert by_name["ipu4p/ipu4p"]["pin"] is None and by_name["ipu4p/ipu4p"]["series_count"] is None
    # The pin/series block describes ipu4-patches/ipu4 although ipu6/ipu6
    # comes first.
    assert out["primary"]["leg"] == "ipu4-patches/ipu4"


def test_base_series_labelled():
    res = _results(base_series=[
        {"dir": "patches/ipu6-ipu7", "count": 45},
        {"dir": "patches/ipu6-ipu4p", "count": 11},
        {"dir": "patches/other", "count": 1},
    ])
    out = parse_ladder.build({"ipu4-patches/ipu4": res})
    base = out["primary"]["base_series"]
    assert [b["label"] for b in base] == ["IPU7 series", "IPU4P series", "patches/other"]
    assert base[1]["source"] == "ruslanbay/linux ipu4p-upstream"
    assert "ruslanbay/linux" in base[1]["url"] and base[2]["url"] is None
    leg = next(v for v in out["legs"] if v["name"] == "ipu4-patches/ipu4")
    assert leg["base_series"] == base


def test_base_series_missing_in_old_results():
    out = parse_ladder.build({"ipu4-patches/ipu4": _results()})
    assert out["primary"]["base_series"] == []
