"""Tests for tools/upstream/divergence_history.py (dashboard divergence chart)."""
from __future__ import annotations

import json
import os
import pathlib
import subprocess
import sys

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "tools" / "upstream"))
import divergence_history as dh  # noqa: E402

DAY = 86400
T0 = 1_700_000_000  # 2023-11-14


def run(repo, *args):
    env = dict(os.environ, GIT_CONFIG_GLOBAL="/dev/null", GIT_CONFIG_SYSTEM="/dev/null")
    return subprocess.run(["git", "-C", str(repo), *args], check=True,
                          capture_output=True, text=True, env=env).stdout.strip()


def commit(repo, files, msg, ts, author_ts=None, remove=()):
    for name, text in files.items():
        p = repo / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
        run(repo, "add", name)
    for name in remove:
        run(repo, "rm", "-q", name)
    env_author = author_ts if author_ts is not None else ts
    env = dict(os.environ, GIT_CONFIG_GLOBAL="/dev/null", GIT_CONFIG_SYSTEM="/dev/null",
               GIT_AUTHOR_DATE=f"@{env_author} +0000", GIT_COMMITTER_DATE=f"@{ts} +0000")
    subprocess.run(["git", "-C", str(repo), "commit", "-q", "-m", msg], check=True, env=env)
    return run(repo, "rev-parse", "HEAD")


def init(path, branch="main"):
    path.mkdir()
    run(path, "init", "-q", "-b", branch)
    run(path, "config", "user.email", "t@example.org")
    run(path, "config", "user.name", "T")
    return path


UP = "drivers/media/pci/intel/ipu6"


@pytest.fixture
def linux(tmp_path):
    """Mainline stand-in: no ipu6 driver until v6.10."""
    repo = init(tmp_path / "linux", branch="master")
    commit(repo, {"README": "x\n"}, "v6.6", T0)
    run(repo, "tag", "v6.6")
    commit(repo, {"README": "y\n"}, "rc", T0 + 20 * DAY)
    run(repo, "tag", "-a", "-m", "rc", "v6.10-rc1")
    commit(repo, {f"{UP}/ipu6-isys.c": "a\nb\nc\n", f"{UP}/ipu6.c": "1\n2\n",
                  f"{UP}/ipu7-only.c": "u\nv\n"}, "v6.10", T0 + 40 * DAY)
    run(repo, "tag", "v6.10")
    commit(repo, {f"{UP}/ipu6.c": "1\n2\n3\n"}, "later", T0 + 50 * DAY)
    run(repo, "tag", "not-a-kernel-tag")
    return repo


@pytest.fixture
def ours(tmp_path):
    """Our history: unrelated newer root, old-dated import, move, cherry-pick."""
    repo = init(tmp_path / "ours")
    commit(repo, {"README.md": "hi\n"}, "Initial commit", T0 + 100 * DAY)
    commit(repo, {f"{UP}/ipu6-isys.c": "a\nb\nX\n", f"{UP}/ipu6.c": "1\n2\n",
                  f"{UP}/ambu-ipu-bridge.c": "local\n",
                  "tools/linux-patches/x/ipu6-isys.c": "copy\n"},
           "import", T0 + 100 * DAY, author_ts=T0 - 5 * DAY)
    commit(repo, {f"{UP}/ipu6-isys.c": "a\nb\nc\n"}, "align", T0 + 100 * DAY,
           author_ts=T0 + 30 * DAY)
    (repo / "kernel" / "ipu4").mkdir(parents=True)
    for name in ("ipu6-isys.c", "ipu6.c", "ambu-ipu-bridge.c"):
        run(repo, "mv", f"{UP}/{name}", f"kernel/ipu4/{name}")
    commit(repo, {}, "move", T0 + 100 * DAY, author_ts=T0 + 45 * DAY)
    commit(repo, {"kernel/ipu4/virt.c": "v\n"}, "harness", T0 + 110 * DAY)
    commit(repo, {"kernel/ipu4/ipu6.c": "1\n2\n3\n"}, "cherry-pick", T0 + 120 * DAY,
           author_ts=T0 + 48 * DAY)
    commit(repo, {"README.md": "bye\n"}, "docs", T0 + 130 * DAY)
    return repo


def test_tag_version():
    assert dh.tag_version("v6.10") == (6, 10, None)
    assert dh.tag_version("v7.3-rc4") == (7, 3, 4)
    assert dh.tag_version("v6.10.1") is None
    assert dh.tag_version("foo") is None


def test_effective_dates_ceil_to_ancestors():
    commits = [("a", [], 10), ("b", ["a"], 5), ("c", ["b", "x"], 7), ("d", ["c"], 30)]
    assert dh.effective_dates(commits) == {"a": 10, "b": 10, "c": 10, "d": 30}


def test_sample():
    assert dh.sample([1, 2, 3], 10) == [1, 2, 3]
    s = dh.sample(list(range(25)), 10)
    assert len(s) == 10 and s[-1] == 24 and s == sorted(set(s))


def test_changed_lines():
    assert dh.changed_lines(["a", "b"], ["a", "b"]) == 0
    assert dh.changed_lines(["a", "x", "c"], ["a", "b", "c"]) == 2
    assert dh.changed_lines(["a"], []) == 1


def test_latest_at():
    dated = [(1, "a"), (5, "b"), (5, "c"), (9, "d")]
    assert dh.latest_at(dated, 0) is None
    assert dh.latest_at(dated, 5) == "c"
    assert dh.latest_at(dated, 100) == "d"


def test_pick_points():
    tags = [(0, "v6.5"), (10, "v6.6"), (15, "v6.7-rc1"), (20, "v6.7")]
    commits = [(5, "c0")] + [(10 + i, f"c{i}") for i in range(1, 9)] + [(25, "c9")]
    pts = dh.pick_points(tags, commits, (25, "c9"))
    kinds = [(k, sha, ref) for k, _, sha, ref in pts]
    assert kinds[0] == ("tag", "c0", "v6.6")  # v6.5 is before FIRST_RELEASE
    assert ("commit", "c6", "v6.7-rc1") in kinds and ("commit", "c4", "v6.6") in kinds
    assert ("tag", "c8", "v6.7") in kinds
    assert kinds[-1] == ("head", "c9", "v6.7")
    assert ("commit", "c9", "v6.7") not in kinds  # head is not repeated


def test_pick_points_caps_commits_per_interval(monkeypatch):
    monkeypatch.setattr(dh, "MAX_BETWEEN", 3)
    tags = [(10, "v6.6"), (100, "v6.7")]
    commits = [(10 + i, f"c{i}") for i in range(1, 20)]
    pts = dh.pick_points(tags, commits, (200, "head"))
    assert sum(1 for p in pts if p[0] == "commit") == 3


def test_local_only():
    names = dh.local_only(_REPO_ROOT)
    assert "ambu-ipu-bridge.c" in names and "ipu6.c" not in names


def test_build(ours, linux):
    data = dh.build(ours, "HEAD", linux, {"ambu-ipu-bridge.c"})
    pts = {(p["kind"], p["ref"]): p for p in data["points"]}
    # v6.6: our import (author date T0-5d, below the unrelated root's date)
    # with no upstream driver yet: every line counts.
    v66 = pts[("tag", "v6.6")]
    assert v66["subject"] == "import"
    assert v66["divergence"] == v66["lines"] == 5
    assert v66["files_missing_upstream"] == 2
    # "align" (author T0+30d) compares with the rc before it, still no driver.
    assert pts[("commit", "v6.10-rc1")]["subject"] == "align"
    # v6.10: tree after the move (author T0+45d > the tag); before it, align.
    v610 = pts[("tag", "v6.10")]
    assert v610["subject"] == "align" and v610["divergence"] == 0
    assert v610["upstream_only_files"] == 1 and v610["upstream_only_lines"] == 2
    # The cherry-pick's T0+48d author date is raised to the harness commit's
    # T0+110d (its latest driver ancestor), not to the unrelated root's.
    picks = [p for p in data["points"] if p["subject"] == "cherry-pick"]
    assert len(picks) == 1 and picks[0]["ts"] == (T0 + 110 * DAY) * 1000
    head = data["points"][-1]
    assert head["kind"] == "head" and head["ref"] == "v6.10"
    assert head["divergence"] == 1 + 1  # ipu6.c +1 line, virt.c not upstream
    assert data["master"]["divergence"] == 1  # master has the third ipu6.c line


def test_driver_dir_ignores_tools(ours):
    assert dh.driver_dir(ours, "HEAD") == "kernel/ipu4"
    assert dh.driver_dir(ours, "HEAD~6") is None


def test_fetch_and_main(ours, linux, tmp_path):
    run(linux, "config", "uploadpack.allowfilter", "true")
    run(linux, "config", "uploadpack.allowanysha1inwant", "true")
    clone = tmp_path / "tags"
    dh.fetch_linux(clone, url=f"file://{linux}")
    names = run(clone, "tag").split()
    assert set(names) == {"v6.6", "v6.10-rc1", "v6.10"}
    dh.fetch_linux(clone, url=f"file://{linux}")  # existing dir: refetch only
    out = tmp_path / "h.json"
    assert dh.main(["--repo", str(ours), "--linux", str(clone), "--out", str(out)]) == 0
    data = json.loads(out.read_text())
    assert data["schema"] == 1 and data["points"][-1]["kind"] == "head"
