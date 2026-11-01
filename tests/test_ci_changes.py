"""Tests for tools/ci/changes.sh (PR job selection)."""

import pathlib
import subprocess

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools" / "ci" / "changes.sh"
AREAS = ("kunit", "pytest", "vm_smoke", "ipu6")


def classify(*paths):
    out = subprocess.run(
        [str(SCRIPT), "--files"],
        input="\n".join(paths) + "\n",
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    result = dict(line.split("=", 1) for line in out.splitlines())
    assert tuple(result) == AREAS
    return {k for k, v in result.items() if v == "true"}


@pytest.mark.parametrize(
    "path, expected",
    [
        ("STATUS.md", set()),
        ("tools/notes/ipu6-ipu4-plan.md", set()),
        ("tools/dashboard/templates/index.html.j2", set()),
        ("tools/dashboard/parse_ladder.py", {"pytest"}),
        ("tools/upstream/divergence_history.py", {"pytest"}),
        ("tests/test_compare.py", {"pytest"}),
        ("kernel/ipu4/ipu6-isys.c", {"kunit", "vm_smoke", "ipu6"}),
        ("tools/build.sh", {"kunit", "vm_smoke", "ipu6"}),
        ("tools/out-of-tree/build.sh", {"ipu6"}),
        ("tools/linux-patches/drivers/x.patch", {"kunit", "vm_smoke"}),
        (".github/kernel-pins/6.18.json", {"kunit", "vm_smoke"}),
        ("tools/qemu-patches/hw/misc/ipu4.c", {"vm_smoke", "ipu6"}),
        ("data/trace.txt", {"vm_smoke", "ipu6"}),
        ("tools/bootstrap.sh", {"kunit", "vm_smoke", "ipu6"}),
        ("patches/ipu6-ipu4/0001-x.patch", {"ipu6"}),
        ("patches/ipu6-ipu4p/0001-x.patch", {"ipu6"}),
        ("patches/ipu6-fixes/0001-x.patch", {"ipu6"}),
        ("tools/tests/ipu6/ladder.sh", {"ipu6"}),
        ("tools/coverage/ladder.sh", {"ipu6"}),
        ("tools/coverage/collect.sh", {"vm_smoke"}),
        ("tools/tests/compare-mmio-baseline.txt", {"vm_smoke"}),
        ("tools/trace/compare.py", {"pytest", "vm_smoke"}),
        ("tools/trace/check_writes.py", {"pytest", "vm_smoke", "ipu6"}),
        (".github/workflows/vm-ipu6.yml", {"ipu6"}),
        (".github/actions/setup-harness/action.yml", set(AREAS)),
        ("tools/ci/changes.sh", set(AREAS)),
    ],
)
def test_single_path(path, expected):
    assert classify(path) == expected


def test_union_of_paths():
    assert classify("tests/test_compare.py", "patches/ipu6-ipu4/0001-x.patch") == {
        "pytest",
        "ipu6",
    }


def test_all():
    out = subprocess.run(
        [str(SCRIPT), "--all"], capture_output=True, text=True, check=True
    ).stdout
    assert out.split() == [f"{a}=true" for a in AREAS]


def test_undiffable_range_selects_everything():
    out = subprocess.run(
        [str(SCRIPT), "no-such-rev", "HEAD"],
        capture_output=True,
        text=True,
        check=True,
        cwd=ROOT,
    ).stdout
    assert out.split() == [f"{a}=true" for a in AREAS]
