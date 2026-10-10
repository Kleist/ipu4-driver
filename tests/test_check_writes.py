"""Unit tests for tools/trace/check_writes.py.

The checker compares the register writes two captures make in a set of
BAR-offset ranges; the ipu6 ladder's probe-mmu and isys-irq tests use it
to hold the guest's writes (QEMU trace log) against data/trace.txt.
"""
from __future__ import annotations

import argparse
import io
import pathlib
import sys

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "tools" / "trace"))
import check_writes  # noqa: E402

MMIOTRACE = """\
VERSION 20070824
PCIDEV 0018 80865a88 15 90000004 0 0 0 0 0 0 1000000 0 0 0 0 0 0
W 4 1.0 1 0x904b0008 0x10 0x0 0
MAP 1037.681661 1 0x90000000 0xffffb7b187000000 0x1000000 0x0 0
R 4 1.1 1 0x90000300 0x37002 0x0 0
W 4 1.2 1 0x904b0004 0x12155 0x0 0
W 4 1.3 1 0x904b0008 0x10 0x0 0
W 4 1.4 1 0x904b010c 0x0 0x0 0
W 4 1.5 1 0x92000000 0x1 0x0 0
"""


def qemu(addr, value, name="ipu4-bar0"):
    return (f"memory_region_ops_write cpu 0 mr 0x55e0999d73d0 addr 0x{addr:x} "
            f"value 0x{value:x} size 4 name '{name}'\n")


def test_mmiotrace_offsets_relative_to_map():
    writes = list(check_writes.read_writes(io.StringIO(MMIOTRACE)))
    # The W before MAP and the write outside the BAR are dropped.
    assert writes == [(0x4b0004, 0x12155), (0x4b0008, 0x10), (0x4b010c, 0)]


def test_qemu_log_filters_region_and_rebases():
    log = (qemu(0xfb4b0008, 0x10) + qemu(0x3f8, 0x41, "serial") +
           "garbage line\n\n" + qemu(0xfb4b010c, 0))
    writes = list(check_writes.read_writes(io.StringIO(log)))
    assert writes == [(0x4b0008, 0x10), (0x4b010c, 0)]


def test_line_range_limits_writes_but_not_map():
    # MAP is on line 4, the writes on lines 3 and 6-9.
    writes = list(check_writes.read_writes(io.StringIO(MMIOTRACE),
                                           line_range=(7, 8)))
    assert writes == [(0x4b0008, 0x10), (0x4b010c, 0)]
    log = qemu(0xfb4b0008, 0x10) + qemu(0xfb4b010c, 0)
    writes = list(check_writes.read_writes(io.StringIO(log),
                                           line_range=(2, 2)))
    assert writes == [(0x4b010c, 0)]


def test_in_ranges_keeps_order():
    writes = [(0x8, 1), (0x4, 2), (0x10c, 3), (0x200, 4), (0x0c, 5)]
    assert check_writes.in_ranges(writes, [(0x8, 0x90), (0x108, 0x190)]) == [
        (0x8, 1), (0x10c, 3), (0x0c, 5)]


@pytest.mark.parametrize("text, expected", [
    ("0x8-0x90", (8, 0x90)),
    ("8-144", (8, 144)),
])
def test_parse_range(text, expected):
    assert check_writes.parse_range(text) == expected


@pytest.mark.parametrize("text", ["0x8", "x-y", "0x90-0x8"])
def test_parse_range_rejects(text):
    with pytest.raises(argparse.ArgumentTypeError):
        check_writes.parse_range(text)


def test_parse_lines():
    assert check_writes.parse_lines("487-599") == (487, 599)
    assert check_writes.parse_lines("5-5") == (5, 5)


@pytest.mark.parametrize("text", ["7", "0x1-0x2", "0-3", "9-8"])
def test_parse_lines_rejects(text):
    with pytest.raises(argparse.ArgumentTypeError):
        check_writes.parse_lines(text)


def test_compare_pass_fail_and_empty():
    out = io.StringIO()
    assert check_writes.compare([(8, 1)], [(8, 1)], out)
    assert "PASS: all 1 writes match" in out.getvalue()

    out = io.StringIO()
    assert not check_writes.compare([(8, 1), (0xc, 2)], [(8, 1)], out)
    text = out.getvalue()
    assert "expected 2 writes, got 1" in text
    assert "-+0x00000c = 0x2" in text

    out = io.StringIO()
    assert not check_writes.compare([], [], out)
    assert "no expected writes" in out.getvalue()


def test_main(tmp_path, capsys):
    silicon = tmp_path / "trace.txt"
    silicon.write_text(MMIOTRACE)
    good = tmp_path / "good.log"
    good.write_text(qemu(0xfb4b0004, 0x3577) + qemu(0xfb4b0008, 0x10) +
                    qemu(0xfb4b010c, 0))
    bad = tmp_path / "bad.log"
    bad.write_text(qemu(0xfb1b0008, 0x200))

    args = ["--expected", str(silicon), "0x4b0008-0x4b0090",
            "0x4b0108-0x4b0190"]
    assert check_writes.main(args + ["--actual", str(good)]) == 0
    assert check_writes.main(args + ["--actual", str(bad)]) == 1
    assert "expected 2 writes, got 0" in capsys.readouterr().out

    # Only the write on line 7 is expected: the good capture has one more.
    args += ["--expected-lines", "7-7", "--actual", str(good)]
    assert check_writes.main(args) == 1
    assert "expected 1 writes, got 2" in capsys.readouterr().out
