"""Unit tests for tools/dashboard/parse_registers.py.

The parser converts the markdown table at the top of
``tools/notes/registers.md`` into a list of rows plus a confidence
histogram. Tests cover the exact format the dashboard contract relies
on; the real registers.md is also smoked at the end so a future format
drift is caught.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "tools" / "dashboard"))
import parse_registers  # noqa: E402


_MINIMAL = """\
# IPU4 register log

Some prose about the file.

| Offset | Block    | Driver caller          | Model behavior        | Confidence |
|--------|----------|------------------------|-----------------------|------------|
| 0x008  | Buttress | `ipu6-buttress.c` WDT  | Write: ignored.       | inferred   |
| 0x100  | CSE IPC  | `ipu6-buttress.c` IU2 | Write latched.        | guess      |
| 0x300  | Buttress | `ipu6-buttress.c` SEC  | R/W latched.          | doc        |

## Some unrelated section

| Foo | Bar |
|-----|-----|
| not a register | row |
"""


def test_counts_are_aggregated_per_confidence():
    out = parse_registers.parse(_MINIMAL)
    assert out["total"] == 3
    assert out["counts"] == {"doc": 1, "inferred": 1, "guess": 1}


def test_rows_carry_offset_block_caller_behaviour():
    out = parse_registers.parse(_MINIMAL)
    first = out["rows"][0]
    assert first["offset"] == "0x008"
    assert first["block"] == "Buttress"
    assert "WDT" in first["caller"]
    assert "Write: ignored." in first["behaviour"]
    assert first["confidence"] == "inferred"


def test_unrelated_table_is_ignored():
    """Only rows whose first cell looks like a hex offset are picked up."""
    out = parse_registers.parse(_MINIMAL)
    # 'not a register' row would inflate the count if we didn't gate
    # on the "0x..." prefix.
    assert out["total"] == 3


def test_unknown_confidence_falls_through_to_guess():
    table = """\
| Offset | Block | Driver caller | Model behavior | Confidence |
|---|---|---|---|---|
| 0x008 | Buttress | foo | bar | tbd |
"""
    out = parse_registers.parse(table)
    assert out["counts"]["guess"] == 1
    assert out["rows"][0]["confidence"] == "guess"


def test_confidence_normalised_to_lowercase():
    table = """\
| Offset | Block | Driver caller | Model behavior | Confidence |
|---|---|---|---|---|
| 0x008 | Buttress | foo | bar | DOC |
| 0x00c | Buttress | foo | bar | `Inferred` |
"""
    out = parse_registers.parse(table)
    assert out["counts"]["doc"] == 1
    assert out["counts"]["inferred"] == 1


def test_behaviour_with_pipes_is_preserved():
    """Some rows include `|` characters inside backticks. The parser
    must keep those in the behaviour cell rather than splitting them
    into spurious extra cells."""
    table = """\
| Offset | Block | Driver caller | Model behavior | Confidence |
|---|---|---|---|---|
| 0x008 | Buttress | foo | uses `a | b | c` | inferred |
"""
    out = parse_registers.parse(table)
    assert out["rows"][0]["behaviour"] == "uses `a | b | c`"


def test_real_registers_md_parses():
    real = _REPO_ROOT / "tools" / "notes" / "registers.md"
    if not real.is_file():
        pytest.skip("tools/notes/registers.md not present in checkout")
    out = parse_registers.parse(real.read_text())
    assert out["total"] > 30
    assert sum(out["counts"].values()) == out["total"]
    for level in ("doc", "inferred", "guess"):
        assert level in out["counts"]


def test_main_emits_json_for_real_file(capsys):
    real = _REPO_ROOT / "tools" / "notes" / "registers.md"
    if not real.is_file():
        pytest.skip("tools/notes/registers.md not present in checkout")
    rc = parse_registers.main([str(real)])
    assert rc == 0
    captured = capsys.readouterr()
    import json
    out = json.loads(captured.out)
    assert "rows" in out and "counts" in out
