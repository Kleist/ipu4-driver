"""Unit tests for tools/trace/normalize_report.py.

The module redacts intrinsically-volatile rows in a compare-mmio report
so the result is byte-stable across runs and can drive a strict diff
gate. Tests build small synthetic reports rather than depending on the
checked-in baseline so a baseline refresh doesn't churn the tests.
"""
from __future__ import annotations

import io
import pathlib
import sys
import textwrap

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "tools" / "trace"))
import normalize_report  # noqa: E402


# ---------- load_volatile -------------------------------------------------

def test_load_volatile_parses_entries(tmp_path):
    p = tmp_path / "vol.txt"
    p.write_text(textwrap.dedent("""
        # comment line
        buttress|0x05c   # inline comment
        isys spc|0x000

        csi2 port0|0x408  # multi-word region
    """))
    got = normalize_report.load_volatile(p)
    assert got == {
        ("buttress", "0x05c"),
        ("isys spc", "0x000"),
        ("csi2 port0", "0x408"),
    }


def test_load_volatile_normalizes_offset_case(tmp_path):
    p = tmp_path / "vol.txt"
    p.write_text("buttress|0X05C\n")
    assert normalize_report.load_volatile(p) == {("buttress", "0x05c")}


def test_load_volatile_rejects_malformed(tmp_path):
    p = tmp_path / "vol.txt"
    p.write_text("buttress 0x05c\n")  # missing '|'
    with pytest.raises(ValueError, match="malformed"):
        normalize_report.load_volatile(p)


# ---------- _parse_address_row -------------------------------------------

def test_parse_address_row_with_name():
    # Exact compare.py layout: 2-space indent, 14-wide region, 1 space,
    # 6-wide offset right-aligned, 2 spaces, name.
    line = "  buttress        0x05c  PWR_STATE"
    assert normalize_report._parse_address_row(line) == ("buttress", "0x05c")


def test_parse_address_row_multi_word_region():
    line = "  isys dmem       0x028  FW_COM_SEND_WR_POS"
    assert normalize_report._parse_address_row(line) == ("isys dmem", "0x028")


def test_parse_address_row_without_name():
    line = "  isys0 mmu       0x004"
    assert normalize_report._parse_address_row(line) == ("isys0 mmu", "0x004")


def test_parse_address_row_rejects_silicon_line():
    # 4 leading spaces — never an address row even if mid-string spans 0x.
    line = "    silicon: 0x0 0x4000"
    assert normalize_report._parse_address_row(line) is None


def test_parse_address_row_rejects_qemu_line():
    line = "    qemu:    0x20 0x1020 0x1028"
    assert normalize_report._parse_address_row(line) is None


def test_parse_address_row_rejects_short_line():
    assert normalize_report._parse_address_row("  (none)") is None


def test_parse_address_row_rejects_no_indent():
    assert normalize_report._parse_address_row("buttress 0x05c") is None


def test_parse_address_row_rejects_offset_without_0x():
    # The header row inside unimpl/extra sections looks like an address
    # row but the offset column reads literally "offset".
    line = "  region         offset  name                              R    W"
    assert normalize_report._parse_address_row(line) is None


# ---------- normalize ----------------------------------------------------

VOL = {
    ("buttress", "0x164"),  # TSC_LO
    ("isys dmem", "0x028"),  # FW_COM_SEND_WR_POS
    ("csi2 port0", "0x508"),  # RX_IRQ_STATUS
}


def test_normalize_redacts_qemu_accesses_header():
    text = "silicon accesses: 100\nqemu accesses:    51\n"
    out = normalize_report.normalize(text, VOL)
    assert "qemu accesses:    <volatile>\n" in out
    assert "qemu accesses:    51" not in out
    # silicon header is left alone.
    assert "silicon accesses: 100\n" in out


def test_normalize_redacts_volatile_qemu_line():
    text = textwrap.dedent("""\
        == read-value mismatches (1) ==
          buttress        0x164  TSC_LO
            silicon: 0xb3c1a58d 0xb3cb58bc
            qemu:    0xdeadbeef
        """)
    out = normalize_report.normalize(text, VOL)
    assert "    qemu:    <volatile>" in out
    assert "0xdeadbeef" not in out
    # silicon line is preserved verbatim.
    assert "    silicon: 0xb3c1a58d 0xb3cb58bc" in out


def test_normalize_keeps_non_volatile_qemu_line():
    text = textwrap.dedent("""\
        == read-value mismatches (1) ==
          csi2 port0      0x008  RX_CONFIG
            silicon: 0x0 0x3
            qemu:    0x0
        """)
    out = normalize_report.normalize(text, VOL)
    assert "    qemu:    0x0" in out


def test_normalize_handles_long_silicon_line_under_volatile_row():
    # Regression: a long silicon line under a volatile address row used
    # to be misparsed as a non-volatile address row, clearing the pending
    # flag so the qemu line below escaped redaction.
    text = textwrap.dedent("""\
        == read-value mismatches (1) ==
          isys dmem       0x028  FW_COM_SEND_WR_POS
            silicon: 0x0 0x1 0x2 0x3 ... (+4 more)
            qemu:    0x0 0x1 0x2 0x3 ... (+1 more)
        """)
    out = normalize_report.normalize(text, VOL)
    assert "    qemu:    <volatile>" in out
    assert "0x0 0x1 0x2 0x3 ... (+1 more)" not in out


def test_normalize_section_transition_resets_pending():
    # An address row in the unimpl section should never set the pending
    # flag — the redaction logic only runs in the mismatch section.
    text = textwrap.dedent("""\
        == unimplemented addresses (1) ==
          buttress        0x164  TSC_LO                              1    0
        == addresses only in qemu (1) ==
          buttress        0x164  TSC_LO                              0    1
        """)
    out = normalize_report.normalize(text, VOL)
    # Both rows preserved verbatim — no <volatile> substitution outside
    # the mismatch section.
    assert "<volatile>" not in out


def test_normalize_does_not_redact_qemu_in_extra_section():
    # The "addresses only in qemu" section uses a different row format
    # (with R/W columns) but happens to fall within an "extra" section.
    # Even if some logic accidentally treated it as mismatch, the qemu:
    # value line doesn't appear here at all — so a sanity check.
    text = textwrap.dedent("""\
        == addresses only in qemu (1) ==
          buttress        0x164  TSC_LO                              0    1
        """)
    out = normalize_report.normalize(text, VOL)
    assert "<volatile>" not in out


def test_normalize_preserves_trailing_newline():
    assert normalize_report.normalize("x\n", VOL).endswith("\n")
    assert not normalize_report.normalize("x", VOL).endswith("\n")


def test_normalize_empty_input():
    assert normalize_report.normalize("", VOL) == ""


# ---------- main / CLI ---------------------------------------------------

def _write_vol(tmp_path):
    p = tmp_path / "vol.txt"
    p.write_text("buttress|0x164\n")
    return p


def test_main_stdin_stdout(tmp_path, monkeypatch, capsys):
    vol_path = _write_vol(tmp_path)
    monkeypatch.setattr("sys.stdin", io.StringIO(
        "qemu accesses:    51\n"
        "== read-value mismatches (1) ==\n"
        "  buttress        0x164  TSC_LO\n"
        "    silicon: 0x1\n"
        "    qemu:    0xdead\n"
    ))
    rc = normalize_report.main(["--volatile", str(vol_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "qemu accesses:    <volatile>" in out
    assert "    qemu:    <volatile>" in out
    assert "0xdead" not in out


def test_main_file_in_file_out(tmp_path):
    vol_path = _write_vol(tmp_path)
    in_path = tmp_path / "report.txt"
    out_path = tmp_path / "report.normalized.txt"
    in_path.write_text(
        "qemu accesses:    51\n"
        "== read-value mismatches (1) ==\n"
        "  buttress        0x164  TSC_LO\n"
        "    silicon: 0x1\n"
        "    qemu:    0xdead\n"
    )
    rc = normalize_report.main([
        "--volatile", str(vol_path),
        "--in", str(in_path),
        "--out", str(out_path),
    ])
    assert rc == 0
    text = out_path.read_text()
    assert "<volatile>" in text
    assert "0xdead" not in text
