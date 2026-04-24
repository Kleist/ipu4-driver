"""Unit tests for tools/trace/compare.py.

The compare module diffs silicon-side and QEMU-side mmiotrace captures
after they've been decoded to the JSONL schema ``postprocess_trace.py
--json`` emits. Tests build small synthetic record streams so the
divergence logic can be exercised without booting the harness.
"""
from __future__ import annotations

import io
import json
import pathlib
import sys

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
# Put tools/trace/ on sys.path so ``import compare`` resolves to the real
# source file. Using a bare ``import`` (rather than
# ``importlib.util.spec_from_file_location``) lets coverage.py attribute
# line hits to the module under its short name.
sys.path.insert(0, str(_REPO_ROOT / "tools" / "trace"))
import compare  # noqa: E402


def _rec(op, address, value, **extras):
    rec = {
        "op": op,
        "timestamp": extras.get("timestamp", 1.0),
        "address": address,
        "region": extras.get("region", "buttress "),
        "offset": extras.get("offset", address & 0xFFFFFF),
        "reg_name": extras.get("reg_name"),
        "value": value,
        "decoded": extras.get("decoded", {}),
    }
    return rec


def _write_jsonl(path, records):
    path.write_text("".join(json.dumps(r) + "\n" for r in records))


def test_is_ipu4_address_window():
    assert compare.is_ipu4_address(0x90000000)
    assert compare.is_ipu4_address(0x90FFFFFF)
    assert not compare.is_ipu4_address(0x8FFFFFFF)
    assert not compare.is_ipu4_address(0x91000000)
    assert not compare.is_ipu4_address(0x92200000)  # igb BAR in trace.txt


def test_load_records_filters_non_ipu4_and_blank_lines(tmp_path):
    path = tmp_path / "mixed.jsonl"
    # Include a blank line, a record missing required keys, and a
    # non-IPU4 address — all should be dropped.
    path.write_text(
        "\n"
        + json.dumps({"op": "R", "value": 0})  # no address
        + "\n"
        + json.dumps(_rec("R", 0x92200000, 0x1))  # igb address
        + "\n"
        + json.dumps(_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE"))
        + "\n"
    )
    recs = list(compare.load_records(path))
    assert len(recs) == 1
    assert recs[0]["reg_name"] == "PWR_STATE"


def test_load_records_accepts_file_object(tmp_path):
    # Passing an already-open file object should also work so the caller
    # can feed in-memory JSONL (useful for tests and future stdin piping).
    payload = json.dumps(_rec("R", 0x9000005C, 0xFA02003)) + "\n"
    fh = io.StringIO(payload)
    recs = list(compare.load_records(fh))
    assert len(recs) == 1


def test_diff_clean_when_streams_match():
    silicon = [_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE")]
    qemu = [_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE")]
    d = compare.diff(silicon, qemu)
    assert d.is_clean()
    assert d.unimplemented == []
    assert d.value_mismatches == []
    assert d.silicon_total == 1
    assert d.qemu_total == 1


def test_diff_reports_unimplemented_addresses():
    silicon = [
        _rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE", offset=0x5C),
        _rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE", offset=0x5C),
        _rec("W", 0x90000030, 0x1, reg_name="FW_RESET_CTL", offset=0x30),
    ]
    qemu = []  # QEMU didn't touch anything.
    d = compare.diff(silicon, qemu)
    assert not d.is_clean()
    assert len(d.unimplemented) == 2
    # Sorted by total access count descending → PWR_STATE first.
    assert d.unimplemented[0]["reg_name"] == "PWR_STATE"
    assert d.unimplemented[0]["reads"] == 2
    assert d.unimplemented[0]["writes"] == 0
    assert d.unimplemented[1]["reg_name"] == "FW_RESET_CTL"
    assert d.unimplemented[1]["writes"] == 1


def test_diff_reports_read_value_mismatch():
    # The M4.5 shift-bug regression: silicon returned 0xfa02003, QEMU
    # was returning the IPU6-layout value 0xf902003. compare must flag
    # the address and show both value sets.
    silicon = [
        _rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE"),
        _rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE"),
    ]
    qemu = [
        _rec("R", 0x9000005C, 0xF902003, reg_name="PWR_STATE"),
    ]
    d = compare.diff(silicon, qemu)
    assert d.unimplemented == []
    assert len(d.value_mismatches) == 1
    mm = d.value_mismatches[0]
    assert mm["address"] == 0x9000005C
    assert mm["silicon_values"] == [0xFA02003]
    assert mm["qemu_values"] == [0xF902003]
    assert 0xFA02003 in mm["only_silicon"]
    assert 0xF902003 in mm["only_qemu"]


def test_diff_write_value_divergence_not_flagged_as_value_mismatch():
    # Writes can legitimately carry different values across runs without
    # meaning the model is wrong; the driver may have taken a different
    # path earlier. value_mismatches only reports read-side divergence,
    # so this pair (same address touched, write values differ, same
    # address also read-accessed identically) must stay clean.
    silicon = [
        _rec("R", 0x9000009C, 0x1, reg_name="ISR_CLEAR"),
        _rec("W", 0x9000009C, 0x17, reg_name="ISR_CLEAR"),
    ]
    qemu = [
        _rec("R", 0x9000009C, 0x1, reg_name="ISR_CLEAR"),
        _rec("W", 0x9000009C, 0x1, reg_name="ISR_CLEAR"),  # different value
    ]
    d = compare.diff(silicon, qemu)
    assert d.value_mismatches == []


def test_diff_extra_in_qemu_counts_writes():
    # Exercise the write-side aggregation in the extra_in_qemu bucket:
    # QEMU wrote an address that silicon never touched.
    silicon = [_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE")]
    qemu = [
        _rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE"),
        _rec("W", 0x9000009C, 0x1, reg_name="ISR_CLEAR", offset=0x9C),
    ]
    d = compare.diff(silicon, qemu)
    assert len(d.extra_in_qemu) == 1
    assert d.extra_in_qemu[0]["reads"] == 0
    assert d.extra_in_qemu[0]["writes"] == 1


def test_format_addr_without_reg_name():
    # Covers the branch in _fmt_addr that fires when a mismatch hits an
    # offset we don't have a name for yet.
    out = compare._fmt_addr({"region": "buttress ", "offset": 0x318})
    assert "buttress" in out
    assert "0x318" in out
    # No reg_name means no trailing name token.
    assert "  " in out and not out.endswith("  ")


def test_diff_extra_in_qemu_is_informational_not_fatal():
    silicon = [_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE")]
    qemu = [
        _rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE"),
        _rec("R", 0x90000008, 0x0, reg_name="WDT"),  # only in QEMU
    ]
    d = compare.diff(silicon, qemu)
    assert d.is_clean(), "extra_in_qemu alone must not fail the diff"
    assert len(d.extra_in_qemu) == 1
    assert d.extra_in_qemu[0]["reg_name"] == "WDT"


def test_diff_value_mismatch_read_only_addresses():
    # Silicon reads an address and writes it with different payload.
    # QEMU has the same address but hasn't implemented it — should be
    # classified as unimplemented (first), not value_mismatch.
    silicon = [_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE")]
    qemu = []
    d = compare.diff(silicon, qemu)
    assert len(d.unimplemented) == 1
    assert d.value_mismatches == []


def test_format_diff_clean_report():
    d = compare.Diff(silicon_total=3, qemu_total=3)
    out = compare.format_diff(d)
    assert "silicon accesses: 3" in out
    assert "qemu accesses:    3" in out
    assert "== unimplemented addresses (0) ==" in out
    assert "== read-value mismatches (0) ==" in out
    assert "== addresses only in qemu (0) ==" in out
    assert "(none)" in out


def test_format_diff_shows_unimplemented_rows():
    silicon = [
        _rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE", offset=0x5C),
    ]
    d = compare.diff(silicon, [])
    out = compare.format_diff(d)
    assert "PWR_STATE" in out
    assert "0x05c" in out


def test_format_diff_shows_value_mismatch_rows():
    silicon = [_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE", offset=0x5C)]
    qemu = [_rec("R", 0x9000005C, 0xF902003, reg_name="PWR_STATE", offset=0x5C)]
    d = compare.diff(silicon, qemu)
    out = compare.format_diff(d)
    assert "silicon: 0xfa02003" in out
    assert "qemu:    0xf902003" in out


def test_format_diff_extra_in_qemu_section():
    silicon = [_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE", offset=0x5C)]
    qemu = [
        _rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE", offset=0x5C),
        _rec("R", 0x90000008, 0x0, reg_name="WDT", offset=0x8),
    ]
    d = compare.diff(silicon, qemu)
    out = compare.format_diff(d)
    assert "== addresses only in qemu (1) ==" in out
    assert "WDT" in out


def test_format_values_truncation():
    values = [i for i in range(10)]
    out = compare._fmt_values(values)
    assert "..." in out
    assert "(+6 more)" in out


def test_format_values_empty():
    assert compare._fmt_values([]) == "(none)"


def test_main_clean_streams_exit_zero(tmp_path, capsys):
    silicon = tmp_path / "silicon.jsonl"
    qemu = tmp_path / "qemu.jsonl"
    _write_jsonl(silicon, [_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE")])
    _write_jsonl(qemu, [_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE")])
    rc = compare.main([str(silicon), str(qemu)])
    captured = capsys.readouterr()
    assert rc == 0
    assert "silicon accesses: 1" in captured.out


def test_main_divergent_streams_exit_nonzero(tmp_path, capsys):
    silicon = tmp_path / "silicon.jsonl"
    qemu = tmp_path / "qemu.jsonl"
    _write_jsonl(silicon, [_rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE")])
    _write_jsonl(qemu, [])
    rc = compare.main([str(silicon), str(qemu)])
    assert rc == 1


def test_main_json_mode(tmp_path, capsys):
    silicon = tmp_path / "silicon.jsonl"
    qemu = tmp_path / "qemu.jsonl"
    _write_jsonl(silicon, [
        _rec("R", 0x9000005C, 0xFA02003, reg_name="PWR_STATE", offset=0x5C),
    ])
    _write_jsonl(qemu, [])
    compare.main([str(silicon), str(qemu), "--json"])
    captured = capsys.readouterr()
    obj = json.loads(captured.out.strip())
    assert obj["silicon_total"] == 1
    assert obj["qemu_total"] == 0
    assert len(obj["unimplemented"]) == 1
    assert obj["unimplemented"][0]["reg_name"] == "PWR_STATE"


def test_main_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        compare.main([str(tmp_path / "nope.jsonl"), str(tmp_path / "nope.jsonl")])


def test_diff_against_real_data_trace(tmp_path):
    """Regression: ensure diff scales to the real 1088-line trace.

    QEMU trace is empty, so the diff is essentially the "what addresses
    does silicon touch" worklist. Assert it reports the plan's priority
    targets (syscom, ISYS unispart IRQ, PWR_STATE) near the top."""
    silicon_path = tmp_path / "silicon.jsonl"
    # Generate the silicon JSONL using the sibling postprocess script.
    import subprocess
    pp = _REPO_ROOT / "postprocess_trace.py"
    trace = _REPO_ROOT / "data" / "trace.txt"
    silicon_path.write_bytes(subprocess.check_output(
        [sys.executable, str(pp), str(trace), "--json"]
    ))
    qemu_path = tmp_path / "qemu.jsonl"
    qemu_path.write_text("")
    d = compare.diff(compare.load_records(silicon_path), compare.load_records(qemu_path))
    assert d.silicon_total == 1088
    reg_names = {row["reg_name"] for row in d.unimplemented if row["reg_name"]}
    assert "FW_COM_RECV_RD_POS" in reg_names
    assert "PWR_STATE" in reg_names
    assert "IRQ_STATUS" in reg_names
