"""Unit tests for postprocess_trace.py.

Migrated from the inline tests that previously lived at the bottom of
postprocess_trace.py. Import the module by file path so the script stays
directly executable (./postprocess_trace.py ...) without being packaged.
"""
from __future__ import annotations

import errno
import importlib.util
import pathlib
import sys

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
_MODULE_PATH = _REPO_ROOT / "postprocess_trace.py"

_spec = importlib.util.spec_from_file_location("postprocess_trace", _MODULE_PATH)
postprocess_trace = importlib.util.module_from_spec(_spec)
sys.modules["postprocess_trace"] = postprocess_trace
_spec.loader.exec_module(postprocess_trace)

process = postprocess_trace.process
address_translate = postprocess_trace.address_translate
NAMED_REGIONS = postprocess_trace.NAMED_REGIONS


def test_strip_timestamp_write():
    elems = process("W 4 123 0x0 0x0 0x0 0")
    assert elems[2] == "(timestamp)"
    assert len(elems) == 5


def test_strip_timestamp_read():
    elems = process("R 4 123 0x0 0x0 0x0 0")
    assert elems[2] == "(timestamp)"


def test_strip_timestamp_map():
    elems = process("MAP 4 123 0x0 0x0 0x0 0")
    assert elems[1] == "(timestamp)"


def test_strip_timestamp_mark():
    elems = process("MARK 4 123 0x0 0x0 0x0 0")
    assert elems[1] == "(timestamp)"


def test_offset_names():
    elems = process("R 4 1.00 1 0x90000318 0x0 0x0 0")
    assert elems[3] == "buttress  0x318"


def test_address_translate_isys_mmu():
    assert address_translate(0x901E0004) == "isys0 mmu 0x004"
    assert address_translate(0x901E0204) == "isys1 mmu 0x104"


def test_address_translate_psys_mmu():
    assert address_translate(0x904B0004) == "psys0 mmu 0x004"
    assert address_translate(0x904B0104) == "psys1 mmu 0x004"
    assert address_translate(0x904B0400) == "psys1 mmu 0x300"


def test_address_translate_spc():
    # ISYS SPC has a SPC_STATUS_CTRL entry at 0x0 via SPC_REGS; PSYS SPC
    # does not, so offset 0 stays address-only.
    assert address_translate(0x90100000) == "isys spc  SPC_STATUS_CTRL(0x000)"
    assert address_translate(0x90400000) == "psys spc  0x000"


def test_address_translate_dmem():
    # ISYS DMEM has no entry at 0x0 (SYSCOM_STATE is at 0x08); PSYS DMEM
    # has BOOTLOADER_STATUS_OFFSET at 0x0.
    assert address_translate(0x90108000) == "isys dmem 0x000"
    assert (
        address_translate(0x90408000) == "psys dmem BOOTLOADER_STATUS_OFFSET(0x000)"
    )


def test_address_translate_out_of_region_warning(capsys):
    # 0x90800000 is past every declared region. The sorted lookup matches
    # the highest-base region ('psys isp3' at 0x90740000, max_offset 0x300)
    # and the resulting 0xc0000 offset triggers the warning on stderr.
    result = address_translate(0x90800000)
    captured = capsys.readouterr()
    assert result is not None
    assert "Warning: offset seems too big" in captured.err


def test_address_translate_named_buttress_reg():
    # 0x5c is PWR_STATE in the buttress region dict.
    assert address_translate(0x9000005C) == "buttress  PWR_STATE(0x05c)"


def test_address_translate_named_csi2_reg():
    # CSI2 port 0 base + RX_ENABLE offset 0x0.
    assert address_translate(0x90164000) == "csi2 port0 RX_ENABLE(0x000)"


def test_replace_ipu6_function_names():
    elems = process("MARK 123 In function ipu6_something")
    assert elems[4] == "ipuX_something"


def test_replace_ipu_function_names():
    elems = process("MARK 123 In function ipu_something")
    assert elems[4] == "ipuX_something"


def test_replace_bus_pm_runtime_device_names_mmu0():
    elems = process("MARK 123 bus_pm_runtime intel-ipu4-mmu0 something")
    assert "intel_ipu4.isys.24" in " ".join(elems)


def test_replace_bus_pm_runtime_device_names_mmu1():
    elems = process("MARK 123 bus_pm_runtime intel-ipu4-mmu1 something")
    assert "intel_ipu4.psys.24" in " ".join(elems)


def test_named_regions_is_sorted():
    sorted_regions = list(reversed(sorted(NAMED_REGIONS)))
    assert sorted_regions == NAMED_REGIONS


def test_reg_address_translate_named():
    reg_dict = {0x30: "FW_RESET_CTL"}
    assert (
        postprocess_trace.reg_address_translate(0x30, reg_dict)
        == "FW_RESET_CTL(0x030)"
    )


def test_reg_address_translate_unnamed():
    assert postprocess_trace.reg_address_translate(0x30, {}) == "0x030"


def test_main_runs_on_data_trace(capsys, monkeypatch):
    """Drive main() end-to-end on the real capture to exercise the argparse
    loop. Run in-process so pytest-cov sees the lines."""
    monkeypatch.setattr(
        sys, "argv", ["postprocess_trace.py", str(_REPO_ROOT / "data" / "trace.txt")]
    )
    postprocess_trace.main()
    captured = capsys.readouterr()
    # First line after PCIDEV headers should decode the first buttress read.
    assert "buttress" in captured.out
    # We expect thousands of lines of decoded output from trace.txt.
    assert captured.out.count("\n") > 100


def test_main_begin_end_filtering(tmp_path, capsys, monkeypatch):
    trace = tmp_path / "mini.txt"
    trace.write_text(
        "VERSION 20070824\n"
        "R 4 1.0 1 0x90000008 0x0 0x0 0\n"
        "MARK 1.1 BEGIN_MARKER\n"
        "R 4 1.2 1 0x9000005c 0xfa02003 0x0 0\n"
        "MARK 1.3 END_MARKER\n"
        "R 4 1.4 1 0x90000090 0x0 0x0 0\n"
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "postprocess_trace.py",
            str(trace),
            "--begin",
            "BEGIN_MARKER",
            "--end",
            "END_MARKER",
        ],
    )
    postprocess_trace.main()
    captured = capsys.readouterr()
    # Pre-begin WDT read and post-end ISR_STATUS read must be filtered out.
    assert "0x90000008" not in captured.out
    assert "ISR_STATUS" not in captured.out
    # The PWR_STATE read between the markers must be present.
    assert "PWR_STATE" in captured.out
    # Begin/End marker diagnostics land on stderr.
    assert "Begin found" in captured.err
    assert "End found" in captured.err


def test_eval_simple_expr_plain_int():
    assert postprocess_trace._eval_simple_expr("0x20") == 0x20
    assert postprocess_trace._eval_simple_expr("42") == 42
    assert postprocess_trace._eval_simple_expr("0") == 0


def test_eval_simple_expr_bit():
    assert postprocess_trace._eval_simple_expr("BIT(0)") == 1
    assert postprocess_trace._eval_simple_expr("BIT(22)") == (1 << 22)


def test_eval_simple_expr_genmask():
    assert postprocess_trace._eval_simple_expr("GENMASK(13, 12)") == 0x3000
    # Inclusive on both ends: GENMASK(23, 19) = bits 19..23 = 0xf80000.
    assert postprocess_trace._eval_simple_expr("GENMASK(23, 19)") == 0xF80000


def test_eval_simple_expr_shift():
    assert postprocess_trace._eval_simple_expr("(0xf << 20)") == 0xF00000
    assert postprocess_trace._eval_simple_expr("(0x1f << 24)") == 0x1F000000


def test_eval_simple_expr_unknown():
    assert postprocess_trace._eval_simple_expr("(REG_BASE + 4)") is None
    assert postprocess_trace._eval_simple_expr("FOO | BAR") is None
    assert postprocess_trace._eval_simple_expr("") is None


def test_eval_simple_expr_strips_trailing_comment():
    assert postprocess_trace._eval_simple_expr("0x5c  /* PWR_STATE */") == 0x5C


def test_parse_header_missing_file(tmp_path):
    # Graceful fallback when the header dir doesn't ship with the script.
    assert postprocess_trace.parse_header(tmp_path / "nope.h") == {}


def test_parse_header_simple_defines(tmp_path):
    header = tmp_path / "regs.h"
    header.write_text(
        "#define FOO_REG_A 0x10\n"
        "#define FOO_REG_B\t0x20   /* inline comment */\n"
        "#define FOO_FIELD_MASK (0xf << 4)\n"
        "#define FOO_FIELD_SHIFT 4\n"
        "#define FOO_BIT BIT(3)\n"
        "#define FOO_MULTI_MASK\tGENMASK(7, 4)\n"
        "#define FOO_FUNC(x) ((x) << 2)\n"
        "#define FOO_EXPR (FOO_REG_A + 4)\n"
    )
    result = postprocess_trace.parse_header(header)
    assert result["FOO_REG_A"] == 0x10
    assert result["FOO_REG_B"] == 0x20
    assert result["FOO_FIELD_MASK"] == 0xF0
    assert result["FOO_FIELD_SHIFT"] == 4
    assert result["FOO_BIT"] == 8
    assert result["FOO_MULTI_MASK"] == 0xF0
    assert "FOO_FUNC" not in result
    assert "FOO_EXPR" not in result


def test_parse_header_line_continuation(tmp_path):
    header = tmp_path / "multi.h"
    header.write_text(
        "#define LONG_MASK \\\n"
        "\tGENMASK(12, 8)\n"
    )
    assert postprocess_trace.parse_header(header)["LONG_MASK"] == 0x1F00


def test_lowest_set_bit():
    assert postprocess_trace._lowest_set_bit(0x8) == 3
    assert postprocess_trace._lowest_set_bit(0xF00000) == 20
    assert postprocess_trace._lowest_set_bit(0) == 0


def test_prefix_rank_priority():
    ipu4 = postprocess_trace._prefix_rank("IPU4_BUTTRESS_")
    butt = postprocess_trace._prefix_rank("BUTTRESS_")
    ipu6 = postprocess_trace._prefix_rank("IPU6_BUTTRESS_")
    unknown = postprocess_trace._prefix_rank("FOO_")
    assert ipu4 < butt < ipu6
    # Unknown prefix ranks below every known one.
    assert unknown >= max(ipu4, butt, ipu6)


def test_build_field_tables_pwr_state_prefers_ipu4():
    defs = {
        # IPU4 winner.
        "IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_SHIFT": 20,
        "IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_MASK": 0xF00000,
        "IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_IS_RDY": 0xA,
        # Competing IPU6 definition with different shift/mask.
        "BUTTRESS_PWR_STATE_IS_PWR_FSM_MASK": 0xF80000,
        # Shift-only entry with no MASK should be ignored.
        "STRAY_PWR_STATE_ONLY_SHIFT_SHIFT": 4,
    }
    tables = postprocess_trace.build_field_tables(defs)
    fields = {name: (shift, mask, enums) for name, shift, mask, enums in tables["PWR_STATE"]}
    shift, mask, enums = fields["IS_PWR_FSM"]
    assert shift == 20
    assert mask == 0xF00000
    assert enums[0xA] == "IS_RDY"
    assert "ONLY_SHIFT" not in fields


def test_build_field_tables_drops_ipu6_only_fields_when_ipu4_defines_any():
    # Real-silicon case: IPU4 defines IS_PWR_FSM at bits 23:20 while the
    # upstream IPU6 header defines IS_PWR at bits 4:3. Those bits mean
    # something different on IPU4 silicon, so when *any* IPU4 field is
    # present for a register the IPU6-only fields must be dropped from
    # the decoder for that register.
    defs = {
        "IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_SHIFT": 20,
        "IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_MASK": 0xF00000,
        "IPU6_BUTTRESS_PWR_STATE_IS_PWR_SHIFT": 3,
        "IPU6_BUTTRESS_PWR_STATE_IS_PWR_MASK": 0x18,
    }
    tables = postprocess_trace.build_field_tables(defs)
    names = {f[0] for f in tables["PWR_STATE"]}
    assert "IS_PWR_FSM" in names
    assert "IS_PWR" not in names


def test_build_field_tables_keeps_ipu6_fields_when_no_ipu4_exists():
    # A register with no IPU4 defines at all should still decode via
    # whatever header we have. Exercise this with a fake register named
    # 'PWR_STATE' whose only define comes from the IPU6 prefix — the
    # filter must not strip it.
    defs = {
        "IPU6_BUTTRESS_PWR_STATE_FALLBACK_SHIFT": 4,
        "IPU6_BUTTRESS_PWR_STATE_FALLBACK_MASK": 0xF0,
    }
    tables = postprocess_trace.build_field_tables(defs)
    names = {f[0] for f in tables["PWR_STATE"]}
    assert "FALLBACK" in names


def test_build_field_tables_derives_shift_from_mask():
    defs = {
        "BUTTRESS_PWR_STATE_HH_STATUS_MASK": 0x3000,
    }
    tables = postprocess_trace.build_field_tables(defs)
    fields = {name: shift for name, shift, _mask, _enums in tables["PWR_STATE"]}
    assert fields["HH_STATUS"] == 12


def test_field_tables_populated_from_repo_headers():
    # Sanity-check that the module-level FIELD_TABLES picked up PWR_STATE
    # fields from the real headers under kernel/ipu4/.
    fields = postprocess_trace.FIELD_TABLES.get("PWR_STATE", [])
    field_names = {f[0] for f in fields}
    assert {"IS_PWR_FSM", "PS_PWR_FSM", "HH_STATUS"} <= field_names


def test_decode_bitfields_pwr_state():
    out = postprocess_trace.decode_bitfields("PWR_STATE", 0x0FA02003)
    # The driver's target "all power islands ready" constant.
    assert "IS_PWR_FSM=0xa(IS_RDY)" in out
    assert "PS_PWR_FSM=0xf(PS_PWR_UP)" in out


def test_decode_bitfields_unknown_register():
    assert postprocess_trace.decode_bitfields("NOT_A_REG", 0xFFFFFFFF) == ""


def test_decode_bitfields_zero_value_yields_empty():
    # Every field extracts to 0, so the compact decoder returns an empty
    # string — the caller drops the annotation rather than render "[]".
    assert postprocess_trace.decode_bitfields("PWR_STATE", 0) == ""


def test_decode_bitfields_dict_includes_zero_values():
    # Value with only HH_STATUS set: IS_PWR_FSM and PS_PWR_FSM are zero
    # but must still appear in the dict so consumers can assert on them.
    out = postprocess_trace.decode_bitfields_dict("PWR_STATE", 0x3000)
    assert "HH_STATUS" in out
    assert out["HH_STATUS"]["value"] == 3
    assert "IS_PWR_FSM" in out
    assert out["IS_PWR_FSM"] == {"value": 0, "enum": "IDLE"}
    assert "PS_PWR_FSM" in out
    assert out["PS_PWR_FSM"]["value"] == 0


def test_decode_bitfields_dict_unknown_register():
    assert postprocess_trace.decode_bitfields_dict("NOT_A_REG", 0) == {}


def test_lookup_address_inside_region():
    res = postprocess_trace.lookup_address(0x9000005C)
    assert res is not None
    assert res.region == "buttress "
    assert res.reg_name == "PWR_STATE"
    assert res.offset == 0x5C
    assert "PWR_STATE" in res.display


def test_lookup_address_below_any_region():
    assert postprocess_trace.lookup_address(0x0) is None


def test_process_annotates_pwr_state():
    elems = process("R 4 1.0 1 0x9000005c 0x0fa02003 0x0 0")
    joined = " ".join(elems)
    assert "PWR_STATE(0x05c)" in joined
    # Bitfield annotation is appended to the value token.
    assert "IS_PWR_FSM=0xa(IS_RDY)" in joined


def test_process_unannotated_when_register_unknown():
    # Address in the middle of the buttress region but without a named reg.
    elems = process("R 4 1.0 1 0x90000318 0x0 0x0 0")
    assert elems[3] == "buttress  0x318"
    # No named register → no bitfield annotation.
    assert "[" not in " ".join(elems)


def test_parse_rw_valid_line():
    rec = postprocess_trace._parse_rw("R 4 1.5 1 0x9000005c 0xfa02003 0x0 0")
    assert rec["op"] == "R"
    assert rec["timestamp"] == 1.5
    assert rec["address"] == 0x9000005C
    assert rec["reg_name"] == "PWR_STATE"
    assert rec["value"] == 0xFA02003


def test_parse_rw_rejects_non_rw():
    assert postprocess_trace._parse_rw("MARK 1.0 hello") is None


def test_parse_rw_rejects_malformed():
    assert postprocess_trace._parse_rw("R 4 notanum 1 0xZZZ 0x0 0 0") is None


def test_summarise_counts_reads_and_writes():
    lines = [
        "R 4 1.0 1 0x9000005c 0xfa02003 0x0 0",
        "R 4 1.1 1 0x9000005c 0xfa02003 0x0 0",
        "W 4 1.2 1 0x9000009c 0x1 0x0 0",
        "MARK 1.3 some marker",
        "R 4 1.4 1 0x9000005c 0xfa02003 0x0 0",
    ]
    rows = postprocess_trace.summarise(lines)
    by_offset = {(r["region"], r["offset"]): r for r in rows}
    pwr = by_offset[("buttress ", 0x5C)]
    assert pwr["reads"] == 3
    assert pwr["writes"] == 0
    assert pwr["distinct_values"] == 1
    assert pwr["name"] == "PWR_STATE"
    clear = by_offset[("buttress ", 0x9C)]
    assert clear["writes"] == 1
    # Rows are sorted by total access count descending.
    assert rows[0] == pwr


def test_format_summary_header_and_rows():
    rows = [
        {"region": "buttress ", "offset": 0x5C, "name": "PWR_STATE",
         "reads": 3, "writes": 0, "distinct_values": 1}
    ]
    out = postprocess_trace.format_summary(rows)
    assert "region" in out  # header row
    assert "PWR_STATE" in out
    assert "0x5c" in out


def test_record_to_json_shape():
    rec = postprocess_trace._parse_rw("R 4 1.0 1 0x9000005c 0xfa02003 0x0 0")
    obj = postprocess_trace.record_to_json(rec)
    assert obj["reg_name"] == "PWR_STATE"
    assert "IS_PWR_FSM" in obj["decoded"]
    assert obj["decoded"]["IS_PWR_FSM"]["enum"] == "IS_RDY"


def test_record_to_json_no_decoder():
    rec = postprocess_trace._parse_rw("W 4 1.0 1 0x90164400 0xff 0x0 0")
    # PART_IRQ_EDGE has no bitfield decoder, so decoded is empty.
    obj = postprocess_trace.record_to_json(rec)
    assert obj["decoded"] == {}


def test_main_summary_mode(tmp_path, capsys, monkeypatch):
    trace = tmp_path / "mini.txt"
    trace.write_text(
        "R 4 1.0 1 0x9000005c 0xfa02003 0x0 0\n"
        "R 4 1.1 1 0x9000005c 0xfa02003 0x0 0\n"
    )
    monkeypatch.setattr(sys, "argv", ["postprocess_trace.py", str(trace), "--summary"])
    postprocess_trace.main()
    captured = capsys.readouterr()
    assert "PWR_STATE" in captured.out
    assert "0x5c" in captured.out


def test_main_json_mode(tmp_path, capsys, monkeypatch):
    trace = tmp_path / "mini.txt"
    trace.write_text(
        "R 4 1.0 1 0x9000005c 0xfa02003 0x0 0\n"
        "MARK 1.1 junk\n"
        "W 4 1.2 1 0x90000098 0x1 0x0 0\n"
    )
    monkeypatch.setattr(sys, "argv", ["postprocess_trace.py", str(trace), "--json"])
    postprocess_trace.main()
    captured = capsys.readouterr()
    import json
    recs = [json.loads(line) for line in captured.out.strip().split("\n")]
    # Two JSON records (MARK line is skipped by _parse_rw).
    assert len(recs) == 2
    assert recs[0]["reg_name"] == "PWR_STATE"
    assert recs[0]["decoded"]["IS_PWR_FSM"]["enum"] == "IS_RDY"
    assert recs[1]["op"] == "W"


def test_main_json_respects_begin_end(tmp_path, capsys, monkeypatch):
    trace = tmp_path / "mini.txt"
    trace.write_text(
        "R 4 1.0 1 0x90000008 0x0 0x0 0\n"
        "MARK 1.1 BEGIN_MARKER\n"
        "R 4 1.2 1 0x9000005c 0xfa02003 0x0 0\n"
        "MARK 1.3 END_MARKER\n"
        "R 4 1.4 1 0x90000090 0x0 0x0 0\n"
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "postprocess_trace.py", str(trace), "--json",
            "--begin", "BEGIN_MARKER", "--end", "END_MARKER",
        ],
    )
    postprocess_trace.main()
    captured = capsys.readouterr()
    import json
    recs = [json.loads(line) for line in captured.out.strip().split("\n") if line]
    assert len(recs) == 1
    assert recs[0]["reg_name"] == "PWR_STATE"


def test_main_swallows_epipe(tmp_path, monkeypatch):
    """Covers the EPIPE catch: if stdout is broken mid-run (piping into
    `head`, etc.) main() must exit cleanly with status 0 rather than
    propagating BrokenPipeError."""
    trace = tmp_path / "tiny.txt"
    trace.write_text("R 4 1.0 1 0x90000008 0x0 0x0 0\n")
    monkeypatch.setattr(sys, "argv", ["postprocess_trace.py", str(trace)])

    def broken_print(*_args, **_kwargs):
        raise BrokenPipeError(errno.EPIPE, "broken pipe")

    monkeypatch.setattr("builtins.print", broken_print)
    with pytest.raises(SystemExit) as exc_info:
        postprocess_trace.main()
    assert exc_info.value.code == 0


def test_main_reraises_non_epipe(tmp_path, monkeypatch):
    """Non-EPIPE OSErrors must propagate, not be swallowed by the EPIPE
    handler."""
    trace = tmp_path / "tiny.txt"
    trace.write_text("R 4 1.0 1 0x90000008 0x0 0x0 0\n")
    monkeypatch.setattr(sys, "argv", ["postprocess_trace.py", str(trace)])

    def broken_print(*_args, **_kwargs):
        raise OSError(errno.EACCES, "permission denied")

    monkeypatch.setattr("builtins.print", broken_print)
    with pytest.raises(OSError):
        postprocess_trace.main()
