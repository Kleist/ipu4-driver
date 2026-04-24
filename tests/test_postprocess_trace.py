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
