"""Unit tests for tools/dashboard/parse_status.py.

The parser turns the milestone bullets in ``STATUS.md`` into
``{id, title, state, summary}`` records. The dashboard relies on this
shape; tests pin the parsing rules so a STATUS.md format drift surfaces
loudly.
"""
from __future__ import annotations

import pathlib
import sys

import pytest

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "tools" / "dashboard"))
import parse_status  # noqa: E402


def test_done_bullet_classified_as_done():
    text = "- **M0 — in-tree migration:** done. lots of detail."
    out = parse_status.parse(text)
    assert out["total"] == 1
    m = out["milestones"][0]
    assert m["id"] == "M0"
    assert m["title"] == "in-tree migration"
    assert m["state"] == "done"


def test_continuation_lines_are_joined_into_summary():
    text = (
        "- **M0 — in-tree migration:**\n"
        "  done. continuation\n"
        "  line two of the same bullet.\n"
    )
    out = parse_status.parse(text)
    assert out["total"] == 1
    assert out["milestones"][0]["state"] == "done"
    assert "line two" in out["milestones"][0]["summary"]


def test_dotted_minor_id_supported():
    """M4.5 is a valid milestone id in STATUS.md."""
    text = "- **M4.5 — virt-sensor bridge bypass:** done."
    out = parse_status.parse(text)
    assert out["milestones"][0]["id"] == "M4.5"


def test_hyphenated_substep_id_supported():
    """M5b-3 carries a sub-step suffix; the regex must let it through."""
    text = "- **M5b-3 — CSI2 active route auto-installed:** done."
    out = parse_status.parse(text)
    assert out["milestones"][0]["id"] == "M5b-3"


def test_title_with_asterisk_supported():
    """``/dev/video*`` appears verbatim in M5a's title."""
    text = "- **M5a — probe completes, /dev/video* appears:** done."
    out = parse_status.parse(text)
    assert out["milestones"][0]["id"] == "M5a"
    assert "/dev/video*" in out["milestones"][0]["title"]


def test_multiline_title_supported():
    """STATUS.md occasionally wraps a long title across two lines."""
    text = (
        "- **M5c-1 — graceful STREAMON short-circuit when there's no\n"
        "  firmware:** done. ipu6_configure_spc panicked.\n"
    )
    out = parse_status.parse(text)
    assert out["total"] == 1
    assert out["milestones"][0]["id"] == "M5c-1"
    assert out["milestones"][0]["state"] == "done"


def test_deferred_bullet_classified_as_deferred():
    text = "- **M7 — mainline jobs:** after M5 is green."
    out = parse_status.parse(text)
    assert out["milestones"][0]["state"] == "deferred"


def test_mixed_state_with_not_yet_classified_as_in_progress():
    text = (
        "- **M6/M7/M8 — rebase cadence + mainline:** "
        "cron workflow in place; 6.18 not yet added."
    )
    out = parse_status.parse(text)
    assert out["milestones"][0]["state"] == "in_progress"


def test_unclassifiable_bullet_marked_unknown():
    text = "- **M99 — wibble:** total nonsense body text here."
    out = parse_status.parse(text)
    assert out["milestones"][0]["state"] == "unknown"


def test_state_counts_sum_to_total():
    text = (
        "- **M0 — alpha:** done.\n"
        "- **M1 — beta:** done.\n"
        "- **M2 — gamma:** after M1.\n"
    )
    out = parse_status.parse(text)
    assert out["total"] == 3
    assert sum(out["counts"].values()) == 3
    assert out["counts"].get("done") == 2
    assert out["counts"].get("deferred") == 1


def test_real_status_md_parses():
    real = _REPO_ROOT / "STATUS.md"
    if not real.is_file():
        pytest.skip("STATUS.md not present in checkout")
    out = parse_status.parse(real.read_text())
    # STATUS.md grows over time but never below ~10 milestones.
    assert out["total"] >= 10
    assert sum(out["counts"].values()) == out["total"]
    # Every milestone has the expected schema.
    for m in out["milestones"]:
        assert m["id"].startswith("M")
        assert m["state"] in {"done", "in_progress", "pending", "deferred", "unknown"}
        assert m["title"]


def test_main_emits_json_for_real_file(capsys):
    real = _REPO_ROOT / "STATUS.md"
    if not real.is_file():
        pytest.skip("STATUS.md not present in checkout")
    rc = parse_status.main([str(real)])
    assert rc == 0
    captured = capsys.readouterr()
    import json
    out = json.loads(captured.out)
    assert "milestones" in out
