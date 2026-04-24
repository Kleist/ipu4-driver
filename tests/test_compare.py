"""Placeholder test module for tools/trace/compare.py.

The comparison harness is added in the Step 2 commit; at Step 0 this file
exists only to anchor pytest's test-discovery glob and to fail loudly if
the compare module ever gets deleted without its tests being cleaned up.
"""
from __future__ import annotations

import pathlib

_COMPARE_PATH = (
    pathlib.Path(__file__).resolve().parent.parent / "tools" / "trace" / "compare.py"
)


def test_compare_module_pending():
    """compare.py does not exist yet. When Step 2 lands, this test is
    deleted and replaced by the real compare-diff tests."""
    assert not _COMPARE_PATH.exists(), (
        "tools/trace/compare.py now exists; delete this placeholder and "
        "replace tests/test_compare.py with the real compare-diff tests."
    )
