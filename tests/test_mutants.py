"""Unit tests for tools/mutation/mutants.py (the mutant generator)."""
from __future__ import annotations

import pathlib
import sys

_REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "tools" / "mutation"))
import mutants  # noqa: E402

SRC = """\
#include <linux/io.h>
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static const int tbl[] = { 1, 2 };

static int f(struct dev *d, int x)
{
	/* x < 1 in a comment */
	dev_dbg(d, "x %d\\n", x + 1);
	if (x < 1 && d)
		return -EINVAL;
	writel(x | 4, d->base + 8);
	usleep_range(500, 1000);
#ifdef IPU6
	x = x - 2;
#else
	x = x * 3;
#endif
	return 0;
}
"""


def _descs():
    return {(m["line"], m["op"], m["desc"]) for m in mutants.generate(SRC)}


def test_operators():
    d = _descs()
    assert (10, "ror", "< -> <=") in d
    assert (10, "lcr", "&& -> ||") in d
    assert (10, "neg", "if (!(...))") in d
    assert (11, "ret", "return -EINVAL -> return 0") in d
    assert (12, "bor", "| -> &") in d
    assert (12, "aor", "+ -> -") in d
    assert (12, "sdl", "writel(...); deleted") in d
    assert (12, "crp", "8 -> 9") in d
    assert (19, "ret", "return 0 -> return -EINVAL") in d


def test_skips_noise():
    lines = {line for line, _, _ in _descs()}
    # File scope, macros, comments and the log call's arguments.
    assert not lines & {2, 4, 8, 9}
    # Never-compiled #ifdef IPU6 branch; its #else branch is mutated.
    assert 15 not in lines
    assert (17, "crp", "3 -> 4") in _descs()
    # Delay arguments.
    assert not any(line == 13 for line, op, _ in _descs() if op == "crp")


def test_apply():
    m = next(m for m in mutants.generate(SRC) if m["op"] == "neg")
    assert "if (!(x < 1 && d))" in mutants.apply(SRC, m["edits"])
