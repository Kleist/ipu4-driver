"""Parse tools/notes/registers.md into rows + a confidence histogram.

The file's a single markdown table whose first column is the BAR offset
and last column is one of ``doc`` / ``inferred`` / ``guess``. Anything
outside the table (the prose intro, the trailing M3/M4 sections) is
ignored. The parser is permissive about whitespace/case in the
confidence cell so tiny rewordings don't break the dashboard.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

CONFIDENCE_LEVELS = ("doc", "inferred", "guess")

_TABLE_RE = re.compile(r"^\|\s*(0x[0-9a-fA-F][^|]*?)\s*\|")


def parse(text: str) -> dict:
    rows = []
    counts = {level: 0 for level in CONFIDENCE_LEVELS}
    for line in text.splitlines():
        if not _TABLE_RE.match(line):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        # Expected: offset | block | driver | behaviour | confidence
        if len(cells) < 5:
            continue
        offset, block, caller, behaviour, confidence = cells[0], cells[1], cells[2], " | ".join(cells[3:-1]), cells[-1]
        confidence = confidence.lower().strip("` ")
        if confidence not in CONFIDENCE_LEVELS:
            # Unknown classification — count under "guess" so it shows
            # up as needing attention rather than disappearing silently.
            confidence = "guess"
        counts[confidence] += 1
        rows.append({
            "offset": offset,
            "block": block,
            "caller": caller,
            "behaviour": behaviour,
            "confidence": confidence,
        })
    return {"rows": rows, "counts": counts, "total": len(rows)}


def main(argv=None):
    argv = argv or sys.argv[1:]
    if not argv:
        path = Path(__file__).resolve().parents[1] / "notes" / "registers.md"
    else:
        path = Path(argv[0])
    import json
    print(json.dumps(parse(path.read_text()), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
