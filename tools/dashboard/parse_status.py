"""Parse STATUS.md into a milestone checklist.

Looks for bullets of the form ``- **M<id> — <title>:** <body>`` where
``<title>`` may wrap onto a continuation line. The body is the prose
that follows until the next top-level ``- **M`` bullet or the next
non-list paragraph. State is classified from the body keyword:
``done``, ``in progress``, ``deferred``, ``not yet`` (treated as
in-progress when mixed with other text).
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

# Match a milestone bullet across the whole file. The title is
# non-greedy so it ends at the first `**`, even if it wraps a line.
# The body extends until the next milestone bullet or the end of the
# document.
_BULLET_RE = re.compile(
    r"^- \*\*(?P<id>M[0-9][\w./]*(?:-\w+)?)\s*[—-]\s*(?P<title>.+?)\*\*"
    r"[:\s]*(?P<body>.*?)(?=^- \*\*M[0-9]|\Z)",
    re.DOTALL | re.MULTILINE,
)


def _classify(body: str) -> str:
    head = body.lower().lstrip()
    if head.startswith("done"):
        return "done"
    if head.startswith(("in progress", "pending")):
        return "in_progress"
    if head.startswith(("deferred", "after ", "not yet")):
        return "deferred"
    if "not yet" in head or "deferred" in head:
        return "in_progress"
    return "unknown"


def parse(text: str) -> dict:
    milestones = []
    counts: dict[str, int] = {}
    for m in _BULLET_RE.finditer(text):
        body = re.sub(r"\s+", " ", m.group("body")).strip()
        state = _classify(body)
        counts[state] = counts.get(state, 0) + 1
        title = re.sub(r"\s+", " ", m.group("title")).strip().rstrip(":")
        milestones.append({
            "id": m.group("id").strip(),
            "title": title,
            "state": state,
            "summary": body,
        })
    return {"milestones": milestones, "counts": counts, "total": len(milestones)}


def main(argv=None):
    argv = argv or sys.argv[1:]
    if not argv:
        path = Path(__file__).resolve().parents[2] / "STATUS.md"
    else:
        path = Path(argv[0])
    import json
    print(json.dumps(parse(path.read_text()), indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
