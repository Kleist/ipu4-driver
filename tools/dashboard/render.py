"""Render the GitHub Pages dashboard.

Reads a single JSON document on stdin describing the dashboard inputs
(see tools/dashboard/build.sh for what shape that is) and writes
``site/index.html`` plus a copy of the embedded history at
``site/data/history.json`` so other consumers can read the trend
without scraping the page.

The whole site is static — Chart.js is loaded from a CDN at runtime,
so trend graphs render in the browser from JSON inlined into the
page. No server, no JS build step.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import jinja2

_HERE = Path(__file__).resolve().parent


def render(payload: dict, out_dir: Path) -> None:
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(str(_HERE / "templates")),
        autoescape=jinja2.select_autoescape(["html"]),
        trim_blocks=True,
        lstrip_blocks=True,
    )
    env.filters["pct"] = _pct
    env.filters["json_dump"] = lambda v: json.dumps(v, separators=(",", ":"))
    tmpl = env.get_template("index.html.j2")
    html = tmpl.render(**payload)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "index.html").write_text(html)
    (out_dir / "style.css").write_text((_HERE / "templates" / "style.css").read_text())
    data_dir = out_dir / "data"
    data_dir.mkdir(exist_ok=True)
    (data_dir / "history.json").write_text(json.dumps(payload.get("history", {}), indent=2))


def _pct(value):
    if value is None:
        return "—"
    return f"{float(value):.1f}%"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", required=True, help="output directory (will be created)")
    p.add_argument("--input", help="JSON input file; default reads stdin")
    args = p.parse_args(argv)

    if args.input:
        payload = json.loads(Path(args.input).read_text())
    else:
        payload = json.load(sys.stdin)
    render(payload, Path(args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
