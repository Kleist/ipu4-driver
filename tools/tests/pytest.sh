#!/usr/bin/env bash
# Python unit tests + coverage gate for the internal trace tools.
#
# Scope: postprocess_trace.py today; tools/trace/compare.py once Step 2
# lands. Config lives in //pyproject.toml (tool.pytest.ini_options and
# tool.coverage.*). The gate fails if any test fails *or* combined line
# coverage across the --cov= sources drops below 90%.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
cd "$ROOT"

# CI image installs pytest + pytest-cov via apt/pip; local devs may need to
# `pip install --user pytest pytest-cov`. Surface that clearly rather than
# failing with a cryptic "command not found".
if ! command -v pytest >/dev/null 2>&1; then
	echo "pytest not found; install with 'pip3 install --user pytest pytest-cov'" >&2
	exit 2
fi

exec pytest tests/ \
	--cov=postprocess_trace \
	--cov=compare \
	--cov=normalize_report \
	--cov=parse_registers \
	--cov=parse_status \
	--cov-report=term-missing \
	--cov-fail-under=90 \
	"$@"
