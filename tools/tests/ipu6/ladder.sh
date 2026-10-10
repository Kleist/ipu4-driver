#!/usr/bin/env bash
# The rung_* functions are only called indirectly ("rung_$r").
# shellcheck disable=SC2317
#
# Run the ipu6-flavor test ladder (plan: tools/notes/ipu6-ipu4-plan.md).
#
#   tools/tests/ipu6/ladder.sh            run every known rung
#   tools/tests/ipu6/ladder.sh RUNG...    run only these rungs (in ladder order)
#   tools/tests/ipu6/ladder.sh --list     print the rung names
#
# Every rung ends as PASS, FAIL, SKIP (a rung it depends on did not pass)
# or NOT-IMPLEMENTED. The exit status is non-zero only when a rung listed
# in tools/tests/ipu6/enabled-tests ends in anything but PASS, or when
# that file names an unknown rung. Per-rung logs go to
# tools/tests/out/ipu6-ladder/<rung>.log, the whole table as JSON to
# tools/tests/out/ipu6-ladder/results.json (read by the Pages dashboard).
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$ROOT/tools/linux-ipu6}"
PATCH_DIR="$ROOT/patches/ipu6-ipu4"
ENABLED_FILE="${IPU4_IPU6_ENABLED_TESTS:-$HERE/enabled-tests}"
OUT="$ROOT/tools/tests/out/ipu6-ladder"

# Ladder order. "rung:dependency": when both are run, a rung is SKIPped
# unless its dependency PASSed.
RUNGS=(
	series-applies
	build:series-applies
	checkpatch
	vm-boot:build
	module-load:vm-boot
	probe-identify:module-load
	probe-buttress:probe-identify
	probe-mmu:probe-buttress
	probe-fw:probe-mmu
	probe-complete:probe-fw
	fw-open:probe-complete
	stream-on:fw-open
	stream-pattern:stream-on
	csi2-sof-irq:stream-on
	native-bind:vm-boot
)

# --- rungs ---------------------------------------------------------------
# Each rung_<name> function runs with stdout/stderr going to its log and
# returns 0 for PASS, non-zero for FAIL. A rung in RUNGS without a
# rung_<name> function is NOT-IMPLEMENTED.

rung_series-applies() {
	"$ROOT/tools/ipu6/bootstrap.sh"
}

rung_build() {
	local log="$OUT/build.compile.log"
	IPU4_IPU6_CLEAN=1 IPU4_IPU6_BUILD_LOG="$log" "$ROOT/tools/ipu6/build.sh" ||
		return 1
	# "cleanly": no compiler warnings at W=1. modpost's "WARNING:" lines
	# about unresolved symbols are expected (no vmlinux) and not matched.
	if grep -E '(warning|error):' "$log"; then
		echo "!!! compiler warnings in the W=1 build (see above)"
		return 1
	fi
}

rung_checkpatch() {
	local cp="$LINUX_DIR/scripts/checkpatch.pl" p rc=0
	shopt -s nullglob
	local patches=("$PATCH_DIR"/*.patch)
	shopt -u nullglob
	if (( ${#patches[@]} == 0 )); then
		echo "no patches in ${PATCH_DIR#"$ROOT"/}; nothing to check"
		return 0
	fi
	if [[ ! -f "$cp" ]]; then
		echo "missing $cp; run tools/ipu6/bootstrap.sh first"
		return 1
	fi
	for p in "${patches[@]}"; do
		echo ">>> checkpatch --strict ${p#"$ROOT"/}"
		(cd "$LINUX_DIR" && perl "$cp" --strict --color=never "$p") || rc=1
	done
	return "$rc"
}

# The VM rungs (vm-boot onwards) are implemented by later PRs of the plan.

# --- driver --------------------------------------------------------------

declare -A DEP=() RESULT=() KNOWN=() DUR=()
ORDER=()
for entry in "${RUNGS[@]}"; do
	name="${entry%%:*}"
	ORDER+=("$name")
	KNOWN[$name]=1
	[[ "$entry" == *:* ]] && DEP[$name]="${entry#*:}"
done

if [[ "${1:-}" == "--list" ]]; then
	printf '%s\n' "${ORDER[@]}"
	exit 0
fi

declare -A ENABLED=()
bad_config=0
if [[ -f "$ENABLED_FILE" ]]; then
	while IFS= read -r line || [[ -n "$line" ]]; do
		line="${line%%#*}"
		line="$(echo "$line" | tr -d '[:space:]')"
		[[ -z "$line" ]] && continue
		if [[ -z "${KNOWN[$line]:-}" ]]; then
			echo "enabled-tests: unknown rung '$line'" >&2
			bad_config=1
			continue
		fi
		ENABLED[$line]=1
	done < "$ENABLED_FILE"
fi

declare -A WANT=()
if (( $# > 0 )); then
	for r in "$@"; do
		if [[ -z "${KNOWN[$r]:-}" ]]; then
			echo "unknown rung '$r' (see --list)" >&2
			exit 2
		fi
		WANT[$r]=1
	done
else
	for r in "${ORDER[@]}"; do WANT[$r]=1; done
fi

mkdir -p "$OUT"
for r in "${ORDER[@]}"; do
	[[ -z "${WANT[$r]:-}" ]] && continue
	dep="${DEP[$r]:-}"
	log="$OUT/$r.log"
	if ! declare -F "rung_$r" > /dev/null; then
		RESULT[$r]=NOT-IMPLEMENTED
		echo "not implemented yet" > "$log"
		continue
	fi
	if [[ -n "$dep" && -n "${WANT[$dep]:-}" && "${RESULT[$dep]:-}" != PASS ]]; then
		RESULT[$r]=SKIP
		echo "skipped: depends on $dep (${RESULT[$dep]:-not run})" > "$log"
		continue
	fi
	echo "::group::rung $r"
	start=$SECONDS
	"rung_$r" 2>&1 | tee "$log"
	rc=${PIPESTATUS[0]}
	echo "::endgroup::"
	if (( rc == 0 )); then
		RESULT[$r]=PASS
	else
		RESULT[$r]=FAIL
	fi
	DUR[$r]=$((SECONDS - start))
	echo "rung $r: ${RESULT[$r]} ($((SECONDS - start))s)"
done

status=$bad_config
echo
printf '%-16s %-8s %s\n' RUNG ENABLED RESULT
printf '%-16s %-8s %s\n' ---- ------- ------
for r in "${ORDER[@]}"; do
	[[ -z "${WANT[$r]:-}" ]] && continue
	en=no
	if [[ -n "${ENABLED[$r]:-}" ]]; then
		en=yes
		[[ "${RESULT[$r]}" != PASS ]] && status=1
	fi
	printf '%-16s %-8s %s\n' "$r" "$en" "${RESULT[$r]}"
done
# Enabled rungs that weren't selected on the command line aren't run,
# so they can't fail; say so rather than hiding it.
for r in "${!ENABLED[@]}"; do
	[[ -z "${WANT[$r]:-}" ]] && echo "note: enabled rung '$r' not selected"
done

if [[ -n "${GITHUB_STEP_SUMMARY:-}" ]]; then
	{
		echo "### ipu6 ladder"
		echo
		echo "| Rung | Enabled | Result |"
		echo "|---|---|---|"
		for r in "${ORDER[@]}"; do
			[[ -z "${WANT[$r]:-}" ]] && continue
			echo "| \`$r\` | ${ENABLED[$r]:+yes} | ${RESULT[$r]} |"
		done
	} >> "$GITHUB_STEP_SUMMARY"
fi

# Machine-readable copy of the table for the Pages dashboard
# (tools/dashboard/parse_ladder.py; schema in tools/dashboard/README.md).
# IPU4_VARIANT names the QEMU model variant the ladder ran against
# (ipu4 | ipu4p | ipu6; plan "Parallel track"). Rungs not selected on the
# command line are recorded as NOT-RUN.
write_results_json() {
	local r rows=""
	for r in "${ORDER[@]}"; do
		rows+="$r	${DEP[$r]:-}	${ENABLED[$r]:+1}	"
		rows+="${RESULT[$r]:-NOT-RUN}	${DUR[$r]:-}"$'\n'
	done
	# shellcheck source=tools/ipu6/pin
	( . "$ROOT/tools/ipu6/pin"
	  LADDER_ROWS="$rows" LADDER_STATUS="$1" LADDER_ROOT="$ROOT" \
	  LADDER_PATCH_DIR="$PATCH_DIR" LADDER_ENABLED_FILE="$ENABLED_FILE" \
	  LADDER_PIN_URL="$IPU6_LINUX_URL" LADDER_PIN_SHA="$IPU6_LINUX_SHA" \
	  python3 - > "$OUT/results.json" <<'PY'
import datetime, email.parser, glob, json, os, re, subprocess, sys
e = os.environ
root = e["LADDER_ROOT"]
patches = []
for p in sorted(glob.glob(os.path.join(e["LADDER_PATCH_DIR"], "*.patch"))):
    with open(p, errors="replace") as f:
        hdr = email.parser.Parser().parse(f, headersonly=True)
    subj = " ".join((hdr.get("Subject") or "").split())
    patches.append({"file": os.path.basename(p),
                    "subject": re.sub(r"^\[PATCH[^\]]*\]\s*", "", subj)})
rungs = []
for line in e["LADDER_ROWS"].splitlines():
    name, dep, en, res, dur = line.split("\t")
    rungs.append({"name": name, "depends_on": dep or None,
                  "enabled": en == "1", "result": res,
                  "duration_s": int(dur) if dur else None})
try:
    sha = subprocess.check_output(["git", "-C", root, "rev-parse", "HEAD"],
                                  stderr=subprocess.DEVNULL, text=True).strip()
except Exception:
    sha = None
run_id = e.get("GITHUB_RUN_ID") or None
run_url = (f"{e.get('GITHUB_SERVER_URL', 'https://github.com')}/"
           f"{e['GITHUB_REPOSITORY']}/actions/runs/{run_id}"
           if run_id and e.get("GITHUB_REPOSITORY") else None)
enabled_file = os.path.relpath(e["LADDER_ENABLED_FILE"], root)
json.dump({
    "schema": 1,
    "variant": e.get("IPU4_VARIANT") or "ipu4",
    "generated_at": datetime.datetime.now(datetime.timezone.utc)
                    .strftime("%Y-%m-%dT%H:%M:%SZ"),
    "commit_sha": e.get("GITHUB_SHA") or sha,
    "run_id": run_id,
    "run_url": run_url,
    "pin": {"url": e["LADDER_PIN_URL"], "sha": e["LADDER_PIN_SHA"]},
    "series": {"count": len(patches), "patches": patches},
    "enabled_tests": enabled_file,
    "exit_status": int(e["LADDER_STATUS"]),
    "rungs": rungs,
}, sys.stdout, indent=2)
print()
PY
	) || echo "warning: could not write $OUT/results.json" >&2
}
write_results_json "$status"

exit "$status"
