#!/usr/bin/env bash
# The test_* functions are only called indirectly ("test_$r").
# shellcheck disable=SC2317
#
# Run the ipu6-flavor test ladder (plan: tools/notes/ipu6-ipu4-plan.md).
#
#   tools/tests/ipu6/ladder.sh            run every known test
#   tools/tests/ipu6/ladder.sh TEST...    run only these tests (in ladder order)
#   tools/tests/ipu6/ladder.sh --list     print the test names
#
# Every test ends as PASS, FAIL, SKIP (a test it depends on did not pass)
# or NOT-IMPLEMENTED. The exit status is non-zero only when a test listed
# in the leg's enabled-tests file ends in anything but PASS, or when
# that file names an unknown test.
#
# Every run is one leg: a driver (IPU4_DRIVER) under one QEMU model
# variant (IPU4_VARIANT: ipu4 | ipu4p | ipu6, read by tools/run-vm.sh and
# the tests). The drivers are the three unmodified baselines tests are
# developed against, one per IPU, and the upstreamable combination:
#
#   driver            IPU    flavor       tree
#   ipu4-out-of-tree  IPU4   out-of-tree  kernel/ipu4/ built against
#                                         tools/out-of-tree/pin (v7.3-rc4)
#                                         in tools/linux-ipu4-oot/
#   ipu4p             IPU4P  ipu6         tools/ipu6/pin (Ruslan's tree),
#                                         no series, in tools/linux-ipu6-ruslan/
#   ipu6              IPU6   ipu6         tools/ipu6/pin.upstream (mainline),
#                                         in tools/linux-ipu6-upstream/
#   ipu4-patches      all    ipu6         tools/ipu6/pin + patches/ipu6-ipu4/
#   (default)                             in tools/linux-ipu6/
#
# IPU4_VARIANT defaults to the driver's IPU (ipu4 for ipu4-patches).
# vm-ipu6.yml runs each baseline under its own variant and ipu4-patches
# under all three. Each leg gates on
# tools/tests/ipu6/enabled-tests.<driver>.<variant>, owned by the
# driver's track (plan "Tracks and file ownership"). IPU4_IPU6_LINUX_DIR
# and IPU4_IPU6_ENABLED_TESTS override the tree and the file. Per-test
# logs go to tools/tests/out/ipu6-ladder/<test>.log, the whole table as
# JSON to tools/tests/out/ipu6-ladder/results.json (read by the Pages
# dashboard).
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
DRIVER="${IPU4_DRIVER:-ipu4-patches}"
FLAVOR=ipu6
PATCH_DIR=
case "$DRIVER" in
ipu4-out-of-tree)
	FLAVOR=out-of-tree
	LINUX_DIR="$ROOT/tools/linux-ipu4-oot"
	PIN="$ROOT/tools/out-of-tree/pin"
	VARIANT=ipu4
	;;
ipu4p)
	LINUX_DIR="$ROOT/tools/linux-ipu6-ruslan"
	PIN="$ROOT/tools/ipu6/pin"
	VARIANT=ipu4p
	;;
ipu6)
	LINUX_DIR="$ROOT/tools/linux-ipu6-upstream"
	PIN="$ROOT/tools/ipu6/pin.upstream"
	VARIANT=ipu6
	;;
ipu4-patches)
	LINUX_DIR="$ROOT/tools/linux-ipu6"
	PIN="$ROOT/tools/ipu6/pin"
	PATCH_DIR="$ROOT/patches/ipu6-ipu4"
	VARIANT=ipu4
	;;
*)
	echo "unknown IPU4_DRIVER '$DRIVER' (expected ipu4-out-of-tree, ipu4p," \
		"ipu6 or ipu4-patches)" >&2
	exit 2
	;;
esac
VARIANT="${IPU4_VARIANT:-$VARIANT}"
case "$VARIANT" in
ipu4 | ipu4p | ipu6) ;;
*) echo "unknown IPU4_VARIANT '$VARIANT' (expected ipu4, ipu4p or ipu6)" >&2; exit 2 ;;
esac
LEG="$DRIVER/$VARIANT"
ENABLED_FILE="${IPU4_IPU6_ENABLED_TESTS:-$HERE/enabled-tests.$DRIVER.$VARIANT}"
LINUX_DIR="${IPU4_IPU6_LINUX_DIR:-$LINUX_DIR}"
# Read by tools/ipu6/*.sh, tools/out-of-tree/build.sh, build-kernel.sh,
# rootfs/build.sh, run-vm.sh and virt-sensor/build.sh.
export IPU4_DRIVER="$DRIVER" IPU4_FLAVOR="$FLAVOR" IPU4_VARIANT="$VARIANT"
export IPU4_IPU6_LINUX_DIR="$LINUX_DIR"
export IPU4_IPU6_PIN="$PIN" IPU4_IPU6_PATCH_DIR="$PATCH_DIR"
# The driver's module, device and node names (IPU_*), as the guest sees
# them (tools/rootfs/build.sh stages the same file).
# shellcheck source=tools/rootfs/ipu-driver.ipu6.conf
. "$ROOT/tools/rootfs/ipu-driver.$FLAVOR.conf"
OUT="$ROOT/tools/tests/out/ipu6-ladder"

# The tests. Each one lives in its own file, tools/tests/ipu6/tests.d/
# NNN-<name>.sh (README.md there), so adding a test adds a file instead
# of editing a list here. The files are sourced in name order, which is
# the ladder order; each registers itself with ladder_test and defines
# test_<name>, which runs with stdout/stderr going to the test's log and
# returns 0 for PASS, non-zero for FAIL. A registered test without a
# test_<name> function is NOT-IMPLEMENTED. "ladder_test NAME DEP": when
# both are run, NAME is SKIPped unless DEP PASSed.
declare -A DEP=() RESULT=() KNOWN=() DUR=()
ORDER=()
ladder_test() {
	ORDER+=("$1")
	KNOWN[$1]=1
	[[ -n "${2:-}" ]] && DEP[$1]="$2"
	return 0
}

# Helpers shared by several tests.
# shellcheck source=tools/tests/ipu6/probe-boot.sh
. "$HERE/probe-boot.sh"
# shellcheck source=tools/tests/ipu6/stream-boot.sh
. "$HERE/stream-boot.sh"

for f in "$HERE"/tests.d/[0-9]*.sh; do
	# shellcheck source=/dev/null
	. "$f"
done

# --- driver --------------------------------------------------------------

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
			echo "enabled-tests: unknown test '$line'" >&2
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
			echo "unknown test '$r' (see --list)" >&2
			exit 2
		fi
		WANT[$r]=1
	done
else
	for r in "${ORDER[@]}"; do WANT[$r]=1; done
fi

mkdir -p "$OUT"
rm -f "$PROBE_STAMP" "$STREAM_STAMP"
for r in "${ORDER[@]}"; do
	[[ -z "${WANT[$r]:-}" ]] && continue
	dep="${DEP[$r]:-}"
	log="$OUT/$r.log"
	if ! declare -F "test_$r" > /dev/null; then
		RESULT[$r]=NOT-IMPLEMENTED
		echo "not implemented yet" > "$log"
		continue
	fi
	if [[ -n "$dep" && -n "${WANT[$dep]:-}" && "${RESULT[$dep]:-}" != PASS ]]; then
		RESULT[$r]=SKIP
		echo "skipped: depends on $dep (${RESULT[$dep]:-not run})" > "$log"
		continue
	fi
	echo "::group::test $r"
	start=$SECONDS
	"test_$r" 2>&1 | tee "$log"
	rc=${PIPESTATUS[0]}
	echo "::endgroup::"
	if (( rc == 0 )); then
		RESULT[$r]=PASS
	else
		RESULT[$r]=FAIL
	fi
	DUR[$r]=$((SECONDS - start))
	echo "test $r: ${RESULT[$r]} ($((SECONDS - start))s)"
done

status=$bad_config
echo
printf '%-16s %-8s %s\n' TEST ENABLED RESULT
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
# Enabled tests that weren't selected on the command line aren't run,
# so they can't fail; say so rather than hiding it.
for r in "${!ENABLED[@]}"; do
	[[ -z "${WANT[$r]:-}" ]] && echo "note: enabled test '$r' not selected"
done

if [[ -n "${GITHUB_STEP_SUMMARY:-}" ]]; then
	{
		echo "### ipu6 ladder ($LEG)"
		echo
		echo "| Test | Enabled | Result |"
		echo "|---|---|---|"
		for r in "${ORDER[@]}"; do
			[[ -z "${WANT[$r]:-}" ]] && continue
			echo "| \`$r\` | ${ENABLED[$r]:+yes} | ${RESULT[$r]} |"
		done
	} >> "$GITHUB_STEP_SUMMARY"
fi

# Machine-readable copy of the table for the Pages dashboard
# (tools/dashboard/parse_ladder.py; schema in tools/dashboard/README.md).
# "leg" is "<driver>/<variant>" (IPU4_DRIVER, IPU4_VARIANT). Tests
# not selected on the command line are recorded as NOT-RUN.
write_results_json() {
	local r rows=""
	for r in "${ORDER[@]}"; do
		rows+="$r	${DEP[$r]:-}	${ENABLED[$r]:+1}	"
		rows+="${RESULT[$r]:-NOT-RUN}	${DUR[$r]:-}"$'\n'
	done
	# shellcheck source=tools/ipu6/pin
	( . "$PIN"
	  LADDER_ROWS="$rows" LADDER_STATUS="$1" LADDER_ROOT="$ROOT" \
	  LADDER_PATCH_DIR="$PATCH_DIR" LADDER_ENABLED_FILE="$ENABLED_FILE" \
	  LADDER_PIN_URL="$IPU6_LINUX_URL" LADDER_PIN_SHA="$IPU6_LINUX_SHA" \
	  python3 - > "$OUT/results.json" <<'PY'
import datetime, email.parser, glob, json, os, re, subprocess, sys
e = os.environ
root = e["LADDER_ROOT"]
patches = []
pdir = e["LADDER_PATCH_DIR"]
for p in sorted(glob.glob(os.path.join(pdir, "*.patch")) if pdir else []):
    with open(p, errors="replace") as f:
        hdr = email.parser.Parser().parse(f, headersonly=True)
    subj = " ".join((hdr.get("Subject") or "").split())
    patches.append({"file": os.path.basename(p),
                    "subject": re.sub(r"^\[PATCH[^\]]*\]\s*", "", subj)})
tests = []
for line in e["LADDER_ROWS"].splitlines():
    name, dep, en, res, dur = line.split("\t")
    tests.append({"name": name, "depends_on": dep or None,
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
    "leg": e["IPU4_DRIVER"] + "/" + e["IPU4_VARIANT"],
    "driver": e["IPU4_DRIVER"],
    "variant": e["IPU4_VARIANT"],
    "generated_at": datetime.datetime.now(datetime.timezone.utc)
                    .strftime("%Y-%m-%dT%H:%M:%SZ"),
    "commit_sha": e.get("GITHUB_SHA") or sha,
    "run_id": run_id,
    "run_url": run_url,
    "pin": {"url": e["LADDER_PIN_URL"], "sha": e["LADDER_PIN_SHA"]},
    "series": {"count": len(patches), "patches": patches},
    "enabled_tests": enabled_file,
    "exit_status": int(e["LADDER_STATUS"]),
    "tests": tests,
}, sys.stdout, indent=2)
print()
PY
	) || echo "warning: could not write $OUT/results.json" >&2
}
write_results_json "$status"

exit "$status"
