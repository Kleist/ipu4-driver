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
# tools/tests/out/ipu6-ladder/<rung>.log.
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

# Full guest kernel (bzImage + modules, same .config as `build`) +
# initramfs with the vm-smoke init, booted under the QEMU IPU4 model;
# passes when the guest sees 8086:5a88 on its PCI bus. Needs the patched
# QEMU (tools/bootstrap.sh --qemu-only && tools/build-qemu.sh).
rung_vm-boot() {
	local qemu="${IPU4_QEMU_BIN:-$ROOT/tools/qemu/build/qemu-system-x86_64}"
	if [[ ! -x "$qemu" ]]; then
		echo "missing $qemu; run tools/bootstrap.sh --qemu-only &&" \
			"tools/build-qemu.sh"
		return 1
	fi
	IPU4_FLAVOR=ipu6 "$ROOT/tools/build-kernel.sh" || return 1
	IPU4_FLAVOR=ipu6 "$ROOT/tools/tests/vm-smoke.sh"
}

# Boot the vm-boot guest kernel with tools/rootfs/init.ipu6-modules:
# load the vb2 modules, the harness's virt-sensor (tools/virt-sensor/),
# intel-ipu6 and intel-ipu6-isys, check that 8086:5a88 stays unbound (no
# ID-table entry yet), unload, and scan dmesg. The guest prints
# MODLOAD: PASS|FAIL. Rebuilds the kernel only when vm-boot (which does
# that) is not part of this run.
rung_module-load() {
	local log="$ROOT/tools/tests/out/ipu6-module-load.serial"
	if [[ -z "${WANT[vm-boot]:-}" ]]; then
		IPU4_FLAVOR=ipu6 "$ROOT/tools/build-kernel.sh" || return 1
	fi
	"$ROOT/tools/virt-sensor/build.sh" || return 1
	IPU4_FLAVOR=ipu6 IPU4_INIT=ipu6-modules "$ROOT/tools/rootfs/build.sh" ||
		return 1
	rm -f "$log"
	timeout --preserve-status "${IPU4_SMOKE_TIMEOUT:-90}" \
		env IPU4_FLAVOR=ipu6 "$ROOT/tools/run-vm.sh" > "$log" 2>&1
	grep -E '^MODLOAD:|virt-sensor|ipu6' "$log"
	grep -q '^MODLOAD: PASS' "$log"
}

# Boot with tools/rootfs/init.ipu6-probe: load the modules as
# module-load does, bind intel-ipu6 to 8086:5a88 through new_id
# (harness-only; the series adds the PCI ID table entry last) and check
# that probe identified the IPU4 (requested intel/ipu/ipu4_fw.bin rather
# than rejecting the device). Probe itself may still fail further on;
# that is what the later probe-* rungs check. Fails on any PROBE:fail
# marker or oops. Serial log: tools/tests/out/ipu6-probe.serial.
rung_probe-identify() {
	local log="$ROOT/tools/tests/out/ipu6-probe.serial"
	if [[ -z "${WANT[vm-boot]:-}" ]]; then
		IPU4_FLAVOR=ipu6 "$ROOT/tools/build-kernel.sh" || return 1
	fi
	"$ROOT/tools/virt-sensor/build.sh" || return 1
	IPU4_FLAVOR=ipu6 IPU4_INIT=ipu6-probe "$ROOT/tools/rootfs/build.sh" ||
		return 1
	rm -f "$log"
	timeout --preserve-status "${IPU4_SMOKE_TIMEOUT:-90}" \
		env IPU4_FLAVOR=ipu6 "$ROOT/tools/run-vm.sh" > "$log" 2>&1
	grep -E '^PROBE:|intel-ipu6|virt-sensor' "$log"
	if grep -q '^PROBE:fail' "$log"; then
		echo "!!! PROBE:fail markers (see above)"
		return 1
	fi
	grep -q '^PROBE: DONE' "$log" &&
		grep -q '^PROBE:oops none' "$log" &&
		grep -q '^PROBE:identify ok' "$log"
}

# The later VM rungs (probe-buttress onwards) are implemented by later
# PRs of the plan.

# --- driver --------------------------------------------------------------

declare -A DEP=() RESULT=() KNOWN=()
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

exit "$status"
