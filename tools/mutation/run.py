#!/usr/bin/env python3
"""Mutation testing of kernel/ipu4/ against the ipu4-out-of-tree ladder leg.

    tools/mutation/run.py baseline [--reuse]
    tools/mutation/run.py run FILE [--sample N] [--seed S] [--ids I,J,...]
    tools/mutation/run.py report [FILE...]

`baseline` runs the leg once with IPU4_GCOV=1 and records, per VM boot,
which driver lines it executes (tools/tests/out/mutation/baseline.json).
`run` mutates FILE (tools/mutation/mutants.py) one mutant at a time,
rebuilds the module and runs only the boots that execute the mutated
line, stopping at the first failing test. A mutant on a line no boot
executes is recorded as no-coverage without running anything. Results
are appended to tools/tests/out/mutation/<file>.jsonl, so an interrupted
run resumes where it stopped. The leg must be built and passing first
(tools/tests/ipu6/ladder.sh with IPU4_DRIVER=ipu4-out-of-tree).
"""
import argparse
import json
import os
import random
import re
import subprocess
import sys
import tarfile
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
DRV = ROOT / "kernel" / "ipu4"
OUT = ROOT / "tools" / "tests" / "out" / "mutation"
LADDER = ROOT / "tools" / "tests" / "ipu6" / "ladder.sh"
LADDER_OUT = ROOT / "tools" / "tests" / "out" / "ipu6-ladder"
sys.path.insert(0, str(HERE))
import mutants  # noqa: E402

# The leg's VM boots (gcov tar name) and the tests that read each one, in
# the order they run: cheapest and earliest-failing first.
BOOTS = {
    "ipu6-modules": ["module-load", "pci-reauth", "frame-desc"],
    "ipu6-probe": ["probe-identify", "probe-buttress", "probe-mmu",
                   "isys-resume", "isys-irq", "v4l2-query"],
    "ipu6-stream": ["fw-open", "stream-on", "stream-pattern",
                    "csi2-sof-irq"],
    "ipu6-stream-error": ["stream-start-error"],
}
ENV = {"IPU4_DRIVER": "ipu4-out-of-tree", "IPU4_VARIANT": "ipu4",
       "IPU4_ACCEL": os.environ.get("IPU4_ACCEL", "tcg")}
# How far back from its statement's first line a mutant may borrow a
# preceding line's gcov count: a statement folded to a constant has no
# count of its own, and comments may sit in between.
LOOKBACK = 12


def env(**extra):
    e = dict(os.environ)
    e.update(ENV)
    e.update({k: str(v) for k, v in extra.items()})
    return e


def lcov_lines(tar):
    """{file: {line: hits}} for the driver's sources in one gcov tar."""
    with tempfile.TemporaryDirectory() as work:
        with tarfile.open(tar) as t:
            t.extractall(work, filter="fully_trusted")
        stage = Path(work + str(DRV))
        if not stage.is_dir():
            return {}
        for gcno in DRV.glob("*.gcno"):
            (stage / gcno.name).write_bytes(gcno.read_bytes())
        info = Path(work) / "out.info"
        subprocess.run(["lcov", "--quiet", "--capture", "--directory",
                        str(stage), "--output-file", str(info)],
                       check=True, stderr=subprocess.DEVNULL)
        cov, cur = {}, None
        for ln in info.read_text().splitlines():
            if ln.startswith("SF:"):
                cur = Path(ln[3:]).name
                cov.setdefault(cur, {})
            elif ln.startswith("DA:") and cur:
                line, hits = ln[3:].split(",")[:2]
                cov[cur][int(line)] = int(hits)
        return cov


def results_json():
    rows = json.loads((LADDER_OUT / "results.json").read_text())
    tests = rows.get("tests", rows) if isinstance(rows, dict) else rows
    return {t["name"]: t for t in tests}


def cmd_baseline(args):
    OUT.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    if not args.reuse:
        rc = subprocess.run([str(LADDER)], env=env(IPU4_GCOV=1)).returncode
        if rc != 0:
            sys.exit(f"baseline ladder failed (rc {rc}); fix the leg first")
    res = results_json()
    boots = {}
    for b in BOOTS:
        tar = ROOT / "tools" / "tests" / "out" / f"gcov-{b}.tar"
        boots[b] = lcov_lines(tar) if tar.exists() else {}
        print(f"{b}: {sum(1 for f in boots[b].values() for h in f.values() if h)}"
              " lines hit")
    base = {"wall_s": round(time.time() - t0),
            "tests": {n: {"result": r.get("result"),
                          "duration_s": r.get("duration_s")}
                      for n, r in res.items()},
            "boots": boots}
    (OUT / "baseline.json").write_text(json.dumps(base))
    print(f">>> {OUT / 'baseline.json'}")


def boots_for(base, fname, line, stmt_line):
    """Boots that execute the mutated statement; None when none instruments it."""
    hit, instrumented = [], False
    for b in BOOTS:
        cov = base["boots"].get(b, {}).get(fname, {})
        cands = [ln for ln in range(line, stmt_line - 1, -1)]
        cands += range(stmt_line - 1, stmt_line - LOOKBACK - 1, -1)
        for ln in cands:
            if ln in cov or str(ln) in cov:
                instrumented = True
                if cov.get(ln, cov.get(str(ln), 0)) > 0:
                    hit.append(b)
                break
    return hit if instrumented else None


def build(log):
    """0 ok, 1 build error, 2 compiler warning (kills, as the build test)."""
    rc = subprocess.run([str(ROOT / "tools" / "out-of-tree" / "build.sh")],
                        env=env(IPU4_IPU6_BUILD_LOG=log, IPU4_GCOV=0),
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL).returncode
    if rc:
        return 1
    return 2 if re.search(r"(warning|error):", Path(log).read_text()) else 0


def run_boot(boot, timeout):
    """Name of the first failing test of `boot`, or None when all pass."""
    subprocess.run([str(LADDER), *BOOTS[boot]],
                   env=env(IPU4_GCOV=0, IPU4_SMOKE_TIMEOUT=timeout,
                           IPU4_IPU6_ENABLED_TESTS="/dev/null"),
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    res = results_json()
    for t in BOOTS[boot]:
        if res.get(t, {}).get("result") != "PASS":
            return t
    return None


def cmd_run(args):
    base = json.loads((OUT / "baseline.json").read_text())
    path = (ROOT / args.file).resolve()
    fname = path.name
    orig = path.read_bytes()
    src = orig.decode()
    muts = mutants.generate(src)
    ids = list(range(len(muts)))
    if args.ids:
        ids = [int(i) for i in args.ids.split(",")]
    elif args.sample:
        ids = sorted(random.Random(args.seed).sample(ids, min(args.sample,
                                                              len(ids))))
    out = OUT / f"{fname}.jsonl"
    done = set()
    if out.exists():
        done = {json.loads(ln)["id"] for ln in out.read_text().splitlines()}
    lines = src.splitlines()
    build_log = str(OUT / "build.log")
    try:
        for n, k in enumerate(ids):
            if k in done:
                continue
            m = muts[k]
            rec = {"id": k, "file": fname, "line": m["line"], "op": m["op"],
                   "desc": m["desc"], "orig": lines[m["line"] - 1].strip()}
            t0 = time.time()
            boots = boots_for(base, fname, m["line"], m["stmt_line"])
            if not boots:
                rec["status"] = ("no-coverage" if boots == [] else
                                 "not-instrumented")
            else:
                path.write_text(mutants.apply(src, m["edits"]))
                b = build(build_log)
                rec["build_s"] = round(time.time() - t0, 1)
                if b == 1:
                    rec["status"] = "build-error"
                elif b == 2:
                    rec["status"], rec["killed_by"] = "killed", "build"
                else:
                    rec["status"] = "survived"
                    rec["boots"] = {}
                    for boot in boots:
                        tb = time.time()
                        failed = run_boot(boot, args.timeout)
                        rec["boots"][boot] = round(time.time() - tb, 1)
                        if failed:
                            rec["status"], rec["killed_by"] = "killed", failed
                            break
                path.write_bytes(orig)
            rec["total_s"] = round(time.time() - t0, 1)
            with out.open("a") as f:
                f.write(json.dumps(rec) + "\n")
            print(f"[{n + 1}/{len(ids)}] #{k} {fname}:{m['line']} {m['op']} "
                  f"{m['desc']}: {rec['status']}"
                  f"{' by ' + rec['killed_by'] if 'killed_by' in rec else ''}"
                  f" ({rec['total_s']}s)", flush=True)
    finally:
        path.write_bytes(orig)
        build(build_log)


def cmd_report(args):
    files = args.files or sorted(p.name for p in OUT.glob("*.jsonl"))
    tot = {}
    for f in files:
        f = f if f.endswith(".jsonl") else Path(f).name + ".jsonl"
        recs = [json.loads(ln) for ln in (OUT / f).read_text().splitlines()]
        st = {}
        for r in recs:
            st[r["status"]] = st.get(r["status"], 0) + 1
            tot[r["status"]] = tot.get(r["status"], 0) + 1
        ran = [r for r in recs if "boots" in r or r.get("killed_by")]
        secs = [r["total_s"] for r in ran]
        k, s = st.get("killed", 0), st.get("survived", 0)
        score = f"{100 * k / (k + s):.0f}%" if k + s else "-"
        print(f"{f[:-6]}: {len(recs)} mutants {st} score {score} (covered); "
              f"run mean {sum(secs) / max(len(secs), 1):.0f}s")
        for r in recs:
            if r["status"] == "survived":
                print(f"  survived #{r['id']} :{r['line']} {r['op']} "
                      f"{r['desc']}   | {r['orig']}")
    k, s = tot.get("killed", 0), tot.get("survived", 0)
    if len(files) > 1 and k + s:
        print(f"total {tot} score {100 * k / (k + s):.0f}%")


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("baseline")
    b.add_argument("--reuse", action="store_true",
                   help="use the last IPU4_GCOV=1 ladder run's results")
    r = sub.add_parser("run")
    r.add_argument("file")
    r.add_argument("--sample", type=int)
    r.add_argument("--seed", type=int, default=1)
    r.add_argument("--ids")
    r.add_argument("--timeout", type=int, default=90,
                   help="per-boot timeout (IPU4_SMOKE_TIMEOUT)")
    rp = sub.add_parser("report")
    rp.add_argument("files", nargs="*")
    a = p.parse_args()
    {"baseline": cmd_baseline, "run": cmd_run, "report": cmd_report}[a.cmd](a)


if __name__ == "__main__":
    main()
