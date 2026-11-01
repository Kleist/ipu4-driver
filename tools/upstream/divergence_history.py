#!/usr/bin/env python3
"""Upstream-divergence history of the out-of-tree driver, for the dashboard.

Walks a timeline of mainline tags from v6.6 on and our driver commits,
counting how many lines the driver tree (``kernel/ipu4/``, or wherever
``ipu6-isys.c`` lived before it moved there) differs from upstream
``drivers/media/pci/intel/ipu6/`` after each. Steps that move the count by
at least MIN_EVENT lines are marked as events, with the tag or commit that
caused them; releases and the head are always events.

Dates come from git. A commit's date is its author date, raised to the
latest date of its ancestors when it is older (rebases and cherry-picks
keep the original author date), so the series never runs backwards. A
tag is compared with our tree as it was at the tag's date; one of our
commits is compared with the newest tag, release or -rc, from before it. Before v6.10 upstream has no ipu6 driver, so every line counts.

Writes ``tools/notes/upstream-diff/history.json`` (gitignored), which
``tools/dashboard/build_payload.py`` embeds in the page.

Usage:
  tools/upstream/divergence_history.py --linux DIR [--fetch] [--target]
                                       [--repo DIR] [--rev REV] [--out FILE]

With --target it also reports the divergence from the target tree of
tools/notes/ipu4-out-of-tree-plan.md (the ladder's ipu4-patches driver:
mainline pin + IPU4P + fixes + IPU4 series), outside the permanent delta
listed in tools/out-of-tree/delta.txt.
"""
from __future__ import annotations

import argparse
import bisect
import datetime as _dt
import difflib
import fnmatch
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

UPSTREAM_REL = "drivers/media/pci/intel/ipu6"
MARKER = "ipu6-isys.c"
FIRST_RELEASE = (6, 6)
MIN_EVENT = 100  # lines a step must move the divergence to be an event
LINUX_URL = "https://github.com/torvalds/linux.git"
TAG_RE = re.compile(r"^v(\d+)\.(\d+)(?:-rc(\d+))?$")
# Paths that hold copies of driver files but are not the driver.
NOT_DRIVER = ("tools/", "patches/")
# The target tree: the ipu4-patches driver of the ladder, i.e. the mainline
# pin (tools/ipu6/pin) with these series applied in order.
TARGET_SERIES = ("ipu6-ipu4p", "ipu6-fixes", "ipu6-ipu4")


def git(repo: Path, *args: str, input: str | None = None) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True, capture_output=True, text=True, input=input,
    ).stdout


# --- pure logic -----------------------------------------------------------


def tag_version(name: str):
    """(major, minor, rc) for a mainline tag, rc = None for a release."""
    m = TAG_RE.match(name)
    if not m:
        return None
    rc = int(m.group(3)) if m.group(3) else None
    return int(m.group(1)), int(m.group(2)), rc


def effective_dates(commits) -> dict:
    """{sha: date} with each date at least the latest of its parents'.

    ``commits`` is an iterable of (sha, parents, author_ts) in an order
    where parents come first (``git rev-list --reverse --topo-order``).
    Parents outside the list (shallow history) are ignored.
    """
    eff = {}
    for sha, parents, ts in commits:
        eff[sha] = max([ts] + [eff[p] for p in parents if p in eff])
    return eff


def changed_lines(ours: list[str], theirs: list[str]) -> int:
    """Lines added plus lines removed to get from theirs to ours."""
    n = 0
    sm = difflib.SequenceMatcher(None, theirs, ours, autojunk=False)
    for op, i1, i2, j1, j2 in sm.get_opcodes():
        if op != "equal":
            n += (i2 - i1) + (j2 - j1)
    return n


def latest_at(dated, ts, strict: bool = False):
    """Last element of ``dated`` (sorted by [0]) with date <= ts (< ts if
    ``strict``), or None."""
    find = bisect.bisect_left if strict else bisect.bisect_right
    i = find([d for d, _ in dated], ts)
    return dated[i - 1][1] if i else None


def timeline(tags, commits, head):
    """Every state of (our tree, upstream tag) in date order.

    ``tags``: [(ts, name)] sorted by date, releases and rcs.
    ``commits``: [(eff_ts, sha)] of driver-touching commits, sorted.
    ``head``: (eff_ts, sha) of the revision being reported.
    Returns [(kind, ts, sha, tag_name)]: a "release" or "rc" step for each
    tag from FIRST_RELEASE on once our driver exists (our tree just before
    the tag's date), a "commit" step for each of our commits after the
    first such release (against the newest tag up to its date) and a final
    "head" step.
    """
    steps = []
    first = next((ts for ts, t in tags if tag_version(t)[2] is None
                  and tag_version(t)[:2] >= FIRST_RELEASE), None)
    for ts, t in tags:
        v = tag_version(t)
        ours = latest_at(commits, ts, strict=True)
        if v[:2] >= FIRST_RELEASE and ours:
            steps.append((ts, 0, "release" if v[2] is None else "rc", ours, t))
    for ts, sha in commits:
        ref = latest_at(tags, ts)
        if first is not None and ts > first and ref and sha != head[1]:
            steps.append((ts, 1, "commit", sha, ref))
    steps.sort(key=lambda s: (s[0], s[1]))
    ref = latest_at(tags, head[0])
    if ref:
        steps.append((head[0], 2, "head", head[1], ref))
    return [(kind, ts, sha, ref) for ts, _, kind, sha, ref in steps]


def mark_events(points, min_event: int = MIN_EVENT):
    """Set each point's ``delta`` (change from the previous step) and
    ``event``; drop steps that change nothing, except releases and head."""
    out, prev = [], None
    for p in points:
        p["delta"] = p["divergence"] - prev if prev is not None else 0
        prev = p["divergence"]
        if p["delta"] == 0 and out and p["kind"] in ("rc", "commit"):
            continue
        p["event"] = (not out or p["kind"] in ("release", "head")
                      or abs(p["delta"]) >= min_event)
        out.append(p)
    return out


# --- git access -------------------------------------------------------------


class Blobs:
    """Reads blobs (as lists of lines) from one repository, cached."""

    def __init__(self, repo: Path):
        self.repo = repo
        self.cache = {}

    def lines(self, oid: str) -> list[str]:
        if oid not in self.cache:
            data = subprocess.run(
                ["git", "-C", str(self.repo), "cat-file", "blob", oid],
                check=True, capture_output=True).stdout
            self.cache[oid] = data.decode("utf-8", "replace").splitlines()
        return self.cache[oid]


class Files:
    """Reads files below a directory, with the same interface as Blobs."""

    def __init__(self, base: Path):
        self.base = base

    def lines(self, name: str) -> list[str]:
        return (self.base / name).read_text(errors="replace").splitlines()


def source_files(repo: Path, rev: str, path: str) -> dict:
    """{name: blob oid} of the .c/.h files directly in ``path`` at ``rev``."""
    out = git(repo, "ls-tree", f"{rev}:{path}") if path else ""
    files = {}
    for line in out.splitlines():
        meta, name = line.split("\t", 1)
        mode, kind, oid = meta.split()
        if kind == "blob" and name.endswith((".c", ".h")):
            files[name] = oid
    return files


def driver_dir(repo: Path, rev: str) -> str | None:
    """Directory holding ipu6-isys.c in ``rev``, ignoring tools/ and patches/."""
    out = git(repo, "ls-tree", "-r", "--name-only", rev)
    for p in out.splitlines():
        if p.rsplit("/", 1)[-1] == MARKER and not p.startswith(NOT_DRIVER):
            return p.rsplit("/", 1)[0] if "/" in p else ""
    return None


def local_only(root: Path) -> set:
    """IPU4_LOCAL_ONLY from tools/upstream/_lib.sh."""
    text = (root / "tools" / "upstream" / "_lib.sh").read_text()
    m = re.search(r"IPU4_LOCAL_ONLY=\((.*?)\)", text, re.S)
    return set(m.group(1).split()) if m else set()


def driver_paths(repo: Path, rev: str) -> list[str]:
    """Every directory that held the driver (ipu6-isys.c) in ``rev``'s history."""
    out = git(repo, "log", "--format=", "--name-only", rev, "--",
              f":(glob)**/{MARKER}")
    dirs = {p.rsplit("/", 1)[0] for p in out.splitlines()
            if "/" in p and not p.startswith(NOT_DRIVER)}
    return sorted(dirs)


def our_history(repo: Path, rev: str):
    """([(eff_ts, sha)] of driver-touching commits, their subjects, head eff).

    The date ceiling runs over driver-touching ancestors only (git's
    path-simplified history): the import was rebased onto the repository's
    unrelated "Initial commit", which would otherwise lift all of it to
    that commit's date.
    """
    log = git(repo, "rev-list", "--reverse", "--topo-order", "--parents",
              "--format=%at%x09%s", rev, "--", *driver_paths(repo, rev))
    commits, subjects, cur = [], {}, None
    for line in log.splitlines():
        if line.startswith("commit "):
            cur = line.split()[1:]
        else:
            ts, subj = line.split("\t", 1)
            commits.append((cur[0], cur[1:], int(ts)))
            subjects[cur[0]] = subj
    eff = effective_dates(commits)
    # Stable sort keeps topological order among equal dates.
    driver = sorted(((eff[c[0]], c[0]) for c in commits), key=lambda e: e[0])
    head_ts = int(git(repo, "log", "-1", "--format=%at", rev))
    head_eff = max([head_ts] + [ts for ts, _ in driver[-1:]])
    return driver, subjects, head_eff


def upstream_tags(linux: Path):
    """[(ts, name)] of the mainline tags in ``linux``, sorted by date."""
    out = git(linux, "for-each-ref", "--format=%(refname:short)\t"
              "%(*committerdate:unix)%(committerdate:unix)", "refs/tags")
    tags = []
    for line in out.splitlines():
        name, ts = line.split("\t")
        if tag_version(name):
            tags.append((int(ts), name))
    return sorted(tags)


_CHANGED = {}  # (our oid, upstream oid) -> changed lines


def divergence(ours: dict, theirs: dict, our_blobs: Blobs, up_blobs: Blobs,
               skip: set) -> dict:
    """Changed lines between our files and upstream's, plus a breakdown."""
    total = missing = lines = 0
    files = 0
    for name, oid in sorted(ours.items()):
        if name in skip:
            continue
        files += 1
        mine = our_blobs.lines(oid)
        lines += len(mine)
        if name in theirs:
            key = (oid, theirs[name])
            if key not in _CHANGED:
                _CHANGED[key] = changed_lines(mine, up_blobs.lines(theirs[name]))
            total += _CHANGED[key]
        else:
            missing += 1
            total += len(mine)
    up_only = [n for n in theirs if n not in ours]
    return {
        "divergence": total,
        "lines": lines,
        "files": files,
        "files_missing_upstream": missing,
        "upstream_only_files": len(up_only),
        "upstream_only_lines": sum(len(up_blobs.lines(theirs[n])) for n in up_only),
    }


def pin_sha(root: Path) -> str:
    """The commit tools/ipu6/pin names."""
    text = (root / "tools" / "ipu6" / "pin").read_text()
    return re.search(r"IPU6_LINUX_SHA=.*?([0-9a-f]{40})", text).group(1)


def allowed_delta(root: Path) -> list[str]:
    """File patterns of the permanent delta (tools/out-of-tree/delta.txt)."""
    text = (root / "tools" / "out-of-tree" / "delta.txt").read_text()
    return [ln.split("#", 1)[0].strip() for ln in text.splitlines()
            if ln.split("#", 1)[0].strip()]


def target_tree(root: Path, linux: Path, work: Path) -> dict:
    """Build the target's ipu6 directory in ``work``: the pin's files with
    TARGET_SERIES applied. Returns {name: name} of its .c/.h files."""
    out = work / UPSTREAM_REL
    out.mkdir(parents=True)
    for line in git(linux, "ls-tree", f"{pin_sha(root)}:{UPSTREAM_REL}").splitlines():
        meta, name = line.split("\t", 1)
        oid = meta.split()[2]
        data = subprocess.run(["git", "-C", str(linux), "cat-file", "blob", oid],
                              check=True, capture_output=True).stdout
        (out / name).write_bytes(data)
    for series in TARGET_SERIES:
        for patch in sorted((root / "patches" / series).glob("*.patch")):
            subprocess.run(["git", "apply", f"--include={UPSTREAM_REL}/*",
                            str(patch)], cwd=work, check=True,
                           capture_output=True)
    return {f.name: f.name for f in out.iterdir()
            if f.name.endswith((".c", ".h"))}


def target_divergence(ours: dict, target: dict, our_blobs, target_files,
                      allowed: list[str]) -> dict:
    """Divergence from the target tree, outside the allowed delta.

    Shared files count their changed lines, our files the target lacks and
    target files we lack count every line. Returns the total and the
    per-file worklist, largest first.
    """
    def is_allowed(name):
        return any(fnmatch.fnmatch(name, pat) for pat in allowed)

    files = []
    for name, oid in ours.items():
        if is_allowed(name):
            continue
        mine = our_blobs.lines(oid)
        if name in target:
            n = changed_lines(mine, target_files.lines(target[name]))
            if n:
                files.append({"name": name, "status": "differs", "lines": n})
        else:
            files.append({"name": name, "status": "not in target", "lines": len(mine)})
    for name in target:
        if name not in ours and not is_allowed(name):
            files.append({"name": name, "status": "missing here",
                          "lines": len(target_files.lines(target[name]))})
    files.sort(key=lambda f: (-f["lines"], f["name"]))
    return {"divergence": sum(f["lines"] for f in files), "files": files}


def fetch_linux(linux: Path, url: str = LINUX_URL, extra=()) -> None:
    """Fetch the mainline tags from FIRST_RELEASE on (tips only, no history)
    and master into ``linux``, then the ipu6 driver's blobs in one go."""
    if not (linux / "HEAD").exists():
        linux.mkdir(parents=True, exist_ok=True)
        git(linux, "init", "-q", "--bare")
        git(linux, "remote", "add", "origin", url)
        git(linux, "config", "remote.origin.promisor", "true")
        git(linux, "config", "remote.origin.partialclonefilter", "blob:none")
    names = []
    for line in git(linux, "ls-remote", "--tags", "--refs", "origin").splitlines():
        name = line.split("\t")[1].removeprefix("refs/tags/")
        v = tag_version(name)
        if v and v[:2] >= FIRST_RELEASE:
            names.append(name)
    specs = [f"+refs/tags/{n}:refs/tags/{n}" for n in names]
    git(linux, "fetch", "-q", "--no-tags", "--depth=1", "--filter=blob:none", "origin",
        *specs, "+refs/heads/master:refs/heads/master", *extra)
    oids = set()
    for ref in names + ["master", *extra]:
        try:
            oids.update(source_files(linux, ref, UPSTREAM_REL).values())
        except subprocess.CalledProcessError:
            continue  # before v6.10: no ipu6 driver
    missing = git(linux, "cat-file", "--batch-check", input="\n".join(oids) + "\n")
    want = [ln.split()[0] for ln in missing.splitlines() if ln.endswith("missing")]
    if want:
        git(linux, "-c", "fetch.negotiationAlgorithm=noop", "fetch", "-q",
            "--no-tags", "--no-write-fetch-head", "--filter=blob:none",
            "origin", *want)


def iso(ts: int) -> str:
    return _dt.datetime.fromtimestamp(ts, _dt.timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%SZ")


def build(repo: Path, rev: str, linux: Path, skip: set, root: Path | None = None) -> dict:
    driver, subjects, head_eff = our_history(repo, rev)
    head_sha = git(repo, "rev-parse", rev).strip()
    tags = upstream_tags(linux)
    tag_ts = {t: ts for ts, t in tags}
    our_blobs, up_blobs = Blobs(repo), Blobs(linux)
    dirs, up_files = {}, {}

    def ours_at(sha):
        if sha not in dirs:
            d = driver_dir(repo, sha)
            dirs[sha] = source_files(repo, sha, d) if d is not None else {}
        return dirs[sha]

    def upstream_at(ref):
        if ref not in up_files:
            try:
                up_files[ref] = source_files(linux, ref, UPSTREAM_REL)
            except subprocess.CalledProcessError:
                up_files[ref] = {}
        return up_files[ref]

    points = []
    for kind, ts, sha, ref in timeline(tags, driver, (head_eff, head_sha)):
        point = {
            "kind": kind,
            "date": iso(ts),
            "ts": ts * 1000,
            "sha": sha,
            "subject": subjects.get(sha, ""),
            "ref": ref,
            "ref_date": iso(tag_ts[ref]),
        }
        point.update(divergence(ours_at(sha), upstream_at(ref), our_blobs,
                                up_blobs, skip))
        points.append(point)
    points = mark_events(points)
    master = None
    if git(linux, "for-each-ref", "refs/heads/master").strip():
        m = divergence(ours_at(head_sha), upstream_at("master"), our_blobs,
                       up_blobs, skip)
        m["sha"] = git(linux, "rev-parse", "master").strip()
        m["date"] = iso(int(git(linux, "log", "-1", "--format=%ct", "master")))
        master = m
    target = None
    if root is not None:
        with tempfile.TemporaryDirectory() as work:
            files = target_tree(root, linux, Path(work))
            target = target_divergence(ours_at(head_sha), files, our_blobs,
                                       Files(Path(work) / UPSTREAM_REL),
                                       allowed_delta(root))
        target["pin"] = pin_sha(root)
        target["series"] = {s: len(list((root / "patches" / s).glob("*.patch")))
                            for s in TARGET_SERIES}
    return {
        "schema": 2,
        "generated_at": iso(int(_dt.datetime.now(_dt.timezone.utc).timestamp())),
        "head": head_sha,
        "upstream_path": UPSTREAM_REL,
        "min_event": MIN_EVENT,
        "points": points,
        "master": master,
        "target": target,
    }


def main(argv=None) -> int:
    root = Path(__file__).resolve().parents[2]
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--repo", type=Path, default=root, help="our repository")
    p.add_argument("--rev", default="HEAD", help="revision to report on")
    p.add_argument("--linux", type=Path, required=True,
                   help="git dir with the mainline tags (see --fetch)")
    p.add_argument("--fetch", action="store_true",
                   help=f"fetch the tags and ipu6 blobs from {LINUX_URL} first")
    p.add_argument("--target", action="store_true",
                   help="also diff against the target tree (pin + "
                        + " + ".join(TARGET_SERIES) + ")")
    p.add_argument("--out", type=Path,
                   default=root / "tools" / "notes" / "upstream-diff" / "history.json")
    args = p.parse_args(argv)

    if args.fetch:
        fetch_linux(args.linux, extra=[pin_sha(root)] if args.target else [])
    data = build(args.repo, args.rev, args.linux, local_only(root),
                 root if args.target else None)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(data, indent=1) + "\n")
    print(f"{len(data['points'])} points -> {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
