#!/usr/bin/env python3
"""Generate source mutants of one kernel/ipu4/ C file.

    tools/mutation/mutants.py FILE [--json]

A small token-level mutator: no parser dependency, good enough for the
driver's kernel C. Only code inside function bodies is mutated; comments,
strings, preprocessor lines, `#ifdef IPU6` blocks (never compiled) and the
arguments of logging calls are skipped. Each mutant is a list of
(offset, length, replacement) edits on the file's text.

Operators:
  ror     relational: < <= > >= == != swapped with a neighbour
  lcr     && <-> ||
  aor     binary + <-> -, += <-> -=, << <-> >>
  bor     binary | <-> &, |= <-> &=, unary ~ removed
  crp     integer constant: 0 -> 1, 1 -> 0, N -> N+1
  neg     if (C) -> if (!(C))
  sdl     call statement `f(...);` deleted
  ret     return -EFOO; -> return 0;   return 0; -> return -EINVAL;
"""
import json
import re
import sys

MULTI = sorted("""<<= >>= ... -> ++ -- << >> <= >= == != && || += -= *= /= %=
&= |= ^= ##""".split(), key=len, reverse=True)

# Calls whose deletion or argument changes only alter log output.
LOG_CALLS = re.compile(
    r"^(dev_(dbg|info|notice|warn|err|crit|err_ratelimited|warn_once|"
    r"err_probe|printk)|pr_\w+|printk|trace_\w+|WARN(_ON)?(_ONCE)?|"
    r"dev_dbg_ratelimited|dev_err_once|dev_info_once|dev_WARN\w*|"
    r"seq_printf|ipu6_dbg\w*)$")
# Statement-deletion targets whose removal cannot compile or is noise.
# Calls whose numeric arguments are delays or timeouts.
DELAY_CALLS = re.compile(r"^(\w*poll_timeout\w*|\w*sleep\w*|\w*delay|"
                         r"msecs_to_jiffies|usecs_to_jiffies|"
                         r"wait_for_completion\w*|wait_event\w*)$")
NO_SDL = re.compile(r"^(BUILD_BUG_ON\w*|might_sleep|lockdep_assert\w*|"
                    r"static_assert|barrier|fallthrough)$")
OPERAND_END = {"ident", "num"}

ROR = {"<": ["<=", ">="], "<=": ["<", ">"], ">": [">=", "<="],
       ">=": [">", "<"], "==": ["!="], "!=": ["=="]}
SWAP = {"&&": ("lcr", "||"), "||": ("lcr", "&&"),
        "+=": ("aor", "-="), "-=": ("aor", "+="),
        "<<": ("aor", ">>"), ">>": ("aor", "<<"),
        "|=": ("bor", "&="), "&=": ("bor", "|=")}


class Tok:
    __slots__ = ("kind", "text", "pos", "line", "live")

    def __init__(self, kind, text, pos, line, live):
        self.kind, self.text, self.pos, self.line = kind, text, pos, line
        self.live = live


def tokenize(src):
    """Tokens outside comments, strings and preprocessor lines."""
    toks = []
    i, line, n = 0, 1, len(src)
    bol = True  # at beginning of line (only whitespace so far)
    # Preprocessor conditionals: [this branch dead, #else branch dead].
    # Only IPU6 (never defined) makes a branch dead.
    stack = []

    def dead():
        return any(d for d, _ in stack)

    while i < n:
        c = src[i]
        if c == "\n":
            line += 1
            bol = True
            i += 1
            continue
        if c in " \t\r\f\v":
            i += 1
            continue
        if bol and c == "#":
            j = i
            while True:
                k = src.find("\n", j)
                if k < 0:
                    k = n
                if src[k - 1:k] == "\\" and k < n:
                    j = k + 1
                    continue
                break
            directive = src[i:k]
            # Comments inside the directive line count lines too.
            line += directive.count("\n")
            d = re.sub(r"/\*.*?\*/|//.*", "", directive, flags=re.S)
            d = re.sub(r"\s+", " ", d.replace("\\\n", " ")).strip()
            m = re.match(r"#\s*(\w+)\s*(.*)", d)
            if m:
                kw, arg = m.group(1), m.group(2)
                ipu6 = arg in ("IPU6", "defined(IPU6)", "defined IPU6")
                if kw in ("ifdef", "if"):
                    stack.append([ipu6, False])
                elif kw == "ifndef":
                    stack.append([False, ipu6])
                elif kw in ("else", "elif") and stack:
                    stack[-1] = [stack[-1][1], False]
                elif kw == "endif" and stack:
                    stack.pop()
            i = k
            continue
        bol = False
        if src.startswith("//", i):
            k = src.find("\n", i)
            i = n if k < 0 else k
            continue
        if src.startswith("/*", i):
            k = src.find("*/", i + 2)
            k = n if k < 0 else k + 2
            line += src.count("\n", i, k)
            i = k
            continue
        if c in "\"'":
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            toks.append(Tok("str", src[i:j + 1], i, line, not dead()))
            i = j + 1
            continue
        m = re.compile(r"[A-Za-z_]\w*").match(src, i)
        if m:
            toks.append(Tok("ident", m.group(), i, line, not dead()))
            i = m.end()
            continue
        m = re.compile(r"(0[xX][0-9a-fA-F]+|\d+)[uUlL]*").match(src, i)
        if m:
            toks.append(Tok("num", m.group(), i, line, not dead()))
            i = m.end()
            continue
        for op in MULTI:
            if src.startswith(op, i):
                break
        else:
            op = c
        toks.append(Tok("op", op, i, line, not dead()))
        i += len(op)
    return toks


def match_paren(toks, i):
    """Index of the token closing the bracket at toks[i]."""
    o = toks[i].text
    c = {"(": ")", "[": "]", "{": "}"}[o]
    depth = 0
    for j in range(i, len(toks)):
        if toks[j].text == o:
            depth += 1
        elif toks[j].text == c:
            depth -= 1
            if depth == 0:
                return j
    return None


def mutable_mask(toks):
    """True for tokens inside a function body and not in a log call."""
    mask = [False] * len(toks)
    depth = 0
    in_func = False
    skip_until = -1
    delay_until = -1
    for i, t in enumerate(toks):
        if t.text == "{":
            if depth == 0:
                prev = toks[i - 1].text if i else ""
                in_func = prev == ")" or (
                    toks[i - 1].kind == "ident" and i > 1 and
                    toks[i - 2].text == ")")
            depth += 1
        elif t.text == "}":
            depth -= 1
            if depth == 0:
                in_func = False
        if (in_func and depth > 0 and t.kind == "ident" and
                LOG_CALLS.match(t.text) and i + 1 < len(toks) and
                toks[i + 1].text == "("):
            end = match_paren(toks, i + 1)
            if end is not None:
                skip_until = max(skip_until, end)
        if (t.kind == "ident" and DELAY_CALLS.match(t.text) and
                i + 1 < len(toks) and toks[i + 1].text == "("):
            end = match_paren(toks, i + 1)
            if end is not None:
                delay_until = max(delay_until, end)
        ok = in_func and depth > 0 and t.live and i > skip_until
        mask[i] = "crp-skip" if ok and i <= delay_until else ok
    return mask


def binary_context(toks, i):
    p = toks[i - 1]
    return p.kind in OPERAND_END or p.text in (")", "]")


def generate(src):
    toks = tokenize(src)
    mask = mutable_mask(toks)
    out = []

    # Line each token's statement starts on (gcov counts it there).
    stmt, start = [], None
    for i, t in enumerate(toks):
        if start is None:
            start = t.line
        stmt.append(start)
        if t.text in (";", "{", "}"):
            start = None

    def add(op, i, edits, desc):
        out.append({"op": op, "line": toks[i].line, "stmt_line": stmt[i],
                    "edits": edits, "desc": desc})

    for i, t in enumerate(toks):
        if not mask[i]:
            continue
        x = t.text
        if t.kind == "op":
            if x in ROR:
                for r in ROR[x]:
                    add("ror", i, [(t.pos, len(x), r)], f"{x} -> {r}")
            elif x in SWAP:
                op, r = SWAP[x]
                add(op, i, [(t.pos, len(x), r)], f"{x} -> {r}")
            elif x in ("+", "-") and binary_context(toks, i):
                r = "-" if x == "+" else "+"
                add("aor", i, [(t.pos, 1, r)], f"{x} -> {r}")
            elif x in ("|", "&") and binary_context(toks, i):
                r = "&" if x == "|" else "|"
                add("bor", i, [(t.pos, 1, r)], f"{x} -> {r}")
            elif x == "~":
                add("bor", i, [(t.pos, 1, "")], "~ removed")
        elif t.kind == "num":
            # Not in an array declarator or a case label, not a delay or
            # timeout (mostly equivalent mutants).
            if toks[i - 1].text in ("[", "case") or mask[i] == "crp-skip":
                continue
            m = re.match(r"(0[xX][0-9a-fA-F]+|\d+)(\w*)", x)
            v = int(m.group(1), 0)
            if v >= 100 and m.group(1)[:2].lower() != "0x":
                continue
            nv = {0: 1, 1: 0}.get(v, v + 1)
            r = (hex(nv) if m.group(1)[:2].lower() == "0x" else str(nv)) \
                + m.group(2)
            add("crp", i, [(t.pos, len(x), r)], f"{x} -> {r}")
        elif t.kind == "ident":
            nxt = toks[i + 1] if i + 1 < len(toks) else None
            if x == "if" and nxt and nxt.text == "(":
                end = match_paren(toks, i + 1)
                if end is not None:
                    add("neg", i, [(nxt.pos + 1, 0, "!("),
                                   (toks[end].pos, 0, ")")], "if (!(...))")
            elif x == "return" and nxt:
                if (nxt.text == "-" and i + 3 < len(toks) and
                        toks[i + 2].text.startswith("E") and
                        toks[i + 3].text == ";"):
                    a, b = nxt.pos, toks[i + 2].pos + len(toks[i + 2].text)
                    add("ret", i, [(a, b - a, "0")],
                        f"return -{toks[i + 2].text} -> return 0")
                elif (nxt.text == "0" and i + 2 < len(toks) and
                        toks[i + 2].text == ";"):
                    add("ret", i, [(nxt.pos, 1, "-EINVAL")],
                        "return 0 -> return -EINVAL")
            elif (nxt and nxt.text == "(" and
                    toks[i - 1].text in (";", "{", "}", "else") and
                    x != x.upper() and not LOG_CALLS.match(x) and
                    not NO_SDL.match(x)):
                end = match_paren(toks, i + 1)
                if (end is not None and end + 1 < len(toks) and
                        toks[end + 1].text == ";"):
                    b = toks[end + 1].pos + 1
                    add("sdl", i, [(t.pos, b - t.pos, ";")],
                        f"{x}(...); deleted")
    return out


def apply(src, edits):
    for pos, length, rep in sorted(edits, key=lambda e: e[0], reverse=True):
        src = src[:pos] + rep + src[pos + length:]
    return src


def main():
    path = sys.argv[1]
    src = open(path).read()
    muts = generate(src)
    lines = src.splitlines()
    for k, m in enumerate(muts):
        m["id"] = k
        m["orig"] = lines[m["line"] - 1].strip()
    if "--json" in sys.argv:
        json.dump(muts, sys.stdout, indent=1)
        return
    for m in muts:
        print(f"{m['id']:5} {path}:{m['line']:<5} {m['op']}  {m['desc']}")
    by = {}
    for m in muts:
        by[m["op"]] = by.get(m["op"], 0) + 1
    print(f"{len(muts)} mutants", by, file=sys.stderr)


if __name__ == "__main__":
    main()
