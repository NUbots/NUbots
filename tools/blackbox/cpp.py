#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2026 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

"""Scan a NUClear module's C++ sources for what it reacts to (on<...>) and what it emits."""

import glob
import os
import re
from dataclasses import dataclass, field

from termcolor import cprint

DSL_MODIFIERS = {"Single", "Sync", "Pool", "Priority", "Buffer", "Inline", "Group", "MainThread", "Always", "Idle"}

# Role of each port. (accent colour, human label, sort priority)
ROLES = {
    "provide": ("#c084fc", "Provide", 0),
    "start": ("#c084fc", "Start", 1),
    "stop": ("#c084fc", "Stop", 1),
    "trigger": ("#38bdf8", "Trigger", 2),
    "with": ("#818cf8", "With", 3),
    "uses": ("#fbbf24", "Uses", 4),
    "when": ("#fbbf24", "When", 4),
    "network": ("#fb923c", "Network", 5),
    "watchdog": ("#fb923c", "Watchdog", 5),
    "every": ("#facc15", "Every", 6),
    "config": ("#2dd4bf", "Configuration", 7),
    "lifecycle": ("#94a3b8", "Lifecycle", 8),
    "other": ("#64748b", "?", 9),
    # outputs
    "emit": ("#4ade80", "emit", 0),
    "task": ("#fb923c", "Task", 1),
    "needs": ("#fb923c", "Needs", 1),
    "causing": ("#fbbf24", "Causing", 2),
    "graph": ("#2dd4bf", "graph", 3),
    "unknown": ("#64748b", "?", 9),
}


@dataclass
class Port:
    """One thing the module consumes or produces."""

    role: str
    cpp_type: str = ""  # canonical C++ name (after `using` resolution)
    label: str = ""  # display title override (e.g. "Every 10 ms", "Walk.yaml")
    tags: list = field(default_factory=list)  # extra tags e.g. "Optional", "Scope::DELAY", "last 3"
    extra: str = ""  # e.g. When condition, file name for config

    @property
    def key(self):
        return (self.cpp_type or self.label, self.role if self.role in ("every", "config") else "")


@dataclass
class ModuleInfo:
    name: str  # localisation/BallLocalisation
    path: str
    namespace: str
    class_name: str
    director: bool
    files: list
    reactions: int
    inputs: list
    outputs: list
    config_files: list
    text: str
    members: set = field(default_factory=set)  # every `x.name` / `x->name` token in the module sources
    keys: set = field(default_factory=set)  # every `["name"]` token (config lookups)
    scopes: set = field(default_factory=set)  # every `Foo::name` token (enum values)


def strip_cpp_comments(text):
    return re.sub(r"//[^\n]*|/\*.*?\*/", "", text, flags=re.S)


def match_bracket(text, i, o, c):
    """Index of the bracket closing the one at text[i], or -1."""
    depth = 0
    for j in range(i, len(text)):
        ch = text[j]
        if ch == o:
            depth += 1
        elif ch == c:
            depth -= 1
            if depth == 0:
                return j
    return -1


def split_top(s, sep=","):
    """Split on `sep` but only at depth 0 of <>, () and []."""
    out, depth, cur = [], 0, ""
    for ch in s:
        if ch in "<([":
            depth += 1
        elif ch in ">)]":
            depth -= 1
        if ch == sep and depth == 0:
            out.append(cur)
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur)
    return [x.strip() for x in out]


def parse_aliases(text):
    aliases = {}
    for m in re.finditer(r"\busing\s+([\w:]+)\s*;", text):
        target = m.group(1).strip(":")
        short = target.split("::")[-1]
        parts = target.split("::")
        if len(parts) >= 2 and parts[-1] == parts[-2]:  # inheriting constructor: using Foo::Foo;
            continue
        aliases[short] = target
    for m in re.finditer(r"\busing\s+(\w+)\s*=\s*([\w:<>, ]+?)\s*;", text):
        aliases[m.group(1)] = m.group(2).strip().strip(":")
    return aliases


def canon_cpp(t):
    t = re.sub(r"\b(const|volatile|typename|struct|class)\b", "", t)
    t = t.replace("&", "").replace("*", "").strip().lstrip(":")
    return " ".join(t.split())


def resolve_cpp(t, aliases):
    t = canon_cpp(t)
    tmpl = ""
    if "<" in t:
        i = t.index("<")
        t, tmpl = t[:i], t[i:]
    parts = t.split("::")
    for _ in range(6):
        if parts and parts[0] in aliases:
            target = aliases[parts[0]].split("::")
            if target == parts[:1]:
                break
            parts = target + parts[1:]
        else:
            break
    return "::".join(parts) + tmpl


UNIT_NAMES = {
    "nanoseconds": "ns",
    "microseconds": "µs",
    "milliseconds": "ms",
    "seconds": "s",
    "minutes": "min",
    "hours": "h",
}


def format_every(args, module_text):
    """Every<N, unit> / Every<N, Per<unit>> -> 'Every 10 ms' / '90 per second'."""
    if not args:
        return "Every tick"
    n = args[0].strip()
    if not re.match(r"^[\d.]+$", n):
        cm = re.search(rf"\b{re.escape(n)}\s*=\s*([\d.]+)", module_text)
        if cm:
            n = cm.group(1)
    unit = args[1].strip() if len(args) > 1 else "std::chrono::milliseconds"
    per = unit.startswith("Per<")
    if per:
        unit = unit[4:-1]
    short = UNIT_NAMES.get(unit.split("::")[-1], unit.split("::")[-1])
    if per:
        return f"{n} per {short}" if short != "s" else f"{n} Hz"
    return f"Every {n} {short}"


def parse_dsl_item(item, aliases, module_text, args_text, inputs, outputs, optional=False):
    m = re.match(r"^([\w:]+)\s*(?:<(.*)>)?\s*$", item.strip(), re.S)
    if not m:
        inputs.append(Port("other", label=item.strip()))
        return
    full, inner = m.group(1), (m.group(2) or "").strip()
    if full.startswith("Priority::") or full.split("::")[-1] in DSL_MODIFIERS:
        return
    word = full.split("::")[-1]
    args = split_top(inner) if inner else []
    tags = ["Optional"] if optional else []

    def add_in(role, typ, extra_tags=(), extra=""):
        inputs.append(Port(role, resolve_cpp(typ, aliases), tags=tags + list(extra_tags), extra=extra))

    if word in ("Trigger", "With", "Provide", "Start", "Stop", "Uses", "Network"):
        for a in args:
            add_in(word.lower(), a)
    elif word == "Optional":
        for a in args:
            parse_dsl_item(a, aliases, module_text, args_text, inputs, outputs, optional=True)
    elif word == "Last":
        for a in args[1:]:
            parse_dsl_item(a, aliases, module_text, args_text, inputs, outputs, optional)
            if inputs:
                inputs[-1].tags.append(f"last {args[0]}")
    elif word == "When":
        cond = f"{args[1].split('::')[-1] if len(args) > 1 else ''} {args[2] if len(args) > 2 else ''}".strip()
        add_in("when", args[0], extra=cond)
    elif word == "Causing":
        outputs.append(Port("causing", resolve_cpp(args[0], aliases), extra=args[1] if len(args) > 1 else ""))
    elif word == "Needs":
        for a in args:
            outputs.append(Port("needs", resolve_cpp(a, aliases)))
    elif word == "Every":
        inputs.append(Port("every", label=format_every(args, module_text), tags=tags))
    elif word == "Watchdog":
        add_in("watchdog", args[0], extra=" ".join(a.split("::")[-1] for a in args[1:]))
    elif word == "Configuration":
        fm = re.search(r'"([^"]+)"', args_text)
        inputs.append(Port("config", cpp_type="extension::Configuration", label=fm.group(1) if fm else "config"))
    elif word in ("Startup", "Shutdown"):
        inputs.append(Port("lifecycle", label=word))
    elif word == "IO":
        inputs.append(Port("other", label="IO (file descriptor)"))
    elif word in ("TCP", "UDP"):
        inputs.append(Port("network", label=f"{word} socket"))
    else:
        inputs.append(Port("other", label=item.strip()))


def find_declared_type(ident, text):
    """Find the T in `auto ident = std::make_unique<T>(...)` / `std::unique_ptr<T> ident` somewhere in the module."""
    for pat in (
        rf"\bauto\s+{ident}\s*=\s*(?:std::)?make_(?:unique|shared)\s*<",
        rf"\b{ident}\s*=\s*(?:std::)?make_(?:unique|shared)\s*<",
    ):
        m = re.search(pat, text)
        if m:
            j = match_bracket(text, m.end() - 1, "<", ">")
            if j > 0:
                return text[m.end() : j].strip()
    for m in re.finditer(r"(?:std::)?(?:unique|shared)_ptr\s*<", text):
        j = match_bracket(text, m.end() - 1, "<", ">")
        if j > 0 and re.match(rf"\s*&?\s*{ident}\b", text[j + 1 :]):
            return text[m.end() : j].strip()
    return None


def parse_emit_arg(arg, aliases, text):
    """Return (role, cpp_type or None, raw label) for the first argument of an emit call."""
    arg = arg.strip()
    mm = re.match(r"^(?:std::)?make_(?:unique|shared)\s*<", arg)
    if mm:
        j = match_bracket(arg, mm.end() - 1, "<", ">")
        return resolve_cpp(arg[mm.end() : j], aliases) if j > 0 else None
    mm = re.match(r"^(?:std::)?(?:unique|shared)_ptr\s*<", arg)
    if mm:
        j = match_bracket(arg, mm.end() - 1, "<", ">")
        return resolve_cpp(arg[mm.end() : j], aliases) if j > 0 else None
    mm = re.match(r"^std::move\s*\((.*)\)$", arg, re.S)
    if mm:
        return parse_emit_arg(mm.group(1), aliases, text)
    if re.match(r"^graph\s*\(", arg):
        return "utility::nusight::graph"
    if re.match(r"^\w+$", arg):
        t = find_declared_type(arg, text)
        return resolve_cpp(t, aliases) if t else None
    return None


def scan_module(module_dir, rel_name):
    files = sorted(
        p
        for ext in ("cpp", "hpp", "h", "cc")
        for p in glob.glob(os.path.join(module_dir, "src", "**", f"*.{ext}"), recursive=True)
    )
    text = ""
    for p in files:
        with open(p, "r", encoding="utf-8", errors="replace") as f:
            text += "\n" + strip_cpp_comments(f.read())

    aliases = parse_aliases(text)
    inputs, outputs = [], []
    reactions = 0

    # on<...>(args)
    for m in re.finditer(r"\bon\s*<", text):
        j = match_bracket(text, m.end() - 1, "<", ">")
        if j < 0:
            continue
        dsl = text[m.end() : j]
        k = j + 1
        while k < len(text) and text[k].isspace():
            k += 1
        args_text = ""
        if k < len(text) and text[k] == "(":
            e = match_bracket(text, k, "(", ")")
            args_text = text[k + 1 : e] if e > 0 else ""
        reactions += 1
        for item in split_top(dsl):
            parse_dsl_item(item, aliases, text, args_text, inputs, outputs)

    # emit<...>(args)
    for m in re.finditer(r"(?<![\w.>])(?<!->)\bemit\s*", text):
        k = m.end()
        scope = ""
        if k < len(text) and text[k] == "<":
            e = match_bracket(text, k, "<", ">")
            if e < 0:
                continue
            scope = text[k + 1 : e].strip()
            k = e + 1
            while k < len(text) and text[k].isspace():
                k += 1
        if k >= len(text) or text[k] != "(":
            continue
        e = match_bracket(text, k, "(", ")")
        if e < 0:
            continue
        args = split_top(text[k + 1 : e])
        if not args:
            continue
        first = args[0]
        if first == "powerplant" and len(args) > 1:  # static emit(powerplant, msg) helpers
            first = args[1]
        if re.search(r"&\s*\w+$", first) or first.startswith("NUClear::PowerPlant"):
            continue  # function declaration, not a call

        typ = parse_emit_arg(first, aliases, text)
        tags = []
        role = "emit"
        if scope.endswith("Task"):
            role = "task"
        elif scope:
            tags.append(scope.split("::")[-1].title())
        if re.match(r"^(?:std::)?(?:unique|shared)_ptr\s*<.*>\s*\(\s*nullptr\s*\)$", first, re.S):
            tags.append("nullptr")
        if typ == "utility::nusight::graph":
            role, typ = "graph", "message::eye::DataPoint"
        if typ is None:
            outputs.append(Port("unknown", label=first.split("(")[0][:40], tags=tags))
        else:
            outputs.append(Port(role, typ, tags=tags))

    # Module identity
    cm = re.search(r"\bclass\s+(\w+)\s*:\s*public\s+([\w:]+)", text)
    class_name = cm.group(1) if cm else os.path.basename(module_dir)
    director = bool(cm and "BehaviourReactor" in cm.group(2))
    nm = re.search(r"\bnamespace\s+([\w:]+)\s*\{", text)
    namespace = nm.group(1) if nm else "module::" + rel_name.replace("/", "::").rsplit("::", 1)[0]

    return ModuleInfo(
        name=rel_name,
        path=module_dir,
        namespace=namespace,
        class_name=class_name,
        director=director,
        files=[os.path.relpath(p, module_dir) for p in files],
        reactions=reactions,
        inputs=dedupe(inputs),
        outputs=dedupe(outputs),
        config_files=[p.label for p in inputs if p.role == "config"],
        text=text,
        members=set(re.findall(r"(?:\.|->)\s*(\w+)", text)),
        keys=set(re.findall(r'\[\s*"(\w+)"\s*\]', text)),
        scopes=set(re.findall(r"::\s*(\w+)", text)),
    )


def dedupe(ports):
    """Merge ports that refer to the same type, collecting their roles into tags."""
    merged = {}
    for p in ports:
        if p.key in merged:
            q = merged[p.key]
            if p.role != q.role and ROLES[p.role][1] not in q.tags:
                q.tags.append(ROLES[p.role][1])
                if ROLES[p.role][2] < ROLES[q.role][2]:
                    q.tags.append(ROLES[q.role][1])
                    q.role = p.role
                    q.tags.remove(ROLES[p.role][1])
            for t in p.tags:
                if t not in q.tags:
                    q.tags.append(t)
            if p.extra and p.extra not in q.extra:
                q.extra = (q.extra + ", " + p.extra).strip(", ")
        else:
            merged[p.key] = p
    out = list(merged.values())
    out.sort(key=lambda p: (ROLES[p.role][2], (p.cpp_type or p.label).split("::")[-1].lower()))
    return out


def all_modules(modules_path):
    found = []
    for folder, _, files in os.walk(modules_path):
        if "CMakeLists.txt" in files:
            with open(os.path.join(folder, "CMakeLists.txt"), "r") as f:
                if re.search(r"^\s*nuclear_module\s*\(", f.read(), re.M | re.I):
                    found.append(os.path.relpath(folder, modules_path))
    return sorted(found)


def find_modules(names, modules_path):
    available = all_modules(modules_path)
    chosen = []
    for name in names:
        n = name.strip().rstrip("/").replace("::", "/").replace("\\", "/")
        n = re.sub(r"^(\./)?module/", "", n)
        if n in available:
            chosen.append(n)
            continue
        matches = [a for a in available if a.endswith("/" + n) or a.split("/")[-1].lower() == n.lower()]
        if len(matches) == 1:
            chosen.append(matches[0])
        elif not matches:
            cprint(f"No module matches '{name}'", "red")
        else:
            cprint(f"'{name}' is ambiguous: {', '.join(matches)}", "red")
    return list(dict.fromkeys(chosen))
