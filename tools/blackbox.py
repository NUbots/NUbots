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

"""
Render a "black box" diagram for one or more NUClear modules.

The left side of the image lists everything the module reacts to (Trigger, With, Provide, Every, Configuration, ...)
and the right side lists everything it emits (emit, emit<Task>, Needs, ...). Every protobuf message is broken down
into its fields and their types by reading the .proto definitions in shared/message and nuclear/message/proto.

Example:
    ./b blackbox localisation/BallLocalisation skill/Walk
    ./b blackbox --all --format svg
"""

import glob
import os
import re
from dataclasses import dataclass, field

from termcolor import cprint

import b

# --------------------------------------------------------------------------------------------------------------------
# Protobuf index
# --------------------------------------------------------------------------------------------------------------------

SCALAR_TYPES = {
    "double",
    "float",
    "int32",
    "int64",
    "uint32",
    "uint64",
    "sint32",
    "sint64",
    "fixed32",
    "fixed64",
    "sfixed32",
    "sfixed64",
    "bool",
    "string",
    "bytes",
}


@dataclass
class ProtoField:
    name: str
    type: str  # type as written in the .proto (for enum values this is the numeric value)
    label: str = ""  # "", "repeated", "optional", "map", "value"
    key_type: str = ""  # for maps
    doc: str = ""


@dataclass
class ProtoMessage:
    fqn: str
    name: str
    file: str
    is_enum: bool = False
    neutron: bool = False
    doc: str = ""
    fields: list = field(default_factory=list)
    nested: dict = field(default_factory=dict)


class ProtoIndex:
    """Parses every .proto file it is given and indexes messages/enums by their fully qualified name."""

    FIELD_RE = re.compile(
        r"^(?:(repeated|optional|required)\s+)?"
        r"(map\s*<\s*([\w.]+)\s*,\s*([\w.]+)\s*>|[\w.]+)"
        r"\s+(\w+)\s*=\s*-?\d+(?:\s*\[.*\])?$"
    )
    ENUM_VALUE_RE = re.compile(r"^(\w+)\s*=\s*(-?\d+)(?:\s*\[.*\])?$")

    def __init__(self, roots):
        self.messages = {}
        for root, neutron in roots:
            for path in sorted(glob.glob(os.path.join(root, "**", "*.proto"), recursive=True)):
                self._load(path, neutron)

    def _load(self, path, neutron):
        with open(path, "r", encoding="utf-8") as f:
            raw = f.read()
        raw = re.sub(r"/\*.*?\*/", "", raw, flags=re.S)

        # Keep `///` doc comments as tokens, drop every other comment
        lines = []
        for line in raw.splitlines():
            if "///" in line:
                code, doc = line.split("///", 1)
                if code.strip():
                    lines.append(code)
                    lines.append("\x01T" + doc.strip() + "\x02")  # trailing doc -> previous field
                else:
                    lines.append("\x01L" + doc.strip() + "\x02")  # leading doc -> next statement
            else:
                lines.append(re.sub(r"//.*", "", line))
        text = "\n".join(lines)

        package = ""
        stack = []  # ProtoMessage or None (for oneof/service scopes)
        pending = []
        last_field = None
        buf = ""

        for m in re.finditer(r"\x01([LT])(.*?)\x02|([{};])|([^{};\x01]+)", text, re.S):
            if m.group(1):
                if m.group(1) == "L":
                    pending.append(m.group(2))
                elif last_field is not None:
                    last_field.doc = (last_field.doc + " " + m.group(2)).strip()
                continue
            if m.group(4):
                buf += m.group(4)
                continue

            tok = m.group(3)
            stmt = " ".join(buf.split())
            buf = ""

            if tok == "{":
                hm = re.match(r"(message|enum|oneof|extend|service)\s+([\w.]+)", stmt)
                if hm and hm.group(1) in ("message", "enum"):
                    name = hm.group(2)
                    parent = next((s for s in reversed(stack) if s is not None), None)
                    fqn = f"{parent.fqn}.{name}" if parent else (f"{package}.{name}" if package else name)
                    pm = ProtoMessage(fqn, name, path, hm.group(1) == "enum", neutron, " ".join(pending))
                    pending = []
                    self.messages[fqn] = pm
                    if parent:
                        parent.nested[name] = pm
                    stack.append(pm)
                else:
                    stack.append(None)
                last_field = None
            elif tok == "}":
                if stack:
                    stack.pop()
                last_field = None
            else:  # ';'
                if stmt.startswith("package "):
                    package = stmt[8:].strip()
                    continue
                owner = next((s for s in reversed(stack) if s is not None), None)
                if owner is None or not stmt:
                    pending = []
                    continue
                doc = " ".join(pending)
                pending = []
                if owner.is_enum:
                    em = self.ENUM_VALUE_RE.match(stmt)
                    if em:
                        last_field = ProtoField(em.group(1), em.group(2), "value", doc=doc)
                        owner.fields.append(last_field)
                    continue
                fm = self.FIELD_RE.match(stmt)
                if fm:
                    label, typ, key_t, val_t, name = fm.groups()
                    if typ.startswith("map"):
                        last_field = ProtoField(name, val_t, "map", key_t, doc)
                    else:
                        last_field = ProtoField(name, typ, label or "", doc=doc)
                    owner.fields.append(last_field)

    def resolve(self, typ, scope_fqn=""):
        """Resolve a type name as written inside `scope_fqn` to a ProtoMessage (or None for scalars/unknown)."""
        if typ.startswith("."):
            return self.messages.get(typ[1:])
        parts = scope_fqn.split(".") if scope_fqn else []
        for i in range(len(parts), -1, -1):
            candidate = ".".join(parts[:i] + [typ])
            if candidate in self.messages:
                return self.messages[candidate]
        return None

    def lookup_cpp(self, cpp_name):
        """Look up a C++ qualified name such as message::localisation::Ball or message::input::GameState::Phase."""
        return self.messages.get(cpp_name.replace("::", "."))


# --------------------------------------------------------------------------------------------------------------------
# C++ module scanning
# --------------------------------------------------------------------------------------------------------------------

# Words in an on<> DSL that carry no data and are simply skipped
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


# --------------------------------------------------------------------------------------------------------------------
# Module discovery
# --------------------------------------------------------------------------------------------------------------------


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


# --------------------------------------------------------------------------------------------------------------------
# Building cards (the content of each port box)
# --------------------------------------------------------------------------------------------------------------------

NEUTRON_HINT = {"iso": "transform", "quat": "quaternion", "vec": "vector", "mat": "matrix"}


@dataclass
class Row:
    indent: int
    name: str
    type: str
    kind: str  # scalar | message | enum | neutron | value | yaml
    doc: str = ""


@dataclass
class Card:
    title: str
    subtitle: str
    accent: str
    tags: list
    rows: list
    note: str = ""
    role: str = ""
    # layout (filled in later)
    x: int = 0
    y: int = 0
    w: int = 0
    h: int = 0


def short_type(t):
    t = t.replace("google.protobuf.", "")
    parts = t.split(".")
    return ".".join(parts[-2:]) if len(parts) > 2 else t


def message_rows(index, msg, depth, indent=0, seen=()):
    rows = []
    if msg.is_enum:
        for f in msg.fields:
            rows.append(Row(indent, f.name, f"= {f.type}", "value", f.doc))
        return rows
    for f in msg.fields:
        target = index.resolve(f.type, msg.fqn) if f.type not in SCALAR_TYPES else None
        if target is None:
            kind = "scalar"
            tdisp = short_type(f.type)
        elif target.neutron:
            kind = "neutron"
            tdisp = target.name
        elif target.is_enum:
            kind = "enum"
            tdisp = f"enum {target.name}"
        else:
            kind = "message"
            tdisp = target.name
        if f.label == "map":
            tdisp = f"map<{short_type(f.key_type)}, {tdisp}>"
        elif f.label == "repeated":
            tdisp = f"{tdisp}[]"
        rows.append(Row(indent, f.name, tdisp, kind, f.doc))
        if kind == "message" and depth > 1 and target.fqn not in seen:
            rows.extend(message_rows(index, target, depth - 1, indent + 1, seen + (msg.fqn,)))
    return rows


def yaml_rows(path, depth, indent=0):
    try:
        from ruamel.yaml import YAML

        with open(path, "r") as f:
            data = YAML(typ="safe").load(f)
    except Exception:
        return []
    if not isinstance(data, dict):
        return []

    def describe(v):
        if isinstance(v, bool):
            return "bool"
        if isinstance(v, int):
            return "int"
        if isinstance(v, float):
            return "float"
        if isinstance(v, str):
            return "string"
        if isinstance(v, list):
            return f"list[{len(v)}]"
        if isinstance(v, dict):
            return f"map{{{len(v)}}}"
        return type(v).__name__

    rows = []
    for k, v in data.items():
        rows.append(Row(indent, str(k), describe(v), "message" if isinstance(v, dict) else "yaml"))
        if isinstance(v, dict) and depth > 1:
            rows.extend(yaml_rows_from(v, depth - 1, indent + 1, describe))
    return rows


def yaml_rows_from(data, depth, indent, describe):
    rows = []
    for k, v in data.items():
        rows.append(Row(indent, str(k), describe(v), "message" if isinstance(v, dict) else "yaml"))
        if isinstance(v, dict) and depth > 1:
            rows.extend(yaml_rows_from(v, depth - 1, indent + 1, describe))
    return rows


def local_struct_rows(name, text):
    """Members of a `struct Name { ... };` declared inside the module itself (not a protobuf message)."""
    m = re.search(rf"\b(?:struct|class)\s+{re.escape(name)}\b[^;{{]*\{{", text)
    if not m:
        return None
    end = match_bracket(text, m.end() - 1, "{", "}")
    if end < 0:
        return None
    body, rows, depth, cur = text[m.end() : end], [], 0, ""
    for ch in body:
        if ch in "{(":
            depth += 1
        elif ch in "})":
            depth -= 1
        if ch == ";" and depth == 0:
            stmt = " ".join(cur.split())
            cur = ""
            dm = re.match(
                r"^(?:static\s+|const\s+|constexpr\s+|mutable\s+)*([\w:]+(?:<.*>)?)\s+(\w+)\s*(?:=|\{|$)", stmt
            )
            if dm and dm.group(1) not in ("return", "using", "public", "private", name) and "(" not in dm.group(1):
                rows.append(Row(0, dm.group(2), canon_cpp(dm.group(1)).split("::")[-1], "scalar"))
        else:
            cur += ch
    return rows


def build_card(port, index, module, depth):
    accent, role_label, _ = ROLES[port.role]
    tags = ([role_label] if port.role not in ("every", "lifecycle", "other", "unknown") else []) + port.tags
    if port.extra:
        tags.append(port.extra)

    if port.role == "config":
        found = glob.glob(os.path.join(module.path, "data", "config", "**", port.label), recursive=True)
        rows = yaml_rows(found[0], depth) if found else []
        sub = os.path.relpath(found[0], module.path) if found else "extension::Configuration"
        return Card(port.label, sub, accent, tags, rows, role=port.role)

    if not port.cpp_type:
        return Card(port.label, "", accent, tags, [], role=port.role)

    cpp = port.cpp_type
    base, tmpl = (cpp.split("<", 1) + [""])[:2]
    wrapper = ""
    if base in ("std::vector", "std::shared_ptr", "std::unique_ptr", "std::array") and tmpl:
        wrapper = base.split("::")[-1]
        cpp = split_top(tmpl.rstrip(">"))[0]
        base = cpp.split("<", 1)[0]

    msg = index.lookup_cpp(base)
    title = base.split("::")[-1]
    if base.count("::") >= 1 and msg is not None and msg.fqn.count(".") >= 2:
        # nested type such as GameState::Phase reads better with its parent
        title = "::".join(base.split("::")[-2:]) if msg.fqn.split(".")[-2][0].isupper() else title
    if wrapper:
        title = f"{wrapper}<{title}>"
    if port.role == "graph":
        title = "graph(...)"

    if msg is None:
        rows = local_struct_rows(base.split("::")[-1], module.text)
        if rows is not None:
            return Card(title, f"{cpp} · local struct", accent, tags, rows, role=port.role)
        return Card(title, f"{cpp} · C++ type, no .proto", accent, tags, [], role=port.role)
    rows = message_rows(index, msg, depth)
    if msg.is_enum:
        tags.append("enum")
    return Card(title, msg.fqn, accent, tags, rows, note=msg.doc, role=port.role)


# --------------------------------------------------------------------------------------------------------------------
# Rendering
# --------------------------------------------------------------------------------------------------------------------

THEME = {
    "bg": "#0b0d14",
    "grid": "#141826",
    "card": "#151926",
    "card_border": "#262c3f",
    "card_shadow": "#06070c",
    "box": "#10131d",
    "box_border": "#3b4260",
    "text": "#e8ebf5",
    "muted": "#7d859e",
    "faint": "#4b5169",
    "divider": "#232839",
    "scalar": "#8d95ad",
    "message": "#7dd3fc",
    "enum": "#fbbf24",
    "neutron": "#c4b5fd",
    "value": "#fbbf24",
    "yaml": "#5eead4",
}

FONT_FILES = {
    ("sans", False): ["Inter-Regular.ttf", "Inter.ttf", "Ubuntu-R.ttf", "DejaVuSans.ttf"],
    ("sans", True): ["Inter-Bold.ttf", "Ubuntu-B.ttf", "DejaVuSans-Bold.ttf"],
    ("mono", False): ["JetBrainsMono-Regular.ttf", "JetBrainsMonoNerdFont-Regular.ttf", "DejaVuSansMono.ttf"],
    ("mono", True): ["JetBrainsMono-Bold.ttf", "JetBrainsMonoNerdFont-Bold.ttf", "DejaVuSansMono-Bold.ttf"],
}
FONT_FAMILIES = {
    "sans": "Inter, Ubuntu, 'DejaVu Sans', Helvetica, Arial, sans-serif",
    "mono": "'JetBrains Mono', 'DejaVu Sans Mono', Menlo, Consolas, monospace",
}


def _find_font_files():
    dirs = [
        "/usr/share/fonts",
        "/usr/local/share/fonts",
        os.path.expanduser("~/.fonts"),
        os.path.expanduser("~/.local/share/fonts"),
        os.path.expanduser("~/.nix-profile/share/fonts"),
        "/Library/Fonts",
        "C:/Windows/Fonts",
    ]
    found = {}
    for d in dirs:
        if os.path.isdir(d):
            for root, _, files in os.walk(d):
                for f in files:
                    found.setdefault(f, os.path.join(root, f))
    return found


class Fonts:
    def __init__(self, scale):
        from PIL import ImageFont

        self.ImageFont = ImageFont
        self.scale = scale
        self.available = _find_font_files()
        self.cache = {}
        self.paths = {}
        for key, names in FONT_FILES.items():
            self.paths[key] = next((self.available[n] for n in names if n in self.available), None)

    def get(self, family, size, bold=False, raw_scale=None):
        s = raw_scale if raw_scale is not None else self.scale
        px = max(1, int(round(size * s)))
        k = (family, px, bold)
        if k not in self.cache:
            path = self.paths[(family, bold)]
            try:
                self.cache[k] = self.ImageFont.truetype(path, px) if path else self.ImageFont.load_default(px)
            except Exception:
                self.cache[k] = self.ImageFont.load_default(px)
        return self.cache[k]

    def width(self, text, family, size, bold=False):
        """Width in logical pixels."""
        return self.get(family, size, bold, raw_scale=4).getlength(text) / 4

    def ascent(self, family, size, bold=False):
        a, _ = self.get(family, size, bold, raw_scale=4).getmetrics()
        return a / 4


def hex_rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i : i + 2], 16) for i in (0, 2, 4))


def mix(a, b, t):
    ra, rb = hex_rgb(a), hex_rgb(b)
    return "#%02x%02x%02x" % tuple(int(ra[i] * (1 - t) + rb[i] * t) for i in range(3))


def bezier(p0, p1, p2, p3, n=40):
    pts = []
    for i in range(n + 1):
        t = i / n
        u = 1 - t
        x = u**3 * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t**3 * p3[0]
        y = u**3 * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t**3 * p3[1]
        pts.append((x, y))
    return pts


class PngCanvas:
    """Draws at a supersampled resolution and downsamples for antialiasing."""

    def __init__(self, w, h, fonts, scale, ss=2):
        from PIL import Image, ImageDraw

        self.Image = Image
        self.s = scale * ss
        self.ss = ss
        self.fonts = fonts
        self.img = Image.new("RGB", (int(w * self.s), int(h * self.s)), THEME["bg"])
        self.d = ImageDraw.Draw(self.img)

    def _p(self, v):
        return int(round(v * self.s))

    def rect(self, x, y, w, h, fill=None, outline=None, width=1, radius=0):
        self.d.rounded_rectangle(
            [self._p(x), self._p(y), self._p(x + w), self._p(y + h)],
            radius=self._p(radius),
            fill=fill,
            outline=outline,
            width=self._p(width) if outline else 0,
        )

    def circle(self, cx, cy, r, fill, outline=None, width=1):
        self.d.ellipse(
            [self._p(cx - r), self._p(cy - r), self._p(cx + r), self._p(cy + r)],
            fill=fill,
            outline=outline,
            width=self._p(width) if outline else 0,
        )

    def line(self, pts, fill, width=1):
        self.d.line([(self._p(x), self._p(y)) for x, y in pts], fill=fill, width=max(1, self._p(width)), joint="curve")

    def text(self, x, y, s, family, size, fill, bold=False, anchor="la"):
        f = self.fonts.get(family, size, bold, raw_scale=self.s)
        self.d.text((self._p(x), self._p(y)), s, font=f, fill=fill, anchor=anchor)

    def save(self, path):
        if self.ss != 1:
            w, h = self.img.size
            self.img = self.img.resize((w // self.ss, h // self.ss), self.Image.Resampling.LANCZOS)
        self.img.save(path, optimize=True)


class SvgCanvas:
    def __init__(self, w, h, fonts, scale=1, ss=1):
        self.w, self.h = w, h
        self.fonts = fonts
        self.parts = [f'<rect width="{w}" height="{h}" fill="{THEME["bg"]}"/>']

    def rect(self, x, y, w, h, fill=None, outline=None, width=1, radius=0):
        self.parts.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{radius}" '
            f'fill="{fill or "none"}" stroke="{outline or "none"}" stroke-width="{width if outline else 0}"/>'
        )

    def circle(self, cx, cy, r, fill, outline=None, width=1):
        self.parts.append(
            f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r}" fill="{fill}" stroke="{outline or "none"}" '
            f'stroke-width="{width if outline else 0}"/>'
        )

    def line(self, pts, fill, width=1):
        d = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
        self.parts.append(
            f'<polyline points="{d}" fill="none" stroke="{fill}" stroke-width="{width}" '
            f'stroke-linecap="round" stroke-linejoin="round"/>'
        )

    def text(self, x, y, s, family, size, fill, bold=False, anchor="la"):
        s = s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        ta = {"l": "start", "r": "end", "m": "middle"}[anchor[0]]
        y = y + self.fonts.ascent(family, size, bold)
        weight = "700" if bold else "400"
        self.parts.append(
            f'<text x="{x:.1f}" y="{y:.1f}" font-family="{FONT_FAMILIES[family]}" font-size="{size}" '
            f'font-weight="{weight}" fill="{fill}" text-anchor="{ta}">{s}</text>'
        )

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            f.write(
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
                f'viewBox="0 0 {self.w} {self.h}">\n' + "\n".join(self.parts) + "\n</svg>\n"
            )


# Layout constants (logical pixels)
L = dict(
    margin=56,
    header=64,
    card_w=372,
    card_gap=26,
    col_gap=34,
    box_w=340,
    gap=170,
    pad=16,
    row_h=21,
    doc_h=15,
    title=17,
    sub=10.5,
    field=12.5,
    tag=9.5,
    tag_h=17,
    max_col_h=2300,
)


def fit_text(fonts, s, family, size, max_w, bold=False):
    if fonts.width(s, family, size, bold) <= max_w:
        return s
    while len(s) > 1 and fonts.width(s + "…", family, size, bold) > max_w:
        s = s[:-1]
    return s + "…"


def measure_card(card, fonts, opts):
    """Compute card height, truncating rows to opts.max_fields."""
    pad, row_h = L["pad"], L["row_h"]
    h = pad + L["title"] + 6 + L["sub"] + 6
    if card.tags:
        h += L["tag_h"] + 8
    if card.note and opts.docs:
        h += L["doc_h"] + 2
    rows = card.rows
    if len(rows) > opts.max_fields:
        card.rows = rows[: opts.max_fields] + [Row(0, f"… {len(rows) - opts.max_fields} more", "", "scalar")]
    if card.rows:
        h += 8  # divider
        for r in card.rows:
            h += row_h + (L["doc_h"] if opts.docs and r.doc else 0)
    else:
        h += 4
    h += pad - 4
    card.w = L["card_w"]
    card.h = int(h)
    return card.h


def pack_columns(cards, n_cols):
    """Greedy shortest-column packing. Returns list of columns (lists of cards)."""
    cols = [[] for _ in range(n_cols)]
    heights = [0] * n_cols
    for c in cards:
        i = min(range(n_cols), key=lambda k: heights[k])
        cols[i].append(c)
        heights[i] += c.h + L["card_gap"]
    return cols, max(heights) - L["card_gap"] if cards else 0


def draw_tag(cv, fonts, x, y, text, accent):
    w = fonts.width(text, "sans", L["tag"], True) + 12
    cv.rect(
        x, y, w, L["tag_h"], fill=mix(accent, THEME["card"], 0.82), outline=mix(accent, THEME["card"], 0.55), radius=4
    )
    cv.text(x + 6, y + 3, text, "sans", L["tag"], accent, bold=True)
    return w


def draw_card(cv, fonts, card, opts):
    x, y, w, h = card.x, card.y, card.w, card.h
    pad = L["pad"]
    cv.rect(x + 3, y + 5, w, h, fill=THEME["card_shadow"], radius=10)
    cv.rect(x, y, w, h, fill=THEME["card"], outline=THEME["card_border"], width=1, radius=10)
    # accent stripe
    cv.rect(x, y + 10, 4, h - 20, fill=card.accent, radius=2)

    cy = y + pad
    cv.text(
        x + pad + 4,
        cy,
        fit_text(fonts, card.title, "sans", L["title"], w - 2 * pad - 8, True),
        "sans",
        L["title"],
        THEME["text"],
        bold=True,
    )
    cy += L["title"] + 6
    if card.subtitle:
        cv.text(
            x + pad + 4,
            cy,
            fit_text(fonts, card.subtitle, "mono", L["sub"], w - 2 * pad - 8),
            "mono",
            L["sub"],
            THEME["muted"],
        )
    cy += L["sub"] + 6
    if card.tags:
        tx = x + pad + 4
        for t in card.tags:
            if tx > x + w - pad - 40:
                break
            tx += draw_tag(cv, fonts, tx, cy, t, card.accent) + 6
        cy += L["tag_h"] + 8
    if card.note and opts.docs:
        cv.text(
            x + pad + 4,
            cy,
            fit_text(fonts, card.note, "sans", L["sub"], w - 2 * pad - 8),
            "sans",
            L["sub"],
            THEME["faint"],
        )
        cy += L["doc_h"] + 2
    if card.rows:
        cv.line([(x + pad, cy + 3), (x + w - pad, cy + 3)], THEME["divider"], 1)
        cy += 8
        for r in card.rows:
            ind = r.indent * 16
            tcol = THEME.get(r.kind, THEME["scalar"])
            tw = fonts.width(r.type, "mono", L["field"])
            cv.text(x + w - pad, cy + 3, r.type, "mono", L["field"], tcol, anchor="ra")
            max_name_w = w - 2 * pad - tw - 14 - ind
            name_col = THEME["text"] if r.kind != "value" else THEME["value"]
            if r.indent:
                cv.text(x + pad + 4 + ind - 12, cy + 3, "└", "mono", L["field"], THEME["faint"])
            cv.text(
                x + pad + 4 + ind,
                cy + 3,
                fit_text(fonts, r.name, "mono", L["field"], max_name_w),
                "mono",
                L["field"],
                name_col,
            )
            cy += L["row_h"]
            if opts.docs and r.doc:
                cv.text(
                    x + pad + 4 + ind,
                    cy - 3,
                    fit_text(fonts, r.doc, "sans", L["sub"], w - 2 * pad - 8 - ind),
                    "sans",
                    L["sub"],
                    THEME["faint"],
                )
                cy += L["doc_h"]
    else:
        cv.text(
            x + pad + 4,
            cy + 2,
            "no fields" if card.role != "unknown" else "type could not be resolved",
            "sans",
            L["sub"],
            THEME["faint"],
        )


def draw_box(cv, fonts, module, bx, by, bw, bh, n_in, n_out):
    cv.rect(bx + 4, by + 8, bw, bh, fill=THEME["card_shadow"], radius=16)
    cv.rect(bx, by, bw, bh, fill=THEME["box"], outline=THEME["box_border"], width=2, radius=16)
    # inner glow line
    cv.rect(bx + 6, by + 6, bw - 12, bh - 12, outline=mix(THEME["box_border"], THEME["box"], 0.6), radius=12)

    name = module.name.split("/")[-1]
    size = 26
    while fonts.width(name, "sans", size, True) > bw - 48 and size > 14:
        size -= 1
    cy = by + 30
    cv.text(bx + bw / 2, cy, name, "sans", size, THEME["text"], bold=True, anchor="ma")
    cy += size + 8
    cv.text(
        bx + bw / 2, cy, fit_text(fonts, module.namespace, "mono", 11, bw - 40), "mono", 11, THEME["muted"], anchor="ma"
    )
    cy += 20
    if module.director:
        tw = fonts.width("DIRECTOR", "sans", L["tag"], True) + 12
        draw_tag(cv, fonts, bx + bw / 2 - tw / 2, cy, "DIRECTOR", ROLES["provide"][0])
        cy += L["tag_h"] + 8
    cy += 8
    cv.line([(bx + 28, cy), (bx + bw - 28, cy)], THEME["divider"], 1)
    cy += 18

    stats = [("reactions", module.reactions), ("inputs", n_in), ("outputs", n_out)]
    slot = (bw - 40) / 3
    for i, (label, val) in enumerate(stats):
        cx = bx + 20 + slot * i + slot / 2
        cv.text(cx, cy, str(val), "sans", 22, THEME["text"], bold=True, anchor="ma")
        cv.text(cx, cy + 28, label, "sans", 10, THEME["muted"], anchor="ma")
    cy += 56
    cv.line([(bx + 28, cy), (bx + bw - 28, cy)], THEME["divider"], 1)
    cy += 16

    cv.text(bx + 28, cy, "SOURCES", "sans", 9, THEME["faint"], bold=True)
    cy += 16
    for f in module.files[:8]:
        cv.text(bx + 28, cy, fit_text(fonts, f, "mono", 10.5, bw - 56), "mono", 10.5, THEME["muted"])
        cy += 16
    if len(module.files) > 8:
        cv.text(bx + 28, cy, f"… {len(module.files) - 8} more", "mono", 10.5, THEME["faint"])
        cy += 16

    # side labels
    cv.text(bx + 14, by + bh - 22, "IN", "sans", 10, THEME["faint"], bold=True)
    cv.text(bx + bw - 14, by + bh - 22, "OUT", "sans", 10, THEME["faint"], bold=True, anchor="ra")


def render_module(module, index, opts, fonts, out_dir):
    in_cards = [build_card(p, index, module, opts.depth) for p in module.inputs]
    out_cards = [build_card(p, index, module, opts.depth) for p in module.outputs]

    for c in in_cards + out_cards:
        measure_card(c, fonts, opts)

    def choose_cols(cards):
        if opts.columns:
            return opts.columns
        total = sum(c.h + L["card_gap"] for c in cards)
        return 1 if total <= L["max_col_h"] else 2

    left_cols, left_h = pack_columns(in_cards, choose_cols(in_cards))
    right_cols, right_h = pack_columns(out_cards, choose_cols(out_cols := out_cards))

    side_w = lambda cols: len(cols) * L["card_w"] + (len(cols) - 1) * L["col_gap"]
    box_h = max(330, min(max(left_h, right_h), 700))
    content_h = max(left_h, right_h, box_h)
    width = L["margin"] * 2 + side_w(left_cols) + L["gap"] + L["box_w"] + L["gap"] + side_w(right_cols)
    height = L["margin"] + L["header"] + content_h + L["margin"]
    top = L["margin"] + L["header"]

    # Positions: left columns run outer -> inner, so the inner column touches the gap
    x = L["margin"]
    for col in reversed(left_cols):
        y = top
        for c in col:
            c.x, c.y = x, y
            y += c.h + L["card_gap"]
        x += L["card_w"] + L["col_gap"]
    x = L["margin"] + side_w(left_cols) + L["gap"]
    bx, by = x, top + (content_h - box_h) / 2
    x += L["box_w"] + L["gap"]
    for col in right_cols:
        y = top
        for c in col:
            c.x, c.y = x, y
            y += c.h + L["card_gap"]
        x += L["card_w"] + L["col_gap"]

    Canvas = SvgCanvas if opts.fmt == "svg" else PngCanvas
    cv = Canvas(int(width), int(height), fonts, opts.scale, 2)

    # subtle dot grid
    if opts.fmt != "svg":
        for gy in range(0, int(height), 28):
            for gx in range(0, int(width), 28):
                cv.circle(gx, gy, 0.8, THEME["grid"])

    # header
    cv.text(L["margin"], L["margin"] - 10, "NUbots · module black box", "sans", 12, THEME["muted"], bold=True)
    cv.text(L["margin"], L["margin"] + 10, module.name.replace("/", " / "), "mono", 11, THEME["faint"])
    cv.text(width - L["margin"], L["margin"] - 10, "./b blackbox", "mono", 11, THEME["faint"], anchor="ra")

    # connectors (under everything)
    def connect(cards, side):
        ordered = sorted(cards, key=lambda c: c.y + c.h / 2)
        n = len(ordered)
        for i, c in enumerate(ordered):
            py = by + box_h * (i + 1) / (n + 1)
            cy = c.y + min(c.h / 2, 60)
            if side == "in":
                p0, p3 = (c.x + c.w, cy), (bx, py)
            else:
                p0, p3 = (bx + L["box_w"], py), (c.x, cy)
            mid = (p0[0] + p3[0]) / 2
            pts = bezier(p0, (mid, p0[1]), (mid, p3[1]), p3)
            cv.line(pts, mix(c.accent, THEME["bg"], 0.55), 2.5)
            cv.line(pts, c.accent, 1.2)
            cv.circle(p0[0], p0[1], 4, THEME["bg"], outline=c.accent, width=1.5)
            cv.circle(p3[0], p3[1], 4, c.accent)

    connect(in_cards, "in")
    connect(out_cards, "out")

    draw_box(cv, fonts, module, bx, by, L["box_w"], box_h, len(in_cards), len(out_cards))
    for c in in_cards + out_cards:
        draw_card(cv, fonts, c, opts)

    if not in_cards:
        cv.text(L["margin"] + L["card_w"] / 2, top + 20, "no inputs found", "sans", 12, THEME["faint"], anchor="ma")
    if not out_cards:
        cv.text(
            bx + L["box_w"] + L["gap"] + L["card_w"] / 2,
            top + 20,
            "no outputs found",
            "sans",
            12,
            THEME["faint"],
            anchor="ma",
        )

    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, module.name.replace("/", "_") + "." + opts.fmt)
    cv.save(out)
    return out, (int(width), int(height))


# --------------------------------------------------------------------------------------------------------------------
# Entry points
# --------------------------------------------------------------------------------------------------------------------


def register(command):
    command.description = "Render black-box diagrams (inputs/outputs with message fields) for NUClear modules"
    command.add_argument("modules", nargs="*", help="Module names, e.g. localisation/BallLocalisation or just Walk")
    command.add_argument("--all", action="store_true", help="Render every module")
    command.add_argument("--list", action="store_true", help="List available modules and exit")
    command.add_argument(
        "-o", "--output", default=os.path.join(b.project_dir, "build", "blackbox"), help="Output directory"
    )
    command.add_argument("-f", "--format", dest="fmt", choices=("png", "svg"), default="png", help="Image format")
    command.add_argument("-d", "--depth", type=int, default=1, help="How deep to expand nested messages (default 1)")
    command.add_argument("--max-fields", type=int, default=28, help="Maximum rows shown per message")
    command.add_argument("--docs", action="store_true", help="Show /// doc comments under each field")
    command.add_argument("--columns", type=int, default=0, help="Force N card columns per side (default auto)")
    command.add_argument("--scale", type=float, default=2.0, help="PNG pixel density multiplier (default 2)")


def run(modules, all, list, output, fmt, depth, max_fields, docs, columns, scale, **kwargs):
    modules_path = os.path.join(b.project_dir, "module")
    if list:
        for m in all_modules(modules_path):
            print(m)
        return
    names = all_modules(modules_path) if all else find_modules(modules, modules_path)
    if not names:
        cprint("No modules to render. Try `./b blackbox --list`.", "yellow")
        return

    class Opts:
        pass

    opts = Opts()
    opts.fmt, opts.depth, opts.max_fields, opts.docs, opts.columns, opts.scale = (
        fmt,
        depth,
        max_fields,
        docs,
        columns,
        scale,
    )

    index = ProtoIndex(
        [
            (os.path.join(b.project_dir, "shared", "message"), False),
            (os.path.join(b.project_dir, "nuclear", "message", "proto"), True),
        ]
    )
    fonts = Fonts(scale)
    cprint(f"Indexed {len(index.messages)} protobuf types", "cyan")

    for name in names:
        module = scan_module(os.path.join(modules_path, name), name)
        out, size = render_module(module, index, opts, fonts, output)
        ins = ", ".join((p.cpp_type or p.label).split("::")[-1] for p in module.inputs) or "-"
        outs = ", ".join((p.cpp_type or p.label).split("::")[-1] for p in module.outputs) or "-"
        cprint(f"{name}", "green", attrs=["bold"])
        print(f"  in : {ins}")
        print(f"  out: {outs}")
        print(f"  -> {out} ({size[0]}x{size[1]})")
