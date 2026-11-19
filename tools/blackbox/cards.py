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

"""Turn each port into a card: a title plus the rows (fields) shown inside its box."""

import glob
import os
import re
from dataclasses import dataclass

from .cpp import ROLES, canon_cpp, match_bracket, split_top
from .proto import SCALAR_TYPES


@dataclass
class Row:
    indent: int
    name: str
    type: str
    kind: str  # scalar | message | enum | neutron | value | yaml
    doc: str = ""
    used: bool = True  # False -> dimmed: no reference to this field anywhere in the module sources


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


def mark_usage(rows, module, yaml=False):
    """Flag rows whose name never appears as a member / config key / enum value token in the module sources.

    This is a plain token search, so a field shared by several messages (timestamp, id, ...) counts as used if any
    of them is touched, and a message handed whole to a utility function outside the module looks untouched.
    """
    for r in rows:
        if r.kind == "value":
            r.used = r.name in module.scopes
        elif r.kind == "yaml" or (r.kind == "message" and yaml):
            r.used = r.name in module.keys or r.name in module.members
        else:
            r.used = r.name in module.members
    return rows


def build_card(port, index, module, depth, usage=False):
    card = _build_card(port, index, module, depth)
    if usage:
        mark_usage(card.rows, module, yaml=port.role == "config")
    return card


def _build_card(port, index, module, depth):
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
