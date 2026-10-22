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

"""Parse .proto files into an index of messages and enums keyed by fully qualified name."""

import glob
import os
import re
from dataclasses import dataclass, field

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
