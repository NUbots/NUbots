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
Input fields that are never referenced anywhere in the module's sources (no `.field` / `->field` / `["key"]` /
`::VALUE` token) are dimmed. That is a plain token search: a field name shared by several messages counts
as used if any of them is touched, and a message handed whole to a utility function outside the module looks unused.

Example:
    ./b blackbox localisation/BallLocalisation skill/Walk
    ./b blackbox --all --format svg
"""

import os

from termcolor import cprint

import b

from .cpp import all_modules, find_modules, scan_module
from .proto import ProtoIndex
from .render import Fonts, RenderOptions, render_module


def register(command):
    command.description = "Render black-box diagrams (inputs/outputs with message fields) for NUClear modules"
    command.add_argument("modules", nargs="*", help="Module names, e.g. localisation/BallLocalisation or just Walk")
    command.add_argument("--all", action="store_true", help="Render every module")
    command.add_argument("--list", action="store_true", help="List available modules and exit")
    command.add_argument(
        "-o", "--output", default=os.path.join(b.project_dir, "recordings", "blackbox"), help="Output directory"
    )
    command.add_argument("-f", "--format", dest="fmt", choices=("png", "svg"), default="png", help="Image format")
    command.add_argument("-d", "--depth", type=int, default=1, help="How deep to expand nested messages (default 1)")
    command.add_argument("--max-fields", type=int, default=28, help="Maximum rows shown per message")
    command.add_argument("--docs", action="store_true", help="Show /// doc comments under each field")
    command.add_argument(
        "--no-usage",
        dest="usage",
        action="store_false",
        help="Don't dim input fields that are never referenced in the module sources",
    )
    command.add_argument("--columns", type=int, default=0, help="Force N card columns per side (default auto)")
    command.add_argument("--scale", type=float, default=2.0, help="PNG pixel density multiplier (default 2)")


def run(modules, all, list, output, fmt, depth, max_fields, docs, usage, columns, scale, **kwargs):
    modules_path = os.path.join(b.project_dir, "module")
    if list:
        for m in all_modules(modules_path):
            print(m)
        return
    names = all_modules(modules_path) if all else find_modules(modules, modules_path)
    if not names:
        cprint("No modules to render. Try `./b blackbox --list`.", "yellow")
        return

    opts = RenderOptions(fmt, depth, max_fields, docs, usage, columns, scale)

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
