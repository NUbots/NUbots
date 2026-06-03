#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2020 NUbots
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

from utility.nbs import MessageTypes, resolve_nbs_paths
from utility.nbs.load_nbs import load_nbs


def register(command):
    command.description = "List all message types present in one or more nbs files"

    command.add_argument("files", metavar="files", nargs="+", help="The nbs files or directories to inspect")


def run(files, **kwargs):
    nbs_files = resolve_nbs_paths(files)
    if not nbs_files:
        print("No .nbs files found")
        exit(1)

    index, maps = load_nbs(*nbs_files, show_progress=True)

    type_names = []
    for h in set(index["type_hash"]):
        if h in MessageTypes:
            type_names.append(MessageTypes[h].name)
        else:
            type_names.append(f"<unknown:{h:#018x}>")

    for name in sorted(type_names):
        print(name)
