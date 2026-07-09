#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2019 NUbots
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

import re

from google.protobuf.json_format import MessageToJson

from utility.nbs import LinearDecoder, resolve_nbs_paths


def register(command):
    command.description = "Decode an nbs file and convert it to json"

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to convert to json")
    command.add_argument(
        "--keep-zeros",
        action="store_true",
        default=True,
        help="Include fields whose value is zero in the JSON output (true by default)",
    )


def run(files, keep_zeros, **kwargs):
    for packet in LinearDecoder(*resolve_nbs_paths(files)):
        out = re.sub(r"\s+", " ", MessageToJson(packet.msg, always_print_fields_with_no_presence=keep_zeros))
        out = '{{ "type": "{}", "timestamp": {}, "data": {} }}'.format(packet.type.name, packet.emit_timestamp, out)
        # Print as a json object
        print(out)
