#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2021 NUbots
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
from tqdm import tqdm

from utility.nbs import Encoder, LinearDecoder


def register(command):
    command.description = (
        "Take a group of nbs files and fuse them back into a single set of nbs files ordered by timestamp"
    )

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to convert to json")
    command.add_argument("-o", "--output", help="The output file to store the fused nbs in")


def run(files, output, **kwargs):
    # If we don't have a output, choose a default output name
    if output is None:
        output = "fused.nbs"

    # Index and load all the nbs files
    index = LinearDecoder(*files, show_progress=True)

    # Write out the timestamp sorted data to a single location
    with Encoder(output) as out:
        for packet in tqdm(index, unit="packet", dynamic_ncols=True):
            out.write(packet.emit_timestamp, packet.msg)
