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

import fnmatch
import os

from tqdm import tqdm

from utility.nbs import Encoder, LinearDecoder


def register(command):
    command.description = "Filter an nbs stream, removing messages from it"
    command.epilog = """
        The command will first remove any patterns provided with -r
        (or everything if no remove patterns are given)
        and then keep any of those removed that match the -k patterns.
        Message names can be filtered with shell globbing syntax
        (e.g. *Image will match message.input.Image and message.output.CompressedImage).
        Message subtypes can also be filtered by providing the subtype after the message
        filter as #SubType (e.g. *CompressedImage#Left). A globbing filter can also
        be used for this. The subtype filter is matched against the messages 'name' field,
        if it has one. Message IDs can be filters using :ID  (e.g. *CompressedImage:12345).
        An exact match must be provided for filtering IDs.
    """

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to filter")
    command.add_argument("-k", "--keep", action="append", help="A message pattern to keep")
    command.add_argument("-r", "--remove", action="append", help="A message pattern to remove")
    command.add_argument("-o", "--output", help="The output file to store the filtered nbs in")


def _filter_to(input_files, output, keep, remove):
    with Encoder(output) as out:
        for packet in tqdm(
            LinearDecoder(*input_files, show_progress=True),
            unit="packet",
            unit_scale=True,
            dynamic_ncols=True,
        ):
            # To start with we keep messages
            valid = True

            # If remove flags any message mark it as invalid
            for pattern in remove:
                if match_pattern(packet, pattern):
                    valid = False

            # If keep flags any message mark it as valid
            for pattern in keep:
                if match_pattern(packet, pattern):
                    valid = True

            # If it's still valid write it to the new file
            if valid:
                out.write(packet.emit_timestamp, packet.msg)


def run(files, keep, remove, output, **kwargs):
    # If no remove was provided, we assume we remove all
    if remove is None:
        if keep is None:
            print("You must provide either something to remove or keep")
            exit(1)
        else:
            remove = ["*"]

    # If we didn't provide any keep arguments we at least need to ignore them
    if keep is None:
        keep = []

    if output is not None:
        # Explicit output path: merge all input files into one output
        _filter_to(files, output, keep, remove)
    else:
        # Default: write each input to a "filtered" subfolder alongside it.
        # Directories are expanded to their contained .nbs files, each filtered individually.
        for f in files:
            f_abs = os.path.abspath(f)
            if os.path.isdir(f_abs):
                nbs_files = sorted(
                    os.path.join(f_abs, name)
                    for name in os.listdir(f_abs)
                    if name.endswith(".nbs")
                )
                if not nbs_files:
                    print(f"No .nbs files found in directory: {f}")
                    continue
                for nbs_file in nbs_files:
                    out_dir = os.path.join(f_abs, "filtered")
                    os.makedirs(out_dir, exist_ok=True)
                    out_path = os.path.join(out_dir, os.path.basename(nbs_file))
                    _filter_to([nbs_file], out_path, keep, remove)
            else:
                out_dir = os.path.join(os.path.dirname(f_abs), "filtered")
                os.makedirs(out_dir, exist_ok=True)
                out_path = os.path.join(out_dir, os.path.basename(f_abs))
                _filter_to([f_abs], out_path, keep, remove)


def match_pattern(packet, pattern):
    # Separate the pattern for the message name and the subtype
    # If no subtype is specified it is set to *
    name_pattern = pattern
    subtype_pattern = "*"
    match_id = False
    if "#" in pattern:
        name_pattern, subtype_pattern = pattern.split("#")
    elif ":" in pattern:
        name_pattern, subtype_pattern = pattern.split(":")
        subtype_pattern = int(subtype_pattern)
        match_id = True

    # Check to see if the message name matches the provided name pattern
    # This match can use wildcards to match single/multiple characters
    name_match = fnmatch.fnmatch(packet.type.name, name_pattern)

    # Check to see if the subtype/name matches
    subtype_match = True
    if subtype_pattern != "*":
        if not match_id:
            # Check to see if the name field matches the supplied subtype pattern
            # This match can use wildcards to match single/multiple characters
            if hasattr(packet.msg, "name"):
                subtype_match = fnmatch.fnmatch(packet.msg.name, subtype_pattern)
        elif packet.subtype != 0:
            # Check to see if the message subtype matches the supplied subtype
            # This has to be an exact match as the subtypes are hashed
            subtype_match = packet.subtype == subtype_pattern

    # Return the matching status
    return name_match and subtype_match
