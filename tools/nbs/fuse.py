#!/usr/bin/env python3

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
