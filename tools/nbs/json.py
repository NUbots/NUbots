#!/usr/bin/env python3

import re

from google.protobuf.json_format import MessageToJson

from utility.nbs import LinearDecoder


def register(command):
    command.description = "Decode an nbs file and convert it to json"

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to convert to json")


def run(files, **kwargs):
    for packet in LinearDecoder(*files):
        out = re.sub(r"\s+", " ", MessageToJson(packet.msg, True))
        out = '{{ "type": "{}", "timestamp": {}, "data": {} }}'.format(packet.type.name, packet.emit_timestamp, out)
        # Print as a json object
        print(out)
