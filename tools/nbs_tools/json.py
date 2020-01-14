#!/usr/bin/env python3

import re

from google.protobuf.json_format import MessageToJson

from .nbs import Decoder


def register(command):
    command.help = "Decode an nbs file and convert it to json"

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to convert to json")


def run(files, **kwargs):

    for packet in Decoder(*files):
        out = re.sub(r"\s+", " ", MessageToJson(packet.msg, True))
        out = '{{ "type": "{}", "timestamp": {}, "data": {} }}'.format(packet.type, packet.timestamp, out)
        # Print as a json object
        print(out)
