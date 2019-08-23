#!/usr/bin/env python3

import re
from . import decoder
from google.protobuf.json_format import MessageToJson


def register(command):
    command.help = "Decode an nbs file and convert it to json"

    # Command arguments
    command.add_argument("path", metavar="path", help="The nbs file to convert to json")


def run(path, **kwargs):

    for packet in decoder.decode(path):
        out = re.sub(r"\s+", " ", MessageToJson(packet.msg, True))
        out = '{{ "type": "{}", "timestamp": {}, "data": {} }}'.format(packet.type, packet.timestamp, out)
        # Print as a json object
        print(out)
