#!/usr/bin/env python3

try:
    # Import the nbs decoder
    import re
    from util import nbs_decoder
    from google.protobuf.json_format import MessageToJson

    def register(command):

        # Install help
        command.help = "Decode an nbs file into a series of json objects"

        # Drone arguments
        command.add_argument("path", metavar="path", help="The file to decode into a series of json objects")

    def run(path, **kwargs):

        for type_name, timestamp, msg in nbs_decoder.decode(path):
            out = re.sub(r"\s+", " ", MessageToJson(msg, True))
            out = '{{ "type": "{}", "timestamp": {}, "data": {} }}'.format(type_name, timestamp, out)
            # Print as a json object
            print(out)


except BaseException as e:
    # Capture the exception in a variable
    ex = e

    def register(command):
        # Swallow arguments
        command.add_argument("_", nargs="*")

    def run(**kwargs):
        print("Cannot run this command due to the following error")
        print(ex)
