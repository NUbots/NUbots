#!/usr/bin/env python3

import xxhash
import gzip
import struct
import sys
import os
import re
import b
import pkgutil
import google.protobuf.message
from google.protobuf.json_format import MessageToJson

class NUsightDecoder:
    def __init__(self, pb_type):
        self.pb_type = pb_type

    def FromString(self, payload):
        # Strip off the filterid and timestamp (first 9 bytes)
        return self.pb_type.FromString(payload[9:])

def register(command):

    # Install help
    command.help = 'Decode an nbs file into a series of json objects'

    # Drone arguments
    command.add_argument('file'
        , metavar='file'
        , help='The file to decode into a series of json objects')

def run(file, **kwargs):

    # Open up our message output directory to get our protobuf types
    shared_path = os.path.join(b.binary_dir, 'shared')

    sys.path.append(shared_path)

    # Load all of the protobuf files as modules to use
    for dir_name, subdir, files in os.walk(shared_path):
        modules = pkgutil.iter_modules(path=[dir_name])
        for loader, module_name, ispkg in modules:
            if module_name.endswith('pb2'):

                # Load our protobuf module
                module = loader.find_module(module_name).load_module(module_name)

    decoders = {}

    # Now that we've imported them all get all the subclasses of protobuf message
    for message in google.protobuf.message.Message.__subclasses__():

        # Work out our original protobuf type
        pb_type = '.'.join(message.DESCRIPTOR.full_name.split('.')[1:])
        # Reverse to little endian
        pb_hash = bytes(reversed(xxhash.xxh64(pb_type, seed=0x4e55436c).digest()))
        decoders[pb_hash] = (pb_type, message)

        # We can also decode NUsight<> wrapped packets
        nusight_type = 'NUsight<{}>'.format(pb_type)
        # Reverse to little endian
        nusight_hash = bytes(reversed(xxhash.xxh64(nusight_type, seed=0x4e55436c).digest()))
        decoders[nusight_hash] = (nusight_type, NUsightDecoder(message))


    # Now open the passed file
    with gzip.open(file, 'rb') if file.endswith('nbz') or file.endswith('.gz') else open(file, 'rb') as f:

        # NBS File Format:
        # 3 Bytes - NUClear radiation symbol header, useful for synchronisation when attaching to an existing stream
        # 4 Bytes - The remaining packet length i.e. 16 bytes + N payload bytes
        # 8 Bytes - 64bit timestamp in microseconds. Note: this is not necessarily a unix timestamp
        # 8 Bytes - 64bit bit hash of the message type
        # N bytes - The binary packet payload

        # While we can read a header
        while len(f.read(3)) == 3:

            # Read our size
            size = struct.unpack('<I', f.read(4))[0]

            # Read our payload
            payload = f.read(size)

            # Read our timestamp
            timestamp = struct.unpack('<Q', payload[:8])[0]

            # Read our hash
            type_hash = payload[8:16]

            # If we know how to parse this type, parse it
            if type_hash in decoders:
                msg = decoders[type_hash][1].FromString(payload[16:])

                # Put anything you don't want to output here, it will still output a message to show it is in the file
                if decoders[type_hash][0] in ['message.input.Image', 'NUsight<message.input.Image>']:
                    out = '{{ "type": "{}", "timestamp": {}, "data": null }}'.format(decoders[type_hash][0], timestamp)
                    print(out)

                # By default output as json
                else:
                    out = re.sub(r'\s+', ' ', MessageToJson(msg, True))
                    out = '{{ "type": "{}", "timestamp": {}, "data": {} }}'.format(decoders[type_hash][0], timestamp, out)
                    # Print as a json object
                    print(out)
