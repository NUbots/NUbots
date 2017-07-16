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

def register(command):

    # Install help
    command.help = 'Decode an nbs file into a series of json objects'

    # Drone arguments
    command.add_argument('file'
        , metavar='file'
        , help='The file to decode into a strint')

def run(file, **kwargs):

    # Open up our message output directory to get our protobuf types
    shared_path = os.path.join(b.cmake_cache[b.cmake_cache["CMAKE_PROJECT_NAME"] + '_BINARY_DIR'], 'shared')

    sys.path.append(shared_path)

    # Load all of the protobuf files as modules to use
    for dir_name, subdir, files in os.walk(shared_path):
        modules = pkgutil.iter_modules(path=[dir_name])
        for loader, module_name, ispkg in modules:
            if module_name.endswith('pb2'):

                # Load our protobuf module
                module = loader.find_module(module_name).load_module(module_name)

    parsers = {}

    # Now that we've imported them all get all the subclasses of protobuf message
    for message in google.protobuf.message.Message.__subclasses__():

        # Work out our original protobuf type
        pb_type = message.__module__.split('.')[:-1]
        pb_type.append(message.__name__)
        pb_type = '.'.join(pb_type).encode('utf-8')
        pb_hash = xxhash.xxh64(pb_type, seed=0x4e55436c).digest()

        pb_hash2 = xxhash.xxh64('NUsight<{}>'.format(pb_type), seed=0x4e55436c).digest()

        parsers[pb_hash] = (pb_type, message)

    # Now open the passed file
    with gzip.open(file, 'rb') if file.endswith('nbz') or file.endswith('.gz') else open(file, 'rb') as f:

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
            if type_hash in parsers:
                msg = parsers[type_hash][1].FromString(payload[16:])

                # So the else can always exist
                if False:
                    pass

                # If there is a special way to decode a particular message do it here
                # if parsers[type_hash][0] == b'message.input.Sensors':
                    # Read our servo

                else:
                    out = re.sub(r'\s+', ' ', MessageToJson(msg, True))
                    out = '{{ "type": "{}", "timestamp": {}, "data": {} }}'.format(parsers[type_hash][0].decode('utf-8'), timestamp, out)
                    # Print as a json object
                    print(out)

