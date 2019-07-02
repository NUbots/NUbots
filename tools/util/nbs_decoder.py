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

# Open up our message output directory to get our protobuf types
if b.binary_dir and os.path.isdir(b.binary_dir):
    shared_path = os.path.join(b.binary_dir, "shared")
    sys.path.append(shared_path)

    # Load all of the protobuf files as modules to use
    for dir_name, subdir, files in os.walk(shared_path):
        modules = pkgutil.iter_modules(path=[dir_name])
        for loader, module_name, ispkg in modules:
            if module_name.endswith("pb2"):

                # Load our protobuf module
                module = loader.find_module(module_name).load_module(module_name)

decoders = {}

# Now that we've imported them all get all the subclasses of protobuf message
for message in google.protobuf.message.Message.__subclasses__():

    # Work out our original protobuf type
    pb_type = ".".join(message.DESCRIPTOR.full_name.split(".")[1:])
    # Reverse to little endian
    pb_hash = bytes(reversed(xxhash.xxh64(pb_type, seed=0x4E55436C).digest()))
    decoders[pb_hash] = (pb_type, message)


def decode(path):

    # Now open the passed file
    with gzip.open(path, "rb") if path.endswith("nbz") or path.endswith(".gz") else open(path, "rb") as f:

        # NBS File Format:
        # 3 Bytes - NUClear radiation symbol header, useful for synchronisation when attaching to an existing stream
        # 4 Bytes - The remaining packet length i.e. 16 bytes + N payload bytes
        # 8 Bytes - 64bit timestamp in microseconds. Note: this is not necessarily a unix timestamp
        # 8 Bytes - 64bit bit hash of the message type
        # N bytes - The binary packet payload

        # While we can read a header
        while f.read(3).decode("utf-8") == "☢":

            # Read our size
            size = struct.unpack("<I", f.read(4))[0]

            # Read our payload
            payload = f.read(size)

            # Read our timestamp
            timestamp = struct.unpack("<Q", payload[:8])[0]

            # Read our hash
            type_hash = payload[8:16]

            # If we know how to parse this type, parse it
            if type_hash in decoders:
                # Yield a message
                try:
                    packet = (decoders[type_hash][0], timestamp, decoders[type_hash][1].FromString(payload[16:]))
                    yield packet
                except:
                    pass
