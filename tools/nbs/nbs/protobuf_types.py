#!/usr/bin/env python3

import glob
import os
import pkgutil
import shutil
import struct
import sys
import tempfile

import google.protobuf.message

import b
import xxhash

if shutil.which("protoc") is None:
    sys.stderr.write("protoc is required to process nbs files\n")


class NBSType:
    def __init__(self, type_hash, name, type):
        self.type_hash = type_hash
        self.name = name
        self.type = type


MessageTypes = {}
with tempfile.TemporaryDirectory() as protobuf_path:

    # Find protobufs to build
    nuclear_proto_dir = os.path.join(b.nuclear_dir, "message", "proto")
    user_proto_dir = os.path.join(b.project_dir, "shared")

    proto_files = [
        *glob.glob(os.path.join(nuclear_proto_dir, "**", "*.proto"), recursive=True),
        *glob.glob(os.path.join(user_proto_dir, "**", "*.proto"), recursive=True),
    ]

    # Build the protocol buffers
    if (
        os.system(
            "protoc --python_out={protobuf_path} -I{nuclear_dir} -I{user_dir} {proto_files}".format(
                protobuf_path=protobuf_path,
                nuclear_dir=nuclear_proto_dir,
                user_dir=user_proto_dir,
                proto_files=" ".join(proto_files),
            )
        )
        != 0
    ):
        raise RuntimeError("Unable to build the protocol buffers for decoding")

    # Open up our message output directory to get our protobuf types
    sys.path.append(protobuf_path)

    # Load all of the protobuf files as modules to use
    for dir_name, subdir, files in os.walk(protobuf_path):
        modules = pkgutil.iter_modules(path=[dir_name])
        for loader, module_name, ispkg in modules:
            if module_name.endswith("pb2"):

                # Load our protobuf module
                module = loader.find_module(module_name).load_module(module_name)

    # Now that we've imported them all get all the subclasses of protobuf message
    for message in google.protobuf.message.Message.__subclasses__():

        # Unpack the big endian 64 bit integer
        (type_hash,) = struct.unpack(">Q", xxhash.xxh64(message.DESCRIPTOR.full_name, seed=0x4E55436C).digest())

        # Add the string to hash, and hash to string conversions
        MessageTypes[type_hash] = NBSType(type_hash, message.DESCRIPTOR.full_name, message)
        MessageTypes[message.DESCRIPTOR.full_name] = MessageTypes[type_hash]
