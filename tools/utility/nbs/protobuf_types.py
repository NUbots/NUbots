#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2017 NUbots
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

import glob
import os
import pkgutil
import shutil
import struct
import sys
import tempfile

import google.protobuf.message
import xxhash

import b

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
        *glob.glob(os.path.join(user_proto_dir, "message", "**", "*.proto"), recursive=True),
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
