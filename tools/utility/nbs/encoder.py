#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2019 NUbots
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

import gzip
import struct

import xxhash
from numpy import pad

from utility.nbs.nbs_packet import RawNBSPacket


class Encoder:
    def __init__(self, path):
        # Open the file and index file
        self._file = gzip.open(path, "rb") if path.endswith("nbz") or path.endswith(".gz") else open(path, "wb")
        self._index = gzip.open("{}.idx".format(path), "wb")

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def write(self, timestamp, message):
        # Serialise the message and get the type hash
        payload = message.SerializeToString()
        (type_hash,) = struct.unpack(">Q", (xxhash.xxh64(message.DESCRIPTOR.full_name, seed=0x4E55436C).digest()))

        self.write_packet(RawNBSPacket(type_hash=type_hash, emit_timestamp=timestamp, payload=payload))

    def write_packet(self, packet, type_hash=None):
        # This lets us override the type hash if we are renaming protocol buffers
        type_hash = packet.type_hash if type_hash == None else type_hash

        # Get our current position in the file for the offset
        offset = self._file.tell()

        # Write out the data
        self._file.write(packet.raw)

        # Write the updated index file
        self._index.write(
            struct.pack("<QIQQI", type_hash, packet.subtype, packet.index_timestamp, offset, len(packet.raw))
        )

    def close(self):
        self._file.close()
        self._index.close()
