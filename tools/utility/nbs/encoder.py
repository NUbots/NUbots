#!/usr/bin/env python3

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
