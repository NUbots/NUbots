#!/usr/bin/env python3

import gzip
import struct

import xxhash


class Encoder:
    def __init__(self, path):
        self._file = gzip.open(path, "rb") if path.endswith("nbz") or path.endswith(".gz") else open(path, "wb")

    def write(self, timestamp, message):
        # NBS File Format:
        # 3 Bytes - NUClear radiation symbol header, useful for synchronisation when attaching to an existing stream
        # 4 Bytes - The remaining packet length i.e. 16 bytes + N payload bytes
        # 8 Bytes - 64bit timestamp in microseconds. Note: this is not necessarily a unix timestamp
        # 8 Bytes - 64bit bit hash of the message type
        # N bytes - The binary packet payload

        # Serialise the message and get the type hash
        data = message.SerializeToString()
        (pb_hash,) = struct.unpack(">Q", (xxhash.xxh64(message.DESCRIPTOR.full_name, seed=0x4E55436C).digest()))

        # Header
        self._file.write(b"\xE2\x98\xA2")
        # Length timestamp and hash
        self._file.write(struct.pack("<IQQ", len(data) + 16, timestamp, pb_hash))
        # Payload
        self._file.write(data)

    def close(self):
        self._file.close()
