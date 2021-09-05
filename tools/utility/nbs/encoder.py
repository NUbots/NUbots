#!/usr/bin/env python3

import gzip
import struct

import xxhash


class Encoder:
    def __init__(self, path):
        self._file = gzip.open(path, "rb") if path.endswith("nbz") or path.endswith(".gz") else open(path, "wb")
        self._index = gzip.open("{}.idx".format(path), "wb")

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

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

        # Get the current position so we can add it to the index
        offset = self._file.tell()

        # If we have a id field, we can use them as the id for the index
        id = 0
        if hasattr(message, "id"):
            m_id = message.id
            if isinstance(m_id, int):
                id = m_id

        # If this type has a timestamp field, use it instead of the message timestamp for the index
        # and make it nanoseconds
        if hasattr(message, "timestamp"):
            ts = message.timestamp
            index_timestamp = int(ts.seconds * 1e9 + ts.nanos)
        else:
            index_timestamp = timestamp * 1000

        # Write the type_hash, id, timestamp, offset, and size into the index
        self._index.write(struct.pack("<QIQQI", pb_hash, id, index_timestamp, offset, len(data) + 23))

        # Header
        self._file.write(b"\xE2\x98\xA2")
        # Length timestamp and hash
        self._file.write(struct.pack("<IQQ", len(data) + 16, timestamp, pb_hash))
        # Payload
        self._file.write(data)

    def close(self):
        self._file.close()
        self._index.close()
