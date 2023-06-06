#!/usr/bin/env python3

import struct
from functools import cached_property

from .protobuf_types import MessageTypes


# This class holds all the derived properties that can be calculated once we have the raw data
class NBSPacket:
    def __init__(self, raw=None):
        self.raw = raw

    @cached_property
    def type_hash(self):
        return struct.unpack("<Q", self.raw[15 : 15 + 8])[0]

    @cached_property
    def type(self):
        try:
            return MessageTypes[self.type_hash]
        except KeyError:
            raise RuntimeError(
                f"Unknown message with type hash {self.type_hash}: perhaps a protobuf type in the nbs file has been renamed?"
            )

    @cached_property
    def emit_timestamp(self):
        return struct.unpack("<Q", self.raw[7 : 7 + 8])[0]

    @cached_property
    def index_timestamp(self):
        if hasattr(self.msg, "timestamp"):
            ts = self.msg.timestamp
            return int(ts.seconds * 1e9 + ts.nanos)
        else:
            return self.emit_timestamp * 1000

    @cached_property
    def subtype(self):
        if hasattr(self.msg, "id"):
            m_id = self.msg.id
            if isinstance(m_id, int):
                return m_id
        return 0

    @cached_property
    def msg(self):
        return self.type.type.FromString(self.raw_payload)

    @cached_property
    def raw_payload(self):
        return self.raw[23:]


class RawNBSPacket(NBSPacket):
    def __init__(self, type_hash, emit_timestamp, payload):
        # NBS File Format:
        # 3 Bytes - NUClear radiation symbol header, useful for synchronisation when attaching to an existing stream
        # 4 Bytes - The remaining packet length i.e. 16 bytes + N payload bytes
        # 8 Bytes - 64bit timestamp in microseconds. Note: this is not necessarily a unix timestamp
        # 8 Bytes - 64bit bit hash of the message type
        # N bytes - The binary packet payload

        raw = bytearray(b"\xE2\x98\xA2")
        raw.extend(struct.pack("<IQQ", 16 + len(payload), emit_timestamp, type_hash))
        raw.extend(payload)
        super(RawNBSPacket, self).__init__(bytes(raw))


class MappedNBSPacket(NBSPacket):
    def __init__(self, file_maps, fileno, offset, size, type_hash, subtype, index_timestamp):
        self._file_maps = file_maps
        self._fileno = fileno
        self._offset = offset
        self._size = size

        # Override the local properties with known information
        self.type_hash = type_hash
        self.subtype = subtype
        self.index_timestamp = index_timestamp

    @property
    def raw(self):
        return self._file_maps[self._fileno]["map"][self._offset : self._offset + self._size]
