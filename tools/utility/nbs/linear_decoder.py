#!/usr/bin/env python3

import struct

import numpy as np

from .load_nbs import load_nbs
from .nbs_packet import NBSPacket
from .protobuf_types import MessageTypes


class LinearDecoder:
    def __init__(self, *paths, types=None, show_progress=False):
        self.index, self._maps = load_nbs(*paths, types=types, show_progress=show_progress)

        self._total_bytes = np.sum(self.index["size"])
        self._total_messages = len(self.index)

    def __len__(self):
        return self._total_messages

    def total_bytes(self):
        return self._total_bytes

    def __iter__(self):
        for type_hash, _, index_timestamp, fileno, offset, size in self.index:
            type = MessageTypes[type_hash].name
            raw = self._maps[fileno]["map"][offset : offset + size]
            (emit_timestamp,) = struct.unpack("<Q", self._maps[fileno]["map"][int(offset - 16) : int(offset - 8)])

            yield NBSPacket(type, index_timestamp, emit_timestamp, MessageTypes[type_hash].type.FromString(raw), raw)
