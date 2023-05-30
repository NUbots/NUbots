#!/usr/bin/env python3

import numpy as np

from .load_nbs import load_nbs
from .nbs_packet import MappedNBSPacket
from .protobuf_types import MessageTypes


class LinearDecoder:
    def __init__(self, *paths, types=None, show_progress=False):
        self.index, self._maps = load_nbs(*paths, types=types, show_progress=show_progress)

        self._total_bytes = np.sum(self.index["size"])
        self._total_messages = len(self.index)

    def __len__(self):
        return self._total_messages

    def __getitem__(self, key):
        (type_hash, subtype, index_timestamp, fileno, offset, size) = self.index[key]

        return MappedNBSPacket(self._maps, fileno, offset, size, type_hash, subtype, index_timestamp)

    def total_bytes(self):
        return self._total_bytes

    def __iter__(self):
        for (
            type_hash,
            subtype,
            index_timestamp,
            fileno,
            offset,
            size,
        ) in self.index:
            if offset + size <= len(self._maps[fileno]["map"]):
                yield MappedNBSPacket(self._maps, fileno, offset, size, type_hash, subtype, index_timestamp)
