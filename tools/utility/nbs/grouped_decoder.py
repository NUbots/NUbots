#!/usr/bin/env python3

import numpy as np
import xxhash
from numpy.lib.npyio import load

from .load_nbs import load_nbs
from .nbs_packet import MappedNBSPacket
from .protobuf_types import MessageTypes


class GroupDecoderReader:
    def __init__(self, index, maps):
        self._index = index
        self._maps = maps
        self._pos = {t: 0 for t in index.keys()}

    def seek(self, timestamp):
        # Seek to the given timestamp and return the first packet that is <= timestamp - 1
        for k, v in self._index.items():
            self._pos[k] = max(0, np.searchsorted(v["timestamp"], timestamp) - 1)
        return self.current()

    def current(self):
        data = {}
        ts = None
        for k, v in self._index.items():
            type_hash, subtype, timestamp, fileno, offset, size = v[self._pos[k]]

            data[k] = MappedNBSPacket(self._maps, fileno, offset, size, type_hash, subtype, timestamp)

            ts = timestamp if ts is None else max(timestamp, ts)
        # Returns a timestamp with an array of NBSPackets
        return ts, data

    def step(self, name, current_time, steps=1):
        # Return column containing timestamps for given camera.
        # Convert String camera name to hash. ThermalCam = "ABC" etc.
        hashed_name = int.from_bytes(xxhash.xxh32(name, seed=0x4E55436C).digest(), byteorder="big")
        msg_type = MessageTypes["message.output.CompressedImage"].type_hash
        timestamps = [time_list[2] for time_list in self._index[(msg_type, hashed_name)]]
        # Increment by 1 frame if flag == true.

        index = max(0, np.searchsorted(timestamps, current_time) + steps)

        self.seek(timestamps[index])
        return timestamps[index]


class GroupDecoder:
    def __init__(self, *paths, types=None, show_progress=False):
        indices, self._maps = load_nbs(*paths, types=types, show_progress=show_progress)

        types = np.unique(indices[["type_hash", "subtype"]])

        self._index = {(k[0], k[1]): indices[np.where(k == indices[["type_hash", "subtype"]])] for k in types}

    def type_reader(self, type):
        if isinstance(type, tuple):
            return GroupDecoderReader({type: self._index[type]}, self._maps)
        else:
            return GroupDecoderReader({(type, 0): self._index[(type, 0)]}, self._maps)

    def reader(self):
        return GroupDecoderReader(self._index, self._maps)

    # Return the earliest and latest timestamp
    def extents(self):
        return (
            min(v["timestamp"][0] for v in self._index.values()),
            max(v["timestamp"][-1] for v in self._index.values()),
        )
