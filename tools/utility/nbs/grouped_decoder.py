#!/usr/bin/env python3

import numpy as np
from numpy.lib.npyio import load

from .load_nbs import load_nbs
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
            _, _, timestamp, fileno, offset, size = v[self._pos[k]]
            data[k] = self._maps[fileno]["map"][offset : offset + size]

            ts = timestamp if ts is None else max(timestamp, ts)

        return ts, data

    # def step(steps=1, types=None):
    #     for k, idx in self._pos:
    #         pass
    #     # Move steps as many types as possible without double stepping one
    #     # If types is none check all types, otherwise only check types that are in the list
    #     pass


class GroupDecoder:
    def __init__(self, *paths, types=None, show_progress=False):
        indices, self._maps = load_nbs(*paths, types=types, show_progress=show_progress)

        types = np.unique(indices[["type_hash", "id"]])

        self._index = {(k[0], k[1]): indices[np.where(k == indices[["type_hash", "id"]])] for k in types}

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
