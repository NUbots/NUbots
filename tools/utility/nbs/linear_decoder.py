#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2021 NUbots
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
