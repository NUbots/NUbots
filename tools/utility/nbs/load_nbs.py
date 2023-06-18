#!/usr/bin/env python3

import gzip
import mmap
import os
import struct

import numpy as np
from tqdm import tqdm

from .nbs_packet import NBSPacket
from .protobuf_types import MessageTypes


def _build_index(mem, show_progress):
    """Returns a list of tuples in the form (type_hash, subtype, timestamp, offset of packet, size of packet)"""

    index = []

    progress = (
        tqdm(total=mem.size(), unit="B", unit_scale=True, dynamic_ncols=True, leave=False, desc="Generating")
        if show_progress
        else None
    )
    offset = 0
    while offset >= 0:
        # Find the next header
        offset = mem.find(b"\xE2\x98\xA2", offset)

        if offset != -1:
            # Get the size of this packet
            size = struct.unpack("<I", mem[offset + 3 : offset + 7])[0]

            # If we don't have enough memory left for this to be a packet, skip
            if offset + 7 + size >= len(mem):
                offset = -1
                continue

            # Load this data into an NBS packet structure
            packet = NBSPacket(raw=mem[offset : offset + 7 + size])

            index.append((packet.type_hash, packet.subtype, packet.index_timestamp, offset, size + 7))

            # Skip the offset to the next packet
            offset += 7 + size

            if show_progress:
                # Update the progress bar
                progress.n = offset
                progress.update(0)

        else:
            if show_progress:
                progress.n = len(mem)

    if show_progress:
        progress.close()
    return index


def load_nbs(*paths, types=None, show_progress=False):
    """Returns tuple (indexes, maps) where the offset and size of each entry in indexes is of the entire nbs packet."""

    index_dtype = [
        ("type_hash", "u8"),
        ("subtype", "u4"),
        ("timestamp", "u8"),
        ("fileno", "u4"),
        ("offset", "u8"),
        ("size", "u4"),
    ]

    # Open and memory map all of the files
    maps = []
    for p in paths:
        f = open(p, "rb")
        maps.append(
            {
                "path": p,
                "file": f,
                "map": mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ) if os.path.getsize(p) != 0 else None,
            }
        )

    # Return empty indexes if all files empty
    if all([m["map"] is None for m in maps]):
        return np.array([], dtype=index_dtype), maps

    # Go through each of the files and build up the type indexes
    if show_progress:
        print("Loading indexes")
    indexes = []
    progress = tqdm(maps, unit="files", leave=False, dynamic_ncols=True) if show_progress else maps
    for fileno, v in enumerate(progress):
        if show_progress:
            progress.set_description(os.path.basename(v["path"]))

        # Skip empty nbs files
        if v["map"] is not None:
            # Find all the packets in this memory mapped file
            mem = v["map"]
            path = v["path"]

            # Where we will cache the index file
            index_path = "{}.idx".format(path)

            # If an index file has not been built yet, build one
            if not os.path.isfile(index_path):
                # Build the index for this memory
                index = _build_index(mem, show_progress)

                with gzip.open(index_path, "wb") as idx:
                    for v in index:
                        # Write the type_hash, subtype, timestamp, offset (of the entire packet)
                        # and size (of the entire packet)
                        idx.write(struct.pack("<QIQQI", v[0], v[1], v[2], v[3], v[4]))

            # Load in the index file
            with gzip.open(index_path, "rb") as idx:
                try:
                    read_index = np.frombuffer(
                        idx.read(),
                        dtype=[
                            ("type_hash", "u8"),
                            ("subtype", "u4"),
                            ("timestamp", "u8"),
                            ("offset", "u8"),
                            ("size", "u4"),
                        ],
                    )

                    index = np.empty(read_index.size, dtype=index_dtype)
                    index[["type_hash", "subtype", "timestamp", "offset", "size"]] = read_index
                    index["fileno"] = fileno

                    indexes.append(index)

                except Exception as e:
                    print(e)
    if show_progress:
        progress.close()

    indexes = np.concatenate(indexes)

    if types:
        elements = []
        for t in types:
            if isinstance(t, tuple):
                elements.append(indexes[["type_hash", "subtype"]] == [MessageTypes[t[0]].type_hash, t[1]])
            else:
                elements.append(indexes["type_hash"] == MessageTypes[t].type_hash)
        indexes = indexes[np.any(np.stack(elements, axis=-1), axis=-1)]

    if show_progress:
        print("Sorting indices")

    # Sort packets by timestamp, then file offset.
    return np.sort(indexes, order=["timestamp", "offset"]), maps
