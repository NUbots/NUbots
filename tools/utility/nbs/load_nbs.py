#!/usr/bin/env python3

import gzip
import mmap
import os
import struct

import numpy as np
from tqdm import tqdm

from .protobuf_types import MessageTypes


def _build_index(mem, show_progress=False):
    # Returns a list of tuples in the form (type_hash, id, timestamp, offset)
    index = []

    progress = (
        tqdm(total=mem.size(), unit="B", unit_scale=True, dynamic_ncols=True, leave=False, desc="Generating")
        if show_progress
        else None
    )
    offset = 0
    while offset >= 0:

        # Find the next header and then skip past it
        offset = mem.find(b"\xE2\x98\xA2", offset)

        if offset != -1:
            # Skip over the header
            offset = offset + 3

            # Read our size header information
            size, timestamp, type_hash = struct.unpack("<IQQ", mem[offset : offset + 20])

            # If we don't have enough memory left for this to be a packet, skip
            if len(mem) - offset - size + 16 < 0:
                offset = -1
                continue

            # Skip to the payload
            offset += 20

            # If this hash matches to a type with an id, load that as the id
            id = 0
            if type_hash in MessageTypes:
                t = MessageTypes[type_hash]

                # If this type has a timestamp field, use it instead of the message timestamp
                # and make it nanoseconds
                if hasattr(t.type, "timestamp"):
                    ts = t.type.FromString(mem[offset : offset + size - 16]).timestamp
                    timestamp = int(ts.seconds * 1e9 + ts.nanos)
                else:
                    timestamp *= 1000

                # If we have a id field, we can use them as the id
                if hasattr(t.type, "id"):
                    m_id = t.type.FromString(mem[offset : offset + size - 16]).id
                    if isinstance(m_id, int):
                        id = m_id

            index.append((type_hash, id, timestamp, offset - 23, size + 7))

            # Skip ahead over the rest of the packet
            offset += size - 16

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
    # Open and memory map all of the files
    maps = []
    for p in paths:
        # Open the file and map the contents
        f = open(p, "rb")
        m = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        maps.append({"path": p, "file": f, "map": m})

    # Go through each of the files and build up the type indexes
    if show_progress:
        print("Loading indexes")
    indexes = []
    progress = tqdm(maps, unit="files", leave=False, dynamic_ncols=True) if show_progress else maps
    for fileno, v in enumerate(progress):
        if show_progress:
            progress.set_description(os.path.basename(v["path"]))

        # Find all the packets in this memory mapped file
        mem = v["map"]
        path = v["path"]

        # Where we will cache the index file
        index_path = "{}.idx".format(path)

        # If an index file has not been built yet, build one
        if not os.path.isfile(index_path):
            # Build the index for this memory
            index = _build_index(mem)

            with gzip.open(index_path, "wb") as idx:
                for v in index:
                    # Write the type_hash, id, timestamp and offset
                    idx.write(struct.pack("<QIQQI", v[0], v[1], v[2], v[3], v[4]))

        # Load in the index file
        with gzip.open(index_path, "rb") as idx:
            try:
                read_index = np.frombuffer(
                    idx.read(),
                    dtype=[
                        ("type_hash", "u8"),
                        ("id", "u4"),
                        ("timestamp", "u8"),
                        ("offset", "u8"),
                        ("size", "u4"),
                    ],
                )

                index = np.empty(
                    read_index.size,
                    dtype=[
                        ("type_hash", "u8"),
                        ("id", "u4"),
                        ("timestamp", "u8"),
                        ("fileno", "u4"),
                        ("offset", "u8"),
                        ("size", "u4"),
                    ],
                )

                index[["type_hash", "id", "timestamp", "offset", "size"]] = read_index
                index["timestamp"] = index["timestamp"] * 1e-3  # Convert from nano to micro seconds
                index["fileno"] = fileno
                index["size"] += np.full(index["size"].size, -23, dtype=("u4"))

                index["offset"] += np.full(index["offset"].size, 23, dtype=("u8"))

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
                elements.append(indexes[["type_hash", "id"]] == [MessageTypes[t[0]].type_hash, t[1]])
            else:
                elements.append(indexes["type_hash"] == MessageTypes[t].type_hash)
        indexes = indexes[np.any(np.stack(elements, axis=-1), axis=-1)]

    if show_progress:
        print("Sorting indices")
    return np.sort(indexes, order=["timestamp"]), maps
