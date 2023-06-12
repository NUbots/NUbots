#!/usr/bin/env python3

import fnmatch
import gzip
import struct
import sys

import xxhash
from tqdm import tqdm

from utility.nbs import Encoder, LinearDecoder


def register(command):
    command.description = "Rename one or more message types in an nbs stream."

    # Command arguments
    command.add_argument("input_file", help="The nbs file to rename the message types in")
    command.add_argument(
        "-m",
        "--map",
        dest="mappings",
        action="append",
        required=True,
        help="A mapping from the current message type name to the new one, specified as from:to",
    )
    command.add_argument("-o", "--output", required=True, help="The output file to store the updated nbs in")


def compute_type_hash(type_name):
    (type_hash,) = struct.unpack(">Q", xxhash.xxh64(type_name, seed=0x4E55436C).digest())
    return type_hash


def run(input_file, mappings, output, **kwargs):
    # If we don't have a output, choose a default output name
    if output is None:
        output = "renamed.nbs"

    # List of (from, to) message types
    mappings_parsed = [x.split(":") for x in mappings]

    # Validate the mapping arguments are in the correct format
    for i, mapping in enumerate(mappings_parsed):
        if len(mapping) < 2:
            print(f"Invalid mapping: {mappings[i]}")
            sys.exit(1)

        (_from, to) = mapping

        if _from is None or len(_from.strip()) == 0:
            print(f"Invalid mapping, missing from: {mappings[i]}")
            sys.exit(1)

        if to is None or len(to.strip()) == 0:
            print(f"Invalid mapping, missing to: {mappings[i]}")
            sys.exit(1)

    # Dict of { hash(from): hash(to) }
    mapping_hashes = {compute_type_hash(k): compute_type_hash(v) for (k, v) in mappings_parsed}

    # Dict of { hash(from): count of renames from type 'from' }
    renamed_type_counts = {}

    with Encoder(output) as out:
        for packet in tqdm(
            LinearDecoder(input_file, show_progress=True),
            unit="packet",
            unit_scale=True,
            dynamic_ncols=True,
            desc="Renaming type hashes in nbs and index",
        ):
            # Get the new hash if we have a mapping, otherwise use the current hash
            type_hash = mapping_hashes.get(packet.type_hash, packet.type_hash)

            # Increment count of renames for this type if the type is being renamed
            if type_hash != packet.type_hash:
                current_count = renamed_type_counts.get(packet.type_hash, 0)
                renamed_type_counts[packet.type_hash] = current_count + 1

            # Write the updated packet
            out.write_packet(packet, type_hash)

    if len(renamed_type_counts) > 0:
        print("Renamed the following types:")

        for _from, to in mappings_parsed:
            type_hash = compute_type_hash(_from)
            count = renamed_type_counts.get(type_hash, 0)
            instances = "instance" if count == 1 else "instances"
            print(f"  {_from} -> {to} ({count} {instances})")
    else:
        print("No types were renamed")
