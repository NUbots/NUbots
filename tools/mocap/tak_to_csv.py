#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2026 NUbots
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
"""Extract position/rotation channels from a Motive .tak recording to CSV.

Motive's .tak format is an undocumented, proprietary OLE Compound File. NaturalPoint
has never published a spec, and no public parser for it exists (checked GitHub; the
only known converter drives NaturalPoint's own NMotive.dll rather than parsing the
binary itself). What is implemented here was recovered by inspecting sample .tak
files byte-by-byte, and is NOT guaranteed to match every Motive version.

What we found inside the OLE container's "Channels.dat" stream:
  - It holds a flat sequence of animation-curve objects, each prefixed with a
    Pascal-style string naming its C++ class (e.g. "class MoCapCore::Vector3fChannel").
  - "Vector3fChannel" objects are position curves: one (frame_index, x, y, z) record
    per sample, in metres.
  - "QuaternionChannel" objects are rotation curves: one (frame_index, x, y, z, w)
    record per sample. Decoded values were verified to be unit quaternions
    (norm == 1.0) across every frame checked, which is strong evidence the field
    order and layout are correct.
  - "FloatChannel" / "cFloat32Channel" objects also exist (likely scale/aux data) but
    aren't decoded here since they're not needed for position/rotation.

Each channel object's header has a few fields (an internal ID, flags) whose exact
byte offset shifts depending on whether it's the first object of its class in the
stream (Motive appears to write extra schema bytes only once per class, similar to
classic MFC CArchive serialisation) -- we never fully pinned that down. Rather than
guess an offset and risk silently misreading the data, this script instead brute
-force searches for the (header_length, frame_count) pair that makes the record
size exactly consume every byte up to the start of the next channel. If that search
doesn't land on exactly one unambiguous answer, the channel is skipped and reported
rather than guessed at.

Caveats (read before trusting this for anything that drives a real robot):
  - There is no per-channel name or skeleton/bone hierarchy in this stream, so
    channels are only identified by their position in the file (0, 1, 2, ...) -- you
    cannot tell from this CSV alone which physical marker/bone a channel is. In the
    sample takes used to build this tool, human-body captures were raw, unlabelled
    marker clouds with no solved skeleton at all (skeletons.proto's use case);
    only a single tracked rigid body (e.g. a prop) had position+rotation together.
  - Frame indices come from each record's own embedded index, not an assumed
    sequential counter, so different channels can have different frame counts
    (dropped/occluded samples) -- do not assume rows line up across channels.
  - Validated against 24 sample .tak files (Motive ~2015, protocol-era unknown).
    Different Motive versions may use a different layout entirely.
"""

import csv
import re
import struct
from pathlib import Path

import olefile

# (number of float32 values per sample, column labels) for the channel classes we
# understand. Order matches the Simple_Rotation sample's verified unit-quaternion
# decode (x, y, z, w).
CHANNEL_SPECS = {
    "Vector3fChannel": ("position", ("x", "y", "z")),
    "QuaternionChannel": ("rotation", ("x", "y", "z", "w")),
}

# How far past the class name to search for the (header_length, frame_count) pair.
# Every channel observed in the sample data resolved within this window.
_HEADER_SEARCH_WINDOW = 200
_MAX_PLAUSIBLE_FRAMES = 500_000


def register(command):
    command.description = (
        "Extract position/rotation channels from a Motive .tak take file to CSV. "
        "The .tak format is undocumented and this parser is reverse-engineered from "
        "sample files -- see the module docstring for what is and isn't verified."
    )
    command.add_argument("file", help="The .tak file to convert.")
    command.add_argument(
        "-o",
        "--output",
        help="Output CSV path. Defaults to the input file name with a .csv extension.",
    )
    command.add_argument(
        "-l",
        "--list",
        dest="list_only",
        action="store_true",
        help="List the channels found (type, index, frame count) without writing a CSV.",
    )


def _iter_channel_blocks(data: bytes):
    """Split Channels.dat into (class_name, block_bytes) for each serialised object."""
    starts = [m.start() - 2 for m in re.finditer(rb"class ", data)]
    starts.append(len(data))
    for i in range(len(starts) - 1):
        block = data[starts[i] : starts[i + 1]]
        name_len = struct.unpack_from("<H", block, 0)[0]
        class_name = block[2 : 2 + name_len].decode("ascii").rsplit("::", 1)[-1]
        yield class_name, block


def _locate_frame_table(block: bytes, name_end: int, record_size: int):
    """Find the unique (header_length, frame_count) that exactly accounts for the
    rest of the block. Returns None if zero or more than one candidate is found."""
    matches = []
    search_end = min(name_end + _HEADER_SEARCH_WINDOW, len(block) - 4)
    for header_len in range(name_end, search_end):
        frame_count = struct.unpack_from("<i", block, header_len)[0]
        if 0 < frame_count <= _MAX_PLAUSIBLE_FRAMES:
            if header_len + 4 + frame_count * record_size == len(block):
                matches.append((header_len, frame_count))
    return matches[0] if len(matches) == 1 else None


def decode_channel(class_name: str, block: bytes):
    """Decode one channel block into (kind, labels, [(frame_index, values), ...]),
    or None if the class is unsupported or the frame table couldn't be pinned down
    unambiguously."""
    spec = CHANNEL_SPECS.get(class_name)
    if spec is None:
        return None
    kind, labels = spec
    record_size = 4 + 4 * len(labels)  # int32 frame index + N float32 values

    name_len = struct.unpack_from("<H", block, 0)[0]
    name_end = 2 + name_len

    located = _locate_frame_table(block, name_end, record_size)
    if located is None:
        return None
    header_len, frame_count = located

    offset = header_len + 4
    samples = []
    for _ in range(frame_count):
        frame_index = struct.unpack_from("<i", block, offset)[0]
        values = struct.unpack_from(f"<{len(labels)}f", block, offset + 4)
        samples.append((frame_index, values))
        offset += record_size

    return kind, labels, samples


def run(file, output, list_only, **kwargs):
    tak_path = Path(file)
    ole = olefile.OleFileIO(tak_path)
    try:
        data = ole.openstream("Channels.dat").read()
    finally:
        ole.close()

    channels = []
    skipped = []
    for index, (class_name, block) in enumerate(_iter_channel_blocks(data)):
        decoded = decode_channel(class_name, block)
        if decoded is None:
            skipped.append((index, class_name, len(block)))
            continue
        kind, labels, samples = decoded
        channels.append((index, class_name, kind, labels, samples))

    print(f"Found {len(channels)} decodable channel(s), skipped {len(skipped)}:")
    for index, class_name, kind, labels, samples in channels:
        print(f"  channel {index:3d}  {kind:8s}  {class_name:18s}  {len(samples)} frames")
    for index, class_name, block_len in skipped:
        reason = "not position/rotation" if class_name not in CHANNEL_SPECS else "no unambiguous frame table found"
        print(f"  channel {index:3d}  (skipped)  {class_name} ({block_len} bytes, {reason})")

    if list_only:
        return

    if not channels:
        print("Nothing decodable, not writing a CSV.")
        return

    output_path = Path(output) if output else tak_path.with_suffix(".csv")
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["channel", "class", "type", "frame", "x", "y", "z", "w"])
        for index, class_name, kind, labels, samples in channels:
            for frame_index, values in samples:
                row_values = list(values) + [""] * (4 - len(values))
                writer.writerow([index, class_name, kind, frame_index, *row_values])

    print(f"Wrote {output_path}")
