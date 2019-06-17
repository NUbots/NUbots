#!/usr/bin/env python3

import xxhash
import struct
import sys
import os
import re
import io
import numpy as np
import pkgutil
import google.protobuf.message
from tqdm import tqdm
from google.protobuf.json_format import MessageToJson

# Open up our message output directory to get our protobuf types
shared_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "shared")

sys.path.append(shared_path)

# Load all of the protobuf files as modules to use
for dir_name, subdir, files in os.walk(shared_path):
    modules = pkgutil.iter_modules(path=[dir_name])
    for loader, module_name, ispkg in modules:
        if module_name.endswith("pb2"):

            # Load our protobuf module
            module = loader.find_module(module_name).load_module(module_name)

parsers = {}

# Now that we've imported them all get all the subclasses of protobuf message
for message in google.protobuf.message.Message.__subclasses__():

    # Work out our original protobuf type
    pb_type = message.DESCRIPTOR.full_name.split(".")[1:]
    pb_type = ".".join(pb_type)

    # Reverse to little endian
    pb_hash = bytes(reversed(xxhash.xxh64(pb_type, seed=0x4E55436C).digest()))

    parsers[pb_hash] = (pb_type, message)


def decode(file):

    # Now open the passed file
    with gzip.open(file, "rb") if file.endswith("nbz") or file.endswith(
        ".gz"
    ) else open(file, "rb") as f:

        # Open another file
        with open(file.replace("nbs", "json"), "w") as of:

            with tqdm(
                total=os.path.getsize(file), unit="b", unit_scale=True
            ) as progress:

                # While we can read a header
                while len(f.read(3)) == 3:

                    # Read our size
                    size = struct.unpack("<I", f.read(4))[0]

                    # Read our payload
                    payload = f.read(size)

                    # Update our progress bar
                    progress.update(7 + size)

                    # Read our timestamp
                    timestamp = struct.unpack("<Q", payload[:8])[0]

                    # Read our hash
                    type_hash = payload[8:16]

                    # If we know how to parse this type, parse it
                    if type_hash in parsers:
                        msg = parsers[type_hash][1].FromString(payload[16:])
                        yield msg


# An individual h264 stream
class H264StreamIO(io.BufferedIOBase):
    def __init__(self, streams, i):
        self.streams = streams
        self.index = i

    def readable(self):
        return True

    def seekable(self):
        return False

    def _read1(self, n=None):
        while not self.streams.buffers[self.index]:
            try:
                self.streams.load()
            except StopIteration:
                break

        # Get the data and return it
        data = self.streams.buffers[self.index][:n]
        self.streams.buffers[self.index] = self.streams.buffers[self.index][len(data) :]
        return data

    def read(self, n=None):
        output = []
        l = []
        if n is None or n < 0:
            while True:
                m = self._read1()
                if not m:
                    break
                l.append(m)
        else:
            while n > 0:
                m = self._read1(n)
                if not m:
                    break
                n -= len(m)
                l.append(m)
        return b"".join(l)

    def seek(self, offset, whence):
        return -1


# The buffered collection of h264 streams
class H264Streams:
    def __init__(self, file):
        self.iter = decode(file)
        self.buffers = [b""] * 4

    def load(self):
        self.buffers = [a + b for a, b in zip(self.buffers, next(self.iter))]

    def stream(self, i):
        return H264StreamIO(self, i)


if __name__ == "__main__":

    input_file = sys.argv[1]

    # Open our output file
    output_file = input_file.replace(".nbs", ".mp4")
    output_video = av.open(output_file, mode="w")
    stream = output_video.add_stream("h264", rate=30)
    stream.width = 1280
    stream.height = 1024
    stream.pix_fmt = "yuv420p"
    stream.options = {"preset": "ultrafast", "tune": "film", "crf": "0"}

    streams = H264Streams(input_file)

    containers = [av.open(streams.stream(i)) for i in range(4)]
    decoders = [container.decode(video=0) for container in containers]

    for frames in zip(*decoders):
        tl, tr, bl, br = [
            np.frombuffer(f.planes[0], np.uint8).reshape(f.height, f.width)
            for f in frames
        ]

        data = np.zeros((stream.height, stream.width), dtype=np.uint8)

        data[0::2, 0::2] = tl
        data[0::2, 1::2] = tr
        data[1::2, 0::2] = bl
        data[1::2, 1::2] = br

        # Because OpenCV names things backwards (format is named relative to (1, 1))
        rgb_frame = cv2.cvtColor(data, cv2.COLOR_BAYER_BG2RGB)
        frame = av.VideoFrame.from_ndarray(rgb_frame, format="rgb24")
        for packet in stream.encode(frame):
            output_video.mux(packet)

    # Flush stream
    for packet in stream.encode():
        output_video.mux(packet)

    # Close the file
    output_video.close()
