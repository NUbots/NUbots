#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2019 NUbots
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

import multiprocessing
import os

# Tell tensorflow to shut up
if "TF_CPP_MIN_LOG_LEVEL" not in os.environ:
    os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"

import numpy as np
import tensorflow as tf
from tqdm import tqdm

from utility.nbs import LinearDecoder

from .images import decode_image
from .images.fourcc import fourcc_to_string
from .images.video_recorder import Recorder

# Configure TensorFlow to use CPU only for multiprocessing compatibility
tf.config.set_visible_devices([], "GPU")

# The message types each --source option reads.
source_types = {
    "compressed": ["message.output.CompressedImage"],
    "raw": ["message.input.Image"],
    "both": ["message.output.CompressedImage", "message.input.Image"],
}


def register(command):
    command.description = "Decode an nbs file and extract any compressed jpeg files into jpeg files"

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to extract the videos from")
    command.add_argument("--output", "-o", default=os.getcwd(), help="The folder to create the videos in")
    command.add_argument("--quality", "-q", default="30M", help="The quality to encode the videos at")
    command.add_argument(
        "--encoder",
        "-e",
        default="h264_nvenc",
        choices=["libx264", "libopenh264", "h264_nvenc"],
        help="The encoder to use when encoding video",
    )
    command.add_argument(
        "--unix_ts",
        action="store_true",
        help="Write unix timestamps to timecode.txt instead of relative millisecond timestamps.",
    )
    command.add_argument(
        "--source",
        "-s",
        default="compressed",
        choices=sorted(source_types.keys()),
        help="Which images to make the videos from. Either compressed, raw or both.",
    )


def init_worker():
    """Initialize worker process with TensorFlow CPU-only configuration"""
    tf.config.set_visible_devices([], "GPU")


def process_frame(item):
    data = decode_image(item["data"], item["format"], item["dimensions"])

    return {
        "timestamp": item["timestamp"],
        "data": [
            {"image": d["image"].numpy(), "name": "{}{}".format(item["camera_name"], d["name"]), "fourcc": d["fourcc"]}
            for d in data
        ],
    }


def packetise_stream(decoder):
    for packet in decoder:
        # Get some useful info into a pickleable format
        yield {
            "camera_name": packet.msg.name,
            "timestamp": (packet.msg.timestamp.seconds, packet.msg.timestamp.nanos),
            "data": packet.msg.data,
            "format": packet.msg.format,
            "dimensions": (packet.msg.dimensions.x, packet.msg.dimensions.y),
        }


def run(files, output, encoder, quality, unix_ts, source, **kwargs):
    os.makedirs(output, exist_ok=True)

    recorders = {}

    with multiprocessing.Pool(multiprocessing.cpu_count(), initializer=init_worker) as pool:

        def record_frame(msg):
            for frame in msg["data"]:
                # A recorder is locked to one pixel format, so each format a camera provides needs its own video
                key = (frame["name"], frame["fourcc"])

                # If we haven't seen this camera before, make a new encoder for it
                if key not in recorders:
                    # Only tell the videos apart by format when we are actually reading more than one of them
                    name = (
                        "{}_{}".format(frame["name"], fourcc_to_string(frame["fourcc"]).strip())
                        if source == "both"
                        else frame["name"]
                    )
                    recorders[key] = Recorder(
                        os.path.join(output, "{}.mp4".format(name)),
                        tf.shape(frame["image"]),
                        frame["fourcc"],
                        encoder,
                        quality,
                        unix_ts=unix_ts,
                    )

                # Push the next packet
                recorders[key].encode({"timestamp": msg["timestamp"], "image": frame["image"]})

        results = []
        for msg in packetise_stream(
            tqdm(
                LinearDecoder(*files, types=source_types[source], show_progress=True),
                unit="packet",
                unit_scale=True,
                dynamic_ncols=True,
            )
        ):
            # Add a task to the pool to process
            results.append(pool.apply_async(process_frame, (msg,)))

            # Only buffer 1024 images for each cpu core to avoid running out of memory
            if len(results) > 1024 * multiprocessing.cpu_count():
                results[0].wait()

            # If the next one is ready process it
            if len(results) > 0 and results[0].ready():
                record_frame(results.pop(0).get())

        # Drain the remainder of the images
        while len(results) > 0:
            record_frame(results.pop(0).get())

    # Once we have finished reading all the frames, we need to finish writing the remainder of the buffer
    # And then use the timecode file we made to setup the VFR timecodes
    for k, r in recorders.items():
        r.close()
