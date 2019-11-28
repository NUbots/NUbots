#!/usr/bin/env python3

import multiprocessing
import os

from tqdm import tqdm

import numpy as np
import tensorflow as tf

from .images import decode_image
from .images.video_recorder import Recorder
from .nbs import Decoder

gpus = tf.config.experimental.list_physical_devices("GPU")
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)
    tf.config.set_soft_device_placement(True)


def register(command):
    command.help = "Decode an nbs file and extract any compressed jpeg files into jpeg files"

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to extract the videos from")
    command.add_argument("--output", "-o", default=os.getcwd(), help="The folder to create the videos in")
    command.add_argument("--quality", "-q", default="30M", help="The quality to encode the videos at")
    command.add_argument(
        "--encoder",
        "-e",
        default="h264_nvenc",
        choices=["libx264", "h264_nvenc"],
        help="The encoder to use when encoding video",
    )


def process_frame(item):
    data = decode_image(item["data"], item["format"])

    return {
        "bytes_read": item["bytes_read"],
        "timestamp": item["timestamp"],
        "data": [
            {"image": d["image"].numpy(), "name": "{}{}".format(item["camera_name"], d["name"]), "fourcc": d["fourcc"]}
            for d in data
        ],
    }


def packetise_stream(decoder):

    bytes_read = 0
    for packet in decoder:

        # Check for compressed images
        if packet.type in ("message.output.CompressedImage", "message.input.Image"):

            # Work out how many bytes we have read to get to this message
            read = decoder.bytes_read() - bytes_read
            bytes_read = decoder.bytes_read()

            # Get some useful info into a pickleable format
            yield {
                "bytes_read": read,
                "camera_name": packet.msg.name,
                "timestamp": (packet.msg.timestamp.seconds, packet.msg.timestamp.nanos),
                "data": packet.msg.data,
                "format": packet.msg.format,
            }


def run(files, output, encoder, quality, **kwargs):

    os.makedirs(output, exist_ok=True)

    recorders = {}

    pool = multiprocessing.Pool(multiprocessing.cpu_count())

    decoder = Decoder(*files)
    with tqdm(total=len(decoder), unit="B", unit_scale=True, dynamic_ncols=True) as progress:

        def record_frame(msg):
            # Update the progress based on the image we are up to
            progress.update(msg["bytes_read"])

            # If we haven't seen this camera before, make a new encoder for it
            for frame in msg["data"]:
                if frame["name"] not in recorders:
                    recorders[frame["name"]] = Recorder(
                        os.path.join(output, "{}.mp4".format(frame["name"])),
                        tf.shape(frame["image"]),
                        frame["fourcc"],
                        encoder,
                        quality,
                    )

                # Push the next packet
                recorders[frame["name"]].encode({"timestamp": msg["timestamp"], "image": frame["image"]})

        results = []
        for msg in packetise_stream(decoder):
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
