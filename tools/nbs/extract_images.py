#!/usr/bin/env python3

import json
import os
import types

import tensorflow as tf
from tqdm import tqdm

from utility.nbs import LinearDecoder

from .images import decode_image


def register(command):
    command.help = "Decode an nbs file and extract any compressed jpeg files into jpeg files"

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs files to extract the compressed images from"
    )
    command.add_argument("--output", "-o", nargs="?", default=os.getcwd(), help="The folder to extract the images into")


def run(files, output, **kwargs):

    os.makedirs(output, exist_ok=True)

    for packet in tqdm(
        LinearDecoder(*files, types=["message.output.CompressedImage"], show_progress=True),
        unit="packet",
        unit_scale=True,
        dynamic_ncols=True,
    ):
        with open(
            os.path.join(
                output,
                "{}_{:012d}.jpg".format(
                    packet.msg.name, int(packet.msg.timestamp.seconds * 1e9 + packet.msg.timestamp.nanos)
                ),
            ),
            "wb",
        ) as f:
            data = decode_image(packet.msg.data, packet.msg.format)
            data = tf.io.encode_jpeg(data[0]["image"])
            f.write(data)
        with open(
            os.path.join(
                output,
                "{}_{:012d}.json".format(
                    packet.msg.name, int(packet.msg.timestamp.seconds * 1e9 + packet.msg.timestamp.nanos)
                ),
            ),
            "w",
        ) as f:
            Hcw = packet.msg.Hcw
            json.dump(
                {
                    "Hcw": [
                        [Hcw.x.x, Hcw.x.y, Hcw.x.z, Hcw.x.t],
                        [Hcw.y.x, Hcw.y.y, Hcw.y.z, Hcw.y.t],
                        [Hcw.z.x, Hcw.z.y, Hcw.z.z, Hcw.z.t],
                        [Hcw.t.x, Hcw.t.y, Hcw.t.z, Hcw.t.t],
                    ],
                    "lens": {
                        "projection": packet.msg.lens.projection,
                        "focal_length": packet.msg.lens.focal_length,
                        "centre": [0, 0],
                        "fov": packet.msg.lens.fov,
                    },
                },
                f,
                indent=4,
                sort_keys=True,
            )
