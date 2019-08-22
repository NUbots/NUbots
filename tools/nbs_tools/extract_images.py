#!/usr/bin/env python3

import os
import json
from . import decoder


def register(command):

    # Install help
    command.help = "Decode an nbs file and extract any compressed jpeg files into jpeg files"

    # Command arguments
    command.add_argument("in_file", metavar="in_file", help="The nbs file to extract the compressed images from")
    command.add_argument(
        "out_path", nargs="?", default=os.getcwd(), metavar="out_path", help="The folder to extract the images into"
    )


def run(in_file, out_path, **kwargs):

    output_path = os.path.join(out_path, os.path.basename(os.path.splitext(in_file)[0]))
    os.makedirs(output_path, exist_ok=True)

    for type_name, timestamp, msg, raw in decoder.decode(in_file):
        if type_name == "message.output.CompressedImage":
            with open(os.path.join(output_path, "{}_{:012d}.jpg".format(msg.name, timestamp)), "wb") as f:
                f.write(msg.data)
            with open(os.path.join(output_path, "{}_{:012d}.json".format(msg.name, timestamp)), "w") as f:
                json.dump(
                    {
                        "Hcw": [
                            [msg.Hcw.x.x, msg.Hcw.x.y, msg.Hcw.x.z, msg.Hcw.x.t],
                            [msg.Hcw.y.x, msg.Hcw.y.y, msg.Hcw.y.z, msg.Hcw.y.t],
                            [msg.Hcw.z.x, msg.Hcw.z.y, msg.Hcw.z.z, msg.Hcw.z.t],
                            [msg.Hcw.t.x, msg.Hcw.t.y, msg.Hcw.t.z, msg.Hcw.t.t],
                        ],
                        "lens": {
                            "projection": msg.lens.projection,
                            "focal_length": msg.lens.focal_length,
                            "centre": [0, 0],
                            "fov": msg.lens.fov.x,
                        },
                    },
                    f,
                )
