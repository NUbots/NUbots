#!/usr/bin/env python3

import json
import os
import types

import cv2
import numpy as np
import yaml
from tqdm import tqdm

from utility.nbs import LinearDecoder

from .images import decode_image, fourcc


def register(command):
    command.description = "Decode an nbs file and extract any compressed jpeg files into jpeg files"

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs files to extract the compressed images from"
    )
    command.add_argument("--output", "-o", nargs="?", default=os.getcwd(), help="The folder to extract the images into")


# Gets the image and debayers if needed, then saves the image
def process_save_image(packet, output):
    image_data = decode_image(packet.msg.data, packet.msg.format)

    img = image_data[0]["image"].numpy()
    fmt = image_data[0]["fourcc"]

    # Debayer if we need to
    if fmt == fourcc("BGGR"):
        img = cv2.cvtColor(img, cv2.COLOR_BayerBG2RGB_VNG)
    elif fmt == fourcc("RGGB"):
        img = cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB_VNG)
    elif fmt == fourcc("GRBG"):
        img = cv2.cvtColor(img, cv2.COLOR_BayerGR2RGB_VNG)
    elif fmt == fourcc("GBRG"):
        img = cv2.cvtColor(img, cv2.COLOR_BayerGB2RGB_VNG)

    # Convert to BGR if the image is in RGB since OpenCV uses BGR format
    if fmt == fourcc("RGB8") or fmt == fourcc("RGB3"):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # Save the image
    file_name = os.path.join(
        output,
        "{}_{:012d}.jpg".format(packet.msg.name, int(packet.msg.timestamp.seconds * 1e9 + packet.msg.timestamp.nanos)),
    )
    cv2.imwrite(file_name, img)


def process_metadata(packet, output):
    # Get the image width for calculations below
    width = packet.msg.dimensions.x
    # Get the projection
    projection = packet.msg.lens.projection
    if projection == 0:
        projection = "UNKNOWN"
    elif projection == 1:
        projection = "RECTILINEAR"
    elif projection == 2:
        projection = "EQUIDISTANT"
    elif projection == 3:
        projection = "EQUISOLID"
    # Get lens parameters
    centre = [packet.msg.lens.centre.x, packet.msg.lens.centre.y]
    centre = [x * width for x in centre]
    k = [packet.msg.lens.k.x / (width**2), packet.msg.lens.k.y / (width**4)]
    fov = packet.msg.lens.fov
    focal_length = packet.msg.lens.focal_length * width

    # Get the Hwc matrix
    Hcw = [
        [packet.msg.Hcw.x.x, packet.msg.Hcw.y.x, packet.msg.Hcw.z.x, packet.msg.Hcw.t.x],
        [packet.msg.Hcw.x.y, packet.msg.Hcw.y.y, packet.msg.Hcw.z.y, packet.msg.Hcw.t.y],
        [packet.msg.Hcw.x.z, packet.msg.Hcw.y.z, packet.msg.Hcw.z.z, packet.msg.Hcw.t.z],
        [packet.msg.Hcw.x.t, packet.msg.Hcw.y.t, packet.msg.Hcw.z.t, packet.msg.Hcw.t.t],
    ]

    Hoc = np.ndarray.tolist(np.linalg.inv(Hcw))

    camera_data = {
        "projection": projection,
        "focal_length": focal_length,
        "centre": centre,
        "k": k,
        "fov": fov,
        "Hoc": Hoc,
    }

    save_path = os.path.join(
        output,
        "{}_{:012d}.yaml".format(packet.msg.name, int(packet.msg.timestamp.seconds * 1e9 + packet.msg.timestamp.nanos)),
    )
    with open(save_path, "w") as file:
        yaml.safe_dump(camera_data, file, default_flow_style=True)


def run(files, output, **kwargs):
    os.makedirs(output, exist_ok=True)

    for packet in tqdm(
        LinearDecoder(*files, types=["message.output.CompressedImage"], show_progress=True),
        unit="packet",
        unit_scale=True,
        dynamic_ncols=True,
    ):
        # Get the image and save
        process_save_image(packet, output)
        # Get the metadata and save
        process_metadata(packet, output)
