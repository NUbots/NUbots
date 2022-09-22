#!/usr/bin/env python3

import json
import os
import types

import cv2
import numpy as np
import tensorflow as tf
import yaml
from tqdm import tqdm
from wand.image import Image

from utility.nbs import LinearDecoder

from .images import decode_image, fourcc


def register(command):
    command.help = "Decode an nbs file and extract any compressed jpeg files into jpeg files"

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs files to extract the compressed images from"
    )
    command.add_argument("--output", "-o", nargs="?", default=os.getcwd(), help="The folder to extract the images into")


# Gets the image and debayers if needed, then saves the image
def process_image(packet, output):
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

    # Save the image
    file_name = os.path.join(
        output,
        "{}_{:012d}.jpg".format(packet.msg.name, int(packet.msg.timestamp.seconds * 1e9 + packet.msg.timestamp.nanos)),
    )
    cv2.imwrite(file_name, img)
    # Hacky! Image saved above has strange behaviour
    Image(filename=file_name).convert("jpg").save(filename=file_name)


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
    Hco = packet.msg.Hcw
    rCOc = np.array([-Hco.x.t, -Hco.y.t, -Hco.z.t])
    Roc = np.array([[Hco.x.x, Hco.y.x, Hco.z.x], [Hco.x.y, Hco.y.y, Hco.z.y], [Hco.x.z, Hco.y.z, Hco.z.z]])
    rCOo = np.matmul(Roc, rCOc).tolist()

    Hoc = [
        [Roc[0][0].item(), Roc[1][0].item(), Roc[2][0].item(), rCOo[0]],
        [Roc[0][1].item(), Roc[1][1].item(), Roc[2][1].item(), rCOo[1]],
        [Roc[0][2].item(), Roc[1][2].item(), Roc[2][2].item(), rCOo[2]],
        [0, 0, 0, 1],
    ]

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
        process_image(packet, output)
        # Get the metadata and save
        process_metadata(packet, output)
