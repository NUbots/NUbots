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

import os
from typing import Dict, Tuple

import numpy as np
from tqdm import tqdm

from utility.nbs import LinearDecoder
from .images import decode_image
from .images.video_recorder import Recorder


def register(command):
    command.description = "Decode an NBS file and export embedded images to MP4 video(s)."

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The NBS files to extract the videos from")
    command.add_argument("--output", "-o", default=os.getcwd(), help="The folder to create the videos in")
    command.add_argument("--quality", "-q", default="30M", help="The target video bitrate/quality (e.g. 18M, 30M)")
    command.add_argument(
        "--encoder",
        "-e",
        default="h264_nvenc",
        choices=["libx264", "h264_nvenc"],
        help="The encoder to use when encoding video",
    )


def _packetise_stream(decoder):
    """
    Yields packets that contain image data in a pickleable format.
    """
    for packet in decoder:
        if packet.type.name in ("message.output.CompressedImage", "message.input.Image"):
            yield {
                "camera_name": packet.msg.name,  # e.g. "top" / "bottom"
                "timestamp": (packet.msg.timestamp.seconds, packet.msg.timestamp.nanos),
                "data": packet.msg.data,
                "format": packet.msg.format,
            }


def _to_numpy_image(x):
    """
    Ensure the image is a NumPy array with shape (H, W) or (H, W, C).
    decode_image() typically returns TF tensors; convert safely.
    """
    # TF eager tensor path
    if hasattr(x, "numpy"):
        x = x.numpy()

    # Already numpy?
    if isinstance(x, np.ndarray):
        return x

    # Fallback: try to coerce bytes-like (e.g. encoded JPEG) into numpy so callers can decide
    if isinstance(x, (bytes, bytearray)):
        # Keep as bytes; decode_image should normally have handled decoding.
        # Returning None signals bad type for raw frame ingest.
        return None

    # Last resort: attempt numpy conversion
    try:
        return np.array(x)
    except Exception:
        return None


def run(files, output, encoder, quality, **kwargs):
    """
    Single-process, TensorFlow-free path:
      - iterate LinearDecoder
      - decode with decode_image()
      - open a Recorder per stream and write frames
    """
    os.makedirs(output, exist_ok=True)

    # name -> Recorder
    recorders: Dict[str, Recorder] = {}
    # name -> (H, W, C)
    sizes: Dict[str, Tuple[int, int, int]] = {}

    try:
        decoder = LinearDecoder(*files, show_progress=True)
        stream = _packetise_stream(
            tqdm(decoder, unit="packet", unit_scale=True, dynamic_ncols=True)
        )

        for msg in stream:
            # decode_image returns a list of dicts like:
            #   {"image": <tensor/ndarray>, "name": "_raw" or suffix, "fourcc": "BGR3"/"GRAY"/etc}
            frames = decode_image(msg["data"], msg["format"])

            # Build output per substream
            for d in frames:
                img = _to_numpy_image(d["image"])
                if img is None:
                    continue

                # Ensure 3D shape for recorder
                if img.ndim == 2:  # grayscale
                    img = img[..., None]
                if img.ndim != 3:
                    # Skip invalid frame shapes gracefully
                    continue

                h, w, c = img.shape
                name = f'{msg["camera_name"]}{d["name"]}'  # e.g. "top_raw" / "bottom_rect"
                fourcc = d.get("fourcc", "BGR3")

                if name not in recorders:
                    # Recorder expects a (H, W, C)-like shape; it formats as WxH internally.
                    recorders[name] = Recorder(
                        os.path.join(output, f"{name}.mp4"),
                        (h, w, c),
                        fourcc,
                        encoder,
                        quality,
                    )
                    sizes[name] = (h, w, c)

                # If the stream size changes mid-log, clamp to the first size (resize here if needed)
                if (h, w, c) != sizes[name]:
                    # Minimal safety: drop mismatched frame (could also resize here if desired)
                    # import cv2; img = cv2.resize(img, (sizes[name][1], sizes[name][0]))
                    continue

                recorders[name].encode({"timestamp": msg["timestamp"], "image": img})

    finally:
        # Always close any open recorders
        for r in recorders.values():
            try:
                r.close()
            except Exception:
                pass
