#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2025 NUbots
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

import cv2
import numpy as np
from tqdm import tqdm

from utility.nbs import Encoder, LinearDecoder

from .images import decode_image, fourcc

CHECKERBOARD = (8, 5)  # 9 by 6 squares
CHECKERBOARD_SQUARE_SIZE = 0.065 # in meters

def register(command):
    command.description = "Calibrate intrinsics from images in an nbs stream"

    # Command arguments
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to filter")

def get_image(packet):
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

    # Turn the image grey
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    return img

def run(files, **kwargs):
    # Set up the checkerboard points
    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= CHECKERBOARD_SQUARE_SIZE  # Scale the points to the real world size

    objpoints = []  # 3d points in real world
    imgpoints = []  # 2d points in image plane
    image_shape = None

    # Get list of all images
    image_dir = "recordings/images"
    image_files = [f for f in os.listdir(image_dir) if f.endswith(".jpg")]

    # Progress loop
    for image in tqdm(image_files, desc="Processing images", unit="img"):
        img_path = os.path.join(image_dir, image)
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

        ret, corners = cv2.findChessboardCorners(img, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret:
            objpoints.append(objp)
            corners_refined = cv2.cornerSubPix(img, corners, (11,11), (-1,-1),
                                            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
            imgpoints.append(corners_refined)
            image_shape = img.shape[::-1]

    N_OK = len(objpoints)
    K = np.zeros((3, 3))  # Camera matrix
    D = np.zeros((4, 1))  # Distortion coefficients
    rvecs = [np.zeros((1,1,3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1,1,3), dtype=np.float64) for i in range(N_OK)]

    rms, _, _, _, _ = cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        image_shape,
        K,
        D,
        rvecs,
        tvecs,
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC,
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    )

    print("RMS error:", rms)
    print("Camera matrix (K):\n", K)
    print("Distortion coefficients (D):\n", D)

    # Get focal length as the normalised focal length: focal length in pixels / image width
    focal_length = K[0, 0] / image_shape[0]

    # Normalised image centre offset: pixels from centre to optical axis / image width
    centre_offset = np.array([K[0, 2], K[1, 2]]) / image_shape[0]
    centre_offset = centre_offset - 0.5  # Normalise to [-0.5, 0.5]
    centre_offset = centre_offset.tolist()

    print("Focal length (normalised):", focal_length)
    print("Centre offset (normalised):", centre_offset)
