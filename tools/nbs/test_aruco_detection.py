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

import cv2
import numpy as np
from tqdm import tqdm

from utility.nbs import LinearDecoder

from .images import decode_image, fourcc

# Hardcoded camera calibration parameters
# RMS error: 0.8892583692230335
CAMERA_MATRIX = np.array([
    [4.43617837e+02, -7.74973607e-02, 6.30864741e+02],
    [0.00000000e+00, 4.42779739e+02, 4.86466115e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float64)

DISTORTION_COEFFICIENTS = np.array([
    [-0.0467533],
    [0.0013722],
    [-0.00462182],
    [0.00085256]
], dtype=np.float64)

# Normalized parameters
FOCAL_LENGTH_NORMALIZED = 0.3465764353790301
CENTRE_OFFSET_NORMALIZED = np.array([-0.00713692086609552, -0.11994834773548346])

# Coordinate system transformation matrix
# OpenCV convention: x right, y down, z forward
# Desired convention: x forward, y left, z up
# This transforms from OpenCV to desired convention
COORDINATE_TRANSFORM = np.array([
    [0, 0, 1],   # x_opencv = z_desired
    [-1, 0, 0],  # y_opencv = -x_desired
    [0, -1, 0]   # z_opencv = -y_desired
], dtype=np.float64)


def register(command):
    command.description = "Test ArUco marker detection and pose estimation from NBS files"

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs files containing compressed images"
    )
    command.add_argument("--aruco-dict", default="DICT_6X6_250", help="ArUco dictionary to use")
    command.add_argument("--aruco-id", type=int, default=0, help="ID of the ArUco marker to track (optional filter)")
    command.add_argument("--aruco-size", type=float, required=True, help="Size of ArUco marker in meters")
    command.add_argument("--camera-name", required=True, help="Name of camera to process")
    command.add_argument("--max-images", type=int, default=50, help="Maximum number of images to process")
    command.add_argument("--output-dir", default="aruco_detection_output", help="Output directory for saved images")


def transform_coordinate_system(translation, rotation_matrix):
    """
    Transform pose from OpenCV coordinate system to desired convention
    OpenCV: x right, y down, z forward
    Desired: x forward, y left, z up
    """
    # Transform translation vector
    transformed_translation = COORDINATE_TRANSFORM @ translation

    # Transform rotation matrix
    transformed_rotation = COORDINATE_TRANSFORM @ rotation_matrix @ COORDINATE_TRANSFORM.T

    return transformed_translation, transformed_rotation


def undistort_image(img):
    """Undistort image using hardcoded camera parameters"""
    h, w = img.shape[:2]

    # Create new camera matrix for undistorted image
    new_camera_matrix = cv2.getOptimalNewCameraMatrix(
        CAMERA_MATRIX, DISTORTION_COEFFICIENTS, (w, h), 1, (w, h)
    )[0]

    # Create undistortion maps
    map1, map2 = cv2.initUndistortRectifyMap(
        CAMERA_MATRIX, DISTORTION_COEFFICIENTS, None, new_camera_matrix, (w, h), cv2.CV_16SC2
    )

    # Undistort image
    undistorted = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

    return undistorted, new_camera_matrix


def get_aruco_detector_params():
    """Get ArUco detector parameters optimized for fisheye cameras"""
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.03
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.05
    params.minCornerDistanceRate = 0.05
    params.minDistanceToBorder = 3
    return params


def detect_and_draw_aruco_markers(image, aruco_dict, marker_size, target_id=None):
    """Detect ArUco markers and draw them with pose axes"""
    # Undistort image for better ArUco detection
    undistorted_img, undistorted_camera_matrix = undistort_image(image)

    # Detect markers in undistorted image
    detector = cv2.aruco.ArucoDetector(aruco_dict, get_aruco_detector_params())
    corners, ids, _ = detector.detectMarkers(undistorted_img)

    # Create output image for visualization
    output_img = undistorted_img.copy()
    detections = []

    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(output_img, corners, ids)

        # Estimate pose for each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, undistorted_camera_matrix, np.zeros(4)
        )

        for i, marker_id in enumerate(ids.flatten()):
            # Skip if we're filtering for a specific ID and this isn't it
            if target_id is not None and marker_id != target_id:
                continue

            rvec = rvecs[i]
            tvec = tvecs[i]

            # Draw pose axes (using original OpenCV coordinates for visualization)
            cv2.drawFrameAxes(output_img, undistorted_camera_matrix, np.zeros(4),
                            rvec, tvec, marker_size * 0.5)

            # Convert to transform matrix
            R, _ = cv2.Rodrigues(rvec)
            H_marker = np.eye(4)
            H_marker[:3, :3] = R
            H_marker[:3, 3] = tvec.flatten()

            # Transform to desired coordinate system
            transformed_translation, transformed_rotation = transform_coordinate_system(
                tvec.flatten(), R
            )

            # Store detection info with transformed coordinates
            detection = {
                'id': int(marker_id),
                'pose_matrix': H_marker,
                'translation': transformed_translation,  # Transformed coordinates
                'rotation_matrix': transformed_rotation,  # Transformed rotation
                'rotation_vector': rvec.flatten(),  # Original OpenCV rotation vector
                'corners': corners[i]
            }
            detections.append(detection)

            # Add text with transformed pose info
            text_pos = tuple(map(int, corners[i][0][0]))
            cv2.putText(output_img, f"ID:{marker_id}",
                       (text_pos[0], text_pos[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(output_img, f"T:[{transformed_translation[0]:.2f},{transformed_translation[1]:.2f},{transformed_translation[2]:.2f}]",
                       (text_pos[0], text_pos[1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

    return output_img, detections, undistorted_img


def run(files, aruco_dict, aruco_id, aruco_size, camera_name, max_images, output_dir, **kwargs):
    # Get ArUco dictionary
    aruco_dict_id = getattr(cv2.aruco, aruco_dict)
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)

    print(f"Testing ArUco detection:")
    print(f"  Dictionary: {aruco_dict}")
    print(f"  Target ID: {aruco_id} (will detect all if not found)")
    print(f"  Marker size: {aruco_size}m")
    print(f"  Camera: {camera_name}")
    print(f"  Max images: {max_images}")
    print(f"  Output directory: {output_dir}")
    print(f"  Using hardcoded camera calibration parameters")
    print(f"  Coordinate system: x forward, y left, z up")

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Collect images
    print("\nLoading images from NBS files...")
    images = []

    for packet in tqdm(LinearDecoder(*files, types=["message.output.CompressedImage"])):
        if packet.type.name == "message.output.CompressedImage":
            if packet.msg.name == camera_name:
                images.append(packet)
                if len(images) >= max_images:
                    break

    print(f"Found {len(images)} images")

    if len(images) == 0:
        print(f"No images found for camera '{camera_name}'")
        return

    # Process images
    total_detections = 0
    target_detections = 0

    print("\nProcessing images and detecting ArUco markers...")

    for i, img_packet in enumerate(tqdm(images, desc="Processing images")):
        try:
            # Decode image
            image_data = decode_image(img_packet.msg.data, img_packet.msg.format)
            img = image_data[0]["image"].numpy()
            fmt = image_data[0]["fourcc"]

            # Debayer if needed
            if fmt == fourcc("BGGR"):
                img = cv2.cvtColor(img, cv2.COLOR_BayerBG2RGB_VNG)
            elif fmt == fourcc("RGGB"):
                img = cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB_VNG)
            elif fmt == fourcc("GRBG"):
                img = cv2.cvtColor(img, cv2.COLOR_BayerGR2RGB_VNG)
            elif fmt == fourcc("GBRG"):
                img = cv2.cvtColor(img, cv2.COLOR_BayerGB2RGB_VNG)

            # Convert to BGR for OpenCV
            if fmt == fourcc("RGB8") or fmt == fourcc("RGB3"):
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            # Detect ArUco markers
            output_img, detections, undistorted_img = detect_and_draw_aruco_markers(
                img, aruco_dict, aruco_size, None  # Detect all markers
            )

            # Count detections
            total_detections += len(detections)
            target_found = any(d['id'] == aruco_id for d in detections)
            if target_found:
                target_detections += 1

            # Add image info overlay
            info_text = f"Image {i+1}/{len(images)}"
            if detections:
                info_text += f" | Found {len(detections)} markers"
                for det in detections:
                    if det['id'] == aruco_id:
                        info_text += f" | TARGET ID {aruco_id} FOUND!"
                        break
            else:
                info_text += " | No markers detected"

            # Add status overlay
            status_img = output_img.copy()
            cv2.rectangle(status_img, (10, 10), (min(800, status_img.shape[1]-10), 60), (0, 0, 0), -1)
            cv2.putText(status_img, info_text, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Save images
            output_filename = os.path.join(output_dir, f"aruco_detection_{i:04d}.jpg")
            undistorted_filename = os.path.join(output_dir, f"undistorted_{i:04d}.jpg")

            cv2.imwrite(output_filename, status_img)
            cv2.imwrite(undistorted_filename, undistorted_img)

            # Print detection info to console
            if detections:
                print(f"\nImage {i+1}: Found {len(detections)} markers")
                for det in detections:
                    distance = np.linalg.norm(det['translation'])
                    print(f"  ID {det['id']}: T=[{det['translation'][0]:.2f}, {det['translation'][1]:.2f}, {det['translation'][2]:.2f}] (dist: {distance:.2f}m)")
                    if det['id'] == aruco_id:
                        print(f"    *** TARGET MARKER FOUND ***")

        except Exception as e:
            print(f"Failed to process image {i}: {e}")

    # Summary
    print(f"\n=== DETECTION SUMMARY ===")
    print(f"Total images processed: {len(images)}")
    print(f"Images with target ID {aruco_id}: {target_detections}")
    print(f"Total marker detections: {total_detections}")
    print(f"Images saved to: {output_dir}")

    if target_detections == 0:
        print(f"\n⚠ WARNING: Target marker ID {aruco_id} was never detected!")
        print("Check:")
        print("- Marker is visible in camera view")
        print("- Marker ID matches what you generated")
        print("- Marker size parameter is correct")
        print("- ArUco dictionary matches")
    else:
        print(f"\n✓ Success: Target marker ID {aruco_id} detected in {target_detections} images")
