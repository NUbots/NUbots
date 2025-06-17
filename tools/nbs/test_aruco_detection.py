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
    command.add_argument("--display-time", type=int, default=1000, help="Display time per image in milliseconds (0 = wait for keypress)")


def create_fisheye_camera_matrix(lens, image_width, image_height):
    """Create camera matrix and distortion coefficients for fisheye lens"""
    # Focal length in pixels
    fx = fy = lens.focal_length * image_width

    # Principal point
    cx = lens.centre.x * image_width
    cy = lens.centre.y * image_height

    camera_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float64)

    # Fisheye distortion coefficients [k1, k2, k3, k4]
    k1, k2 = lens.k.x, lens.k.y
    dist_coeffs = np.array([k1, k2, 0.0, 0.0], dtype=np.float64)

    return camera_matrix, dist_coeffs


def undistort_fisheye_image(img, camera_matrix, dist_coeffs):
    """Undistort fisheye image for ArUco detection"""
    h, w = img.shape[:2]

    # Create new camera matrix for undistorted image
    new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        camera_matrix, dist_coeffs, (w, h), np.eye(3), balance=0.0
    )

    # Create undistortion maps
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, np.eye(3), new_camera_matrix, (w, h), cv2.CV_16SC2
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


def detect_and_draw_aruco_markers(image, aruco_dict, lens, image_width, image_height, marker_size, target_id=None):
    """Detect ArUco markers and draw them with pose axes"""
    # Create fisheye camera parameters
    camera_matrix, dist_coeffs = create_fisheye_camera_matrix(lens, image_width, image_height)

    # Undistort image for better ArUco detection
    undistorted_img, undistorted_camera_matrix = undistort_fisheye_image(image, camera_matrix, dist_coeffs)

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

            # Draw pose axes
            cv2.drawFrameAxes(output_img, undistorted_camera_matrix, np.zeros(4),
                            rvec, tvec, marker_size * 0.5)

            # Convert to transform matrix
            R, _ = cv2.Rodrigues(rvec)
            H_marker = np.eye(4)
            H_marker[:3, :3] = R
            H_marker[:3, 3] = tvec.flatten()

            # Store detection info
            detection = {
                'id': int(marker_id),
                'pose_matrix': H_marker,
                'translation': tvec.flatten(),
                'rotation_vector': rvec.flatten(),
                'corners': corners[i]
            }
            detections.append(detection)

            # Add text with pose info
            text_pos = tuple(map(int, corners[i][0][0]))
            cv2.putText(output_img, f"ID:{marker_id}",
                       (text_pos[0], text_pos[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(output_img, f"T:[{tvec[0][0]:.2f},{tvec[0][1]:.2f},{tvec[0][2]:.2f}]",
                       (text_pos[0], text_pos[1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

    return output_img, detections, undistorted_img


def run(files, aruco_dict, aruco_id, aruco_size, camera_name, max_images, display_time, **kwargs):
    # Get ArUco dictionary
    aruco_dict_id = getattr(cv2.aruco, aruco_dict)
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)

    print(f"Testing ArUco detection:")
    print(f"  Dictionary: {aruco_dict}")
    print(f"  Target ID: {aruco_id} (will detect all if not found)")
    print(f"  Marker size: {aruco_size}m")
    print(f"  Camera: {camera_name}")
    print(f"  Max images: {max_images}")
    print(f"  Display time: {display_time}ms per image")
    print(f"\nControls:")
    print(f"  - Press 'q' to quit")
    print(f"  - Press 's' to skip to next image")
    print(f"  - Press SPACE to pause/continue")
    print(f"  - Press 'r' to restart from beginning")

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

    # Create windows
    cv2.namedWindow("ArUco Detection", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Original (Undistorted)", cv2.WINDOW_NORMAL)

    # Process images
    total_detections = 0
    target_detections = 0
    current_image = 0
    paused = False

    print("\nProcessing images and detecting ArUco markers...")
    print("Starting display loop...")

    try:
        while current_image < len(images):
            img_packet = images[current_image]

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
                    img, aruco_dict, img_packet.msg.lens,
                    img_packet.msg.dimensions.x, img_packet.msg.dimensions.y,
                    aruco_size, None  # Detect all markers
                )

                # Count detections
                total_detections += len(detections)
                target_found = any(d['id'] == aruco_id for d in detections)
                if target_found:
                    target_detections += 1

                # Add image info overlay
                info_text = f"Image {current_image+1}/{len(images)}"
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
                cv2.putText(status_img, "Press 'q' to quit, 's' to skip, SPACE to pause",
                           (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                # Display images
                cv2.imshow("ArUco Detection", status_img)
                cv2.imshow("Original (Undistorted)", undistorted_img)

                # Print detection info to console
                if detections:
                    print(f"\nImage {current_image+1}: Found {len(detections)} markers")
                    for det in detections:
                        distance = np.linalg.norm(det['translation'])
                        print(f"  ID {det['id']}: T=[{det['translation'][0]:.2f}, {det['translation'][1]:.2f}, {det['translation'][2]:.2f}] (dist: {distance:.2f}m)")
                        if det['id'] == aruco_id:
                            print(f"    *** TARGET MARKER FOUND ***")

                # Handle key input
                if not paused:
                    key = cv2.waitKey(display_time) & 0xFF
                else:
                    key = cv2.waitKey(0) & 0xFF  # Wait indefinitely when paused

                if key == ord('q'):
                    print("\nQuitting...")
                    break
                elif key == ord('s'):
                    print(f"Skipping to next image...")
                    current_image += 1
                elif key == ord(' '):
                    paused = not paused
                    print(f"{'Paused' if paused else 'Resumed'}")
                elif key == ord('r'):
                    print("Restarting from beginning...")
                    current_image = 0
                    total_detections = 0
                    target_detections = 0
                    continue
                elif key == 27:  # ESC key
                    print("\nQuitting...")
                    break
                else:
                    if not paused:
                        current_image += 1

            except Exception as e:
                print(f"Failed to process image {current_image}: {e}")
                current_image += 1

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        # Clean up
        cv2.destroyAllWindows()

        # Summary
        print(f"\n=== DETECTION SUMMARY ===")
        print(f"Total images processed: {current_image}")
        print(f"Images with target ID {aruco_id}: {target_detections}")
        print(f"Total marker detections: {total_detections}")

        if target_detections == 0:
            print(f"\n⚠ WARNING: Target marker ID {aruco_id} was never detected!")
            print("Check:")
            print("- Marker is visible in camera view")
            print("- Marker ID matches what you generated")
            print("- Marker size parameter is correct")
            print("- ArUco dictionary matches")
        else:
            print(f"\n✓ Success: Target marker ID {aruco_id} detected in {target_detections} images")
