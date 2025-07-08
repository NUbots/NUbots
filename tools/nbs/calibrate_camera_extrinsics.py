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

import json
import os
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml
from scipy.optimize import minimize
from tqdm import tqdm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

# Expected ArUco rotation in world frame (flat on ground)
# Identity rotation means the ArUco tag is flat on the ground
ARUCO_WORLD_ROTATION = np.eye(3)  # Identity rotation (flat on ground)

# OpenCV camera coordinate system to NUbots coordinate system
# OpenCV: x right, y down, z forward
# NUbots: x forward, y left, z up
OPEN_CV_TO_NU_ROTATION = np.array([
    [0, 0, 1, 0],   # x_nu = z_opencv (forward)
    [-1, 0, 0, 0],  # y_nu = -x_opencv (left)
    [0, -1, 0, 0],  # z_nu = -y_opencv (up)
    [0, 0, 0, 1]
])

def register(command):
    command.description = "Calibrate fisheye camera extrinsic parameters using ArUco markers"

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs files containing compressed images and sensor data"
    )
    command.add_argument("--output", "-o", nargs="?", default=os.getcwd(), help="The folder to save results")
    command.add_argument("--aruco-dict", default="DICT_6X6_250", help="ArUco dictionary to use")
    command.add_argument("--aruco-id", type=int, required=True, help="ID of the ArUco marker to track")
    command.add_argument("--aruco-size", type=float, required=True, help="Size of ArUco marker in meters")
    command.add_argument("--camera-name", required=True, help="Name of camera to calibrate")
    command.add_argument("--max-time-diff", type=float, default=0.01,
                        help="Maximum time difference for sensor-image synchronization (seconds)")


def quaternion_to_rotation_matrix(q):
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix"""
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])


def rotation_matrix_to_quaternion(R):
    """Convert 3x3 rotation matrix to quaternion [x, y, z, w]"""
    trace = np.trace(R)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w])


def extract_transform_matrix(iso3_msg):
    """Extract 4x4 transform matrix from protobuf iso3 message"""
    H = np.eye(4)
    H[:3, 3] = [iso3_msg.t.x, iso3_msg.t.y, iso3_msg.t.z]
    H[:3, :3] = [
        [iso3_msg.x.x, iso3_msg.y.x, iso3_msg.z.x],
        [iso3_msg.x.y, iso3_msg.y.y, iso3_msg.z.y],
        [iso3_msg.x.z, iso3_msg.y.z, iso3_msg.z.z]
    ]
    return H


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
    params.minMarkerPerimeterRate = 0.03  # Lower for distant markers
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.05
    params.minCornerDistanceRate = 0.05
    params.minDistanceToBorder = 3
    return params


def detect_aruco_marker(image, aruco_dict, aruco_id, marker_size):
    """Detect ArUco marker in image using hardcoded camera parameters"""
    # Undistort image for better ArUco detection
    undistorted_img, undistorted_camera_matrix = undistort_image(image)

    # Detect markers in undistorted image
    detector = cv2.aruco.ArucoDetector(aruco_dict, get_aruco_detector_params())
    corners, ids, _ = detector.detectMarkers(undistorted_img)

    if ids is not None and aruco_id in ids:
        idx = np.where(ids.flatten() == aruco_id)[0][0]
        marker_corners = corners[idx]

        # Estimate pose using undistorted camera matrix
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners, marker_size, undistorted_camera_matrix, np.zeros(4)
        )

        # Convert to transform matrix
        R, _ = cv2.Rodrigues(rvec[0])
        H_marker = np.eye(4)
        H_marker[:3, :3] = R
        H_marker[:3, 3] = tvec[0]

        # Convert to NUbots coordinate system
        H_marker = OPEN_CV_TO_NU_ROTATION @ H_marker

        return H_marker, marker_corners[0], undistorted_img

    return None, None, undistorted_img


class CalibrationData:
    def __init__(self):
        self.observations = []  # List of (Hcw, Hca_detected, image_info)

    def add_observation(self, Hcw, Hca_detected, image_info):
        """Add a calibration observation"""
        self.observations.append({
            'Hcw': Hcw,  # Camera to world transform
            'Hca_detected': Hca_detected,  # Detected ArUco pose in camera space
            'image_info': image_info
        })


def objective_function(offsets, calib_data):
    """Objective function for optimization - minimizes rotation error in world space"""
    roll_offset, pitch_offset = offsets

    # Use only the first observation (single image)
    obs = calib_data.observations[0]

    # Create offset rotations
    R_pitch = np.array([
        [np.cos(pitch_offset), -np.sin(pitch_offset), 0],
        [np.sin(pitch_offset), np.cos(pitch_offset), 0],
        [0, 0, 1]
    ])

    R_roll = np.array([
        [np.cos(roll_offset), 0, np.sin(roll_offset)],
        [0, 1, 0],
        [-np.sin(roll_offset), 0, np.cos(roll_offset)]
    ])

    # Apply offsets to camera rotation: R_roll * R_pitch * R_camera_base
    Hwc = np.linalg.inv(obs['Hcw'])
    Hwa = Hwc @ obs['Hca_detected']
    Rwa_corrected = R_roll @ R_pitch @ Hwa[:3, :3]
    Rwa_expected = ARUCO_WORLD_ROTATION

    # Compute rotation error using quaternion difference
    q_detected = rotation_matrix_to_quaternion(Rwa_corrected)
    q_expected = rotation_matrix_to_quaternion(Rwa_expected)

    # Normalize quaternions
    q_detected_norm = np.linalg.norm(q_detected)
    q_expected_norm = np.linalg.norm(q_expected)

    if q_detected_norm < 1e-10 or q_expected_norm < 1e-10:
        return np.pi  # Return maximum error if quaternions are invalid

    q_detected = q_detected / q_detected_norm
    q_expected = q_expected / q_expected_norm

    # Use angle between quaternions as error metric
    dot_product = np.clip(np.dot(q_detected, q_expected), -1.0, 1.0)
    angle_error = 2.0 * np.arccos(np.abs(dot_product))  # Angle in radians

    return angle_error


def plot_3d_coordinate_frames(obs, optimal_roll, optimal_pitch, output_dir, camera_name):
    """Create 3D plot showing world frame, camera frame, and ArUco tag frames"""

    # Create figure and 3D axis
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Define frame size for visualization
    frame_size = 0.1  # 10cm

    def plot_frame(ax, transform_matrix, color, label, alpha=1.0):
        """Plot a coordinate frame given a 4x4 transformation matrix"""
        origin = transform_matrix[:3, 3]
        R = transform_matrix[:3, :3]

        # Create unit vectors
        x_axis = R @ np.array([frame_size, 0, 0])
        y_axis = R @ np.array([0, frame_size, 0])
        z_axis = R @ np.array([0, 0, frame_size])

        # Plot axes
        ax.quiver(origin[0], origin[1], origin[2],
                 x_axis[0], x_axis[1], x_axis[2],
                 color='red', alpha=alpha, arrow_length_ratio=0.2)
        ax.quiver(origin[0], origin[1], origin[2],
                 y_axis[0], y_axis[1], y_axis[2],
                 color='green', alpha=alpha, arrow_length_ratio=0.2)
        ax.quiver(origin[0], origin[1], origin[2],
                 z_axis[0], z_axis[1], z_axis[2],
                 color='blue', alpha=alpha, arrow_length_ratio=0.2)

        # Add label
        ax.text(origin[0], origin[1], origin[2], label, fontsize=10, color=color)

    # World frame (origin)
    world_frame = np.eye(4)
    plot_frame(ax, world_frame, 'black', 'World', alpha=0.8)

    # Camera frame
    camera_frame = np.linalg.inv(obs['Hcw'])  # Camera to world transform
    plot_frame(ax, camera_frame, 'purple', 'Camera', alpha=0.8)

    # Detected ArUco frame (in world coordinates)
    Hwa_detected = np.linalg.inv(obs['Hcw']) @ obs['Hca_detected']
    # plot_frame(ax, Hwa_detected, 'orange', 'ArUco (Detected)', alpha=0.6)

    # Corrected ArUco frame (in world coordinates)
    R_pitch = np.array([
        [np.cos(optimal_pitch), -np.sin(optimal_pitch), 0],
        [np.sin(optimal_pitch), np.cos(optimal_pitch), 0],
        [0, 0, 1]
    ])
    R_roll = np.array([
        [np.cos(optimal_roll), 0, np.sin(optimal_roll)],
        [0, 1, 0],
        [-np.sin(optimal_roll), 0, np.cos(optimal_roll)]
    ])

    # Apply corrections to the detected ArUco pose
    Hwa_corrected = Hwa_detected.copy()
    Hwa_corrected[:3, :3] = R_roll @ R_pitch @ Hwa_detected[:3, :3]
    plot_frame(ax, Hwa_corrected, 'cyan', 'ArUco (Corrected)', alpha=0.8)

    # Expected ArUco frame (should be at origin with identity rotation)
    expected_aruco_frame = np.eye(4)
    plot_frame(ax, expected_aruco_frame, 'red', 'ArUco (Expected)', alpha=0.8)

    # Set axis labels and limits
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'3D Coordinate Frames - {camera_name}\nRoll: {np.degrees(optimal_roll):.2f}°, Pitch: {np.degrees(optimal_pitch):.2f}°')

    # Set equal aspect ratio
    max_range = np.array([
        ax.get_xlim()[1] - ax.get_xlim()[0],
        ax.get_ylim()[1] - ax.get_ylim()[0],
        ax.get_zlim()[1] - ax.get_zlim()[0]
    ]).max() / 2.0

    mid_x = (ax.get_xlim()[1] + ax.get_xlim()[0]) * 0.5
    mid_y = (ax.get_ylim()[1] + ax.get_ylim()[0]) * 0.5
    mid_z = (ax.get_zlim()[1] + ax.get_zlim()[0]) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Add legend
    legend_elements = [
        plt.Line2D([0], [0], color='black', label='World Frame'),
        plt.Line2D([0], [0], color='purple', label='Camera Frame'),
        plt.Line2D([0], [0], color='orange', label='ArUco (Detected)'),
        plt.Line2D([0], [0], color='cyan', label='ArUco (Corrected)'),
        plt.Line2D([0], [0], color='red', label='ArUco (Expected)')
    ]
    ax.legend(handles=legend_elements, loc='upper right')

    # Save plot
    plot_file = os.path.join(output_dir, f"{camera_name}_coordinate_frames_3d.png")
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"3D coordinate frame plot saved to: {plot_file}")

    # Show plot
    plt.show()

    return plot_file


def synchronize_messages(images, sensors, max_time_diff):
    """Synchronize image and sensor messages by timestamp"""
    synchronized = []

    for img_packet in images:
        img_time = img_packet.msg.timestamp.seconds + img_packet.msg.timestamp.nanos * 1e-9

        # Find closest sensor message
        best_sensor = None
        min_time_diff = float('inf')

        for sensor_packet in sensors:
            sensor_time = sensor_packet.msg.timestamp.seconds + sensor_packet.msg.timestamp.nanos * 1e-9
            time_diff = abs(img_time - sensor_time)

            if time_diff < min_time_diff and time_diff <= max_time_diff:
                min_time_diff = time_diff
                best_sensor = sensor_packet

        if best_sensor is not None:
            synchronized.append((img_packet, best_sensor))

    return synchronized


def run(files, output, aruco_dict, aruco_id, aruco_size, camera_name, max_time_diff, **kwargs):
    os.makedirs(output, exist_ok=True)

    print(f"Using world space calibration with ArUco tag flat on ground:")
    print(f"  Expected ArUco rotation: Identity (flat on ground)")
    print(f"  Expected ArUco translation: [0.0, 0.0, 0.0] (world origin)")
    print(f"  Coordinate system: x forward, y left, z up")

    # Get ArUco dictionary - FIXED: Create the actual dictionary object
    aruco_dict_id = getattr(cv2.aruco, aruco_dict)
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)

    # Collect messages
    print("Loading messages from NBS files...")
    images = []
    sensors = []

    for packet in tqdm(LinearDecoder(*files, types=["message.output.CompressedImage", "message.input.Sensors"])):
        if packet.type.name == "message.output.CompressedImage":
            if packet.msg.name == camera_name:
                images.append(packet)
        elif packet.type.name == "message.input.Sensors":
            sensors.append(packet)

    print(f"Found {len(images)} images and {len(sensors)} sensor messages")

    # Synchronize messages
    print("Synchronizing messages...")
    synchronized = synchronize_messages(images, sensors, max_time_diff)
    print(f"Synchronized {len(synchronized)} image-sensor pairs")

    if len(synchronized) == 0:
        print("No synchronized messages found. Try increasing --max-time-diff")
        return

    # Process synchronized messages - use only the first successful detection
    calib_data = CalibrationData()
    successful_detection = None

    print("Processing images and detecting ArUco markers (using last successful detection)...")
    for img_packet, sensor_packet in tqdm(synchronized):
        # Decode image
        try:
            image_data = decode_image(img_packet.msg.data, img_packet.msg.format)
            img = image_data[0]["image"].numpy()
            fmt = image_data[0]["fourcc"]

            # Debayer if needed (same logic as extract_images.py)
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

        except Exception as e:
            print(f"Failed to decode image: {e}")
            continue

        # Detect ArUco marker
        H_marker, corners, undistorted_img = detect_aruco_marker(
            img, aruco_dict, aruco_id, aruco_size
        )

        if H_marker is not None:
            # Extract camera to world transform
            Hcw = extract_transform_matrix(img_packet.msg.Hcw)

            # Store the successful detection (will keep overwriting to get the last one)
            successful_detection = {
                'Hcw': Hcw,
                'H_marker': H_marker,
                'metadata': {
                    'timestamp': img_packet.msg.timestamp.seconds + img_packet.msg.timestamp.nanos * 1e-9,
                    'image_size': (img_packet.msg.dimensions.x, img_packet.msg.dimensions.y)
                }
            }

    # Add the last successful detection to calibration data
    if successful_detection is not None:
        calib_data.add_observation(
            successful_detection['Hcw'],
            successful_detection['H_marker'],
            successful_detection['metadata']
        )

        print(f"Found last successful ArUco detection!")
        print(f"  Image timestamp: {successful_detection['metadata']['timestamp']}")
        print(f"  Image size: {successful_detection['metadata']['image_size'][0]} x {successful_detection['metadata']['image_size'][1]}")

    if len(calib_data.observations) == 0:
        print("No ArUco markers detected. Check your marker ID and dictionary.")
        return

    print(f"Using single image for calibration")

    # Add debugging information
    obs = calib_data.observations[0]

    # Test objective function at initial guess
    initial_error = objective_function([0.0, 0.0], calib_data)
    print(f"Initial rotation error: {initial_error:.6f} radians ({np.degrees(initial_error):.3f} degrees)")

    # Optimize roll and pitch offsets with better options
    print("\nOptimizing roll and pitch offsets (minimizing rotation error in world space)...")
    initial_guess = [0.0, 0.0]  # [roll_offset, pitch_offset]

    result = minimize(
        objective_function,
        initial_guess,
        args=(calib_data,),
        method='BFGS',
        options={
            'disp': True,
            'maxiter': 1000,
            'gtol': 1e-8,
            'eps': 1e-8
        }
    )

    optimal_roll, optimal_pitch = result.x

    print(f"\nOptimization Results:")
    print(f"Optimal roll offset: {optimal_roll:.6f} radians ({np.degrees(optimal_roll):.3f} degrees)")
    print(f"Optimal pitch offset: {optimal_pitch:.6f} radians ({np.degrees(optimal_pitch):.3f} degrees)")
    print(f"Final rotation error: {result.fun:.6f} radians ({np.degrees(result.fun):.3f} degrees)")
    print(f"Success: {result.success}")
    print(f"Number of iterations: {result.nit}")
    print(f"Number of function evaluations: {result.nfev}")

    # Test a few other optimization methods if BFGS doesn't work well
    if result.fun > 0.1:  # If error is still high
        print(f"\nTrying alternative optimization methods...")

        # Try L-BFGS-B with bounds
        result_lbfgs = minimize(
            objective_function,
            initial_guess,
            args=(calib_data,),
            method='L-BFGS-B',
            bounds=[(-np.pi/4, np.pi/4), (-np.pi/4, np.pi/4)],  # Reasonable bounds
            options={'disp': True, 'maxiter': 1000}
        )

        if result_lbfgs.fun < result.fun:
            print(f"L-BFGS-B found better solution:")
            print(f"  Roll: {result_lbfgs.x[0]:.6f} rad ({np.degrees(result_lbfgs.x[0]):.3f} deg)")
            print(f"  Pitch: {result_lbfgs.x[1]:.6f} rad ({np.degrees(result_lbfgs.x[1]):.3f} deg)")
            print(f"  Error: {result_lbfgs.fun:.6f} rad ({np.degrees(result_lbfgs.fun):.3f} deg)")
            optimal_roll, optimal_pitch = result_lbfgs.x
            result = result_lbfgs

    # Save results
    results = {
        'camera_name': camera_name,
        'aruco_id': aruco_id,
        'aruco_size': aruco_size,
        'calibration_method': 'world_space_aruco_flat_ground',
        'aruco_ground_truth': {
            'translation': [0.0, 0.0, 0.0],
            'rotation': 'identity_flat_on_ground'
        },
        'coordinate_system': 'x forward, y left, z up',
        'num_observations': 1,  # Single image
        'optimization': {
            'roll_offset_radians': float(optimal_roll),
            'pitch_offset_radians': float(optimal_pitch),
            'roll_offset_degrees': float(np.degrees(optimal_roll)),
            'pitch_offset_degrees': float(np.degrees(optimal_pitch)),
            'final_rotation_error': float(result.fun),
            'success': bool(result.success),
            'num_iterations': int(result.nit) if hasattr(result, 'nit') else None
        }
    }

    results_file = os.path.join(output, f"{camera_name}_extrinsic_calibration_single_image.yaml")
    with open(results_file, 'w') as f:
        yaml.safe_dump(results, f, default_flow_style=False)

    print(f"\nResults saved to: {results_file}")

    print(f"\nDebug information:")
    print(f"Expected ArUco pose (world space):")
    print(f"  Rotation: Identity (flat on ground)")
    print(f"  Translation: [0.0, 0.0, 0.0]")
    print(f"Detected ArUco pose (world space):")
    # Apply roll and pitch corrections to detected pose
    R_pitch = np.array([
        [np.cos(optimal_pitch), -np.sin(optimal_pitch), 0],
        [np.sin(optimal_pitch), np.cos(optimal_pitch), 0],
        [0, 0, 1]
    ])
    R_roll = np.array([
        [np.cos(optimal_roll), 0, np.sin(optimal_roll)],
        [0, 1, 0],
        [-np.sin(optimal_roll), 0, np.cos(optimal_roll)]
    ])
    Hwa = np.linalg.inv(obs['Hcw']) @ obs['Hca_detected']
    Rwa_corrected = R_roll @ R_pitch @ Hwa[:3, :3]
    print("Original Rotation:")
    print(Hwa[:3, :3])

    print(f"  Corrected Rotation:")
    print(Rwa_corrected)

    print(" Expected Rotation:")
    print(ARUCO_WORLD_ROTATION)

    # Plot 3D coordinate frames
    plot_file = plot_3d_coordinate_frames(obs, optimal_roll, optimal_pitch, output, camera_name)
