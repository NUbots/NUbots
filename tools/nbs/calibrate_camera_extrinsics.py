# #!/usr/bin/env python3
# #
# # MIT License
# #
# # Copyright (c) 2019 NUbots
# #
# # This file is part of the NUbots codebase.
# # See https://github.com/NUbots/NUbots for further info.
# #
# # Permission is hereby granted, free of charge, to any person obtaining a copy
# # of this software and associated documentation files (the "Software"), to deal
# # in the Software without restriction, including without limitation the rights
# # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# # copies of the Software, and to permit persons to whom the Software is
# # furnished to do so, subject to the following conditions:
# #
# # The above copyright notice and this permission notice shall be included in all
# # copies or substantial portions of the Software.
# #
# # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# # SOFTWARE.
# #

# import json
# import os
# from collections import defaultdict
# from typing import Dict, List, Optional, Tuple

# import cv2
# import cv2.fisheye as fisheye
# import numpy as np
# import yaml
# from scipy.optimize import minimize
# from tqdm import tqdm

# from utility.nbs import LinearDecoder

# from .images import decode_image, fourcc

# # Hardcoded ArUco marker ground truth pose in torso space
# # TODO: Update these values to match your actual ArUco marker placement
# ARUCO_GROUND_TRUTH = {
#     'translation': [0.5, 0.0, 0.8],  # x, y, z in meters (torso space)
#     'quaternion': [0.0, 0.0, 0.0, 1.0]  # x, y, z, w (identity rotation)
# }


# def register(command):
#     command.description = "Calibrate fisheye camera extrinsic parameters using ArUco markers"

#     # Command arguments
#     command.add_argument(
#         "files", metavar="files", nargs="+", help="The nbs files containing compressed images and sensor data"
#     )
#     command.add_argument("--output", "-o", nargs="?", default=os.getcwd(), help="The folder to save results")
#     command.add_argument("--aruco-dict", default="DICT_6X6_250", help="ArUco dictionary to use")
#     command.add_argument("--aruco-id", type=int, required=True, help="ID of the ArUco marker to track")
#     command.add_argument("--aruco-size", type=float, required=True, help="Size of ArUco marker in meters")
#     command.add_argument("--camera-name", required=True, help="Name of camera to calibrate")
#     command.add_argument("--max-time-diff", type=float, default=0.01,
#                         help="Maximum time difference for sensor-image synchronization (seconds)")


# def quaternion_to_rotation_matrix(q):
#     """Convert quaternion [x, y, z, w] to 3x3 rotation matrix"""
#     x, y, z, w = q
#     return np.array([
#         [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
#         [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
#         [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
#     ])


# def rotation_matrix_to_quaternion(R):
#     """Convert 3x3 rotation matrix to quaternion [x, y, z, w]"""
#     trace = np.trace(R)
#     if trace > 0:
#         s = np.sqrt(trace + 1.0) * 2
#         w = 0.25 * s
#         x = (R[2, 1] - R[1, 2]) / s
#         y = (R[0, 2] - R[2, 0]) / s
#         z = (R[1, 0] - R[0, 1]) / s
#     else:
#         if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
#             s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
#             w = (R[2, 1] - R[1, 2]) / s
#             x = 0.25 * s
#             y = (R[0, 1] + R[1, 0]) / s
#             z = (R[0, 2] + R[2, 0]) / s
#         elif R[1, 1] > R[2, 2]:
#             s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
#             w = (R[0, 2] - R[2, 0]) / s
#             x = (R[0, 1] + R[1, 0]) / s
#             y = 0.25 * s
#             z = (R[1, 2] + R[2, 1]) / s
#         else:
#             s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
#             w = (R[1, 0] - R[0, 1]) / s
#             x = (R[0, 2] + R[2, 0]) / s
#             y = (R[1, 2] + R[2, 1]) / s
#             z = 0.25 * s
#     return np.array([x, y, z, w])


# def apply_roll_pitch_offsets(base_transform, roll_offset, pitch_offset):
#     """Apply roll and pitch offsets to a base transform, following Camera.cpp logic"""
#     # Extract rotation and translation from base transform
#     R_base = base_transform[:3, :3]
#     t_base = base_transform[:3, 3]

#     # Create offset rotations (following Camera.cpp: pitch around Z, roll around Y)
#     R_pitch = np.array([
#         [np.cos(pitch_offset), -np.sin(pitch_offset), 0],
#         [np.sin(pitch_offset), np.cos(pitch_offset), 0],
#         [0, 0, 1]
#     ])

#     R_roll = np.array([
#         [np.cos(roll_offset), 0, np.sin(roll_offset)],
#         [0, 1, 0],
#         [-np.sin(roll_offset), 0, np.cos(roll_offset)]
#     ])

#     # Apply offsets: R_pitch * R_roll * R_base (following Camera.cpp order)
#     R_corrected = R_pitch @ R_roll @ R_base

#     # Construct corrected transform
#     H_corrected = np.eye(4)
#     H_corrected[:3, :3] = R_corrected
#     H_corrected[:3, 3] = t_base

#     return H_corrected


# def extract_transform_matrix(iso3_msg):
#     """Extract 4x4 transform matrix from protobuf iso3 message"""
#     H = np.eye(4)
#     H[:3, 3] = [iso3_msg.t.x, iso3_msg.t.y, iso3_msg.t.z]
#     H[:3, :3] = [
#         [iso3_msg.x.x, iso3_msg.y.x, iso3_msg.z.x],
#         [iso3_msg.x.y, iso3_msg.y.y, iso3_msg.z.y],
#         [iso3_msg.x.z, iso3_msg.y.z, iso3_msg.z.z]
#     ]
#     return H


# def create_fisheye_camera_matrix(lens, image_width, image_height):
#     """Create camera matrix and distortion coefficients for fisheye lens"""
#     # Focal length in pixels
#     fx = fy = lens.focal_length * image_width

#     # Principal point
#     cx = lens.centre.x * image_width
#     cy = lens.centre.y * image_height

#     camera_matrix = np.array([
#         [fx, 0, cx],
#         [0, fy, cy],
#         [0, 0, 1]
#     ], dtype=np.float64)

#     # Fisheye distortion coefficients [k1, k2, k3, k4]
#     # The lens.k contains [k1, k2] in normalized units
#     k1, k2 = lens.k.x, lens.k.y
#     dist_coeffs = np.array([k1, k2, 0.0, 0.0], dtype=np.float64)

#     return camera_matrix, dist_coeffs


# def undistort_fisheye_image(img, camera_matrix, dist_coeffs):
#     """Undistort fisheye image for ArUco detection"""
#     h, w = img.shape[:2]

#     # Create new camera matrix for undistorted image
#     new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
#         camera_matrix, dist_coeffs, (w, h), np.eye(3), balance=0.0
#     )

#     # Create undistortion maps
#     map1, map2 = cv2.fisheye.initUndistortRectifyMap(
#         camera_matrix, dist_coeffs, np.eye(3), new_camera_matrix, (w, h), cv2.CV_16SC2
#     )

#     # Undistort image
#     undistorted = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)

#     return undistorted, new_camera_matrix


# def get_aruco_detector_params():
#     """Get ArUco detector parameters optimized for fisheye cameras"""
#     params = cv2.aruco.DetectorParameters()
#     params.adaptiveThreshWinSizeMin = 3
#     params.adaptiveThreshWinSizeMax = 23
#     params.adaptiveThreshWinSizeStep = 10
#     params.minMarkerPerimeterRate = 0.03  # Lower for distant markers
#     params.maxMarkerPerimeterRate = 4.0
#     params.polygonalApproxAccuracyRate = 0.05
#     params.minCornerDistanceRate = 0.05
#     params.minDistanceToBorder = 3
#     return params


# def detect_aruco_marker(image, aruco_dict, aruco_id, lens, image_width, image_height, marker_size):
#     """Detect ArUco marker in fisheye image"""
#     # Create fisheye camera parameters
#     camera_matrix, dist_coeffs = create_fisheye_camera_matrix(lens, image_width, image_height)

#     # Undistort image for better ArUco detection
#     undistorted_img, undistorted_camera_matrix = undistort_fisheye_image(image, camera_matrix, dist_coeffs)

#     # Detect markers in undistorted image
#     detector = cv2.aruco.ArucoDetector(aruco_dict, get_aruco_detector_params())
#     corners, ids, _ = detector.detectMarkers(undistorted_img)

#     if ids is not None and aruco_id in ids:
#         idx = np.where(ids.flatten() == aruco_id)[0][0]
#         marker_corners = corners[idx]

#         # Estimate pose using undistorted camera matrix
#         rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
#             marker_corners, marker_size, undistorted_camera_matrix, np.zeros(4)
#         )

#         # Convert to transform matrix
#         R, _ = cv2.Rodrigues(rvec[0])
#         H_marker = np.eye(4)
#         H_marker[:3, :3] = R
#         H_marker[:3, 3] = tvec[0]

#         return H_marker, marker_corners[0], undistorted_img

#     return None, None, undistorted_img


# class CalibrationData:
#     def __init__(self):
#         self.observations = []  # List of (H_detected, H_expected, image_info)
#         self.base_Hpc = None    # Base transform from head pitch to camera

#     def add_observation(self, H_tc, H_aruco_detected, H_aruco_expected, image_info):
#         """Add a calibration observation"""
#         self.observations.append({
#             'H_tc': H_tc,
#             'H_aruco_detected': H_aruco_detected,
#             'H_aruco_expected': H_aruco_expected,
#             'image_info': image_info
#         })


# def objective_function(offsets, calib_data):
#     """Objective function for optimization - minimizes pose error"""
#     roll_offset, pitch_offset = offsets
#     total_error = 0.0

#     for obs in calib_data.observations:
#         # Apply offsets to get corrected camera transform
#         if calib_data.base_Hpc is not None:
#             H_pc_corrected = apply_roll_pitch_offsets(calib_data.base_Hpc, roll_offset, pitch_offset)
#         else:
#             # If we don't have base transform, work with the current estimate
#             H_pc_corrected = apply_roll_pitch_offsets(np.eye(4), roll_offset, pitch_offset)

#         # Transform detected ArUco pose to torso space
#         H_tc = obs['H_tc']
#         H_aruco_in_torso = np.linalg.inv(H_tc) @ obs['H_aruco_detected']

#         # Compare with expected pose
#         H_expected = obs['H_aruco_expected']

#         # Compute pose error (translation + rotation)
#         translation_error = np.linalg.norm(H_aruco_in_torso[:3, 3] - H_expected[:3, 3])

#         # Rotation error using quaternion difference
#         q_detected = rotation_matrix_to_quaternion(H_aruco_in_torso[:3, :3])
#         q_expected = rotation_matrix_to_quaternion(H_expected[:3, :3])
#         rotation_error = 1 - np.abs(np.dot(q_detected, q_expected))  # 1 - |cos(theta/2)|

#         total_error += translation_error + rotation_error * 10.0  # Weight rotation error

#     return total_error


# def synchronize_messages(images, sensors, max_time_diff):
#     """Synchronize image and sensor messages by timestamp"""
#     synchronized = []

#     for img_packet in images:
#         img_time = img_packet.msg.timestamp.seconds + img_packet.msg.timestamp.nanos * 1e-9

#         # Find closest sensor message
#         best_sensor = None
#         min_time_diff = float('inf')

#         for sensor_packet in sensors:
#             sensor_time = sensor_packet.msg.timestamp.seconds + sensor_packet.msg.timestamp.nanos * 1e-9
#             time_diff = abs(img_time - sensor_time)

#             if time_diff < min_time_diff and time_diff <= max_time_diff:
#                 min_time_diff = time_diff
#                 best_sensor = sensor_packet

#         if best_sensor is not None:
#             synchronized.append((img_packet, best_sensor))

#     return synchronized


# def run(files, output, aruco_dict, aruco_id, aruco_size, camera_name, max_time_diff, **kwargs):
#     os.makedirs(output, exist_ok=True)

#     # Use hardcoded ArUco ground truth pose
#     aruco_translation = np.array(ARUCO_GROUND_TRUTH['translation'])
#     aruco_quaternion = np.array(ARUCO_GROUND_TRUTH['quaternion'])  # [x, y, z, w]

#     H_aruco_expected = np.eye(4)
#     H_aruco_expected[:3, :3] = quaternion_to_rotation_matrix(aruco_quaternion)
#     H_aruco_expected[:3, 3] = aruco_translation

#     print(f"Using hardcoded ArUco ground truth pose:")
#     print(f"  Translation: {aruco_translation}")
#     print(f"  Quaternion: {aruco_quaternion}")

#     # Get ArUco dictionary
#     aruco_dict = getattr(cv2.aruco, aruco_dict)

#     # Collect messages
#     print("Loading messages from NBS files...")
#     images = []
#     sensors = []

#     for packet in tqdm(LinearDecoder(*files, types=["message.output.CompressedImage", "message.input.Sensors"])):
#         if packet.type == "message.output.CompressedImage":
#             if packet.msg.name == camera_name:
#                 images.append(packet)
#         elif packet.type == "message.input.Sensors":
#             sensors.append(packet)

#     print(f"Found {len(images)} images and {len(sensors)} sensor messages")

#     # Synchronize messages
#     print("Synchronizing messages...")
#     synchronized = synchronize_messages(images, sensors, max_time_diff)
#     print(f"Synchronized {len(synchronized)} image-sensor pairs")

#     if len(synchronized) == 0:
#         print("No synchronized messages found. Try increasing --max-time-diff")
#         return

#     # Process synchronized messages
#     calib_data = CalibrationData()
#     successful_detections = 0

#     print("Processing images and detecting ArUco markers...")
#     for img_packet, sensor_packet in tqdm(synchronized):
#         # Decode image
#         try:
#             image_data = decode_image(img_packet.msg.data, img_packet.msg.format)
#             img = image_data[0]["image"].numpy()
#             fmt = image_data[0]["fourcc"]

#             # Debayer if needed (same logic as extract_images.py)
#             if fmt == fourcc("BGGR"):
#                 img = cv2.cvtColor(img, cv2.COLOR_BayerBG2RGB_VNG)
#             elif fmt == fourcc("RGGB"):
#                 img = cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB_VNG)
#             elif fmt == fourcc("GRBG"):
#                 img = cv2.cvtColor(img, cv2.COLOR_BayerGR2RGB_VNG)
#             elif fmt == fourcc("GBRG"):
#                 img = cv2.cvtColor(img, cv2.COLOR_BayerGB2RGB_VNG)

#             # Convert to BGR for OpenCV
#             if fmt == fourcc("RGB8") or fmt == fourcc("RGB3"):
#                 img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

#         except Exception as e:
#             print(f"Failed to decode image: {e}")
#             continue

#         # Detect ArUco marker
#         H_marker, corners, undistorted_img = detect_aruco_marker(
#             img, aruco_dict, aruco_id, img_packet.msg.lens,
#             img_packet.msg.dimensions.x, img_packet.msg.dimensions.y, aruco_size
#         )

#         if H_marker is not None:
#             # Compute Htc transform
#             Hcw = extract_transform_matrix(img_packet.msg.Hcw)
#             Htw = extract_transform_matrix(sensor_packet.msg.Htw)
#             Htc = Htw @ np.linalg.inv(Hcw)  # torso to camera

#             # Add observation
#             calib_data.add_observation(Htc, H_marker, H_aruco_expected, {
#                 'timestamp': img_packet.msg.timestamp.seconds + img_packet.msg.timestamp.nanos * 1e-9,
#                 'image_size': (img_packet.msg.dimensions.x, img_packet.msg.dimensions.y)
#             })

#             successful_detections += 1

#     print(f"Successfully detected ArUco marker in {successful_detections} images")

#     if successful_detections == 0:
#         print("No ArUco markers detected. Check your marker ID and dictionary.")
#         return

#     # Optimize roll and pitch offsets
#     print("Optimizing roll and pitch offsets...")
#     initial_guess = [0.0, 0.0]  # [roll_offset, pitch_offset]

#     result = minimize(
#         objective_function,
#         initial_guess,
#         args=(calib_data,),
#         method='BFGS',
#         options={'disp': True}
#     )

#     optimal_roll, optimal_pitch = result.x

#     print(f"\nOptimization Results:")
#     print(f"Optimal roll offset: {optimal_roll:.6f} radians ({np.degrees(optimal_roll):.3f} degrees)")
#     print(f"Optimal pitch offset: {optimal_pitch:.6f} radians ({np.degrees(optimal_pitch):.3f} degrees)")
#     print(f"Final error: {result.fun:.6f}")
#     print(f"Success: {result.success}")

#     # Save results
#     results = {
#         'camera_name': camera_name,
#         'aruco_id': aruco_id,
#         'aruco_size': aruco_size,
#         'aruco_ground_truth': ARUCO_GROUND_TRUTH,
#         'num_observations': len(calib_data.observations),
#         'optimization': {
#             'roll_offset_radians': float(optimal_roll),
#             'pitch_offset_radians': float(optimal_pitch),
#             'roll_offset_degrees': float(np.degrees(optimal_roll)),
#             'pitch_offset_degrees': float(np.degrees(optimal_pitch)),
#             'final_error': float(result.fun),
#             'success': bool(result.success),
#             'num_iterations': int(result.nit) if hasattr(result, 'nit') else None
#         }
#     }

#     results_file = os.path.join(output, f"{camera_name}_extrinsic_calibration.yaml")
#     with open(results_file, 'w') as f:
#         yaml.safe_dump(results, f, default_flow_style=False)

#     print(f"\nResults saved to: {results_file}")

#     # Save debug images with detected markers (undistorted versions)
#     debug_dir = os.path.join(output, "debug_images")
#     os.makedirs(debug_dir, exist_ok=True)

#     saved_debug = 0
#     for i, (img_packet, sensor_packet) in enumerate(synchronized[:10]):  # Save first 10
#         if saved_debug >= 5:  # Limit to 5 debug images
#             break

#         try:
#             image_data = decode_image(img_packet.msg.data, img_packet.msg.format)
#             img = image_data[0]["image"].numpy()
#             fmt = image_data[0]["fourcc"]

#             # Apply same debayering logic
#             if fmt == fourcc("BGGR"):
#                 img = cv2.cvtColor(img, cv2.COLOR_BayerBG2RGB_VNG)
#             elif fmt == fourcc("RGGB"):
#                 img = cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB_VNG)
#             elif fmt == fourcc("GRBG"):
#                 img = cv2.cvtColor(img, cv2.COLOR_BayerGR2RGB_VNG)
#             elif fmt == fourcc("GBRG"):
#                 img = cv2.cvtColor(img, cv2.COLOR_BayerGB2RGB_VNG)

#             if fmt == fourcc("RGB8") or fmt == fourcc("RGB3"):
#                 img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

#             # Detect and draw markers
#             H_marker, corners, undistorted_img = detect_aruco_marker(
#                 img, aruco_dict, aruco_id, img_packet.msg.lens,
#                 img_packet.msg.dimensions.x, img_packet.msg.dimensions.y, aruco_size
#             )

#             if H_marker is not None:
#                 # Draw detected markers on undistorted image
#                 detector = cv2.aruco.ArucoDetector(aruco_dict, get_aruco_detector_params())
#                 corners_undist, ids_undist, _ = detector.detectMarkers(undistorted_img)

#                 if ids_undist is not None:
#                     cv2.aruco.drawDetectedMarkers(undistorted_img, corners_undist, ids_undist)

#                 debug_file = os.path.join(debug_dir, f"debug_{saved_debug:03d}.jpg")
#                 cv2.imwrite(debug_file, undistorted_img)
#                 saved_debug += 1

#         except Exception as e:
#             print(f"Failed to save debug image {i}: {e}")

#     print(f"Saved {saved_debug} debug images to {debug_dir}")
