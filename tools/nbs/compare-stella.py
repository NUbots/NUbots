#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from utility.nbs import LinearDecoder

def unwrap_angles(angles):
    """
    Unwrap angles to remove discontinuities (e.g., jumps from 180 to -180 degrees)

    Args:
        angles: numpy array of angles in degrees

    Returns:
        numpy array of unwrapped angles
    """
    return np.unwrap(np.radians(angles)) * 180 / np.pi

def rotation_matrix_to_euler(R):
    """Convert 3x3 rotation matrix to roll, pitch, yaw in degrees"""
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    if sy > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0

    return np.degrees([roll, pitch, yaw])

def extract_pose_from_transform(Htw_iso3):
    """Extract position and orientation from Htw (world to torso transform)"""
    # Extract the transformation matrix from the iso3 message
    transform_matrix = np.array([
        [Htw_iso3.x.x, Htw_iso3.y.x, Htw_iso3.z.x, Htw_iso3.t.x],
        [Htw_iso3.x.y, Htw_iso3.y.y, Htw_iso3.z.y, Htw_iso3.t.y],
        [Htw_iso3.x.z, Htw_iso3.y.z, Htw_iso3.z.z, Htw_iso3.t.z],
        [Htw_iso3.x.t, Htw_iso3.y.t, Htw_iso3.z.t, Htw_iso3.t.t]
    ])

    # Htw is world-to-torso, so we need to invert it to get torso position in world
    print(transform_matrix)
    Hwt = np.linalg.inv(transform_matrix)

    # Extract position (translation part) - this is torso position in world space
    position = Hwt[:3, 3]

    # Extract rotation matrix
    rotation_matrix = Hwt[:3, :3]

    # Convert rotation matrix to Euler angles (roll, pitch, yaw)
    orientation = rotation_matrix_to_euler(rotation_matrix)

    return position, orientation

# Path to your nbs file
nbs_file = "/home/willburgin/NUbots/recordings/stella-mocap/20250908T14_39_50.nbs"

print("Loading Ground Truth data...")
# Load ground truth messages
gt_decoder = LinearDecoder(nbs_file, types=["message.localisation.RobotPoseGroundTruth"], show_progress=True)
print(f"Found {len(gt_decoder)} ground truth messages")

print("Loading Stella data...")
# Load Stella pose estimates from Sensors messages
stella_decoder = LinearDecoder(nbs_file, types=["message.input.Sensors"], show_progress=True)
print(f"Found {len(stella_decoder)} sensor messages")

# Ground Truth data extraction
gt_timestamps = []
gt_xs, gt_ys, gt_zs = [], [], []
gt_rolls, gt_pitches, gt_yaws = [], [], []

for packet in gt_decoder:
    msg = packet.msg

    # Get timestamp in seconds
    timestamp_sec = packet.index_timestamp / 1e9
    gt_timestamps.append(timestamp_sec)

    # Extract transformation matrix from iso3 message (Hft - field to torso)
    Hft = msg.Hft
    transform_matrix = np.array([
        [Hft.x.x, Hft.y.x, Hft.z.x, Hft.t.x],
        [Hft.x.y, Hft.y.y, Hft.z.y, Hft.t.y],
        [Hft.x.z, Hft.y.z, Hft.z.z, Hft.t.z],
        [Hft.x.t, Hft.y.t, Hft.z.t, Hft.t.t]
    ])

    # Extract translation (position) - field frame = world frame
    position = transform_matrix[:3, 3]
    gt_xs.append(position[0])
    gt_ys.append(position[1])
    gt_zs.append(position[2])

    # Extract rotation matrix and convert to RPY
    rotation_matrix = transform_matrix[:3, :3]
    rpy = rotation_matrix_to_euler(rotation_matrix)
    gt_rolls.append(rpy[0])
    gt_pitches.append(rpy[1])
    gt_yaws.append(rpy[2])

# Stella data extraction
stella_timestamps = []
stella_xs, stella_ys, stella_zs = [], [], []
stella_rolls, stella_pitches, stella_yaws = [], [], []

for packet in stella_decoder:
    msg = packet.msg

    # Get timestamp in seconds
    timestamp_sec = packet.index_timestamp / 1e9
    stella_timestamps.append(timestamp_sec)

    # Extract pose from Htw (world to torso transform)
    position, orientation = extract_pose_from_transform(msg.Htw)

    stella_xs.append(position[0])
    stella_ys.append(position[1])
    stella_zs.append(position[2])
    stella_rolls.append(orientation[0])
    stella_pitches.append(orientation[1])
    stella_yaws.append(orientation[2])

# Convert to numpy arrays and normalize timestamps
gt_timestamps = np.array(gt_timestamps)
stella_timestamps = np.array(stella_timestamps)

# Find common time reference (earliest timestamp)
min_time = min(gt_timestamps[0] if len(gt_timestamps) > 0 else float('inf'),
               stella_timestamps[0] if len(stella_timestamps) > 0 else float('inf'))

gt_timestamps = gt_timestamps - min_time
stella_timestamps = stella_timestamps - min_time

# Convert to numpy arrays and unwrap angles
gt_rolls = unwrap_angles(np.array(gt_rolls))
gt_pitches = unwrap_angles(np.array(gt_pitches))
gt_yaws = unwrap_angles(np.array(gt_yaws))

stella_rolls = unwrap_angles(np.array(stella_rolls))
stella_pitches = unwrap_angles(np.array(stella_pitches))
stella_yaws = unwrap_angles(np.array(stella_yaws))

print(f"Ground Truth: {len(gt_timestamps)} samples over {gt_timestamps[-1]:.2f} seconds")
print(f"Stella: {len(stella_timestamps)} samples over {stella_timestamps[-1]:.2f} seconds")

# Plot translations comparison
plt.figure(figsize=(15, 10))

plt.subplot(3, 1, 1)
plt.plot(gt_timestamps, gt_xs, 'r-', label="Ground Truth", linewidth=2)
plt.plot(stella_timestamps, stella_xs, 'b--', label="Stella", linewidth=2)
plt.ylabel("X Position [m]")
plt.title("Position Comparison: Ground Truth vs Stella")
plt.legend()
plt.grid(True, alpha=0.3)

plt.subplot(3, 1, 2)
plt.plot(gt_timestamps, gt_ys, 'r-', label="Ground Truth", linewidth=2)
plt.plot(stella_timestamps, stella_ys, 'b--', label="Stella", linewidth=2)
plt.ylabel("Y Position [m]")
plt.legend()
plt.grid(True, alpha=0.3)

plt.subplot(3, 1, 3)
plt.plot(gt_timestamps, gt_zs, 'r-', label="Ground Truth", linewidth=2)
plt.plot(stella_timestamps, stella_zs, 'b--', label="Stella", linewidth=2)
plt.ylabel("Z Position [m]")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Plot orientation comparison
plt.figure(figsize=(15, 10))

plt.subplot(3, 1, 1)
plt.plot(gt_timestamps, gt_rolls, 'r-', label="Ground Truth", linewidth=2)
plt.plot(stella_timestamps, stella_rolls, 'b--', label="Stella", linewidth=2)
plt.ylabel("Roll [degrees]")
plt.title("Orientation Comparison: Ground Truth vs Stella")
plt.legend()
plt.grid(True, alpha=0.3)

plt.subplot(3, 1, 2)
plt.plot(gt_timestamps, gt_pitches, 'r-', label="Ground Truth", linewidth=2)
plt.plot(stella_timestamps, stella_pitches, 'b--', label="Stella", linewidth=2)
plt.ylabel("Pitch [degrees]")
plt.legend()
plt.grid(True, alpha=0.3)

plt.subplot(3, 1, 3)
plt.plot(gt_timestamps, gt_yaws, 'r-', label="Ground Truth", linewidth=2)
plt.plot(stella_timestamps, stella_yaws, 'b--', label="Stella", linewidth=2)
plt.ylabel("Yaw [degrees]")
plt.xlabel("Time [s]")
plt.legend()
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Plot 2D trajectory comparison
plt.figure(figsize=(10, 8))
plt.plot(gt_xs, gt_ys, 'r-', label="Ground Truth", linewidth=2, alpha=0.8)
plt.plot(stella_xs, stella_ys, 'b--', label="Stella", linewidth=2, alpha=0.8)
plt.scatter(gt_xs[0], gt_ys[0], color='green', s=100, label='Start', zorder=5)
plt.scatter(gt_xs[-1], gt_ys[-1], color='red', s=100, label='End', zorder=5)
plt.xlabel('X Position [m]')
plt.ylabel('Y Position [m]')
plt.title('2D Trajectory Comparison: Ground Truth vs Stella')
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.legend()
plt.show()

# Print comparison statistics
print(f"\n=== Comparison Statistics ===")
if len(gt_xs) > 0 and len(stella_xs) > 0:
    print(f"Position RMS Error:")

    # Only compare overlapping time periods
    common_start = max(gt_timestamps[0], stella_timestamps[0])
    common_end = min(gt_timestamps[-1], stella_timestamps[-1])

    # Filter to common time range
    gt_mask = (gt_timestamps >= common_start) & (gt_timestamps <= common_end)
    stella_mask = (stella_timestamps >= common_start) & (stella_timestamps <= common_end)

    if np.sum(gt_mask) > 1 and np.sum(stella_mask) > 1:
        try:
            # Simple numpy-based interpolation
            gt_times_common = gt_timestamps[gt_mask]
            stella_times_common = stella_timestamps[stella_mask]

            # Interpolate Stella data to GT timestamps using numpy
            stella_x_interp = np.interp(gt_times_common, stella_times_common, np.array(stella_xs)[stella_mask])
            stella_y_interp = np.interp(gt_times_common, stella_times_common, np.array(stella_ys)[stella_mask])
            stella_z_interp = np.interp(gt_times_common, stella_times_common, np.array(stella_zs)[stella_mask])

            # Calculate RMS errors
            x_error = np.sqrt(np.mean((np.array(gt_xs)[gt_mask] - stella_x_interp)**2))
            y_error = np.sqrt(np.mean((np.array(gt_ys)[gt_mask] - stella_y_interp)**2))
            z_error = np.sqrt(np.mean((np.array(gt_zs)[gt_mask] - stella_z_interp)**2))

            print(f"  X: {x_error:.3f} m")
            print(f"  Y: {y_error:.3f} m")
            print(f"  Z: {z_error:.3f} m")

            # Also compute orientation errors
            stella_roll_interp = np.interp(gt_times_common, stella_times_common, stella_rolls[stella_mask])
            stella_pitch_interp = np.interp(gt_times_common, stella_times_common, stella_pitches[stella_mask])
            stella_yaw_interp = np.interp(gt_times_common, stella_times_common, stella_yaws[stella_mask])

            roll_error = np.sqrt(np.mean((gt_rolls[gt_mask] - stella_roll_interp)**2))
            pitch_error = np.sqrt(np.mean((gt_pitches[gt_mask] - stella_pitch_interp)**2))
            yaw_error = np.sqrt(np.mean((gt_yaws[gt_mask] - stella_yaw_interp)**2))

            print(f"Orientation RMS Error:")
            print(f"  Roll:  {roll_error:.2f} degrees")
            print(f"  Pitch: {pitch_error:.2f} degrees")
            print(f"  Yaw:   {yaw_error:.2f} degrees")

        except Exception as e:
            print(f"  Could not compute RMS error: {e}")
    else:
        print("  Not enough overlapping data for comparison")
