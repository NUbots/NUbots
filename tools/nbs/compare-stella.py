#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import os
from utility.nbs import LinearDecoder

def register(command):
    command.description = "Compare Stella SLAM poses with ground truth data from an NBS file"

    # Command arguments
    command.add_argument(
        "nbs_file", metavar="nbs_file", help="The NBS file containing Stella and ground truth data"
    )
    command.add_argument(
        "--output", "-o", nargs="?", default=None,
        help="Output directory to save plots (if not specified, plots will be displayed)"
    )

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

def run(nbs_file, output=None, **kwargs):
    # Create output directory if specified
    if output:
        os.makedirs(output, exist_ok=True)
        # Set matplotlib to non-interactive backend when saving
        plt.ioff()

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

    # Compute offset from first 100 samples
    n_samples = 100
    if len(gt_xs) >= n_samples and len(stella_xs) >= n_samples:
        print(f"\nComputing offset from first {n_samples} samples...")

        # Calculate mean position offset
        gt_mean_pos = np.array([np.mean(gt_xs[:n_samples]),
                               np.mean(gt_ys[:n_samples]),
                               np.mean(gt_zs[:n_samples])])

        stella_mean_pos = np.array([np.mean(stella_xs[:n_samples]),
                                   np.mean(stella_ys[:n_samples]),
                                   np.mean(stella_zs[:n_samples])])

        position_offset = gt_mean_pos - stella_mean_pos

        # Calculate mean orientation offset
        gt_mean_orient = np.array([np.mean(gt_rolls[:n_samples]),
                                  np.mean(gt_pitches[:n_samples]),
                                  np.mean(gt_yaws[:n_samples])])

        stella_mean_orient = np.array([np.mean(stella_rolls[:n_samples]),
                                      np.mean(stella_pitches[:n_samples]),
                                      np.mean(stella_yaws[:n_samples])])

        orientation_offset = gt_mean_orient - stella_mean_orient

        print(f"Position offset: X={position_offset[0]:.3f}, Y={position_offset[1]:.3f}, Z={position_offset[2]:.3f} m")
        print(f"Orientation offset: Roll={orientation_offset[0]:.2f}, Pitch={orientation_offset[1]:.2f}, Yaw={orientation_offset[2]:.2f} degrees")

        # Apply offset to Stella data
        stella_xs_corrected = np.array(stella_xs) + position_offset[0]
        stella_ys_corrected = np.array(stella_ys) + position_offset[1]
        stella_zs_corrected = np.array(stella_zs) + position_offset[2]

        stella_rolls_corrected = stella_rolls + orientation_offset[0]
        stella_pitches_corrected = stella_pitches + orientation_offset[1]
        stella_yaws_corrected = stella_yaws + orientation_offset[2]

    else:
        print(f"\nNot enough samples for offset calculation (need {n_samples}, got GT:{len(gt_xs)}, Stella:{len(stella_xs)})")
        # Use uncorrected data
        stella_xs_corrected = np.array(stella_xs)
        stella_ys_corrected = np.array(stella_ys)
        stella_zs_corrected = np.array(stella_zs)
        stella_rolls_corrected = stella_rolls
        stella_pitches_corrected = stella_pitches
        stella_yaws_corrected = stella_yaws

    # Plot translations comparison
    plt.figure(figsize=(15, 10))

    plt.subplot(3, 1, 1)
    plt.plot(gt_timestamps, gt_xs, 'r-', label="Ground Truth", linewidth=2)
    plt.plot(stella_timestamps, stella_xs_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
    plt.plot(stella_timestamps, stella_xs, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("X Position [m]")
    plt.title("Position Comparison: Ground Truth vs Stella (with offset correction)")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 2)
    plt.plot(gt_timestamps, gt_ys, 'r-', label="Ground Truth", linewidth=2)
    plt.plot(stella_timestamps, stella_ys_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
    plt.plot(stella_timestamps, stella_ys, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("Y Position [m]")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 3)
    plt.plot(gt_timestamps, gt_zs, 'r-', label="Ground Truth", linewidth=2)
    plt.plot(stella_timestamps, stella_zs_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
    plt.plot(stella_timestamps, stella_zs, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("Z Position [m]")
    plt.xlabel("Time [s]")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()

    if output:
        plt.savefig(os.path.join(output, "position_comparison.png"), dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Saved position comparison plot to {os.path.join(output, 'position_comparison.png')}")
    else:
        plt.show()

    # Plot orientation comparison
    plt.figure(figsize=(15, 10))

    plt.subplot(3, 1, 1)
    plt.plot(gt_timestamps, gt_rolls, 'r-', label="Ground Truth", linewidth=2)
    plt.plot(stella_timestamps, stella_rolls_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
    plt.plot(stella_timestamps, stella_rolls, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("Roll [degrees]")
    plt.title("Orientation Comparison: Ground Truth vs Stella (with offset correction)")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 2)
    plt.plot(gt_timestamps, gt_pitches, 'r-', label="Ground Truth", linewidth=2)
    plt.plot(stella_timestamps, stella_pitches_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
    plt.plot(stella_timestamps, stella_pitches, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("Pitch [degrees]")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 3)
    plt.plot(gt_timestamps, gt_yaws, 'r-', label="Ground Truth", linewidth=2)
    plt.plot(stella_timestamps, stella_yaws_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
    plt.plot(stella_timestamps, stella_yaws, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("Yaw [degrees]")
    plt.xlabel("Time [s]")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()

    if output:
        plt.savefig(os.path.join(output, "orientation_comparison.png"), dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Saved orientation comparison plot to {os.path.join(output, 'orientation_comparison.png')}")
    else:
        plt.show()

    # Plot 2D trajectory comparison
    plt.figure(figsize=(10, 8))
    plt.plot(gt_xs, gt_ys, 'r-', label="Ground Truth", linewidth=2, alpha=0.8)
    plt.plot(stella_xs_corrected, stella_ys_corrected, 'b--', label="Stella (Corrected)", linewidth=2, alpha=0.8)
    plt.plot(stella_xs, stella_ys, 'g:', label="Stella (Original)", linewidth=1, alpha=0.5)
    plt.scatter(gt_xs[0], gt_ys[0], color='red', s=100, label='GT Start', zorder=5)
    plt.scatter(stella_xs_corrected[0], stella_ys_corrected[0], color='blue', s=100, label='Stella Start', zorder=5)
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.title('2D Trajectory Comparison: Ground Truth vs Stella (with offset correction)')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend()

    if output:
        plt.savefig(os.path.join(output, "trajectory_comparison.png"), dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Saved trajectory comparison plot to {os.path.join(output, 'trajectory_comparison.png')}")
    else:
        plt.show()

    # Print comparison statistics (update the interpolation section)
    if len(gt_xs) > 0 and len(stella_xs_corrected) > 0:
        print(f"\n=== Comparison Statistics (After Offset Correction) ===")

        # Only compare overlapping time periods
        common_start = max(gt_timestamps[0], stella_timestamps[0])
        common_end = min(gt_timestamps[-1], stella_timestamps[-1])

        # Filter to common time range
        gt_mask = (gt_timestamps >= common_start) & (gt_timestamps <= common_end)
        stella_mask = (stella_timestamps >= common_start) & (stella_timestamps <= common_end)

        if np.sum(gt_mask) > 1 and np.sum(stella_mask) > 1:
            try:
                # Interpolate CORRECTED Stella data to GT timestamps using numpy
                stella_x_interp = np.interp(gt_timestamps[gt_mask], stella_timestamps[stella_mask], stella_xs_corrected[stella_mask])
                stella_y_interp = np.interp(gt_timestamps[gt_mask], stella_timestamps[stella_mask], stella_ys_corrected[stella_mask])
                stella_z_interp = np.interp(gt_timestamps[gt_mask], stella_timestamps[stella_mask], stella_zs_corrected[stella_mask])

                # Calculate RMS errors with corrected data
                x_error = np.sqrt(np.mean((np.array(gt_xs)[gt_mask] - stella_x_interp)**2))
                y_error = np.sqrt(np.mean((np.array(gt_ys)[gt_mask] - stella_y_interp)**2))
                z_error = np.sqrt(np.mean((np.array(gt_zs)[gt_mask] - stella_z_interp)**2))

                print(f"Position RMS Error (Corrected):")
                print(f"  X: {x_error:.3f} m")
                print(f"  Y: {y_error:.3f} m")
                print(f"  Z: {z_error:.3f} m")

                # Also compute orientation errors with corrected data
                stella_roll_interp = np.interp(gt_timestamps[gt_mask], stella_timestamps[stella_mask], stella_rolls_corrected[stella_mask])
                stella_pitch_interp = np.interp(gt_timestamps[gt_mask], stella_timestamps[stella_mask], stella_pitches_corrected[stella_mask])
                stella_yaw_interp = np.interp(gt_timestamps[gt_mask], stella_timestamps[stella_mask], stella_yaws_corrected[stella_mask])

                roll_error = np.sqrt(np.mean((gt_rolls[gt_mask] - stella_roll_interp)**2))
                pitch_error = np.sqrt(np.mean((gt_pitches[gt_mask] - stella_pitch_interp)**2))
                yaw_error = np.sqrt(np.mean((gt_yaws[gt_mask] - stella_yaw_interp)**2))

                print(f"Orientation RMS Error (Corrected):")
                print(f"  Roll:  {roll_error:.2f} degrees")
                print(f"  Pitch: {pitch_error:.2f} degrees")
                print(f"  Yaw:   {yaw_error:.2f} degrees")

            except Exception as e:
                print(f"  Could not compute RMS error: {e}")
        else:
            print("  Not enough overlapping data for comparison")
