#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import os
from utility.nbs import LinearDecoder

def register(command):
    command.description = "Compare Ground Truth, Stella SLAM, and IMU+Kinematics poses from an NBS file"

    # Command arguments
    command.add_argument(
        "nbs_file", metavar="nbs_file", help="The NBS file containing Ground Truth, Stella, and IMU+Kinematics data"
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
        print(f"Output directory created: {output}")
    else:
        print("No output directory specified - figures will be displayed only")

    print("Loading Sensor data...")
    # Load all sensor messages (contains Htw_gt, Htw_imukin, and Htw)
    sensors_decoder = LinearDecoder(nbs_file, types=["message.input.Sensors"], show_progress=True)
    print(f"Found {len(sensors_decoder)} sensor messages")

    # Data extraction for all three pose estimates
    gt_timestamps = []
    gt_xs, gt_ys, gt_zs = [], [], []
    gt_rolls, gt_pitches, gt_yaws = [], [], []

    stella_timestamps = []
    stella_xs, stella_ys, stella_zs = [], [], []
    stella_rolls, stella_pitches, stella_yaws = [], [], []

    imukin_timestamps = []
    imukin_xs, imukin_ys, imukin_zs = [], [], []
    imukin_rolls, imukin_pitches, imukin_yaws = [], [], []

    # Find the earliest timestamp to establish time reference
    earliest_timestamp = None
    for packet in sensors_decoder:
        timestamp_sec = packet.index_timestamp / 1e9
        if earliest_timestamp is None or timestamp_sec < earliest_timestamp:
            earliest_timestamp = timestamp_sec

    # Reset decoder to beginning
    sensors_decoder = LinearDecoder(nbs_file, types=["message.input.Sensors"], show_progress=False)

    for packet in sensors_decoder:
        msg = packet.msg
        timestamp_sec = packet.index_timestamp / 1e9

        # Skip first 8 seconds of data
        if timestamp_sec - earliest_timestamp < 10.0:
            continue

        # Extract Ground Truth pose from Htw_gt
        if hasattr(msg, 'Htw_gt') and msg.HasField('Htw_gt'):
            gt_timestamps.append(timestamp_sec)
            position, orientation = extract_pose_from_transform(msg.Htw_gt)
            gt_xs.append(position[0])
            gt_ys.append(position[1])
            gt_zs.append(position[2])
            gt_rolls.append(orientation[0])
            gt_pitches.append(orientation[1])
            gt_yaws.append(orientation[2])

        # Extract Stella pose from Htw (sliding window output)
        if hasattr(msg, 'Htw') and msg.HasField('Htw'):
            stella_timestamps.append(timestamp_sec)
            position, orientation = extract_pose_from_transform(msg.Htw)
            stella_xs.append(position[0])
            stella_ys.append(position[1])
            stella_zs.append(position[2])
            stella_rolls.append(orientation[0])
            stella_pitches.append(orientation[1])
            stella_yaws.append(orientation[2])

        # Extract IMU+Kinematics pose from Htw_imukin
        if hasattr(msg, 'Htw_imukin') and msg.HasField('Htw_imukin'):
            imukin_timestamps.append(timestamp_sec)
            position, orientation = extract_pose_from_transform(msg.Htw_imukin)
            imukin_xs.append(position[0])
            imukin_ys.append(position[1])
            imukin_zs.append(position[2])
            imukin_rolls.append(orientation[0])
            imukin_pitches.append(orientation[1])
            imukin_yaws.append(orientation[2])

    # Convert to numpy arrays and normalize timestamps
    gt_timestamps = np.array(gt_timestamps)
    stella_timestamps = np.array(stella_timestamps)
    imukin_timestamps = np.array(imukin_timestamps)

    # Find common time reference (earliest timestamp)
    timestamps = [gt_timestamps, stella_timestamps, imukin_timestamps]
    min_time = min(ts[0] if len(ts) > 0 else float('inf') for ts in timestamps)

    gt_timestamps = gt_timestamps - min_time
    stella_timestamps = stella_timestamps - min_time
    imukin_timestamps = imukin_timestamps - min_time

    # Convert to numpy arrays and unwrap angles
    gt_rolls = unwrap_angles(np.array(gt_rolls))
    gt_pitches = unwrap_angles(np.array(gt_pitches))
    gt_yaws = unwrap_angles(np.array(gt_yaws))

    stella_rolls = unwrap_angles(np.array(stella_rolls))
    stella_pitches = unwrap_angles(np.array(stella_pitches))
    stella_yaws = unwrap_angles(np.array(stella_yaws))

    imukin_rolls = unwrap_angles(np.array(imukin_rolls))
    imukin_pitches = unwrap_angles(np.array(imukin_pitches))
    imukin_yaws = unwrap_angles(np.array(imukin_yaws))

    print(f"Skipped first 8 seconds of data")
    print(f"Ground Truth: {len(gt_timestamps)} samples over {gt_timestamps[-1]:.2f} seconds" if len(gt_timestamps) > 0 else "Ground Truth: 0 samples")
    print(f"Stella: {len(stella_timestamps)} samples over {stella_timestamps[-1]:.2f} seconds" if len(stella_timestamps) > 0 else "Stella: 0 samples")
    print(f"IMU+Kinematics: {len(imukin_timestamps)} samples over {imukin_timestamps[-1]:.2f} seconds" if len(imukin_timestamps) > 0 else "IMU+Kinematics: 0 samples")

    # Compute offset from first 100 samples (Stella vs Ground Truth) - after 8-second skip
    n_samples = 100
    stella_xs_corrected = np.array(stella_xs)
    stella_ys_corrected = np.array(stella_ys)
    stella_zs_corrected = np.array(stella_zs)
    stella_rolls_corrected = stella_rolls
    stella_pitches_corrected = stella_pitches
    stella_yaws_corrected = stella_yaws

    # Initialize IMU+Kinematics corrected arrays
    imukin_xs_corrected = np.array(imukin_xs)
    imukin_ys_corrected = np.array(imukin_ys)
    imukin_zs_corrected = np.array(imukin_zs)
    imukin_rolls_corrected = imukin_rolls
    imukin_pitches_corrected = imukin_pitches
    imukin_yaws_corrected = imukin_yaws

    if len(gt_xs) >= n_samples and len(stella_xs) >= n_samples:
        print(f"\nComputing offset from first {n_samples} samples (Stella vs Ground Truth) - after 8-second skip...")

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

        print(f"Stella Position offset: X={position_offset[0]:.3f}, Y={position_offset[1]:.3f}, Z={position_offset[2]:.3f} m")
        print(f"Stella Orientation offset: Roll={orientation_offset[0]:.2f}, Pitch={orientation_offset[1]:.2f}, Yaw={orientation_offset[2]:.2f} degrees")

        # Apply offset to Stella data
        stella_xs_corrected = np.array(stella_xs) + position_offset[0]
        stella_ys_corrected = np.array(stella_ys) + position_offset[1]
        stella_zs_corrected = np.array(stella_zs) + position_offset[2]

        stella_rolls_corrected = stella_rolls + orientation_offset[0]
        stella_pitches_corrected = stella_pitches + orientation_offset[1]
        stella_yaws_corrected = stella_yaws + orientation_offset[2]

        # Apply the SAME offset to IMU+Kinematics data for fair comparison
        if len(imukin_xs) > 0:
            imukin_xs_corrected = np.array(imukin_xs) + position_offset[0]
            imukin_ys_corrected = np.array(imukin_ys) + position_offset[1]
            imukin_zs_corrected = np.array(imukin_zs) + position_offset[2]

            imukin_rolls_corrected = imukin_rolls + orientation_offset[0]
            imukin_pitches_corrected = imukin_pitches + orientation_offset[1]
            imukin_yaws_corrected = imukin_yaws + orientation_offset[2]

    else:
        print(f"\nNot enough samples for Stella offset calculation (need {n_samples}, got GT:{len(gt_xs)}, Stella:{len(stella_xs)})")

    # Plot translations comparison
    plt.figure(figsize=(15, 10))

    plt.subplot(3, 1, 1)
    if len(gt_timestamps) > 0:
        plt.plot(gt_timestamps, gt_xs, 'r-', label="Ground Truth", linewidth=2)
    if len(stella_timestamps) > 0:
        plt.plot(stella_timestamps, stella_xs_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
        # plt.plot(stella_timestamps, stella_xs, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    if len(imukin_timestamps) > 0:
        plt.plot(imukin_timestamps, imukin_xs_corrected, 'm-', label="IMU+Kinematics (Corrected)", linewidth=2)
        # plt.plot(imukin_timestamps, imukin_xs, 'c:', label="IMU+Kinematics (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("X Position [m]")
    plt.title("Position Comparison: Ground Truth vs Stella vs IMU+Kinematics")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 2)
    if len(gt_timestamps) > 0:
        plt.plot(gt_timestamps, gt_ys, 'r-', label="Ground Truth", linewidth=2)
    if len(stella_timestamps) > 0:
        plt.plot(stella_timestamps, stella_ys_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
        # plt.plot(stella_timestamps, stella_ys, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    if len(imukin_timestamps) > 0:
        plt.plot(imukin_timestamps, imukin_ys_corrected, 'm-', label="IMU+Kinematics (Corrected)", linewidth=2)
        # plt.plot(imukin_timestamps, imukin_ys, 'c:', label="IMU+Kinematics (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("Y Position [m]")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 3)
    if len(gt_timestamps) > 0:
        plt.plot(gt_timestamps, gt_zs, 'r-', label="Ground Truth", linewidth=2)
    if len(stella_timestamps) > 0:
        plt.plot(stella_timestamps, stella_zs_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
        # plt.plot(stella_timestamps, stella_zs, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    if len(imukin_timestamps) > 0:
        plt.plot(imukin_timestamps, imukin_zs_corrected, 'm-', label="IMU+Kinematics (Corrected)", linewidth=2)
        # plt.plot(imukin_timestamps, imukin_zs, 'c:', label="IMU+Kinematics (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("Z Position [m]")
    plt.xlabel("Time [s]")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()

    if output:
        position_fig_path = os.path.join(output, "01_position_comparison.png")
        plt.savefig(position_fig_path, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
        plt.close()
        print(f"✓ Saved position comparison plot: {position_fig_path}")
    else:
        plt.show()

    # Plot orientation comparison
    plt.figure(figsize=(15, 10))

    plt.subplot(3, 1, 1)
    if len(gt_timestamps) > 0:
        plt.plot(gt_timestamps, gt_rolls, 'r-', label="Ground Truth", linewidth=2)
    if len(stella_timestamps) > 0:
        plt.plot(stella_timestamps, stella_rolls_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
        # plt.plot(stella_timestamps, stella_rolls, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    if len(imukin_timestamps) > 0:
        plt.plot(imukin_timestamps, imukin_rolls_corrected, 'm-', label="IMU+Kinematics (Corrected)", linewidth=2)
        # plt.plot(imukin_timestamps, imukin_rolls, 'c:', label="IMU+Kinematics (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("Roll [degrees]")
    plt.title("Orientation Comparison: Ground Truth vs Stella vs IMU+Kinematics")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 2)
    if len(gt_timestamps) > 0:
        plt.plot(gt_timestamps, gt_pitches, 'r-', label="Ground Truth", linewidth=2)
    if len(stella_timestamps) > 0:
        plt.plot(stella_timestamps, stella_pitches_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
        # plt.plot(stella_timestamps, stella_pitches, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    if len(imukin_timestamps) > 0:
        plt.plot(imukin_timestamps, imukin_pitches_corrected, 'm-', label="IMU+Kinematics (Corrected)", linewidth=2)
        # plt.plot(imukin_timestamps, imukin_pitches, 'c:', label="IMU+Kinematics (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("Pitch [degrees]")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.subplot(3, 1, 3)
    if len(gt_timestamps) > 0:
        plt.plot(gt_timestamps, gt_yaws, 'r-', label="Ground Truth", linewidth=2)
    if len(stella_timestamps) > 0:
        plt.plot(stella_timestamps, stella_yaws_corrected, 'b--', label="Stella (Corrected)", linewidth=2)
        # plt.plot(stella_timestamps, stella_yaws, 'g:', label="Stella (Original)", linewidth=1, alpha=0.7)
    if len(imukin_timestamps) > 0:
        plt.plot(imukin_timestamps, imukin_yaws_corrected, 'm-', label="IMU+Kinematics (Corrected)", linewidth=2)
        # plt.plot(imukin_timestamps, imukin_yaws, 'c:', label="IMU+Kinematics (Original)", linewidth=1, alpha=0.7)
    plt.ylabel("Yaw [degrees]")
    plt.xlabel("Time [s]")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()

    if output:
        orientation_fig_path = os.path.join(output, "02_orientation_comparison.png")
        plt.savefig(orientation_fig_path, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
        plt.close()
        print(f"✓ Saved orientation comparison plot: {orientation_fig_path}")
    else:
        plt.show()

    # Plot 2D trajectory comparison
    plt.figure(figsize=(10, 8))
    if len(gt_xs) > 0:
        plt.plot(gt_xs, gt_ys, 'r-', label="Ground Truth", linewidth=2, alpha=0.8)
        plt.scatter(gt_xs[0], gt_ys[0], color='red', s=100, label='GT Start', zorder=5)
    if len(stella_xs_corrected) > 0:
        plt.plot(stella_xs_corrected, stella_ys_corrected, 'b--', label="Stella (Corrected)", linewidth=2, alpha=0.8)
        plt.scatter(stella_xs_corrected[0], stella_ys_corrected[0], color='blue', s=100, label='Stella Start', zorder=5)
    # if len(stella_xs) > 0:
    #     plt.plot(stella_xs, stella_ys, 'g:', label="Stella (Original)", linewidth=1, alpha=0.5)
    if len(imukin_xs_corrected) > 0:
        plt.plot(imukin_xs_corrected, imukin_ys_corrected, 'm-', label="IMU+Kinematics (Corrected)", linewidth=2, alpha=0.8)
        plt.scatter(imukin_xs_corrected[0], imukin_ys_corrected[0], color='magenta', s=100, label='IMU+Kin Start', zorder=5)
    # if len(imukin_xs) > 0:
    #     plt.plot(imukin_xs, imukin_ys, 'c:', label="IMU+Kinematics (Original)", linewidth=1, alpha=0.5)
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.title('2D Trajectory Comparison: Ground Truth vs Stella vs IMU+Kinematics')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend()

    if output:
        trajectory_fig_path = os.path.join(output, "03_trajectory_comparison.png")
        plt.savefig(trajectory_fig_path, dpi=300, bbox_inches='tight', facecolor='white', edgecolor='none')
        plt.close()
        print(f"✓ Saved trajectory comparison plot: {trajectory_fig_path}")
    else:
        plt.show()

    # Create a summary report
    if output:
        summary_path = os.path.join(output, "00_summary_report.txt")
        with open(summary_path, 'w') as f:
            f.write("Stella SLAM vs Ground Truth Comparison Report\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"Data file: {nbs_file}\n")
            f.write(f"Generated: {os.path.basename(output)}\n\n")

            f.write("Generated Figures:\n")
            f.write("- 01_position_comparison.png: X, Y, Z position over time\n")
            f.write("- 02_orientation_comparison.png: Roll, Pitch, Yaw over time\n")
            f.write("- 03_trajectory_comparison.png: 2D trajectory comparison\n\n")

            f.write("Data Summary:\n")
            f.write(f"Ground Truth: {len(gt_timestamps)} samples over {gt_timestamps[-1]:.2f} seconds\n" if len(gt_timestamps) > 0 else "Ground Truth: 0 samples\n")
            f.write(f"Stella: {len(stella_timestamps)} samples over {stella_timestamps[-1]:.2f} seconds\n" if len(stella_timestamps) > 0 else "Stella: 0 samples\n")
            f.write(f"IMU+Kinematics: {len(imukin_timestamps)} samples over {imukin_timestamps[-1]:.2f} seconds\n" if len(imukin_timestamps) > 0 else "IMU+Kinematics: 0 samples\n")

        print(f"✓ Saved summary report: {summary_path}")
        print(f"\n📁 All figures saved to: {output}")
        print(f"📊 Generated {3} figures total")

    # Print comparison statistics
    print(f"\n=== Comparison Statistics ===")

    # Compare Stella vs Ground Truth
    if len(gt_xs) > 0 and len(stella_xs_corrected) > 0:
        print(f"\n--- Stella vs Ground Truth ---")

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

                print(f"Stella Position RMS Error:")
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

                print(f"Stella Orientation RMS Error:")
                print(f"  Roll:  {roll_error:.2f} degrees")
                print(f"  Pitch: {pitch_error:.2f} degrees")
                print(f"  Yaw:   {yaw_error:.2f} degrees")

            except Exception as e:
                print(f"  Could not compute Stella RMS error: {e}")
        else:
            print("  Not enough overlapping data for Stella comparison")

    # Compare IMU+Kinematics vs Ground Truth
    if len(gt_xs) > 0 and len(imukin_xs_corrected) > 0:
        print(f"\n--- IMU+Kinematics vs Ground Truth ---")

        # Only compare overlapping time periods
        common_start = max(gt_timestamps[0], imukin_timestamps[0])
        common_end = min(gt_timestamps[-1], imukin_timestamps[-1])

        # Filter to common time range
        gt_mask = (gt_timestamps >= common_start) & (gt_timestamps <= common_end)
        imukin_mask = (imukin_timestamps >= common_start) & (imukin_timestamps <= common_end)

        if np.sum(gt_mask) > 1 and np.sum(imukin_mask) > 1:
            try:
                # Interpolate CORRECTED IMU+Kinematics data to GT timestamps using numpy
                imukin_x_interp = np.interp(gt_timestamps[gt_mask], imukin_timestamps[imukin_mask], imukin_xs_corrected[imukin_mask])
                imukin_y_interp = np.interp(gt_timestamps[gt_mask], imukin_timestamps[imukin_mask], imukin_ys_corrected[imukin_mask])
                imukin_z_interp = np.interp(gt_timestamps[gt_mask], imukin_timestamps[imukin_mask], imukin_zs_corrected[imukin_mask])

                # Calculate RMS errors
                x_error = np.sqrt(np.mean((np.array(gt_xs)[gt_mask] - imukin_x_interp)**2))
                y_error = np.sqrt(np.mean((np.array(gt_ys)[gt_mask] - imukin_y_interp)**2))
                z_error = np.sqrt(np.mean((np.array(gt_zs)[gt_mask] - imukin_z_interp)**2))

                print(f"IMU+Kinematics Position RMS Error:")
                print(f"  X: {x_error:.3f} m")
                print(f"  Y: {y_error:.3f} m")
                print(f"  Z: {z_error:.3f} m")

                # Also compute orientation errors
                imukin_roll_interp = np.interp(gt_timestamps[gt_mask], imukin_timestamps[imukin_mask], imukin_rolls_corrected[imukin_mask])
                imukin_pitch_interp = np.interp(gt_timestamps[gt_mask], imukin_timestamps[imukin_mask], imukin_pitches_corrected[imukin_mask])
                imukin_yaw_interp = np.interp(gt_timestamps[gt_mask], imukin_timestamps[imukin_mask], imukin_yaws_corrected[imukin_mask])

                roll_error = np.sqrt(np.mean((gt_rolls[gt_mask] - imukin_roll_interp)**2))
                pitch_error = np.sqrt(np.mean((gt_pitches[gt_mask] - imukin_pitch_interp)**2))
                yaw_error = np.sqrt(np.mean((gt_yaws[gt_mask] - imukin_yaw_interp)**2))

                print(f"IMU+Kinematics Orientation RMS Error:")
                print(f"  Roll:  {roll_error:.2f} degrees")
                print(f"  Pitch: {pitch_error:.2f} degrees")
                print(f"  Yaw:   {yaw_error:.2f} degrees")

            except Exception as e:
                print(f"  Could not compute IMU+Kinematics RMS error: {e}")
        else:
            print("  Not enough overlapping data for IMU+Kinematics comparison")
