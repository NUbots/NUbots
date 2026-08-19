#!/usr/bin/env python3

import os
import sys

def iso3_to_matrix(iso3_msg):
    import numpy as np
    try:
        # Columns: x, y, z, t as vec4
        col0 = [iso3_msg.x.x, iso3_msg.x.y, iso3_msg.x.z, iso3_msg.x.t]
        col1 = [iso3_msg.y.x, iso3_msg.y.y, iso3_msg.y.z, iso3_msg.y.t]
        col2 = [iso3_msg.z.x, iso3_msg.z.y, iso3_msg.z.z, iso3_msg.z.t]
        col3 = [iso3_msg.t.x, iso3_msg.t.y, iso3_msg.t.z, iso3_msg.t.t]
        mat = np.column_stack((col0, col1, col2, col3))
    except AttributeError:
        # Fallback if vec4 has no t but w
        col0 = [iso3_msg.x.x, iso3_msg.x.y, iso3_msg.x.z, getattr(iso3_msg.x, 'w', getattr(iso3_msg.x, 't', 0))]
        col1 = [iso3_msg.y.x, iso3_msg.y.y, iso3_msg.y.z, getattr(iso3_msg.y, 'w', getattr(iso3_msg.y, 't', 0))]
        col2 = [iso3_msg.z.x, iso3_msg.z.y, iso3_msg.z.z, getattr(iso3_msg.z, 'w', getattr(iso3_msg.z, 't', 0))]
        col3 = [iso3_msg.t.x, iso3_msg.t.y, iso3_msg.t.z, getattr(iso3_msg.t, 'w', getattr(iso3_msg.t, 't', 0))]
        mat = np.column_stack((col0, col1, col2, col3))
    return mat

def get_pose_from_Htw(Htw_msg):
    import numpy as np
    Htw = iso3_to_matrix(Htw_msg)
    try:
        # Invert Htw to get Hwt (transform from Torso to World, meaning Torso's pose in World space)
        Hwt = np.linalg.inv(Htw)
    except np.linalg.LinAlgError:
        return None, None, None
    
    # Translation
    x = Hwt[0, 3]
    y = Hwt[1, 3]
    
    # Rotation (yaw)
    R = Hwt[0:3, 0:3]
    yaw = np.arctan2(R[1, 0], R[0, 0])
    
    return x, y, yaw

from utility.dockerise import run_on_docker

@run_on_docker
def register(command):
    command.description = "Plot trajectory comparison of ground truth vs neural network odometry estimate directly from an NBS file"
    command.add_argument("nbs_file", help="The nbs file containing OdometryRecord and Sensors messages")

@run_on_docker
def run(nbs_file, **kwargs):
    import numpy as np
    import matplotlib.pyplot as plt
    from utility.nbs import LinearDecoder

    if not os.path.isfile(nbs_file):
        print(f"Error: {nbs_file} not found.")
        return

    print(f"Opening NBS Dataset {nbs_file}...")

    gt_path_x = []
    gt_path_y = []
    gt_yaws = []
    gt_times = []

    pred_path_x = []
    pred_path_y = []
    pred_yaws = []
    pred_times = []
    
    start_time = None

    # Iterate through NBS dataset to load records
    for packet in LinearDecoder(nbs_file):
        
        # Ground Truth and Estimate from filtered Sensors
        if packet.type.name == "message.input.Sensors":
            msg = packet.msg
            t = msg.timestamp.seconds + msg.timestamp.nanos * 1e-9
            if start_time is None:
                start_time = t
            t_rel = t - start_time
            
            mat_gt = iso3_to_matrix(msg.Htw_ground_truth)
            mat_pred = iso3_to_matrix(msg.Htw)

            # Check if either matrix is essentially zero or contains NaNs
            if np.allclose(mat_gt, 0) or np.isnan(mat_gt).any() or np.allclose(mat_pred, 0) or np.isnan(mat_pred).any():
                continue
                
            x_gt, y_gt, yaw_gt = get_pose_from_Htw(msg.Htw_ground_truth)
            x, y, yaw = get_pose_from_Htw(msg.Htw)

            if x_gt is None or x is None or np.isnan(x_gt) or np.isnan(x):
                continue

            gt_path_x.append(x_gt)
            gt_path_y.append(y_gt)
            gt_yaws.append(yaw_gt)
            gt_times.append(t_rel)

            pred_path_x.append(x)
            pred_path_y.append(y)
            pred_yaws.append(yaw)
            pred_times.append(t_rel)

    print(f"Extracted {len(gt_times)} synchronized records from Sensors messages.")

    if len(gt_times) == 0:
        print("No valid records found in the dataset.")
        return

    # Convert to numpy arrays for calculation and plotting
    gt_path_x = np.array(gt_path_x)
    gt_path_y = np.array(gt_path_y)
    gt_yaws = np.degrees(np.unwrap(np.array(gt_yaws)))
    gt_times = np.array(gt_times)

    pred_path_x = np.array(pred_path_x)
    pred_path_y = np.array(pred_path_y)
    pred_yaws = np.degrees(np.unwrap(np.array(pred_yaws)))
    pred_times = np.array(pred_times)
    
    # Calculate Overall Metrics
    ate_rmse = np.sqrt(np.mean((gt_path_x - pred_path_x)**2 + (gt_path_y - pred_path_y)**2))
    yaw_rmse = np.sqrt(np.mean((gt_yaws - pred_yaws)**2))
    
    print("\n================ Trajectory Metrics ================")
    print(f"Absolute Trajectory Error (ATE RMSE):  {ate_rmse:.4f} m")
    print(f"Absolute Yaw Error (Yaw RMSE):         {yaw_rmse:.4f} deg")
    print("====================================================\n")

    # Generate premium layout plots
    fig, (ax_traj, ax_yaw) = plt.subplots(2, 1, figsize=(12, 10))

    # 1. X-Y 2D Path Plot
    if len(gt_times) > 0:
        ax_traj.plot(gt_path_x, gt_path_y, color='#1f77b4', linewidth=2.5, label='Ground Truth')
        ax_traj.scatter(gt_path_x[0], gt_path_y[0], color='green', marker='o', s=100, label='Start Point', zorder=5)
    
    if len(pred_times) > 0:
        ax_traj.plot(pred_path_x, pred_path_y, color='#ff7f0e', linewidth=2.0, linestyle='--', label='Estimate (Sensors filter)')
        if len(gt_times) == 0:
             ax_traj.scatter(pred_path_x[0], pred_path_y[0], color='green', marker='o', s=100, label='Start Point', zorder=5)

    ax_traj.set_xlabel('X Position (m)', fontsize=12)
    ax_traj.set_ylabel('Y Position (m)', fontsize=12)
    ax_traj.set_title(f'2D Absolute Trajectory | ATE RMSE: {ate_rmse:.4f}m', fontsize=14, fontweight='bold')
    ax_traj.grid(True, linestyle=':', alpha=0.6)
    ax_traj.legend(fontsize=11)
    ax_traj.axis('equal')

    # 2. Time-based Yaw angle Plot
    if len(gt_times) > 0:
        ax_yaw.plot(gt_times, gt_yaws, color='#1f77b4', linewidth=2.0, label='Ground Truth')
    
    if len(pred_times) > 0:
        ax_yaw.plot(pred_times, pred_yaws, color='#ff7f0e', linewidth=1.8, linestyle='--', label='Estimate (Sensors filter)')

    ax_yaw.set_xlabel('Time (seconds)', fontsize=12)
    ax_yaw.set_ylabel('Yaw Angle (degrees)', fontsize=12)
    ax_yaw.set_title(f'Yaw Orientation Over Time | Yaw RMSE: {yaw_rmse:.4f}°', fontsize=14, fontweight='bold')
    ax_yaw.grid(True, linestyle=':', alpha=0.6)
    ax_yaw.legend(fontsize=11)

    plt.tight_layout()
    output_plot_path = "trajectory_comparison.png"
    plt.savefig(output_plot_path, dpi=300)
    print(f"Stunning comparison plot saved to: {output_plot_path}")
    
    # Try displaying plot if in interactive environment
    try:
        plt.show()
    except Exception:
        pass

if __name__ == '__main__':
    # Fallback to direct run if not run through b command line
    if len(sys.argv) < 2:
        print("Usage: python3 plot_trajectory.py <nbs_file>")
        sys.exit(1)
    
    nbs = sys.argv[1]
    run(nbs_file=nbs)
