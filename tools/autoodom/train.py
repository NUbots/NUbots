#!/usr/bin/env python3

import math
import os
import sys

# Imports delayed to run in docker where deps are met

def get_model_class():
    import torch
    import torch.nn as nn

    class ResBlock1D(nn.Module):
        def __init__(self, channels):
            super().__init__()
            self.conv1 = nn.Conv1d(channels, channels, kernel_size=3, padding=1)
            self.bn1 = nn.BatchNorm1d(channels)
            self.relu = nn.ReLU()
            self.conv2 = nn.Conv1d(channels, channels, kernel_size=3, padding=1)
            self.bn2 = nn.BatchNorm1d(channels)

        def forward(self, x):
            residual = x
            out = self.conv1(x)
            out = self.bn1(out)
            out = self.relu(out)
            out = self.conv2(out)
            out = self.bn2(out)
            out += residual
            out = self.relu(out)
            return out

    class AutoOdomResNet(nn.Module):
        def __init__(self, input_dim, hidden_dim=64, num_layers=3, mean=None, std=None, target_mean=None, target_std=None):
            super().__init__()
            self.input_dim = input_dim

            if mean is None:
                mean = torch.zeros(input_dim)
            if std is None:
                std = torch.ones(input_dim)
            if target_mean is None:
                target_mean = torch.zeros(3)
            if target_std is None:
                target_std = torch.ones(3)

            self.register_buffer("mean", torch.tensor(mean, dtype=torch.float32))
            self.register_buffer("std", torch.tensor(std, dtype=torch.float32))
            self.register_buffer("target_mean", torch.tensor(target_mean, dtype=torch.float32))
            self.register_buffer("target_std", torch.tensor(target_std, dtype=torch.float32))

            # ResNet feature extractor
            layers = [
                nn.Conv1d(input_dim, hidden_dim, kernel_size=5, padding=2),
                nn.BatchNorm1d(hidden_dim),
                nn.ReLU()
            ]
            for _ in range(num_layers):
                layers.append(ResBlock1D(hidden_dim))

            layers.append(nn.AdaptiveAvgPool1d(1)) # Global Average Pooling
            self.net = nn.Sequential(*layers)

            self.fc = nn.Sequential(
                nn.Linear(hidden_dim, 64),
                nn.ReLU(),
                nn.Linear(64, 3) # Output: dx, dy, dtheta
            )

        def forward(self, x):
            # Reshape inputs to [batch, seq_len, features]
            if len(x.shape) == 2:
                x = x.view(x.size(0), -1, self.input_dim)

            x_norm = (x - self.mean) / (self.std + 1e-8)

            # Conv1D expects [batch, features, seq_len]
            x_norm = x_norm.transpose(1, 2)

            features = self.net(x_norm) # [batch, hidden_dim, 1]
            features = features.squeeze(2) # [batch, hidden_dim]

            out = self.fc(features)

            if not self.training:
                # Scale back to real-world units during inference/ONNX export
                out = out * self.target_std + self.target_mean

            return out
    return AutoOdomResNet

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

def get_world_translation(Htw_msg):
    import numpy as np
    # Htw is transform from World to Torso. Hwt is transform from Torso to World.
    # The Torso's position in the world is the translation component of Hwt.
    Htw = iso3_to_matrix(Htw_msg)
    Hwt = np.linalg.inv(Htw)
    return Hwt[0:2, 3] # (x, y) translation in world frame

def get_world_rotation(Htw_msg):
    import numpy as np
    Htw = iso3_to_matrix(Htw_msg)
    Hwt = np.linalg.inv(Htw)
    return Hwt[0:3, 0:3]

def compute_local_step_displacement(Htw_gt_t, Htw_gt_t_minus_1):
    import numpy as np
    p_w_t = get_world_translation(Htw_gt_t)
    p_w_t_minus_1 = get_world_translation(Htw_gt_t_minus_1)

    # World displacement
    disp_w = p_w_t - p_w_t_minus_1

    # Local rotation
    R_w_t = get_world_rotation(Htw_gt_t)
    R_w_t_minus_1 = get_world_rotation(Htw_gt_t_minus_1)

    # Calculate yaw change dtheta
    yaw_t = np.arctan2(R_w_t[1, 0], R_w_t[0, 0])
    yaw_t_minus_1 = np.arctan2(R_w_t_minus_1[1, 0], R_w_t_minus_1[0, 0])

    # Wrap difference to [-pi, pi]
    dtheta = yaw_t - yaw_t_minus_1
    dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi

    # Create a yaw-only rotation matrix for the 2D local frame (heading frame)
    R_yaw_t_minus_1 = np.array([
        [np.cos(yaw_t_minus_1), -np.sin(yaw_t_minus_1), 0.0],
        [np.sin(yaw_t_minus_1),  np.cos(yaw_t_minus_1), 0.0],
        [0.0, 0.0, 1.0]
    ])

    # Rotate world displacement into the 2D heading frame of t-1
    disp_local = R_yaw_t_minus_1.T @ np.array([disp_w[0], disp_w[1], 0.0])

    return np.array([disp_local[0], disp_local[1], dtheta])

from utility.dockerise import run_on_docker

@run_on_docker(default_gpus="all")
def register(command):
    command.description = "Train a GRU to predict odometry x-y translation from an NBS dataset"
    command.add_argument("nbs_file", help="The nbs file containing OdometryRecord messages")
    command.add_argument("--epochs", type=int, default=100, help="Number of training epochs")
    command.add_argument("--batch-size", type=int, default=32, help="Training batch size")
    command.add_argument("--window-size", type=int, default=50, help="Window size (H)")
    command.add_argument("--rollout-length", type=int, default=100, help="Number of consecutive steps for trajectory rollout loss (100=2s at 50Hz)")
    command.add_argument("--lambda-traj", type=float, default=5.0, help="Weight for multi-step trajectory rollout loss")
    command.add_argument("--lambda-bias", type=float, default=0.5, help="Weight for zero-mean bias penalty")
    command.add_argument("--warmup-epochs", type=int, default=20, help="Epochs of per-step-only training before rollout loss kicks in")
    command.add_argument("--evaluate", action="store_true", help="Evaluate the model on the test split and plot")
    command.add_argument("--export-onnx", action="store_true", help="Export the trained model to ONNX format")

@run_on_docker
def run(nbs_file, epochs, batch_size, window_size, rollout_length=100, lambda_traj=5.0, lambda_bias=0.5, warmup_epochs=20, evaluate=False, export_onnx=False, **kwargs):
    import numpy as np
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import DataLoader, TensorDataset, Dataset
    from utility.nbs import LinearDecoder

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    if not os.path.isfile(nbs_file):
        print(f"Error: {nbs_file} not found.")
        return

    print(f"Opening NBS Dataset {nbs_file} with window size {window_size}...")

    raw_features = []
    raw_positions = []
    raw_timestamps = []

    # Iterate through NBS dataset
    for packet in LinearDecoder(nbs_file):
        if packet.type.name == "message.localisation.OdometryRecord":
            msg = packet.msg

            # Pack observation vector
            fv = []
            fv.extend([msg.velocity_target.x, msg.velocity_target.y, msg.velocity_target.z])
            fv.extend([msg.accelerometer.x, msg.accelerometer.y, msg.accelerometer.z])
            fv.extend([msg.gyroscope.x, msg.gyroscope.y, msg.gyroscope.z])

            # The list of up to 20 joints
            fv.extend(list(msg.present_position))
            fv.extend(list(msg.present_velocity))
            fv.extend(list(msg.goal_position))

            raw_features.append(fv)
            raw_positions.append(msg.Htw_ground_truth)
            raw_timestamps.append(packet.index_timestamp)

    total_records = len(raw_features)
    print(f"Extracted {total_records} OdometryRecords.")

    if total_records < window_size + 2:
        print("Not enough records in the dataset to form a single window and displacement step.")
        return

    # Detect gaps caused by mocap tracking dropouts (OdometryLogger only emits when
    # a valid rigid body is found, so lost tracking produces no records for those frames).
    EXPECTED_PERIOD_NS = 20_000_000   # 20 ms at 50 Hz
    GAP_THRESHOLD_NS   = 3 * EXPECTED_PERIOD_NS  # 60 ms = clearly missing frames

    timestamps = np.array(raw_timestamps, dtype=np.int64)
    is_gap = np.zeros(total_records, dtype=bool)
    if total_records > 1:
        is_gap[1:] = (timestamps[1:] - timestamps[:-1]) > GAP_THRESHOLD_NS
    segment_ids = np.cumsum(is_gap)   # integer segment ID per record

    num_gaps = int(is_gap.sum())
    if num_gaps > 0:
        print(f"Detected {num_gaps} mocap tracking gap(s) — windows crossing gaps will be excluded.")

    # Calculate dataset mean and std for normalization
    all_features = np.array(raw_features)
    mean_val = np.mean(all_features, axis=0)
    std_val = np.std(all_features, axis=0)
    std_val[std_val == 0.0] = 1.0

    # Build windows, skipping any that span a mocap tracking gap
    X = []
    Y = []
    window_seg_ids = []
    for i in range(total_records - window_size):
        # Skip if any record in [i+1, i+window_size-1] follows a gap
        if is_gap[i + 1 : i + window_size].any():
            continue

        # A window is a flattened array of size H * feature_dim
        window = raw_features[i : i + window_size]
        flattened_window = np.concatenate(window)

        # Ground truth delta position that occurred during the final frame of the window
        gt_disp = compute_local_step_displacement(
            raw_positions[i + window_size - 1],
            raw_positions[i + window_size - 2]
        )

        X.append(flattened_window)
        Y.append(gt_disp)
        window_seg_ids.append(segment_ids[i])

    window_seg_ids = np.array(window_seg_ids)

    X = torch.tensor(np.array(X), dtype=torch.float32)

    Y_raw = np.array(Y)
    target_mean_val = np.mean(Y_raw, axis=0)
    target_std_val = np.std(Y_raw, axis=0)
    target_std_val[target_std_val == 0.0] = 1.0

    Y_raw_tensor = torch.tensor(Y_raw, dtype=torch.float32)
    Y_norm_tensor = (Y_raw_tensor - target_mean_val) / target_std_val

    print(f"Dataset compiled. Shape X: {X.shape}, Shape Y: {Y_raw_tensor.shape}")

    # Chronological Train/Test Split (80% / 20%) to prevent data leakage
    split_idx = int(len(X) * 0.8)
    X_train, Y_train = X[:split_idx], Y_norm_tensor[:split_idx]
    X_test, Y_eval = X[split_idx:], Y_raw_tensor[split_idx:]
    seg_ids_train = window_seg_ids[:split_idx]
    print(f"Train split: {len(X_train)} samples | Test split: {len(X_test)} samples")

    AutoOdomModel = get_model_class()
    feature_dim = X.shape[1] // window_size
    model = AutoOdomModel(
        input_dim=feature_dim,
        hidden_dim=256,
        mean=mean_val,
        std=std_val,
        target_mean=target_mean_val,
        target_std=target_std_val
    ).to(device)

    if export_onnx:
        save_dir = os.path.join("module", "input", "SensorFilter", "data")
        model_path = os.path.join(save_dir, "autoodom_mlp.pt")
        if not os.path.exists(model_path):
            print(f"Error: Model not found at {model_path}")
            return

        # Load state dict
        model.load_state_dict(torch.load(model_path))
        model.eval()

        onnx_path = os.path.join(save_dir, "autoodom_mlp.onnx")
        print(f"Exporting model to {onnx_path}...")

        # Create a dummy input tensor on the correct device
        dummy_input = torch.randn(1, X.shape[1]).to(device)
        torch.onnx.export(
            model,
            dummy_input,
            onnx_path,
            export_params=True,
            opset_version=11,
            do_constant_folding=True,
            input_names=['input'],
            output_names=['output'],
            dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}
        )
        print("ONNX export complete.")
        return

    if evaluate:
        save_dir = os.path.join("module", "input", "SensorFilter", "data")
        model_path = os.path.join(save_dir, "autoodom_mlp.pt")
        if not os.path.exists(model_path):
            print(f"Error: Model not found at {model_path}")
            return

        model.load_state_dict(torch.load(model_path))
        model.eval()
        print("Evaluating model on the Test Set...")

        with torch.no_grad():
            # In eval mode, outputs are unnormalized automatically
            predictions = model(X_test.to(device)).cpu()
            # Y_eval is already in real-world units

        # Integrate step-by-step in the world frame using the integrated yaw orientations
        gt_path_x = [0.0]
        gt_path_y = [0.0]
        pred_path_x = [0.0]
        pred_path_y = [0.0]

        gt_yaws = [0.0]
        pred_yaws = [0.0]

        gt_dxs = []
        gt_dys = []
        gt_dthetas = []

        pred_dxs = []
        pred_dys = []
        pred_dthetas = []

        gt_yaw = 0.0
        pred_yaw = 0.0

        for i in range(len(Y_eval)):
            # Ground truth step integration
            gt_dx_local = Y_eval[i][0].item()
            gt_dy_local = Y_eval[i][1].item()
            gt_dtheta = Y_eval[i][2].item()

            gt_dxs.append(gt_dx_local)
            gt_dys.append(gt_dy_local)
            gt_dthetas.append(gt_dtheta)

            gt_yaw += gt_dtheta
            gt_yaws.append(gt_yaw)
            R_w_gt = np.array([
                [np.cos(gt_yaw), -np.sin(gt_yaw), 0.0],
                [np.sin(gt_yaw), np.cos(gt_yaw), 0.0],
                [0.0, 0.0, 1.0]
            ])
            gt_disp_w = R_w_gt @ np.array([gt_dx_local, gt_dy_local, 0.0])
            gt_path_x.append(gt_path_x[-1] + gt_disp_w[0])
            gt_path_y.append(gt_path_y[-1] + gt_disp_w[1])

            # Predicted step integration
            pred_dx_local = predictions[i][0].item()
            pred_dy_local = predictions[i][1].item()
            pred_dtheta = predictions[i][2].item()

            pred_dxs.append(pred_dx_local)
            pred_dys.append(pred_dy_local)
            pred_dthetas.append(pred_dtheta)

            pred_yaw += pred_dtheta
            pred_yaws.append(pred_yaw)
            R_w_pred = np.array([
                [np.cos(pred_yaw), -np.sin(pred_yaw), 0.0],
                [np.sin(pred_yaw), np.cos(pred_yaw), 0.0],
                [0.0, 0.0, 1.0]
            ])
            pred_disp_w = R_w_pred @ np.array([pred_dx_local, pred_dy_local, 0.0])
            pred_path_x.append(pred_path_x[-1] + pred_disp_w[0])
            pred_path_y.append(pred_path_y[-1] + pred_disp_w[1])

        # Segment-based evaluation (10 seconds / 500 frames per segment)
        segment_len = 500
        num_segments = len(Y_eval) // segment_len

        segment_ates = []
        plot_segments = min(4, num_segments)
        segment_plots = []

        for s in range(num_segments):
            start_idx = s * segment_len
            end_idx = start_idx + segment_len

            gt_seg_x = [0.0]
            gt_seg_y = [0.0]
            pred_seg_x = [0.0]
            pred_seg_y = [0.0]

            gt_seg_yaw = 0.0
            pred_seg_yaw = 0.0

            for i in range(start_idx, end_idx):
                gt_dx_local = Y_eval[i][0].item()
                gt_dy_local = Y_eval[i][1].item()
                gt_dtheta = Y_eval[i][2].item()

                gt_seg_yaw += gt_dtheta
                R_w_gt = np.array([
                    [np.cos(gt_seg_yaw), -np.sin(gt_seg_yaw), 0.0],
                    [np.sin(gt_seg_yaw), np.cos(gt_seg_yaw), 0.0],
                    [0.0, 0.0, 1.0]
                ])
                gt_disp_w = R_w_gt @ np.array([gt_dx_local, gt_dy_local, 0.0])
                gt_seg_x.append(gt_seg_x[-1] + gt_disp_w[0])
                gt_seg_y.append(gt_seg_y[-1] + gt_disp_w[1])

                pred_dx_local = predictions[i][0].item()
                pred_dy_local = predictions[i][1].item()
                pred_dtheta = predictions[i][2].item()

                pred_seg_yaw += pred_dtheta
                R_w_pred = np.array([
                    [np.cos(pred_seg_yaw), -np.sin(pred_seg_yaw), 0.0],
                    [np.sin(pred_seg_yaw), np.cos(pred_seg_yaw), 0.0],
                    [0.0, 0.0, 1.0]
                ])
                pred_disp_w = R_w_pred @ np.array([pred_dx_local, pred_dy_local, 0.0])
                pred_seg_x.append(pred_seg_x[-1] + pred_disp_w[0])
                pred_seg_y.append(pred_seg_y[-1] + pred_disp_w[1])

            gt_seg_x = np.array(gt_seg_x)
            gt_seg_y = np.array(gt_seg_y)
            pred_seg_x = np.array(pred_seg_x)
            pred_seg_y = np.array(pred_seg_y)

            seg_ate = np.sqrt(np.mean((gt_seg_x - pred_seg_x)**2 + (gt_seg_y - pred_seg_y)**2))
            segment_ates.append(seg_ate)

            if s < plot_segments:
                segment_plots.append((gt_seg_x, gt_seg_y, pred_seg_x, pred_seg_y, seg_ate))

        mean_seg_ate = np.mean(segment_ates) if segment_ates else 0.0

        gt_dxs = np.array(gt_dxs)
        gt_dys = np.array(gt_dys)
        pred_dxs = np.array(pred_dxs)
        pred_dys = np.array(pred_dys)

        rte_mae = np.mean(np.sqrt((gt_dxs - pred_dxs)**2 + (gt_dys - pred_dys)**2))
        rre_mae = np.mean(np.abs(np.array(gt_dthetas) - np.array(pred_dthetas)))

        print("\n================ Evaluation Metrics (Test Set) ================")
        if num_segments > 0:
            print(f"Mean 10s Segment ATE:                  {mean_seg_ate:.4f} m")
        else:
            print("Mean 10s Segment ATE:                  N/A (Test set < 10s)")
        print(f"Relative Translation Error (RTE MAE):  {rte_mae * 1000.0:.4f} mm/step")
        print(f"Relative Rotation Error (RRE MAE):     {np.degrees(rre_mae):.4f} deg/step")
        print("===============================================================\n")

        if num_segments == 0:
            print("Test set too small to generate segment trajectory plots.")
            return

        import matplotlib.pyplot as plt
        # Create a beautiful, premium grid figure
        fig = plt.figure(figsize=(16, 12))
        gs = fig.add_gridspec(4, 2, width_ratios=[1.0, 1.0])

        # Left side: 4 Segments
        for s in range(plot_segments):
            ax = fig.add_subplot(gs[s, 0])
            gt_x, gt_y, pred_x, pred_y, seg_ate = segment_plots[s]
            ax.plot(gt_x, gt_y, color='#1f77b4', linewidth=2.5, label='Ground Truth')
            ax.plot(pred_x, pred_y, color='#ff7f0e', linewidth=2.0, linestyle='--', label='ResNet Odometry')
            ax.set_xlabel('X (m)', fontsize=9)
            ax.set_ylabel('Y (m)', fontsize=9)
            ax.set_title(f'Segment {s+1} (10s) Trajectory | ATE: {seg_ate:.4f}m', fontsize=10, fontweight='bold')
            ax.legend(fontsize=8, loc='upper left')
            ax.grid(True, linestyle=':', alpha=0.6)
            ax.axis('equal')

        # Right side: Global metrics and tracking
        # Top right: Global Yaw Comparison
        ax_yaw = fig.add_subplot(gs[0:2, 1])
        ax_yaw.plot(np.degrees(gt_yaws), color='#1f77b4', linewidth=2.0, label='GT')
        ax_yaw.plot(np.degrees(pred_yaws), color='#ff7f0e', linewidth=1.5, linestyle='--', label='Pred')
        ax_yaw.set_ylabel('Yaw Angle (deg)', fontsize=11)
        ax_yaw.set_title('Global Yaw Orientation Tracking', fontsize=12, fontweight='bold')
        ax_yaw.legend(fontsize=9)
        ax_yaw.grid(True, linestyle=':', alpha=0.6)

        # Middle right: Global dx velocity
        ax_vel = fig.add_subplot(gs[2, 1])
        ax_vel.plot(gt_dxs * 50.0, color='#1f77b4', linewidth=1.2, label='GT dx')
        ax_vel.plot(pred_dxs * 50.0, color='#ff7f0e', linewidth=1.0, linestyle='--', label='Pred dx')
        ax_vel.set_ylabel('Velocity (m/s)', fontsize=11)
        ax_vel.set_title('Global Forward Velocity (dx)', fontsize=12, fontweight='bold')
        ax_vel.legend(fontsize=9)
        ax_vel.grid(True, linestyle=':', alpha=0.6)

        # Bottom right: Global dtheta rate
        ax_rot = fig.add_subplot(gs[3, 1])
        ax_rot.plot(np.degrees(gt_dthetas) * 50.0, color='#1f77b4', linewidth=1.2, label='GT dtheta')
        ax_rot.plot(np.degrees(pred_dthetas) * 50.0, color='#ff7f0e', linewidth=1.0, linestyle='--', label='Pred dtheta')
        ax_rot.set_xlabel('Timestep (0.02s)', fontsize=11)
        ax_rot.set_ylabel('Yaw Rate (deg/s)', fontsize=11)
        ax_rot.set_title('Global Yaw Rate (dtheta)', fontsize=12, fontweight='bold')
        ax_rot.legend(fontsize=9)
        ax_rot.grid(True, linestyle=':', alpha=0.6)

        plt.tight_layout()
        plot_path = "odometry_evaluation.png"
        plt.savefig(plot_path, dpi=300)
        print(f"Evaluation plot saved to {plot_path}")
        return

    # ============================================================================
    # Differentiable trajectory integration for rollout loss (fully vectorised)
    # ============================================================================
    def integrate_trajectory_torch(steps):
        """
        Integrate local-frame (dx, dy, dθ) steps into world-frame trajectories.
        Uses previous-yaw convention consistent with C++ SensorFilter inference.

        Fully vectorised using cumsum — no Python loops.

        Args:
            steps: [batch, seq_len, 3] tensor of (dx, dy, dtheta) in local frame

        Returns:
            positions: [batch, seq_len+1, 2] world-frame (x, y) positions
            yaws: [batch, seq_len+1] world-frame yaw angles
        """
        B, K, _ = steps.shape
        dx_local = steps[:, :, 0]   # [B, K]
        dy_local = steps[:, :, 1]   # [B, K]
        dtheta = steps[:, :, 2]     # [B, K]

        # Cumulative yaw: yaw[t] = sum(dtheta[0:t])
        cum_yaw = torch.cumsum(dtheta, dim=1)  # [B, K]

        # Previous-yaw convention: rotation at step t uses yaw BEFORE adding dtheta[t]
        # prev_yaw[0] = 0, prev_yaw[1] = dtheta[0], prev_yaw[t] = sum(dtheta[0:t])
        zeros = torch.zeros(B, 1, device=steps.device)
        prev_yaw = torch.cat([zeros, cum_yaw[:, :-1]], dim=1)  # [B, K]

        # Rotate all local displacements to world frame in parallel
        cos_yaw = torch.cos(prev_yaw)
        sin_yaw = torch.sin(prev_yaw)
        dx_world = cos_yaw * dx_local - sin_yaw * dy_local  # [B, K]
        dy_world = sin_yaw * dx_local + cos_yaw * dy_local  # [B, K]

        # Cumulative sum for world-frame positions
        cum_x = torch.cumsum(dx_world, dim=1)  # [B, K]
        cum_y = torch.cumsum(dy_world, dim=1)  # [B, K]

        # Prepend origin (0, 0) and yaw=0
        positions = torch.stack([
            torch.cat([zeros, cum_x], dim=1),
            torch.cat([zeros, cum_y], dim=1)
        ], dim=-1)  # [B, K+1, 2]

        yaws = torch.cat([zeros, cum_yaw], dim=1)  # [B, K+1]

        return positions, yaws

    # ============================================================================
    # Sequential dataset for contiguous rollout sequences
    # ============================================================================
    class OdometrySequenceDataset(Dataset):
        """Yields contiguous sequences of windows for multi-step rollout training.
        Uses a stride to avoid excessive overlap between sequences.
        Sequences that span a mocap tracking gap are excluded."""

        def __init__(self, X, Y_norm, Y_raw, seq_length, segment_ids, stride=None):
            self.X = X
            self.Y_norm = Y_norm
            self.Y_raw = Y_raw
            self.seq_length = seq_length
            # Default stride = half the sequence length (50% overlap)
            self.stride = stride if stride is not None else max(1, seq_length // 2)
            max_start = len(X) - seq_length
            # Only keep starts where the whole sequence lies within the same mocap segment
            self.starts = [
                s for s in range(0, max_start + 1, self.stride)
                if segment_ids[s] == segment_ids[s + seq_length - 1]
            ]

        def __len__(self):
            return len(self.starts)

        def __getitem__(self, idx):
            start = self.starts[idx]
            end = start + self.seq_length
            return self.X[start:end], self.Y_norm[start:end], self.Y_raw[start:end]

    # ============================================================================
    # Training setup
    # ============================================================================
    # Keep raw targets for the train split (needed for rollout and bias losses)
    Y_train_raw = Y_raw_tensor[:split_idx]

    criterion = nn.L1Loss()
    optimizer = optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-4)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=5)

    # Convert normalisation stats to tensors on device for denormalization in rollout
    t_mean = torch.tensor(target_mean_val, dtype=torch.float32).to(device)
    t_std = torch.tensor(target_std_val, dtype=torch.float32).to(device)

    # Clamp rollout length to available training data
    effective_rollout = min(rollout_length, len(X_train) - 1)
    if effective_rollout < 10:
        print(f"Warning: Training data too short for rollout (need >{rollout_length} samples). Disabling rollout loss.")
        warmup_epochs = epochs  # Fallback to per-step only

    print(f"\n{'='*60}")
    print(f"Training Plan:")
    print(f"  Total epochs:       {epochs}")
    print(f"  Phase 1 (warmup):   {warmup_epochs} epochs — per-step L1 loss only")
    print(f"  Phase 2 (rollout):  {epochs - warmup_epochs} epochs — step + trajectory + bias")
    print(f"  Rollout length:     {effective_rollout} steps ({effective_rollout / 50.0:.1f}s at 50Hz)")
    print(f"  λ_traj:             {lambda_traj}")
    print(f"  λ_bias:             {lambda_bias}")
    print(f"{'='*60}\n")

    # ============================================================================
    # Phase 1: Per-step warmup (shuffled, per-step L1 only)
    # ============================================================================
    if warmup_epochs > 0:
        print(f"=== Phase 1: Per-Step Warmup ({warmup_epochs} epochs) ===")
        dataset_step = TensorDataset(X_train, Y_train)
        loader_step = DataLoader(dataset_step, batch_size=batch_size, shuffle=True)

        model.train()
        for epoch in range(warmup_epochs):
            epoch_loss = 0.0
            for inputs, targets in loader_step:
                inputs, targets = inputs.to(device), targets.to(device)
                optimizer.zero_grad()
                outputs = model(inputs)
                loss = criterion(outputs, targets)
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                optimizer.step()
                epoch_loss += loss.item()

            avg_loss = epoch_loss / len(loader_step)
            scheduler.step(avg_loss)

            if (epoch + 1) % 10 == 0 or epoch == 0:
                current_lr = optimizer.param_groups[0]['lr']
                print(f"  [Warmup {epoch + 1}/{warmup_epochs}] Step L1: {avg_loss:.6f} | LR: {current_lr:.6e}")

    # ============================================================================
    # Phase 2: Multi-step rollout training (sequential, combined loss)
    # ============================================================================
    rollout_epochs = epochs - warmup_epochs
    if rollout_epochs > 0 and effective_rollout >= 10:
        print(f"\n=== Phase 2: Rollout Training ({rollout_epochs} epochs) ===")

        # Sequence batch size is smaller since each sample is a full rollout
        seq_batch_size = max(1, min(8, batch_size // 4))

        seq_dataset = OdometrySequenceDataset(X_train, Y_train, Y_train_raw, effective_rollout, seg_ids_train)
        seq_loader = DataLoader(seq_dataset, batch_size=seq_batch_size, shuffle=True)

        # Reset scheduler for Phase 2 with a fresh LR
        current_lr = optimizer.param_groups[0]['lr']
        phase2_lr = max(current_lr, 5e-4)  # Don't start Phase 2 with a decayed LR
        for pg in optimizer.param_groups:
            pg['lr'] = phase2_lr
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=8)

        model.train()
        for epoch in range(rollout_epochs):
            epoch_step_loss = 0.0
            epoch_traj_loss = 0.0
            epoch_bias_loss = 0.0
            epoch_total_loss = 0.0

            for X_seq, Y_norm_seq, Y_raw_seq in seq_loader:
                X_seq = X_seq.to(device)         # [B, K, input_flat]
                Y_norm_seq = Y_norm_seq.to(device)  # [B, K, 3]
                Y_raw_seq = Y_raw_seq.to(device)    # [B, K, 3]

                B, K, input_flat = X_seq.shape
                optimizer.zero_grad()

                # Forward pass: flatten sequences, run model, reshape back
                pred_norm = model(X_seq.reshape(B * K, input_flat))  # [B*K, 3]
                pred_norm = pred_norm.view(B, K, 3)

                # --- Per-step L1 loss (normalised space) ---
                step_loss = criterion(pred_norm, Y_norm_seq)

                # --- Denormalise predictions for trajectory integration ---
                pred_real = pred_norm * t_std + t_mean

                # --- Multi-step rollout trajectory loss (ATE) ---
                pred_pos, pred_yaws = integrate_trajectory_torch(pred_real)
                gt_pos, gt_yaws = integrate_trajectory_torch(Y_raw_seq)

                # Absolute Trajectory Error: L1 distance at each integrated timestep
                pos_errors = torch.sqrt(((pred_pos - gt_pos) ** 2).sum(dim=-1) + 1e-8)
                traj_loss = pos_errors.mean()

                # Also penalise yaw drift over the trajectory
                yaw_errors = torch.abs(pred_yaws - gt_yaws)
                traj_loss = traj_loss + 0.1 * yaw_errors.mean()

                # --- Zero-mean bias penalty ---
                # Penalise batch-mean predicted displacement differing from batch-mean GT
                pred_batch_mean = pred_real.mean(dim=(0, 1))  # [3]
                gt_batch_mean = Y_raw_seq.mean(dim=(0, 1))    # [3]
                bias_loss = torch.abs(pred_batch_mean - gt_batch_mean).mean()

                # --- Combined loss ---
                total_loss = step_loss + lambda_traj * traj_loss + lambda_bias * bias_loss
                total_loss.backward()

                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                optimizer.step()

                epoch_step_loss += step_loss.item()
                epoch_traj_loss += traj_loss.item()
                epoch_bias_loss += bias_loss.item()
                epoch_total_loss += total_loss.item()

            n_batches = len(seq_loader)
            avg_step = epoch_step_loss / n_batches
            avg_traj = epoch_traj_loss / n_batches
            avg_bias = epoch_bias_loss / n_batches
            avg_total = epoch_total_loss / n_batches

            scheduler.step(avg_total)

            if (epoch + 1) % 5 == 0 or epoch == 0:
                current_lr = optimizer.param_groups[0]['lr']
                global_epoch = warmup_epochs + epoch + 1
                print(f"  [Rollout {epoch + 1}/{rollout_epochs} (epoch {global_epoch})] "
                      f"Total: {avg_total:.6f} | Step: {avg_step:.6f} | "
                      f"Traj ATE: {avg_traj:.4f}m | Bias: {avg_bias:.6f} | LR: {current_lr:.6e}")

    print("\nTraining finished.")
    save_dir = os.path.join("module", "input", "SensorFilter", "data")
    os.makedirs(save_dir, exist_ok=True)
    model_path = os.path.join(save_dir, "autoodom_mlp.pt")
    torch.save(model.state_dict(), model_path)
    print(f"Model saved to {model_path}.")

    # Automatically export to ONNX after training
    model.eval()
    onnx_path = os.path.join(save_dir, "autoodom_mlp.onnx")
    print(f"Exporting model to {onnx_path}...")
    dummy_input = torch.randn(1, X.shape[1]).to(device)
    torch.onnx.export(
        model,
        dummy_input,
        onnx_path,
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}
    )
    print("ONNX export complete.")
