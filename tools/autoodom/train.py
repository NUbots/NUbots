#!/usr/bin/env python3

import math
import os
import sys

# Imports delayed to run in docker where deps are met

def get_gru_class():
    import torch
    import torch.nn as nn
    class AutoOdomGRU(nn.Module):
        def __init__(self, input_dim, hidden_dim=128, num_layers=2, mean=None, std=None, target_mean=None, target_std=None):
            super().__init__()
            self.input_dim = input_dim
            self.hidden_dim = hidden_dim

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

            self.gru = nn.GRU(input_dim, hidden_dim, num_layers, batch_first=True, dropout=0.1)
            self.fc = nn.Sequential(
                nn.Linear(hidden_dim, 64),
                nn.ReLU(),
                nn.Linear(64, 32),
                nn.ReLU(),
                nn.Linear(32, 3) # Output: dx, dy, dtheta
            )

        def forward(self, x):
            # Use .size(0) instead of .shape[0] for ONNX dynamic axes compatibility
            if len(x.shape) == 2:
                x = x.view(x.size(0), -1, self.input_dim)

            x_norm = (x - self.mean) / (self.std + 1e-8)
            gru_out, _ = self.gru(x_norm)
            out = self.fc(gru_out[:, -1, :])
            
            if not self.training:
                # Scale back to real-world units during inference/ONNX export
                out = out * self.target_std + self.target_mean
                
            return out
    return AutoOdomGRU

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

@run_on_docker
def register(command):
    command.description = "Train a GRU to predict odometry x-y translation from an NBS dataset"
    command.add_argument("nbs_file", help="The nbs file containing OdometryRecord messages")
    command.add_argument("--epochs", type=int, default=100, help="Number of training epochs")
    command.add_argument("--batch-size", type=int, default=32, help="Training batch size")
    command.add_argument("--window-size", type=int, default=50, help="Window size (H)")
    command.add_argument("--evaluate", action="store_true", help="Evaluate the model on the test split and plot")
    command.add_argument("--export-onnx", action="store_true", help="Export the trained model to ONNX format")

@run_on_docker
def run(nbs_file, epochs, batch_size, window_size, evaluate=False, export_onnx=False, **kwargs):
    import numpy as np
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import DataLoader, TensorDataset
    from utility.nbs import LinearDecoder

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    if not os.path.isfile(nbs_file):
        print(f"Error: {nbs_file} not found.")
        return

    print(f"Opening NBS Dataset {nbs_file} with window size {window_size}...")

    raw_features = []
    raw_positions = []

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

    total_records = len(raw_features)
    print(f"Extracted {total_records} OdometryRecords.")

    if total_records < window_size + 2:
        print("Not enough records in the dataset to form a single window and displacement step.")
        return

    # Calculate dataset mean and std for normalization
    all_features = np.array(raw_features)
    mean_val = np.mean(all_features, axis=0)
    std_val = np.std(all_features, axis=0)
    std_val[std_val == 0.0] = 1.0

    # Build windows
    X = []
    Y = []
    for i in range(total_records - window_size):
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
    print(f"Train split: {len(X_train)} samples | Test split: {len(X_test)} samples")

    AutoOdomGRU = get_gru_class()
    feature_dim = X.shape[1] // window_size
    model = AutoOdomGRU(
        input_dim=feature_dim, 
        hidden_dim=128, 
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
            ax.plot(pred_x, pred_y, color='#ff7f0e', linewidth=2.0, linestyle='--', label='GRU Odometry')
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

    # Proceed with training using X_train and Y_train natively scaled
    dataset = TensorDataset(X_train, Y_train)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=1e-3)

    print(f"Starting Training for {epochs} epochs on {device}...")
    model.train()
    for epoch in range(epochs):
        epoch_loss = 0.0
        for batch_i, (inputs, targets) in enumerate(dataloader):
            inputs, targets = inputs.to(device), targets.to(device)
            optimizer.zero_grad()

            outputs = model(inputs)

            # Loss computed directly on the natively scaled targets
            loss = criterion(outputs, targets)
            loss.backward()
            optimizer.step()

            epoch_loss += loss.item()

        avg_loss = epoch_loss / len(dataloader)
        if (epoch + 1) % 10 == 0 or epoch == 0:
            print(f"Epoch [{epoch + 1}/{epochs}] | MSE Loss: {avg_loss:.6f}")

    print("Training finished.")
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
