#!/usr/bin/env python3

import math
import os
import sys

# Imports delayed to run in docker where deps are met

def get_mlp_class():
    import torch
    import torch.nn as nn
    class AutoOdomMLP(nn.Module):
        def __init__(self, input_dim):
            super().__init__()
            # The paper specifies bypassing LSTM logic, so we directly feed a flattened window into an MLP
            self.net = nn.Sequential(
                nn.Linear(input_dim, 256),
                nn.ReLU(),
                nn.Linear(256, 128),
                nn.ReLU(),
                nn.Linear(128, 64),
                nn.ReLU(),
                nn.Linear(64, 3)  # Output: dx, dy, dtheta
            )

        def forward(self, x):
            return self.net(x)
    return AutoOdomMLP




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
    
    # Rotate world displacement into the local frame of t-1
    disp_local = R_w_t_minus_1.T @ np.array([disp_w[0], disp_w[1], 0.0])
    
    # Calculate yaw change dtheta
    yaw_t = np.arctan2(R_w_t[1, 0], R_w_t[0, 0])
    yaw_t_minus_1 = np.arctan2(R_w_t_minus_1[1, 0], R_w_t_minus_1[0, 0])
    
    # Wrap difference to [-pi, pi]
    dtheta = yaw_t - yaw_t_minus_1
    dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
    
    return np.array([disp_local[0], disp_local[1], dtheta])

from utility.dockerise import run_on_docker

@run_on_docker
def register(command):
    command.description = "Train an MLP to predict odometry x-y translation from an NBS dataset"
    command.add_argument("nbs_file", help="The nbs file containing OdometryRecord messages")
    command.add_argument("--epochs", type=int, default=100, help="Number of training epochs")
    command.add_argument("--batch-size", type=int, default=32, help="Training batch size")
    command.add_argument("--window-size", type=int, default=50, help="Window size (H)")
    command.add_argument("--evaluate", action="store_true", help="Evaluate the model and plot the path instead of training")
    command.add_argument("--export-onnx", action="store_true", help="Export the trained model to ONNX format")

@run_on_docker
def run(nbs_file, epochs, batch_size, window_size, evaluate=False, export_onnx=False, **kwargs):
    import numpy as np
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import DataLoader, TensorDataset
    from utility.nbs import LinearDecoder

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

    if total_records < window_size + 1:
        print("Not enough records in the dataset to form a single window.")
        return

    # Build windows
    X = []
    Y = []
    for i in range(total_records - window_size):
        # A window is a flattened array of size H * feature_dim
        window = raw_features[i : i + window_size]
        flattened_window = np.concatenate(window)
        
        # Ground truth delta position between end-1 and end of the window (0.02s step) in local frame
        gt_disp = compute_local_step_displacement(raw_positions[i + window_size], raw_positions[i + window_size - 1])
        
        X.append(flattened_window)
        Y.append(gt_disp)

    X = torch.tensor(np.array(X), dtype=torch.float32)
    Y = torch.tensor(np.array(Y), dtype=torch.float32)

    print(f"Dataset compiled. Shape X: {X.shape}, Shape Y: {Y.shape}")
    
    dataset = TensorDataset(X, Y)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    AutoOdomMLP = get_mlp_class()
    model = AutoOdomMLP(input_dim=X.shape[1])

    if export_onnx:
        model_path = os.path.join("models", "autoodom_mlp.pt")
        if not os.path.exists(model_path):
            print(f"Error: Model not found at {model_path}")
            return
        
        model.load_state_dict(torch.load(model_path))
        model.eval()
        
        onnx_path = os.path.join("models", "autoodom_mlp.onnx")
        print(f"Exporting model to {onnx_path}...")
        
        # Create a dummy input tensor
        dummy_input = torch.randn(1, X.shape[1])
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
        model_path = os.path.join("models", "autoodom_mlp.pt")
        if not os.path.exists(model_path):
            print(f"Error: Model not found at {model_path}")
            return
        
        model.load_state_dict(torch.load(model_path))
        model.eval()
        print("Evaluating model...")
        
        with torch.no_grad():
            predictions = model(X)
        
        # Integrate step-by-step in the world frame using the integrated yaw orientations
        gt_path_x = [0.0]
        gt_path_y = [0.0]
        pred_path_x = [0.0]
        pred_path_y = [0.0]
        
        gt_yaw = 0.0
        pred_yaw = 0.0
        
        for i in range(len(Y)):
            # Ground truth step integration
            gt_dx_local = Y[i][0].item()
            gt_dy_local = Y[i][1].item()
            gt_dtheta = Y[i][2].item()
            
            gt_yaw += gt_dtheta
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
            
            pred_yaw += pred_dtheta
            R_w_pred = np.array([
                [np.cos(pred_yaw), -np.sin(pred_yaw), 0.0],
                [np.sin(pred_yaw), np.cos(pred_yaw), 0.0],
                [0.0, 0.0, 1.0]
            ])
            pred_disp_w = R_w_pred @ np.array([pred_dx_local, pred_dy_local, 0.0])
            pred_path_x.append(pred_path_x[-1] + pred_disp_w[0])
            pred_path_y.append(pred_path_y[-1] + pred_disp_w[1])
            
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 8))
        plt.plot(gt_path_x, gt_path_y, label='Ground Truth')
        plt.plot(pred_path_x, pred_path_y, label='Predicted', linestyle='--')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Odometry Evaluation')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        plot_path = "odometry_evaluation.png"
        plt.savefig(plot_path)
        print(f"Evaluation plot saved to {plot_path}")
        return

    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=1e-3)

    print(f"Starting Training for {epochs} epochs...")
    model.train()
    for epoch in range(epochs):
        epoch_loss = 0.0
        for batch_i, (inputs, targets) in enumerate(dataloader):
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, targets)
            loss.backward()
            optimizer.step()
            
            epoch_loss += loss.item()
            
        avg_loss = epoch_loss / len(dataloader)
        if (epoch + 1) % 10 == 0 or epoch == 0:
            print(f"Epoch [{epoch + 1}/{epochs}] | MSE Loss: {avg_loss:.6f}")

    print("Training finished.")
    os.makedirs("models", exist_ok=True)
    model_path = os.path.join("models", "autoodom_mlp.pt")
    torch.save(model.state_dict(), model_path)
    print(f"Model saved to {model_path}.")
