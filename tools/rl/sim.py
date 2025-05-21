import mujoco
import mujoco.viewer
import numpy as np
import time
from pathlib import Path

import jax
import jax.numpy as jp
import argparse
import sys
import os

# Add the current directory to the path so we can import nugus
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from nugus.joystick import Joystick, nugus_env_config
from test_onnx import OnnxInfer


def register(command):
    command.description = "Run a simulation of the Nugus robot with joystick control and ONNX policy"
    command.add_argument("--xml_path", type=str, default="nugus/xmls/scene.xml", help="Path to the XML model file")
    command.add_argument("--onnx_model_path", type=str, required=True, help="Path to the ONNX model file")


def get_observation(data, command, last_action):
    """Construct observation from MuJoCo data."""
    # Get gyro (angular velocity)
    gyro_id = data.model.sensor("gyro").id
    gyro_adr = data.model.sensor_adr[gyro_id]
    gyro_dim = data.model.sensor_dim[gyro_id]
    gyro = data.sensordata[gyro_adr:gyro_adr + gyro_dim]

    # Get gravity
    gravity_id = data.model.sensor("upvector").id
    gravity_adr = data.model.sensor_adr[gravity_id]
    gravity_dim = data.model.sensor_dim[gravity_id]
    gravity = data.sensordata[gravity_adr:gravity_adr + gravity_dim]

    # Get joint positions relative to default pose
    default_pose = data.model.keyframe("stand_bent_knees").qpos[7:]
    joint_positions = data.qpos[7:] - default_pose

    # Combine into observation
    obs = np.concatenate([
        gyro,           # 3
        gravity,        # 3
        command,        # 3
        joint_positions, # 20
        last_action,    # 20
    ])

    return obs.astype(np.float32)


def run(**kwargs):
    """
    Runs a simulation of the Nugus robot with joystick control and ONNX policy.
    """
    xml_path = kwargs.get("xml_path", "nugus/xmls/scene.xml")
    onnx_model_path = kwargs.get("onnx_model_path")

    # Get the absolute path to the XML file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, xml_path)

    if not os.path.exists(model_path):
        print(f"Error: XML model not found at {model_path}")
        return

    # Initialize ONNX policy
    policy = OnnxInfer(onnx_model_path, input_name="obs", awd=True)
    print(f"Loaded ONNX model from: {onnx_model_path}")

    # Load model and data
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Set initial pose from keyframe
    data.qpos[:] = model.keyframe("stand_bent_knees").qpos
    data.qvel[:] = np.zeros(model.nv)

    # Initialize control variables
    last_action = np.zeros(model.nu)
    command = np.array([0.0, 0.0, 0.0])  # [lin_vel_x, lin_vel_y, ang_vel_yaw]
    default_pose = model.keyframe("stand_bent_knees").qpos[7:]
    lowers = model.actuator_ctrlrange[:, 0]
    uppers = model.actuator_ctrlrange[:, 1]

    # Set timesteps to match joystick environment
    ctrl_dt = 0.02  # Control timestep
    sim_dt = 0.002  # Simulation timestep
    model.opt.timestep = sim_dt  # Set simulation timestep
    n_substeps = int(ctrl_dt / sim_dt)  # Number of simulation steps per control step

    # Velocity control parameters
    max_lin_vel = 1.5  # Maximum linear velocity
    max_ang_vel = 0.6  # Maximum angular velocity
    vel_step = 0.1     # Velocity change per keypress
    action_scale = 1.0  # Match the config in joystick.py

    print(f"Starting simulation...")
    print(f"Using model from: {model_path}")
    print(f"Control timestep: {ctrl_dt}s, Simulation timestep: {sim_dt}s")
    print("Controls:")
    print("  Arrow keys: Control linear velocity (forward/backward/left/right)")
    print("  A/D: Control angular velocity (turn left/right)")
    print("  ESC or close window to stop")

    def key_callback(keycode):
        nonlocal command
        print(f"Keycode: {keycode}")
        if keycode == 265:  # Up arrow
            command[0] = min(command[0] + vel_step, max_lin_vel)
        elif keycode == 264:  # Down arrow
            command[0] = max(command[0] - vel_step, -max_lin_vel)
        elif keycode == 263:  # Left arrow
            command[1] = min(command[1] + vel_step, max_lin_vel)
        elif keycode == 262:  # Right arrow
            command[1] = max(command[1] - vel_step, -max_lin_vel)
        elif keycode == 65:  # 'A' key
            command[2] = min(command[2] + vel_step, max_ang_vel)
        elif keycode == 68:  # 'D' key
            command[2] = max(command[2] - vel_step, -max_ang_vel)
        elif keycode == 342:  # 'r' key
            # reset command
            command = np.array([0.0, 0.0, 0.0])
        print(f"Command: {command}")

    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        # Track simulation time
        sim_start = time.time()
        last_ctrl_time = time.time()

        while viewer.is_running():
            step_start = time.time()

            # Check if it's time for a control step
            current_time = time.time()
            if current_time - last_ctrl_time >= ctrl_dt:
                # Get observation from MuJoCo data
                obs = get_observation(data, command, last_action)

                # Get action from policy
                action = policy.infer(obs)

                # Apply action to control (with action scaling)
                motor_targets = default_pose + action * action_scale
                motor_targets = np.clip(motor_targets, lowers, uppers)
                data.ctrl[:] = motor_targets

                # Store last action
                last_action = action

                # Update last control time
                last_ctrl_time = current_time

            # Step the simulation
            mujoco.mj_step(model, data)
            viewer.sync()

            # Wait until real time catches up with simulation time
            time_until_next_step = sim_dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

        print(f"Simulation complete. Total time: {time.time() - sim_start:.2f} seconds")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml_path", type=str, default="nugus/xmls/scene.xml")
    parser.add_argument("--onnx_model_path", type=str, required=True)
    args = parser.parse_args()

    run(xml_path=args.xml_path, onnx_model_path=args.onnx_model_path)
