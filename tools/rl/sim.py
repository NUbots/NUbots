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
    command.add_argument("--duration", type=float, default=5.0, help="Simulation duration in seconds")
    command.add_argument("--xml_path", type=str, default="nugus/xmls/scene.xml", help="Path to the XML model file")
    command.add_argument("--onnx_model_path", type=str, required=True, help="Path to the ONNX model file")


def get_observation(data, command, last_action):
    """Construct observation from MuJoCo data."""
    # Get gyro (angular velocity)
    gyro_id = data.model.sensor("gyro").id
    gyro_adr = data.model.sensor_adr[gyro_id]
    gyro_dim = data.model.sensor_dim[gyro_id]
    gyro = data.sensordata[gyro_adr:gyro_adr + gyro_dim]

    # Get accelerometer
    accelerometer_id = data.model.sensor("accelerometer").id
    accelerometer_adr = data.model.sensor_adr[accelerometer_id]
    accelerometer_dim = data.model.sensor_dim[accelerometer_id]
    accelerometer = data.sensordata[accelerometer_adr:accelerometer_adr + accelerometer_dim]

    # Get joint positions relative to default pose
    default_pose = data.model.keyframe("stand_bent_knees").qpos[7:]
    joint_positions = data.qpos[7:] - default_pose

    # Print dimensions for debugging
    print(f"Gyro dim: {gyro_dim}, Accelerometer dim: {accelerometer_dim}, Joint positions dim: {len(joint_positions)}, Last action dim: {len(last_action)}")

    # Combine into observation
    obs = np.concatenate([
        gyro,           # 3
        accelerometer,  # 3
        command,        # 3
        joint_positions, # 20
        last_action,    # 20
    ])

    # Ensure we have exactly 49 dimensions
    if len(obs) != 49:
        print(f"Warning: Observation has {len(obs)} dimensions, expected 49")
        # If we have too many dimensions, truncate
        if len(obs) > 49:
            obs = obs[:49]
        # If we have too few dimensions, pad with zeros
        elif len(obs) < 49:
            obs = np.pad(obs, (0, 49 - len(obs)))

    return obs.astype(np.float32)


def run(**kwargs):
    """
    Runs a simulation of the Nugus robot with joystick control and ONNX policy.
    """
    duration = kwargs.get("duration", 5.0)
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
    command = np.array([0.5, 0.0, 0.0])  # Initial command: move forward
    default_pose = model.keyframe("stand_bent_knees").qpos[7:]
    lowers = model.actuator_ctrlrange[:, 0]
    uppers = model.actuator_ctrlrange[:, 1]

    print(f"Starting simulation for {duration} seconds...")
    print(f"Using model from: {model_path}")

    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Track simulation time
        sim_start = time.time()

        while viewer.is_running() and data.time < duration:
            step_start = time.time()

            # Get observation from MuJoCo data
            obs = get_observation(data, command, last_action)

            # Get action from policy
            action = policy.infer(obs)

            # Apply action to control
            motor_targets = default_pose + action
            motor_targets = np.clip(motor_targets, lowers, uppers)
            data.ctrl[:] = motor_targets

            # Store last action
            last_action = action

            # Step the simulation
            mujoco.mj_step(model, data)
            viewer.sync()

            # Wait until real time catches up with simulation time
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            # Print current command and state occasionally
            if int(data.time * 10) % 10 == 0:
                print(f"Time: {data.time:.2f}s, Command: {command}")
                print(f"Base position: {data.qpos[:3]}")
                print(f"Base orientation: {data.qpos[3:7]}")

        print(f"Simulation complete. Total time: {time.time() - sim_start:.2f} seconds")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--xml_path", type=str, default="nugus/xmls/scene.xml")
    parser.add_argument("--onnx_model_path", type=str, required=True)
    args = parser.parse_args()

    run(duration=args.duration, xml_path=args.xml_path, onnx_model_path=args.onnx_model_path)
