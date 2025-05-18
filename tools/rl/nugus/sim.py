import mujoco
import mujoco.viewer
import numpy as np
import time
from pathlib import Path


def main():
    # Load the model from the XML file
    model_path = Path("xmls/scene.xml")
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    # Set initial pose from keyframe values
    # Instead of using model.keyframe, set joint positions directly
    joint_positions = {
        "left_hip_yaw": 0.0339,
        "left_hip_roll": 0.163,
        "left_hip_pitch": -0.904,
        "left_knee_pitch": 1.2,
        "left_ankle_pitch": -0.51,
        "left_ankle_roll": -0.166,
        "right_hip_yaw": -0.0329,
        "right_hip_roll": -0.162,
        "right_hip_pitch": -0.904,
        "right_knee_pitch": 1.2,
        "right_ankle_pitch": -0.508,
        "right_ankle_roll": 0.167,
        "neck_yaw": -2.22e-08,
        "head_pitch": 6.45e-05,
        "left_shoulder_pitch": 1.71,
        "left_shoulder_roll": 0.197,
        "left_elbow_pitch": -0.713,
        "right_shoulder_pitch": 1.71,
        "right_shoulder_roll": -0.197,
        "right_elbow_pitch": -0.718,
    }

    # Set joint positions based on their names
    for joint_name, position in joint_positions.items():
        # Find the joint ID by name
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id != -1:  # Check if joint exists
            # Get the qpos index for this joint
            qpos_addr = model.jnt_qposadr[joint_id]
            # Set the position
            data.qpos[qpos_addr] = position

    # Floating base
    data.qpos[:3] = [0, 0, 0.473774]
    data.qpos[3:7] = [0.994428, 0.00107808, 0.104931, -0.0100319]

    # Simulation parameters
    duration = 5.0  # seconds

    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Track simulation time
        sim_start = time.time()
        print(f"Running simulation for {duration} seconds...")

        while viewer.is_running() and data.time < duration:
            step_start = time.time()

            # Set control signal dynamically
            for joint_name, position in joint_positions.items():
                joint_id = mujoco.mj_name2id(
                    model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name
                )
                print(joint_id, model.nu)
                if joint_id != -1 and joint_id < model.nu:
                    # Get the qpos index for this joint
                    qpos_addr = model.jnt_qposadr[joint_id]
                    # Set the position
                    data.ctrl[joint_id] = position

            # Step the simulation
            mujoco.mj_step(model, data)
            viewer.sync()

            # Wait until real time catches up with simulation time
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            # Print progress occasionally
            if int(data.time * 10) % 10 == 0:
                print(
                    f"Simulated {data.time:.2f} seconds (real time: {time.time() - sim_start:.2f}s)"
                )

        print(f"Simulation complete. Total time: {time.time() - sim_start:.2f} seconds")


if __name__ == "__main__":
    main()
