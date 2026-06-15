# WalkDataCollector

## Description

Generates training data for walk engine distillation (Stage 1 of the robust locomotion project). 

This tool runs the full walk engine pipeline offline — `WalkGenerator` (spline trajectories) → analytical IK → tinyrobotics numerical IK refinement — across diverse velocity commands, and outputs binary training data for a neural network to learn the mapping from (commands, phase, history) → joint angles.

## Pipeline Replicated

The IK pipeline matches the exact code path used on the robot in `module/actuation/Kinematics`:

1. `WalkGenerator` generates desired foot poses `Htl`, `Htr` in torso frame
2. `calculate_leg_joints()` computes an analytical IK solution (warm start)
3. `tinyrobotics::inverse_kinematics()` refines with numerical optimisation (Levenberg-Marquardt)
4. Final 12 joint angles (6 per leg) are recorded as training targets

## Output Format

Binary float32 files, one per episode:
- Each sample is `58` floats: `46` observation + `12` target
- Load in Python: `np.fromfile("episode_00042.bin", dtype=np.float32).reshape(-1, 58)`

### Observation layout (46 dims)

| Index | Feature | Dim |
|-------|---------|-----|
| 0-2 | Velocity command (vx, vy, vθ) | 3 |
| 3-4 | Phase clock (sin, cos) | 2 |
| 5 | Phase indicator (+1 LEFT, -1 RIGHT) | 1 |
| 6-9 | Engine state one-hot | 4 |
| 10-21 | Previous joint targets (t-1) | 12 |
| 22-33 | Previous joint targets (t-2) | 12 |
| 34-45 | Previous joint targets (t-3) | 12 |

### Target layout (12 dims)

L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL,
R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL

## Usage

Configure parameters in `data/config/WalkDataCollector.yaml`, then run via the NUbots build system. The tool will generate data and shut down automatically.

## Dependencies

- WalkGenerator (shared/utility/skill/)
- InverseKinematics (shared/utility/actuation/)
- tinyrobotics (for numerical IK refinement)
- KinematicsConfiguration.yaml (for robot dimensions)
