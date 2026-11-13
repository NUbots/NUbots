# Walk

## Description

Runs an mjlab K1 velocity-tracking walk policy at 50 Hz and emits the resulting joint position targets as a
Director-arbitrated `message::actuation::Servos` subtask, which `platform::Booster::HardwareIO` forwards to the
Booster SDK `rt/joint_ctrl` topic (honoured only in CUSTOM mode).

`skill::BoosterWalk` (the firmware gait, through the SDK's `Move()` RPC) and `skill::SimK1Walk` (the older
mujoco_playground policy, NUSim only) are legacy alternatives. A role should include exactly one of the three.

## Usage

Include this module in a role with `actuation::Servos` and emit a `message::skill::Walk` Task.

## Consumes

- `message::skill::Walk` Task with the target velocity
- `message::input::Sensors` for joint feedback, gyro and the torso attitude (`Htw`)
- `message::behaviour::state::Stability`, ticks are skipped while FALLEN, when the get-up skill owns the servos
- `message::booster::BoosterHeadRot` for the head targets (unless `head.policy_controlled`)
- `message::booster::BoosterModeState` (optional) to request CUSTOM mode until the robot is in it

## Emits

- `message::actuation::Servos` Task (22 motors, SDK `JointIndexK1` serial order)
- `message::booster::BoosterMode` (CUSTOM) when the walk task starts
- `message::behaviour::state::WalkState`

## Dependencies

- Director
- `input::Sensors`
- ONNX Runtime

## Policy

The shipped checkpoint (`data/k1_walk_mjlab.onnx`) is trained in
[booster_mjlab](https://github.com/IntelligentRoboticsLab/booster_mjlab), task `Mjlab-Velocity-Flat-Booster-K1`,
experiment `k1_velocity_amp_symmetric_muon` (PPO with an AMP style reward, symmetry augmentation and the Muon
optimizer), W&B run `poi732yy`, iteration 29999.

The observation and action contract is configured in `Walk.yaml`, and the module refuses any graph whose input or
output size disagrees with it.

### ONNX graph

| Tensor | Name      | Shape     | Type      |
| ------ | --------- | --------- | --------- |
| Input  | `obs`     | `[1, 75]` | `float32` |
| Output | `actions` | `[1, 22]` | `float32` |

The observation normaliser is part of the graph, so the module applies none. Actions are not clipped.

### Observation (75 floats)

One frame, with no history (`history_window: 1`), and no scaling or clipping.

| Offset | Count | Field                               | Notes                                                    |
| ------ | ----- | ----------------------------------- | -------------------------------------------------------- |
| 0      | 3     | Angular velocity (gyro), body frame | rad/s, from `Sensors::gyroscope`                         |
| 3      | 3     | Projected gravity, body frame       | world `(0, 0, -1)` rotated by `Sensors::Htw`             |
| 6      | 22    | `q - default_pose`                  | rad                                                      |
| 28     | 22    | `dq`                                | rad/s                                                    |
| 50     | 22    | Last action                         | raw network output, before scaling and joint-range clamp |
| 72     | 3     | Command `[vx, vy, wz]`              | m/s, m/s, rad/s                                          |

With `history_window` above 1 the input is that many frames, oldest first.

### Joint order

The policy's joint order is `JointIndexK1`, so `policy_joints` is `0..21`.

```
 0 HeadYaw            1 HeadPitch
 2 LeftShoulderPitch  3 LeftShoulderRoll  4 LeftElbowPitch  5 LeftElbowYaw
 6 RightShoulderPitch 7 RightShoulderRoll 8 RightElbowPitch 9 RightElbowYaw
10 LeftHipPitch  11 LeftHipRoll  12 LeftHipYaw  13 LeftKneePitch  14 LeftAnklePitch  15 LeftAnkleRoll
16 RightHipPitch 17 RightHipRoll 18 RightHipYaw 19 RightKneePitch 20 RightAnklePitch 21 RightAnkleRoll
```

### Action (22 floats)

Joint target offsets around the training keyframe:

```
q_ref[j]    = default_pose[j] + action_scale * action_scale_joint[j] * action[j]
q_ref[head] = clamp(latest BoosterHeadRot)          # unless head.policy_controlled
q_cmd       = clamp(lerp(measured, q_ref, time since walk start / handoff_blend), joint_lower, joint_upper)
```

- `default_pose` is the booster_mjlab `HOME_KEYFRAME`.
- `action_scale_joint[j] = 0.25 * effort_limit[j] / kp[j]`.
- `kp` and `kd` are the training-time actuator gains.
- `handoff_blend` blends from the measured pose into the policy target when a walk starts, as training always
  starts from the default pose.

### Head

The policy observes the head and emits head actions, but the look skills own the head: with
`head.policy_controlled: false` the head targets are the latest `BoosterHeadRot`, tracked with `head.kp` and
`head.kd`. The head stays in `policy_joints`, as the policy was trained observing it.

### Loop rate

The policy is trained at 50 Hz. Ticks whose period is off 0.02 s by more than 5 ms are counted and reported at WARN
every 2 s.

### Command envelope

Training commands reach `vx` in `[-1.5, 1.75]` m/s, `vy` in `[-1.75, 1.75]` m/s and `wz` in `[-1.5, 1.5]` rad/s.
Commands are not clipped.

## Changing the policy

A checkpoint with a different contract needs `history_window`, `policy_joints`, the gains, the action
scales, the default pose and the joint ranges in `Walk.yaml` changed to match it. booster_mjlab exports these as
ONNX metadata props (`joint_names`, `joint_stiffness`, `joint_damping`, `default_joint_pos`, `action_scale`,
`observation_names`, `observation_terms_history_length`), which `onnx.load(path).metadata_props` reads.

## Logging

- `log_level: DEBUG` logs the command, action, joint state and joint targets of each walking tick, and graphs the
  command and loop period.
- `log_level: TRACE` logs every tick as `WALKOBS <tick> mode=<K1Mode> t=<s> dt=<s>` followed by the observation
  frame.
