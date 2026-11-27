# Walk

## Description

Runs the mjlab K1 policies at 50 Hz and emits the resulting joint position targets as a Director-arbitrated
`message::actuation::Servos` subtask, which `platform::Booster::HardwareIO` forwards to the Booster SDK
`rt/joint_ctrl` topic (honoured only in CUSTOM mode):

- the velocity-tracking walk, for the `message::skill::Walk` task
- the six ball skills from mjlab's `brooker/jogaMIT` branch (receive, dribble, ground kick, air shot, directional first
  touch, first-time pass), for the `message::skill::BallSkill` task. See [Ball skills](#ball-skills).

Both run in the one reactor, so a switch between the walk and a ball skill continues from the joint targets last sent,
and a switch between ball skills keeps their shared observation history, as mjlab's controller interface asks.

`skill::BoosterWalk` (the firmware gait, through the SDK's `Move()` RPC) and `skill::SimK1Walk` (the older
mujoco_playground policy, NUSim only) are legacy alternatives. A role should include exactly one of the three.

## Usage

Include this module in a role with `actuation::Servos` and emit a `message::skill::Walk` Task, or a
`message::skill::BallSkill` Task to run a ball skill. Emit only one of the two at a time (remove the other with a null
task), as both drive the servos. `purpose::KeyboardSkills` (`roles/nusim/keyboardskills.role`) plays them all from the
keyboard in NUSim.

## Consumes

- `message::skill::Walk` Task with the target velocity
- `message::skill::BallSkill` Task with the skill and its ball velocity command
- `message::vision::Balls` or `message::localisation::Ball` for the ball skills' ball observation (`BallSkills.yaml`
  `ball.source`)
- `Sensors.yaml`'s `Hpc`, the camera's mounting on the head, to put the ball in the policies' camera frame
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

## Ball skills

`data/config/BallSkills.yaml` configures them. The ONNX files are in `data/ball_skills/`, copied from mjlab's
`policies/` (branch `brooker/jogaMIT`, commit `8237c75`), the re-exported `model_<n>.onnx` of each with the real
robot's gains in its metadata:

| `BallSkill::Type` | File               | mjlab folder / checkpoint                          | Command (m/s, robot yaw frame)                      |
| ----------------- | ------------------ | -------------------------------------------------- | --------------------------------------------------- |
| `RECEIVE`         | `receive.onnx`     | `k1_receive_walk_v5_stable` / `model_19500`        | ignored, always `(0, 0)`                            |
| `DRIBBLE`         | `dribble.onnx`     | `k1_dribble_v13b_arms_low` / `model_33700`         | ball velocity, 0.4 to 1.8, or `(0, 0)` to hold      |
| `KICK`            | `kick.onnx`        | `k1_kick_v10_handover` / `model_40493`             | ball exit velocity, 2 to 7, within 135° (45° > 4.5) |
| `KICK_AIR`        | `kick_air.onnx`    | `k1_kick_air_v6_handover` / `model_43292`          | as `KICK`, ground part; the ball rises at 30°       |
| `FIRST_TOUCH`     | `first_touch.onnx` | `k1_first_touch_v5_aim` / `model_26999`            | ball velocity off the touch, 0.5 to 1.2             |
| `FIRST_PASS`      | `first_pass.onnx`  | `k1_first_touch_pass_v6_long_more` / `model_34290` | ball velocity off the touch, 2 to 4                 |

Commands are clamped to these ranges (`limits`). For the first touch and first-time pass the direction must also be
within 90° of the direction the pass came from, which the caller has to keep to.

### Ball-skill ONNX graph

`obs [1, 234] -> actions [1, 22]`, normaliser in the graph. Joint order is `JointIndexK1`. Each file's metadata gives
the joint names (checked against `JointIndexK1`), `default_joint_pos`, `action_scale`, `joint_stiffness` and
`joint_damping`, which the module uses as they are (placeholder gains, kp 1 / kd 0, fall back to `fallback_kp/kd`).

### Ball-skill observation (234 floats)

Seven terms, each over the last three control steps **oldest first**, concatenated term by term. The first step after
a start or a fall fills the history by repeating itself.

| Term                | Per step | Definition                                                         |
| ------------------- | -------- | ------------------------------------------------------------------ |
| `base_ang_vel`      | 3        | `Sensors::gyroscope`                                               |
| `projected_gravity` | 3        | world `(0, 0, -1)` rotated by `Sensors::Htw`                       |
| `joint_pos`         | 22       | `q - default_joint_pos`                                            |
| `joint_vel`         | 22       | `dq`                                                               |
| `actions`           | 22       | previous raw output                                                |
| `command`           | 2        | ball velocity command, robot yaw frame, after clamping             |
| `ball`              | 4        | `(x, y, z, visible)` in the policies' head camera frame, see below |

The policies' camera frame is the head pitch link frame (`Head_2`: x forward, y left, z up) with its origin moved by
`camera.offset`. A ball estimate in the world {w} is put there with `Sensors::Hcw` and `Sensors.yaml`'s `Hpc`, and
counts as visible if it is younger than `ball.max_age` and inside the symmetric 106.2° × 95° field of view and range.
Otherwise the last sighting is repeated with `visible = 0`; before the first one it is `(0.5, 0, -0.8, 0)`.

`ball.source` picks the estimate:

- `vision`: the `vision::Balls` detection nearest the camera, its ray put on the ball centre (the plane at the ball's
  radius). Closest to training, which used noisy per-frame detections.
- `localisation`: `localisation::Ball`, advanced by its velocity over its age. Only the robot's own sightings are used
  (`confidence > 0`). With `input::NUSimGroundTruth` on, this is NUSim's ground truth ball.

### Ball-skill action

`q = clamp(default_joint_pos + action_scale * action, joint_lower, joint_upper)` with the file's gains. The policy
drives the head (it was trained looking for the ball), rate limited to `head_rate_limit`. A ball skill that does not
follow the walk blends in from the measured pose over `handoff_blend`.

### Hand-overs

Within `handover_window` of the other policy's last targets, the walk and the ball skills continue from them with no
blend: the new policy's previous action is `(last target - its default) / its scale`, and the ball skills' history
starts from the current step (mjlab `CONTROLLER_INTERFACE.md` section 4). Switching between ball skills (changing the
task's type) keeps the history and the previous action. The trained and tested hand-overs are walk → receive /
first touch / first-time pass, receive → dribble (ball still, within 0.2 m), dribble → kick / air shot, and standing
→ kick. After a kick the robot stands and does not chase the ball.

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
