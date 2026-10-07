# K1BlockPolicy

## Description

Runs the K1 goalie's block (save) policy at 50 Hz. It streams joint position targets to `platform::Booster::HardwareIO` as a Director-arbitrated `K1Servos` subtask in CUSTOM mode. This is the same low-level path as `skill::K1KickPolicy` and `skill::K1WalkPolicy`.

The policy stays on its feet.
- While the `Block` task is **inactive**, it holds the goalie's ready stance.
- While it is **active**, it shuffles, side-steps or extends a limb to put a body part on the ball's path.

It is driven by a **command**, not by raw ball state. The save planner owns the ball prediction and the choice of skill. The policy's capability envelope is measured over exactly the quantities the planner commands, so the behaviour system knows what the policy can and can't reach.

The policy is trained in the mjlab goalkeeper task, which must produce a graph matching the contract below.

## Usage

- Add `skill::K1BlockPolicy` and `actuation::K1Servos` to the role.
- Emit a `message::skill::Block` Task and re-emit it every tick with the latest command.
- `purpose::Tester`'s `block_policy_priority` emits an inactive `Block` for smoke testing.

Until a trained checkpoint exists, `model_path` points at `k1_block_hold_stance_T1.onnx`. It outputs zero actions, so the robot holds `default_pose`. Regenerate it, for example for a different history window, with:

```sh
python3 tools/policy/make_block_hold_onnx.py --history-window 1
```

At `log_level: TRACE`, every tick logs `BLOCKOBS <tick>` followed by the observation frame.

**The placeholder does not balance.** Zero actions command a static default pose, and at the soft
training gains (hip 18–30, knee 60 N·m/rad) the robot sags and topples within a few seconds; the
trained policy balances actively. In NUSim (19 Sep 2026) the placeholder held the stance in CUSTOM
mode for 50 s with NUSim's fallback gains (legs 80, ankles 150; `mujoco/config/gains.yaml`) and fell
after about 3 s with the training gains. Use stiffer `kp`/`kd` for smoke tests, and never run the
placeholder on the robot with the training gains.

## Observation / action contract (v0)

This is the single source of truth for the interface between this module and the mjlab goalkeeper task. Change both sides together, and bump the version when you do.

### Frames

The robot frame `{r}` is the planar odometry base frame: origin at the torso projected to the ground, x forward, y left, z up. In mjlab this is the yaw-only frame at the root (Trunk) link's xy position.

### Policy joints (20, in policy order)

These are the JointIndexK1 indices 2–21: left arm (shoulder pitch/roll, elbow pitch/yaw), right arm, left leg (hip pitch/roll/yaw, knee, ankle pitch/roll), right leg. This equals the mjlab joint order of `K1_POLICY_JOINT_REGEX`, since the K1 XML orders its joints in SDK order.

The head (indices 0–1) is **not** policy controlled. It tracks the latest `BoosterHeadRot` (K1Look), because vision owns the head.

### One observation frame (70 floats)

| Offset | Count | Field | Robot source | mjlab term |
|---|---|---|---|---|
| 0 | 3 | Angular velocity, IMU/body frame (rad/s) | `RawSensors.gyroscope` | `builtin_sensor(robot/imu_ang_vel)` |
| 3 | 3 | Projected gravity, body frame (unit, upright is `(0, 0, -1)`) | from `RawSensors.imu_rpy` | `projected_gravity` |
| 6 | 20 | `q − default_pose` (rad) | servo `present_position` | `joint_pos_rel` |
| 26 | 20 | `dq` (rad/s) | servo `present_velocity` | `joint_vel_rel` |
| 46 | 20 | Last action (raw policy output) | previous inference | `last_action` |
| 66 | 4 | Block command `[active, dy, t, v]` | `message::skill::Block` | block command term |

**The command:**
- `active`: 1 while a shot is being defended, otherwise 0.
- `dy`: lateral offset (m, +left) in `{r}` where the ball's predicted path crosses the robot's frontal plane (x = 0). Clipped to ±1.5.
- `t`: predicted time (s) until that crossing. Clipped to [0, 3].
- `v`: current ball speed (m/s). Clipped to [0, 6].
- When `active` is 0, all four entries are 0.
- The planner recomputes the command every tick from the latest ball estimate, so the policy is closed-loop through it. In training, the command must come from a *simulated estimate*, with the noise, delay and update rate of the real ball filter, not from ground truth.

### History

- `history_window` frames of the layout above, time-major with the **oldest frame first**, flattened into one input of `history_window × 70` floats.
- The buffer is seeded by repeating the first frame when a Block task starts, which matches mjlab's `CircularBuffer` backfill on reset.
- `history_window: 1` means a plain MLP. For a history policy, mjlab's `OnnxHistoryPolicy` expects exactly this layout.

### ONNX graph

| Tensor | Name | Shape | Type |
|---|---|---|---|
| Input | `obs` | `[1, history_window × 70]` | float32 |
| Output | `actions` | `[1, 20]` | float32 |

- The observation normalisation is baked into the graph, and the output is the deterministic action. mjlab's exporter does both.
- The module checks both sizes on load and refuses a mismatched graph, even on the CPU fallback.

### Action

- For policy joint `k` (JointIndexK1 index `j`): `q_target[j] = default_pose[j] + action_scale_joint[j] × a[k]`. This is mjlab's `JointPositionAction` with `use_default_offset=True`.
- `kp`, `kd`, `action_scale_joint` and `default_pose` are the training values (see the config).
- Commanded positions are clamped to `joint_lower` / `joint_upper`.

### Rate and hand-off

- The policy runs at 50 Hz, the same as the training control step (0.02 s).
- On `Start<Block>`, the command cross-fades from the measured pose into the policy target over `handoff_blend` seconds.

## Consumes

- `message::skill::Block`: Director task carrying the block command.
- `message::platform::RawSensors`: servo feedback, gyro, IMU attitude.
- `message::booster::BoosterHeadRot`: head targets.

## Emits

- `message::actuation::K1Servos` Task: 22 motors, SDK JointIndexK1 serial order.
- `message::booster::BoosterMode` (CUSTOM) when the Block task starts.

## Dependencies

- Director
- ONNX Runtime through `utility::onnx::ONNXRuntime`
