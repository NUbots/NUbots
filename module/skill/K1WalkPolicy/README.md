# K1WalkPolicy

Runs an mjlab K1 velocity-tracking walk policy at 50 Hz on the robot side and streams the
resulting joint position targets as a Director-arbitrated `message::actuation::K1Servos`
subtask, which `platform::Booster::HardwareIO` forwards to the Booster SDK `rt/joint_ctrl`
topic (honoured only in CUSTOM mode).

This replaces `skill::K1Walk`'s `Move()` RPC path: locomotion inference lives in the NUbots
stack, and the robot/simulator only tracks servo joint commands.

## Consumes

- `message::skill::Walk` (Director task) with the target velocity
- `message::platform::RawSensors` for joint feedback, gyro and the IMU attitude
- `message::behaviour::state::Stability` — ticks are skipped while FALLEN, when
  `skill::K1GetUpPolicy` owns the low-level channel
- `message::booster::BoosterHeadRot` for the head targets (unless `head.policy_controlled`)
- `message::booster::BoosterModeState` (cached, optional) — logged with every observation so
  a capture can be segmented by the mode the robot was actually in

## Emits

- `message::actuation::K1Servos` (22 motors, SDK `JointIndexK1` serial order) as a subtask
- `message::booster::BoosterMode` (CUSTOM) when the walk task starts
- `message::behaviour::state::WalkState`

## Contract

The observation/action contract is **configured, not hardcoded**, because the checkpoints
that have flown on this robot do not share one. `K1WalkPolicy.yaml` pins it and
`load_model()` refuses any graph whose input/output element counts disagree, on the GPU and
on the CPU fallback alike.

The shipped checkpoint is trained in
[booster_mjlab](https://github.com/IntelligentRoboticsLab/booster_mjlab), task
`Mjlab-Velocity-Flat-Booster-K1`, experiment `k1_velocity_amp_symmetric_muon` (PPO with an
AMP style reward, symmetry data augmentation and the Muon optimizer), W&B run `poi732yy`,
iteration 29999 (`data/k1_walk_mjlab_poi732yy_29999.onnx`).

The older checkpoints in `data/` are kept but are **not** loadable against the shipped
config; see [Switching checkpoints](#switching-checkpoints).

### ONNX graph I/O

| Tensor | Name | Shape | Type |
|---|---|---|---|
| Input | `obs` | `[1, 75]` | `float32` |
| Output | `actions` | `[1, 22]` | `float32` |

The graph is `EmpiricalNormalization` → 512/256/128 ELU MLP; the observation normalizer is
**baked in**, so the deployment side applies no normalization of its own. There is no action
clipping either — the training runner left `clip_actions` unset.

### Observation layout (75 floats)

One frame, no observation history (`observation_terms_history_length` is 0 for every term in
the exported metadata), so `history_window: 1`.

| Offset | Count | Field | Notes |
|---|---|---|---|
| 0  | 3  | Angular velocity (gyro), body frame | rad/s, from `RawSensors::gyroscope`; training reads the `imu_ang_vel` gyro on the trunk `imu` site. |
| 3  | 3  | Projected gravity, body frame | world `(0,0,-1)` rotated into the trunk frame by the firmware attitude estimate; unit vector (upright ⇒ `(0,0,-1)`). |
| 6  | 22 | `q − default_pose` | rad. |
| 28 | 22 | `dq` | rad/s, **unscaled**. |
| 50 | 22 | `last_action` | previous **raw** network output, before scaling and before the joint-range clamp. |
| 72 | 3  | Command `[vx, vy, wz]` | body-frame planar velocity (m/s, m/s, rad/s), passed through as-is. |

Total `3 + 3 + 22 + 22 + 22 + 3 = 75`.

No per-term scaling and no clipping: every mjlab observation term has scale 1.0. The
observation noise the actor saw in training (`Unoise` on gyro, gravity, `q` and `dq`) is
domain randomization, not something to reproduce here.

**No base linear velocity.** There is no measured base linear velocity on the real K1 in
CUSTOM mode, so it is a critic-only privileged quantity in training and nothing here
estimates it.

**No gait clock.** Unlike the previous checkpoint, this policy has no clock term: the gait
comes out of the AMP style reward and the air-time/clearance/slip terms. `gait_clock: false`
drops the two clock floats from the frame and stops the phase integrator; `gait_period` and
`command_threshold` are then unused and kept only so a clocked policy is a config change.

### Joint order

The mjlab model's 22 joints (`K1_JOINT_ORDER` in `booster_mjlab/robots/booster_k1/
k1_constants.py`, and the `joint_names` metadata prop on the ONNX) are exactly
`JointIndexK1`, so policy index `j` is `JointIndexK1` index `j` and **no permutation table is
needed** — `policy_joints` is simply `0..21`.

```
 0 HeadYaw            1 HeadPitch
 2 LeftShoulderPitch  3 LeftShoulderRoll  4 LeftElbowPitch  5 LeftElbowYaw
 6 RightShoulderPitch 7 RightShoulderRoll 8 RightElbowPitch 9 RightElbowYaw
10 LeftHipPitch  11 LeftHipRoll  12 LeftHipYaw  13 LeftKneePitch  14 LeftAnklePitch  15 LeftAnkleRoll
16 RightHipPitch 17 RightHipRoll 18 RightHipYaw 19 RightKneePitch 20 RightAnklePitch 21 RightAnkleRoll
```

### Action layout (22 floats) & application

Joint target offsets around the training keyframe:

```
q_ref[j]    = default_pose[j] + action_scale * action_scale_joint[j] * action[j]
q_ref[head] = clamp(latest BoosterHeadRot)          # unless head.policy_controlled
q_cmd       = clamp(lerp(measured, q_ref, handoff_blend), joint_lower, joint_upper)
```

- `default_pose` is the booster_mjlab `HOME_KEYFRAME`: arms down (shoulder roll ∓1.4, elbow
  yaw ∓0.4) over a deeper crouch than the previous policy (hip pitch −0.4, knee 0.8, ankle
  pitch −0.4, against −0.2 / 0.4 / −0.2).
- `action_scale_joint[j] = 0.25 · effort_limit[j] / kp[j]` (booster_train convention);
  `action_scale` is a global multiplier (1.0 for this policy).
- The PD gains sent in the LowCmd are the training-time gains: the actuator groups' explicit
  `stiffness`/`damping`, **not** the armature formula the previous policy used. They are
  substantially stiffer — arms 3.948 → 10, hips 30.2 → 80, knees 60.4 → 80, ankles
  35.7 → 50 — which is the single largest behavioural difference between the two
  checkpoints on hardware.
- `handoff_blend` (0.3 s) cross-fades from the measured pose into the policy target on entry
  to CUSTOM. Training always starts at the default pose; deployment starts from wherever
  PREP left the robot. The deeper crouch makes this a larger move than before.
- `last_action` is the **raw** network output, kept unclamped and unscaled, because that is
  what the observation expects. Only `q_cmd` is clamped.

### The head

The policy emits head actions and observes the head, but on the robot vision owns the head,
so `head.policy_controlled` defaults to `false`: the two head targets are replaced with the
latest `BoosterHeadRot` and tracked with `head.kp`/`head.kd` (10.0 / 0.5, higher than the
training head kp of 4.0 because there is no gravity feed-forward on this side of the wire).

The head stays in `policy_joints` regardless, because the policy was *trained* observing it —
dropping it would change the observation length. The policy therefore still sees its own raw
head action fed back and the measured head position; only the applied target differs from
training. That measured head position can sit up to 0.785 rad off the trained posture (the
`upper_body_posture` reward held the head within 0.05 rad of default), which is the one
knowingly out-of-distribution input on this path. Set `head.policy_controlled: true` to hand
the head back to the policy, e.g. when reproducing a training rollout.

### Loop rate

The trained control period is 0.02 s (mjlab `decimation` 4 × `timestep` 0.005). A loop period
off that by more than 5 ms is counted and reported at WARN every 2 s. That is a bug to fix,
not to compensate for. Without an observation window a slow loop is less damaging than it was
for the previous policy, but the joint velocities and the action-rate dynamics the policy was
trained on still assume 50 Hz.

### Saturation

At `a = ±1` this policy stays inside the mechanical stops on every joint but two, because the
action scale comes from the effort limit rather than from the joint range:

| Joint | `JointIndexK1` | Overshoot |
|---|---|---|
| Head Pitch | 1 | 0.026 rad |
| Left/Right Shoulder Roll | 3, 7 | 0.010 rad |

Clamping is counted per tick and logged at DEBUG. (For comparison, the previous checkpoint
overshot by up to 0.737 rad on the elbow yaws.)

### Command envelope

The training curriculum's final envelope is `vx ∈ [-1.5, 1.75]`, `vy ∈ [-1.75, 1.75]`,
`wz ∈ [-1.5, 1.5]` — wider than the previous policy's on every axis, and wide enough to
contain the `wz = 1.5` that `PlanWalkPath`'s ball-adjust mode can ask for. Commands are
passed through unclipped, as in training.

## Switching checkpoints

The module is parameterised, so moving between checkpoints is a `K1WalkPolicy.yaml` change.
For the previous mjlab checkpoint (`k1_walk_mjlab_2p0glew7_14999.onnx`, `[1, 1775]` →
`[1, 20]`) restore all of:

| Key | poi732yy | 2p0glew7 |
|---|---|---|
| `history_window` | 1 | 25 |
| `gait_clock` | false | true |
| `policy_joints` | 0..21 | 2..21 |
| `kp` (arms / hips / knees / ankles / head) | 10 / 80 / 80 / 50 / 4 | 3.948 / 30.201 / 60.402 / 35.692 / 3.948 |
| `kd` (arms / hips / knees / ankles / head) | 1 / 4 / 4 / 2 / 0.25 | 0.251 / 3.605 / 4.807 / 4.26 / 0.251 |
| `action_scale_joint` | 0.375 head, 0.35 arms, 0.2125/0.2375/0.1197/0.35/0.1915 legs | 0.3799 head, 0.88656 arms, 0.562895/0.885865/0.536534/0.463561/0.268267 legs |
| `default_pose` shoulder roll / elbow yaw / hip pitch / knee / ankle pitch | ∓1.4 / ∓0.4 / −0.4 / 0.8 / −0.4 | ∓1.3 / ∓0.15 / −0.2 / 0.4 / −0.2 |

The `k1_walk*.onnx` mujoco_playground checkpoints (79-obs / 22-action, single frame, their own
observation layout) are older still and have no config in this module.

## Instrumentation

At `log_level: TRACE` every tick emits `WALKOBS <tick> mode=<K1Mode> t=<s> dt=<s>` followed
by the whole newest observation frame. `tools/analysis/segment_walk_log.py` splits a capture
into MOVING/FROZEN runs before reporting statistics — the first hardware log was 81.5%
robot-standing-still, and whole-file statistics from it were misleading enough to be quoted
as findings. Note that the frame width changed with the checkpoint (75 floats here, 71 for
2p0glew7), so a capture is only comparable against itself.

## Re-exporting the policy

booster_mjlab's `VelocityOnPolicyRunner.save` exports an ONNX on every checkpoint save into
the run's log directory (`logs/rsl_rl/<experiment>/<date>/<date>.onnx`), overwriting it each
time, and attaches the contract as ONNX metadata props: `joint_names`, `joint_stiffness`,
`joint_damping`, `default_joint_pos`, `action_scale`, `observation_names`,
`observation_terms_scale`, `observation_terms_history_length`, `observation_terms_clip`,
`command_names`. Those props are the source of truth for the arrays in `K1WalkPolicy.yaml` —
read them with `onnx.load(path).metadata_props` and check them against the config before
trusting a new checkpoint.
