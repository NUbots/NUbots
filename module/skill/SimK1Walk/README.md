# SimK1Walk

Runs the mujoco_playground K1 joystick walk policy (79-obs / 22-action ONNX) on the
robot side at 50 Hz and streams the resulting joint position targets to
`platform::Booster::HardwareIO` as `message::booster::BoosterLowCmd`, which forwards
them to the Booster SDK `rt/joint_ctrl` topic (honoured in CUSTOM mode).

It is the walk used by the `nusim/*` roles: NUSim ignores the firmware `Move()` RPC that
`skill::K1Walk` drives, so in sim locomotion inference lives in the NUbots stack and the
simulator only tracks servo joint commands. The robot roles use `skill::K1Walk`.

## Consumes

- `message::skill::Walk` (Director task) with the target velocity
- `message::platform::RawSensors` for joint feedback, gyro and the IMU attitude
- `message::booster::BoosterHeadRot` for the head targets (the policy does not own the head)
- `message::booster::BoosterModeState` (cached, optional) — logged with every observation
  so a capture can be segmented by the mode the robot was actually in

## Emits

- `message::booster::BoosterLowCmd` (22 motors, SDK JointIndexK1 serial order)
- `message::booster::BoosterMode` (CUSTOM) when the walk task starts
- `message::behaviour::state::WalkState`

## Contract

The observation/action layout is pinned in NUSim `docs/OBS_ACTION_CONTRACT.md`.

Two things are instrumented for hardware bring-up:

- At `log_level: TRACE` every tick emits `WALKOBS <tick> mode=<K1Mode> t=<s> dt=<s>` plus
  all 79 observations.
- Commanded joint positions are clamped to `joint_lower`/`joint_upper` (the trained model's
  ranges) and any clamp is logged at WARN
