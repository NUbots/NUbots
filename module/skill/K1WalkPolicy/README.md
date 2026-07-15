# K1WalkPolicy

Runs the mujoco_playground K1 joystick walk policy (82-obs / 22-action ONNX) on the
robot side at 50 Hz and streams the resulting joint position targets to
`platform::Booster::HardwareIO` as `message::booster::BoosterLowCmd`, which forwards
them to the Booster SDK `rt/joint_ctrl` topic (honoured in CUSTOM mode).

This replaces `skill::K1Walk`'s `Move()` RPC path: locomotion inference lives in the
NUbots stack, and the robot/simulator only tracks servo joint commands.

## Consumes

- `message::skill::Walk` (Director task) with the target velocity
- `message::platform::RawSensors` for joint feedback, gyro and the IMU attitude
- `message::booster::BoosterOdometry` to estimate the body-frame linear velocity
- `message::booster::BoosterHeadRot` for the head targets (the policy does not own the head)

## Emits

- `message::booster::BoosterLowCmd` (22 motors, SDK JointIndexK1 serial order)
- `message::booster::BoosterMode` (CUSTOM) when the walk task starts
- `message::behaviour::state::WalkState`

## Contract

The observation/action layout is pinned in NUSim `docs/OBS_ACTION_CONTRACT.md`. The
base linear velocity observation is estimated by differentiating the Booster odometry
(low-passed, z unobservable), not by a privileged simulator sensor.
