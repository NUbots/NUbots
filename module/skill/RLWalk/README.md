# RLWalk

## Description

Walks the NUgus with a reinforcement-learning policy trained in mjlab, exported to ONNX and run with OpenVINO.

At 50 Hz, while `Stability` is at least `DYNAMIC`, it:

1. **Builds the observation (71 values), in the training layout:**

   - gyroscope (3);
   - gravity in the torso frame (3);
   - joint positions relative to `default_pose` (20);
   - joint velocities (20);
   - the last raw action (20);
   - the walk command `velocity_target` (3);
   - the gait phase as sin/cos (2), advanced by control step with period `gait_period`.

   Joints are reordered between the NUbots `ServoID` order and the mjlab training model's order.

2. **Runs the policy.** The raw action is scaled by `action_scale` and added to `default_pose`, then each servo is clipped to `servo_limits`. Below `command_velocity_threshold` the default pose is held instead, because the policy steps in place at zero command.
3. **Commands the arms and legs** as a `Limbs` task, with separate arm and leg gains. The head is left to `skill::Look`.

`model.device` picks the OpenVINO device, falling back to CPU if the GPU fails. At `log_level: DEBUG`, the observation, actions and control-loop timing (period, jitter, gait-clock drift) are graphed and logged.

## Usage

Include `skill::RLWalk` in the role, and emit `message::skill::Walk` tasks.

- `roles/rl_keyboardwalk.role` drives it with the keyboard on the robot.
- `roles/webots/rl_keyboardwalk.role` does the same in Webots.

The policy is `data/model.onnx`. `action_scale`, `gait_period` and `default_pose` must match its training. `config/webots/RLWalk.yaml` holds the Webots gains and runs inference on the CPU.

## Consumes

- `message::skill::Walk`: the walk task, with the desired velocity (x, y, yaw).
- `message::input::Sensors`: gyroscope, `Htw` for gravity, and servo positions and velocities.
- `message::behaviour::state::Stability`: the policy only runs at `DYNAMIC` or above.

## Emits

- `message::actuation::Limbs` Task: arm and leg servo commands.
- `message::behaviour::state::WalkState`: `STOPPED` when the walk task starts or stops, and `WALKING` on each policy update.
- `message::behaviour::state::Stability`: `UNKNOWN` when the configuration loads, so walk behaviours have an initial state.

## Dependencies

- OpenVINO and TBB
- Eigen
- Director
