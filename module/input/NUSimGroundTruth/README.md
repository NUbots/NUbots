# NUSimGroundTruth

## Description

Bridges NUSim's test surface into NUClear. See NUSim `mujoco/module/SdkBridge/PROTOCOL.md` §6 for the wire contract.

- **Ground truth in:** `rt/nusim/gt/ball` and `rt/nusim/gt/robot` (`nav_msgs/Odometry`, world-frame pose and twist, stamped with the wall clock of the physics snapshot) become `message::booster::NUSimBallGroundTruth` and `NUSimRobotGroundTruth`.
- **Ball commands out:** `message::booster::NUSimBallCommand` goes to `rt/nusim/ball_command`, which places the ball and sets its velocity, in the simulator world or the robot's yaw frame. It can roll the ball without slipping.
- **Target marker out:** the walk planners' target goes to `rt/nusim/target_marker`, and NUSim's viewer draws it as an orange disc on the floor with an arrow along the target heading. It is taken from `message::planning::WalkToDebug` (which PlanWalkPath, MPCWalkPath and ModelledMPCWalkPath emit), in the robot frame, so it shows where the planner is aiming according to localisation. With a ground-truth `Field`, as in the walk benchmarks, that is the true target. Turn it off with `show_walk_target`. A `message::booster::NUSimTargetMarker` sends any other target, and on shutdown the marker is removed. Only the viewer window draws it: the robot's camera never sees it.

The simulator world `{s}` is the scene's field frame, not the NUbots odometry world `{w}`. Compare estimates through the robot frame `{r}` instead.

Only useful against NUSim. On a real robot the topics never appear and the module stays idle.

## Usage

Add `input::NUSimGroundTruth` to a NUSim role, alongside a consumer such as `tools::BallLocalisationBenchmark`.

## Consumes

- `message::booster::NUSimBallCommand`
- `message::booster::NUSimTargetMarker`
- `message::planning::WalkToDebug`

## Emits

- `message::booster::NUSimBallGroundTruth`
- `message::booster::NUSimRobotGroundTruth`

## Dependencies

- Booster robotics SDK (DDS channels)
