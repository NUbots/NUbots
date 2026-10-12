# WalkTrackBenchmark

## Description

Benchmarks following a moving target in NUSim against the simulator's ground truth: how well a walk planner keeps up
with something like a dribbled or rolling ball, rather than how it arrives at a fixed pose (that's
[WalkPathBenchmark](../WalkPathBenchmark/README.md)).

The target is given to the planner every tick as a `WalkToFieldPosition` with `stop_at_target` off, so the whole path
is exercised (WalkToFieldPosition → the role's planner → the walk policy) but WalkToFieldPosition never stops the walk.
Planners see the target's current pose only, not its velocity, so this measures how well each one chases it.

Each track's target first waits at its start until the robot has approached it (within `approach.position_error` and
`approach.heading_error` for `approach.settle_time`, or `approach.timeout`). It then moves along its segments, and every
tick until the end of the last segment is scored. A segment sets the speed, a constant acceleration (a negative one
stops the target, like a rolling ball), the curvature of the path, and a turn at its start (π doubles back). The robot
is asked to hold the travel direction plus the track's `heading_offset`. The defaults are a straight walk, a half
circle, a sideways walk, a rolling ball, an out-and-back and a weave, run at `speed_scales` 1.0 and 1.5. A scale runs
the same paths faster.

With `ground_truth_field` on, it also emits the `Field` from ground truth (the field frame is then NUSim's world), so
localisation error does not mask the planner's behaviour. Run NUSim with `--keyframe kickoff`: the robot starts at
(-3, 0) and the tracks are relative to its start pose.

One `TRACK` line is logged per track and a `SUMMARY` line at the end:

- `approach`: seconds the robot took to reach the start (`(timeout)` if it didn't, and the track ran anyway)
- `duration`: seconds the target moved, and so was scored
- `pos_mean`, `pos_p90`, `pos_max`: position error (m) over the scored ticks
- `yaw_mean`, `yaw_p90`: heading error (rad)
- `lag`: mean error along the travel direction (m), positive when the robot is behind the target
- `cross`: mean distance off the target's path, across the travel direction (m)
- `lost`: share of scored ticks with the position error over `lost_distance`
- `falls`: torso dropped below `fall_height`

## Usage

Run NUSim from the `tumminello/ball-ground-truth-and-shooter` branch (it publishes `rt/nusim/gt/robot`), then a role:

```bash
./b run sim/soccer --keyframe kickoff --headless       # in NUSim
./b run nusim/walktrackbenchmark                       # PlanWalkPath
./b run nusim/walktrackbenchmark_mpc                   # MPCWalkPath
./b run nusim/walktrackbenchmark_modelled_mpc          # ModelledMPCWalkPath
```

Configure the tracks and thresholds in `WalkTrackBenchmark.yaml`. The per-tick trajectory, target and walk command go
to `csv_directory`, one CSV per run named by its local start time as DataLogging names its recordings
(`recordings/walk_track_benchmark/YYYYMMDDTHH_MM_SS.csv`). `phase` is 0 while approaching and 1 while scored, and `t`
is seconds into the phase.

## Consumes

- `message::booster::NUSimRobotGroundTruth` the robot's true pose
- `message::input::Sensors` for Hrw, to build the ground-truth Field
- `message::eye::DataPoint` the planner's "Walk Command" graph, for the CSV

## Emits

- `message::strategy::WalkToFieldPosition` Task for the target, with `stop_at_target` off
- `message::strategy::FallRecovery` Task
- `message::booster::NUSimBallCommand` to park the ball, when `park_ball` is set
- `message::localisation::Field` from ground truth, when `ground_truth_field` is on

## Dependencies

- `input::NUSimGroundTruth`
