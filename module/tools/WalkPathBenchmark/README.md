# WalkPathBenchmark

## Description

Benchmarks walking to a pose in NUSim against the simulator's ground truth. It runs a fixed chain of
`WalkToFieldPosition` trials, so the whole walk-to-pose path is exercised (WalkToFieldPosition → PlanWalkPath →
the walk policy), and scores each trial on the true robot pose rather than the robot's own estimate.

With `ground_truth_field` on, it also emits the `Field` from ground truth (the field frame is then NUSim's world), so
localisation error does not mask the planner's behaviour. Run NUSim with `--keyframe kickoff`: the robot starts at (-3, 0) and the targets are relative to its start pose.

Each trial finishes once the robot has stayed within `reach_position_error` and `reach_heading_error` of the target
for `settle_time`. It fails as `SHORT` if the robot comes to rest outside the target for `settle_time`, and as
`TIMEOUT` after `trial_timeout`. One `TRIAL` line is logged per trial and a `SUMMARY` line at the
end:

- `time`: seconds until the robot entered its final stay at the target (came to rest when `SHORT`, the trial time
  on a timeout)
- `reach`: seconds until it first reached the target
- `pos_err`, `yaw_err`: error when the trial finished
- `path`, `path_ratio`: distance walked, and its ratio to the straight-line distance
- `max_err_after_reach`: the largest position error after first reaching the target (overshoot and drift)
- `falls`: torso dropped below `fall_height`

## Usage

Run NUSim from the `tumminello/ball-ground-truth-and-shooter` branch (it publishes `rt/nusim/gt/robot`), then the
role:

```bash
./b run sim/soccer --keyframe kickoff --headless   # in NUSim
./b run nusim/walkpathbenchmark                    # in NUbots_K1
```

Configure the targets and thresholds in `WalkPathBenchmark.yaml`. The per-tick trajectory and walk command go to
`csv_path`.

## Consumes

- `message::booster::NUSimRobotGroundTruth` the robot's true pose
- `message::input::Sensors` for Hrw, to build the ground-truth Field
- `message::eye::DataPoint` the "Walk Command" graph from PlanWalkPath, for the CSV

## Emits

- `message::strategy::WalkToFieldPosition` Task for the current target
- `message::strategy::FallRecovery` Task
- `message::localisation::Field` from ground truth, when `ground_truth_field` is on

## Dependencies

- `input::NUSimGroundTruth`
