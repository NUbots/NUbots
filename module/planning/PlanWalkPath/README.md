# PlanWalkPath

## Description

Plans walk velocity commands for the Booster K1's RL walk policy (commands flow PlanWalkPath → `skill::Walk` →
K1WalkPolicy, or K1Walk for the SDK's built-in gait).
The structure follows the planners other K1 teams run (Booster robocup_demo, HTWK Firmware-Salvador).

Three types of walk path planning are in this module:

1. Walking towards a target pose, using a two-regime proportional controller:
   - **Far** (beyond `max_align_radius`): face the target and drive forward only — the RL gait is
     fastest and most stable walking forward. Forward speed is proportional to distance and gated by
     the heading error, so the robot rotates on the spot when badly misaligned and slows while turning.
   - **Near**: omnidirectional proportional approach, decelerating to zero at the target and blending
     the commanded heading from "face the target" to the final desired heading.
     It avoids robots by finding the first robot that intersects with the path, grouping it with any
     close by robots, then walking to a spot to the left or right of the group. Once it's cleared the
     group, it continues on to either avoid another robot or walk straight to the target.
2. Turn on the spot, with the small forward velocity `rotate_velocity_x` the walk needs to step. Supports both
   directions.
3. Pivot around a point (the ball). Orbits a point `pivot_radius` ahead while facing it, using the
   orbit kinematics `vy = -vtheta * pivot_radius`. Supports both directions.

4. Adjust: orbit the ball at `adjust_range` to fix the approach angle before a kick.

Every mode passes its velocity through per-axis exponential smoothing (`tau`, assuming the 10 Hz rate the behaviour
tree re-emits tasks at) and then the dead zone, and emits the result as the `skill::Walk` Task. The RL walk stands
still for any command too small to start stepping and never turns on the spot, however fast the turn, so the dead
zone (`walk_path::apply_dead_zone`):

- snaps axes below `zero_tolerance` to zero, and an all-zero command is a stop;
- scales a small translation onto the ellipse with semi-axes `min_velocity` x and y, keeping its direction (the turn
  passes through, as the walk tracks small turns once stepping);
- makes a turn with no translation an on-the-spot turn, with `rotate_velocity_x` forward and at least
  `min_velocity` z;
- clamps everything to `max_velocity`.

`min_velocity` and `max_velocity` are measured from the policy in NUSim; `tools::WalkPathBenchmark` measures the
policy (its command sweep) and scores the whole walk to a pose against ground truth.

The smoother is reset when the robot recovers from a fall (Stability transition to DYNAMIC) so a
stale command cannot produce a velocity spike on resume.

## Usage

Include this module in the role and emit the provide Task for either reaction. The reactions cannot
run at the same time, since they all Need the Walk.

## Consumes

- `message::planning::WalkTo` Task requesting to walk towards a given pose in robot space.
- `message::planning::TurnOnSpot` Task requesting to turn on the spot.
- `message::planning::PivotAroundPoint` Task requesting to rotate around the ball, to align with a direction.
- `message::planning::Adjust` Task requesting to orbit the ball to fix the kick approach angle.
- `message::localisation::Robots` to find any robots to avoid when path planning to a target.
- `message::input::Sensors` to transform robot points into (our own) robot space.
- `message::behaviour::state::Stability` to reset the command smoother after a fall.

## Emits

- `message::skill::Walk` Task requesting to run the walk, giving the xy velocity (m/s) and rotational velocity (radians/s).
- `message::planning::WalkToDebug` and NUsight DataPoints ("Walk Proposal", "Smoothed Walk Command", "Walk Smoothing Difference", "Walk Command") for visualisation.

## Dependencies

- Director
- The K1WalkPolicy skill (or another consumer of `skill::Walk`)
- Mathematics intersection utility
