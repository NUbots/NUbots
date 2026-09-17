# PlanWalkPath

## Description

Plans walk velocity commands for the Booster K1, whose gait is the SDK's built-in closed-loop RL
locomotion policy (commands flow PlanWalkPath → `skill::Walk` → K1Walk → HardwareIO `Move(vx, vy, vtheta)`).
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
2. Turn on the spot. Without moving translationally, turn on the spot. Supports both directions.
3. Pivot around a point (the ball). Orbits a point `pivot_radius` ahead while facing it, using the
   orbit kinematics `vy = -vtheta * pivot_radius`. Supports both directions.

All three emit a `WalkProposal` Task, which this module re-consumes at a fixed rate to apply
per-axis exponential smoothing (`tau`) followed by dead-zone compensation: the RL policy does not
respond to very small commands, so nonzero commands are bumped up to `min_velocity`, while commands
below `zero_tolerance` are snapped to zero. The result is emitted as the `skill::Walk` Task.

The smoother is reset when the robot recovers from a fall (Stability transition to DYNAMIC) so a
stale command cannot produce a velocity spike on resume.

## Usage

Include this module in the role and emit the provide Task for either reaction. The reactions cannot
run at the same time, since they all Need the Walk.

## Consumes

- `message::planning::WalkTo` Task requesting to walk towards a given pose in robot space.
- `message::planning::TurnOnSpot` Task requesting to turn on the spot.
- `message::planning::PivotAroundPoint` Task requesting to rotate around the ball, to align with a direction.
- `message::planning::WalkProposal` its own intermediate velocity proposal, re-consumed for smoothing and dead-zone compensation.
- `message::localisation::Robots` to find any robots to avoid when path planning to a target.
- `message::input::Sensors` to transform robot points into (our own) robot space.
- `message::behaviour::state::Stability` to reset the command smoother after a fall.

## Emits

- `message::planning::WalkProposal` Task carrying the raw planned velocity before smoothing.
- `message::skill::Walk` Task requesting to run the walk, giving the xy velocity (m/s) and rotational velocity (radians/s).
- `message::planning::WalkToDebug` and NUsight DataPoints ("Walk Proposal", "Smoothed Walk Command", "Walk Smoothing Difference", "Walk Command") for visualisation.

## Dependencies

- Director
- The K1Walk skill (or another consumer of `skill::Walk`)
- Mathematics intersection utility
