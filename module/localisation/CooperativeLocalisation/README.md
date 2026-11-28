# CooperativeLocalisation

## Description

Tracks the ball and the other robots cooperatively: this robot's own detections and the detections its teammates share
over team communication are fused into one joint filter per target, on the field {f}. Every robot runs its own copy, so
each robot gets a ball and robot estimate that also uses what its teammates can see.

Ported from the cooperative tracking evaluation in Sebastian Brooker's soccer-sim (FYP). There, on one 12 minute 3v3
Booster sim match with detections from the NUbots YOLO models, the joint filter had the ball inside its own 95 % region
(of area at most 3 m²) 48 % of the time and within 0.5 m 53 % of the time, against 8 % and 16 % for the NUbots per-robot
filters. It complements the single-robot tracking in `localisation::BallLocalisation` and
`localisation::RobotLocalisation`, which are unchanged, with joint updates from every robot.

For now the cooperative ball is used as a backup: `localisation::BallLocalisation` falls back to it when the robot has
not seen the ball itself for a while (see its `use_cooperative_ball`). The cooperative robots are emitted for anything
that wants them, but nothing uses them yet.

### Filter

Each target (the ball, or one robot) has its own square-root extended Kalman filter (`SquareRootEKF.hpp`) over
`[x, y, vx, vy]` on the field. The process model is constant velocity with velocity damping (`friction`, the ball
slowing as it rolls), driven by white acceleration noise. Both updates are a single QR factorisation, so the covariance
stays positive definite even when a detection is very tight across the line of sight and very loose along it.

### Measurement

A detection is a range and bearing from the observing camera, `y = [u; r]` with `u` the unit vector from the camera to
the detection on the field and `r` the range, linearised at the track mean (the extended Kalman filter update). The
noise is:

- bearing: the detector's `bearing_std`, the observer's heading std, and the observer's position std across the line
  of sight divided by the range
- range: `range_base + range_quad * r^2` (ground projecting a pixel), and the observer's position std along the line of
  sight

The observer's position covariance is inflated (`observer_inflation`), as the observer's pose error is correlated over
time while the filter treats each detection as independent.

Teammates' detections arrive late (up to a comms period, 0.5 s). A detection from `lag` seconds ago measures where the
target was then, `p - lag * v`, plus the process noise over the lag, so it can be fused into a track that is already at
the current time.

### Tracking

Each robot's camera frame, own or shared, is one scan:

1. Robot detections consistent with a teammate's reported position (or this robot's own) are teammates, and are dropped.
   Detections off the field are dropped.
2. Every track is predicted to now and every detection is gated against every track of the same kind (squared
   Mahalanobis distance on the full innovation covariance, `gate_chi2`). Detections are assigned one-to-one, lowest
   cost first; the cost adds `log det` of the innovation covariance so a tighter track wins at equal distance. A robot
   track sitting on the observer can't take its detections.
3. Assigned detections are fused. Unassigned detections start a new track at the detection, with the detection's own
   covariance (deep along the line of sight, narrow across it), unless it is a robot closer than `min_spawn_range`.
4. A ball detection with a range std above `spawn_max_range_std` (far away) never starts a track by itself. It waits up
   to `pending_window` for another robot's poor detection whose ray crosses it at a decent angle near both measured
   ranges, and the ball starts at the intersection.
5. If a scan's ball detections all fail the ball track's gate `ball_reset_count` scans in a row, the ball track is
   wrong: it is ended and the detection starts a new one.
6. Tracks that took different detections in the same scan are different objects and are never merged.

Tracks are removed when their largest position std grows beyond `robot_lost_std` / `ball_lost_std`, or they leave the
field. Two confident tracks that agree (`merge_chi2`, `merge_max_std`) are duplicates and the one with fewer fusions is
removed. There is only one ball, so only the ball track with the smallest position covariance is kept.

## Usage

Include this module with `network::RobotCommunication` (which shares the detections, see its `share_detections`) and
`localisation::BallLocalisation` (which falls back to the cooperative ball). Detections are only used, or shared, while
`message::localisation::Field` is `localised`.

In NUSim, team messages reach the other robots because NUSim's match launcher sets `broadcast_ip` to
`255.255.255.255`, and each team uses its own port (`10000 + team_id`), so the two teams never hear each other.

## Consumes

- `message::vision::Balls` this robot's ball detections
- `message::vision::Robots` this robot's robot detections
- `message::input::RoboCup` teammates' positions and the detections they share
- `message::localisation::Field` to put this robot's detections on the field, and the estimates back in world space
- `message::support::FieldDescription` for the field bounds and the ball radius
- `message::support::GlobalConfig` for this robot's player id

## Emits

- `message::localisation::CooperativeBall` the cooperative ball in world {w} space, and which robots have seen it
  recently
- `message::localisation::CooperativeRobots` the cooperative robot tracks (opponents) in world {w} space
- `message::localisation::CooperativeDetections` this robot's latest detections on the field, for
  `network::RobotCommunication` to share

## Dependencies

- Eigen
