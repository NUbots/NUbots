# ExtrinsicsCalibration

## Description

Automatically tunes a single camera's extrinsic offsets (`roll_offset`, `pitch_offset`, `yaw_offset`) so that
the offsets no longer have to be hand-tuned after a game.

The module assumes the robot is placed **static at the centre of the field, looking straight toward the goal**,
and that the **field type is known** (lab or RoboCup field) via the `FieldDescription`. Because the field
dimensions are known, the ground-truth positions of the field landmarks (X, L and T intersections) are known to
a high degree of accuracy.

The YOLO network detects the field landmarks and projects them onto the ground plane. The module re-projects
those detections under candidate extrinsic offsets and minimises the total squared distance between each
detection and the landmark it should correspond to.

### How it works

1. **Recover the geometry without changing the vision pipeline.** Each `FieldIntersection` already carries its
   world-space ground projection `rIWw` and the frame transform `Hcw`. The offset-independent camera-frame ray
   is recovered as `uICc = (Hcw · rIWw).normalized()`. The offset-free `Hwp` (world from head-pitch) and the
   base `Hpc` (head-pitch from camera, from URDF forward kinematics) are recomputed exactly as the `Camera`
   module does, so the candidate transform is `Hwc(θ) = Hwp · R_offset(θ) · Hpc_base`.
2. **Known field pose.** From the placement assumption, the field-from-world transform `Hfw` is synthesised:
   the torso is at the field centre `(x = y = 0)` facing the goal (`field_yaw`), with roll/pitch/height taken
   from `Sensors`.
3. **Initialise with the Hungarian algorithm.** As frames arrive, the detections are projected to field space
   at the current offsets and associated to the same-type ground-truth landmarks with the Hungarian algorithm
   (`utility::algorithm::determine_assignment`). Matched samples are accumulated.
4. **Refine with BOBYQA.** Once the collection window closes, NLopt's BOBYQA minimises
   `Σ ‖Hfw · project(θ) − rLFf‖²` over the offset deltas `del_roll`, `del_pitch`, `del_yaw` (bounded about the
   current offsets).
5. **Write back.** The optimised offsets are written back to `config/<robot>/Cameras/<camera>.yaml` as
   `"<degrees> * pi / 180"` expression strings (when `write_config` is enabled).

## Usage

Place the robot at the centre of the field facing the goal, then run the `extrinsicscalibration` role. Select
the camera to calibrate and the field type via configuration. Run again with the other camera to calibrate both.

## Consumes

- `message::vision::FieldIntersections` the YOLO X/L/T field landmark detections and frame transform
- `message::input::Sensors` for the offset-free kinematics (`Htw`, `Htx[HEAD_PITCH]`)
- `message::support::FieldDescription` for the field dimensions / ground-truth landmark positions

## Emits

- Nothing. The optimised offsets are logged and (optionally) written back to the camera config file.

## Dependencies

- `Eigen`
- `NLopt` for the BOBYQA optimisation
- `tinyrobotics` for camera forward kinematics
- `utility::algorithm::determine_assignment` (Hungarian algorithm)
- `utility::localisation::setup_field_landmarks`
