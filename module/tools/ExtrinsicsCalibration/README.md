# ExtrinsicsCalibration

## Description

Automatically tunes a single camera's extrinsic offsets (`roll_offset`, `pitch_offset`, `yaw_offset`) so that the offsets no longer have to be hand-tuned after a game.

The module assumes the robot is placed **static at the centre of the field, looking straight toward the goal**, and that the **field type is known** (lab or RoboCup field) via the `FieldDescription`. Because the field dimensions are known, the ground-truth positions of the field landmarks (X, L and T intersections) are known to a high degree of accuracy.

**Field dimensions MUST be measured first and the necessary values must be updated in FieldDescription.yaml before running this binary.**

The YOLO network detects the field landmarks and projects them onto the ground plane. The module re-projects those detections under candidate extrinsic offsets and minimises the total squared distance between each detection and the landmark it should correspond to.

### How it works

1. **Recover the geometry without changing the vision pipeline.** Each `FieldIntersection` already carries its
   world-space ground projection $r^{w}_{I/W}$ and the frame transform $H^{c}_{w}$. The offset-independent camera-frame ray
   is recovered as $u_{I/C}^c = (H^{c}_{w} \cdot r^{w}_{I/W}).normalized()$. The offset-free $H^{p}_{w}$ (world from head-pitch) and the
   base $H^{c}_{p}$ (head-pitch from camera, from URDF forward kinematics) are recomputed exactly as the `Camera`
   module does, so the candidate transform is $H^{c}_{w}(\theta) = H^{c}_{p,\text{base}} \cdot R_{\text{offset}}(\theta) \cdot H^{p}_{w}$.
2. **Known field pose.** From the placement assumption, the field-from-world transform $H^{f}_{w}$ is synthesised:
   the torso is at the field centre $(x = y = 0)$ facing the goal, with roll/pitch/height taken
   from `Sensors`.
3. **Initialise with the Hungarian algorithm.** As frames arrive, the detections are projected to field space at the current offsets and associated to the same-type ground-truth landmarks with the Hungarian algorithm
   (`utility::algorithm::determine_assignment`). Matched samples are accumulated.
4. **Refine with BOBYQA.** Once the collection window closes, NLopt's BOBYQA minimises
   $$\sum \left\|H^{f}_{w} \cdot \text{project}(\theta) - r^{f}_{L/F}\right\|^2$$
   over the offset deltas $\Delta\text{roll}$, $\Delta\text{pitch}$, $\Delta\text{yaw}$ (bounded about the
   current offsets).
5. **Write back.** The optimised offsets are written back to `config/<robot>/Cameras/<camera>.yaml` as
   `"<degrees> * pi / 180"` expression strings (where $\text{degrees}$ is the angle in degrees).

## Usage

Place the robot at the centre of the field facing the goal, then run the `extrinsicscalibration` role. Select
the camera to calibrate and the field type via configuration. Run again with the other camera to calibrate both.

## Consumes

- `message::vision::FieldIntersections` the YOLO X/L/T field landmark detections and frame transform
- `message::input::Sensors` for the offset-free kinematics ($H^{w}_{t}$, $H^{x[\text{HEAD\_PITCH}]}_{t}$)
- `message::support::FieldDescription` for the field dimensions / ground-truth landmark positions

## Emits

- Nothing. The optimised offsets are logged and written back to the camera config file when the optimisation succeeds.

## Dependencies

- `Eigen`
- `NLopt` for the BOBYQA optimisation
- `tinyrobotics` for camera forward kinematics
- `utility::algorithm::determine_assignment` (Hungarian algorithm)
- `utility::localisation::setup_field_landmarks`
