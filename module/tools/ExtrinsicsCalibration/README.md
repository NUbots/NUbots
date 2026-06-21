# ExtrinsicsCalibration

## Description

Automatically tunes a single camera's extrinsic transform `Hpc` (the camera `{c}` to head-pitch `{p}` transform) so that it no longer has to be hand-tuned. The full 6-DOF pose is optimised directly — three rotation parameters (`roll`, `pitch`, `yaw`, ZYX-intrinsic) and three translation parameters (`tx`, `ty`, `tz`) — and written back as `translation` + `rpy` (replacing the old `roll_offset`/`pitch_offset`/`yaw_offset` scheme).

The module assumes the robot is placed at the **centre of the field, looking straight toward the goal**, and that the **field type is known** (lab or RoboCup field) via the `FieldDescription`. Because the field dimensions are known, the ground-truth positions of the field landmarks (X, L and T intersections) are known to a high degree of accuracy.

Once running, the robot **stands itself up and automatically sweeps its head** through a yaw/pitch grid, so the field landmarks are seen across the whole image without any manual input. The sweep is delegated to the `PlanLook` provider (the module emits a `LookAround` task); the grid is configured per-role in `config/bin/extrinsicscalibration/PlanLook.yaml` so it does not affect the shared soccer ball-search sweep. Samples are gathered only once the head has moved far enough since the last capture, spreading them across distinct viewpoints.

**Field dimensions MUST be measured first and the necessary values must be updated in FieldDescription.yaml before running this binary.**

The YOLO network detects the field landmarks and projects them onto the ground plane. The module re-projects those detections under a candidate `Hpc` and minimises the total squared distance between each detection and the landmark it should correspond to.

### How it works

Notation: $H^{a}_{b}$ is the transform that maps a point in frame `{b}` into frame `{a}`.

1. **Recover the geometry without changing the vision pipeline.** Each `FieldIntersection` already carries its
   world-space ground projection $r^{w}_{I/W}$ and the frame transform $H^{c}_{w}$. The pose-independent camera-frame ray
   is recovered as $u_{I/C}^c = (H^{c}_{w} \cdot r^{w}_{I/W}).\text{normalized}()$. The $H^{w}_{p}$ (world from head-pitch)
   is recomputed exactly as the `Camera` module does, so the candidate camera-to-world transform is
   $H^{w}_{c}(\xi) = H^{w}_{p} \cdot H^{p}_{c}(\xi)$, where $H^{p}_{c}(\xi)$ is built directly from the 6-DOF pose
   $\xi = (\text{roll}, \text{pitch}, \text{yaw}, t_x, t_y, t_z)$ (ZYX-intrinsic rpy + translation).
2. **Known field pose.** From the placement assumption, the field-from-world transform $H^{f}_{w}$ is synthesised:
   the torso is at the field centre $(x = y = 0)$ facing the goal, with roll/pitch/height taken
   from `Sensors`.
3. **Collect raw detections.** As frames arrive, only the pose-independent camera-frame rays (plus that frame's
   transforms) are stored. No landmark association is committed at collection time, so the same detections can be
   re-matched as the pose improves.
4. **Train/validation split.** Each collected detection is randomly tagged (seeded, so it is reproducible and
   independent of the head-sweep order) as training or held-out validation, controlled by `validation.split_ratio`
   and `validation.seed`.
5. **Refine with ICP (training set).** Once enough detections have been gathered, the module runs an Iterative
   Closest Point loop over the training detections:
   - **Associate:** project every detection to field space at the _current best_ pose and match each frame's
     detections to the same-type ground-truth landmarks with the Hungarian algorithm
     (`utility::algorithm::determine_assignment`), gated by `max_association_distance`.
   - **Optimise:** with those associations held fixed, NLopt's BOBYQA minimises
     $$\sum \left\|H^{f}_{w} \cdot \text{project}(\xi) - r^{f}_{L/F}\right\|^2$$
     over the 6-DOF pose $\xi$, box-bounded about the **URDF nominal** by `rotation_bounds` (rad) and
     `translation_bounds` (m).
   - **Re-centre and repeat:** the refined pose becomes the new warm start for the next association. The loop stops
     once the associations stop changing or `max_icp_iterations` is reached. This lets a detection that was
     initially mis-matched (because the starting pose was off) snap to the correct landmark as the fit improves,
     rather than being permanently frozen to the wrong target.
6. **Validate.** The warm-start and optimised poses are scored (per-sample RMS re-projection error) on both the
   training and the held-out validation sets. A lower _optimised validation_ RMS indicates the extra DOF
   generalise. If the warm start fits validation better, the optimiser is chasing noise / placement error rather
   than real mounting error. The translation delta from the URDF nominal is also logged as a cm-scale sanity check.
7. **Write back.** The optimised extrinsics are written to `config/<robot>/Cameras/<camera>.yaml` as
   `translation: [x, y, z]` (metres) and `rpy: [r, p, y]` (`"<degrees> * pi / 180"` expression strings). The robot
   keeps standing and holds its head still afterwards (it does **not** exit, which would drop it) — stop the binary
   once the extrinsics are written.

## Usage

Place the robot at the centre of the field facing the goal, then run the `extrinsicscalibration` role. The robot
stands itself up and sweeps its head automatically. Select the camera to calibrate and
the field type via configuration. Run again with the other camera to calibrate both.

The camera configs are seeded with the URDF nominal `Hpc` (translation from forward kinematics, the nominal
rotation). The module logs this seed on startup so it can be confirmed against the config values.

## Consumes

- `message::vision::FieldIntersections` the YOLO X/L/T field landmark detections and frame transform
- `message::input::Sensors` for the kinematics ($H^{w}_{t}$, $H^{x[\text{HEAD\_PITCH}]}_{t}$) and head servo angles

## Emits

- `message::skill::Walk` (zero velocity) and `message::strategy::FallRecovery` tasks to stand the robot, and a
  `message::planning::LookAround` task so the `PlanLook` provider sweeps the head through the scan grid.
- The optimised extrinsics are logged and written back to the camera config file when the optimisation succeeds.

## Dependencies

- `Eigen`
- `NLopt` for the BOBYQA optimisation
- `tinyrobotics` for the URDF nominal camera forward kinematics (bound centre / seed)
- `utility::math::euler` for the rpy <-> rotation matrix conversions
- `utility::algorithm::determine_assignment` (Hungarian algorithm)
- `utility::localisation::setup_field_landmarks`
- The Director behaviour stack (`Walk`, `PlanLook`, `Look`, `FallRecovery`, `GetUp`) to stand and move the head
