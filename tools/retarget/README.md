# Motion retargeting for the K1

`retarget_k1.py` turns human motion capture (BVH) into Booster K1 joint trajectories by solving inverse kinematics with
[mink](https://github.com/kevinzakka/mink) on the K1 model in `shared/utility/platform/models/k1/`.

Supported inputs:

- [LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset) BVH files
- OptiTrack Motive skeleton exports as BVH, using either Motive's own naming (`Skeleton_Hip`, `Skeleton_LThigh`, ...)
  or the standard naming (`Hips`, `LeftUpLeg`, ...). Joint naming is detected automatically. Any other skeleton only
  needs a new entry in `JOINT_NAME_PRESETS`.

## Usage

This is a standalone script, **not** a `./b` command. It needs mujoco/mink and a display for the viewer, and the
build container has neither. Run it on the host with [uv](https://docs.astral.sh/uv/), which installs the
dependencies declared at the top of the script into a throwaway environment:

```sh
uv run tools/retarget/retarget_k1.py clip.bvh --view             # retarget, write clip_k1.npz, play it back
uv run tools/retarget/retarget_k1.py clip.bvh -o out.npz --csv   # also write joint angles as CSV
uv run tools/retarget/retarget_k1.py clip.bvh --start 300 --end 900 --fps 50
```

`--view` is kinematic playback: the robot is posed exactly as retargeted, with no physics. `--physics` plays it with
physics instead. Gravity and floor contact are on, and each frame's joint angles go to position servos, but there is
no balance control. Expect anything that shifts the centre of mass to fall over, and use it to see how far a clip
is from something the real robot could do. The script prints when the robot falls, restarts the clip two seconds
later, and space restarts it early. Servo gains are rough guesses (`SERVO_*` constants), not measured from the K1.

```sh
uv run tools/retarget/retarget_k1.py clip.bvh --physics      # retarget, then simulate
uv run tools/retarget/retarget_k1.py clip_k1.npz --physics   # replay a saved result without re-solving
uv run tools/retarget/retarget_k1.py clip_k1.npz --view
```

In the viewer, blue spheres are the K1's keypoint sites and orange spheres are the retargeted human targets they track.

Exporting from Motive: solve the skeleton, clean up gaps, then export the take as BVH. Both LAFAN1 and Motive are
Y-up, which is the default (`--up y`). Units don't matter, because every bone is rescaled to K1 proportions.

## Output

An `.npz` with:

| Key                | Shape     | Meaning                                                 |
| ------------------ | --------- | ------------------------------------------------------- |
| `fps`              | scalar    | Output frame rate                                       |
| `joint_names`      | (22,)     | K1 joint names, in the column order of `joint_pos`      |
| `joint_pos`        | (T, 22)   | Joint angles in radians, within the URDF limits         |
| `root_pos`         | (T, 3)    | Trunk position in the world (Z-up, metres)              |
| `root_quat_wxyz`   | (T, 4)    | Trunk orientation in the world                          |
| `keypoint_names`   | (K,)      | Keypoints used for retargeting                          |
| `keypoint_targets` | (T, K, 3) | Retargeted human keypoints the IK tracked               |
| `keypoint_errors`  | (T, K)    | Distance between each K1 site and its target, in metres |

## How it works

1. **Human keypoints.** Forward kinematics on the BVH gives the world positions of the pelvis, hips, knees, ankles,
   toes, shoulders, elbows, wrists and head. These are converted to Z-up and resampled to `--fps`.
2. **Scaling to the K1.** The keypoints are rebuilt as a tree from the pelvis. Each bone keeps the human's direction
   but takes the K1's length. Hips and shoulders are instead placed from the trunk orientation and the K1's own
   geometry, since the K1 has no waist joint. The pelvis path is scaled by the leg-length ratio, so steps keep the
   right length, and the whole motion is shifted so the feet sit on the ground.
3. **IK.** mink tracks every keypoint with a `FrameTask`. The trunk and feet also get orientation targets: the feet
   are kept flat and yawed to match the human's feet. A low-weight `PostureTask` pulls joints the targets don't
   determine towards arms-down, and a `ConfigurationLimit` keeps every joint inside the URDF limits. Each frame starts
   from the previous solution, which keeps the trajectory smooth. If a frame still tracks badly (mean error above
   `--restart-error`, 3 cm by default), it is re-solved from the neutral pose and the better result is kept, so the
   solver can't stay stuck in a bad pose after a move the K1 can't follow.

Task weights, site placements and the neutral pose are constants at the top of the script.

## Balance-aware mode (`--balance`, experimental)

`--balance` adds three things on top of plain keypoint tracking:

- **Foot contacts.** A foot counts as planted when its ankle target is near the floor and nearly still. A planted foot
  is pinned flat at its mean position and heading for the whole contact, which removes foot sliding.
- **Static balance.** If the centre of mass falls outside the area under the planted feet (shrunk by
  `SUPPORT_MARGIN`), the frame is re-solved with a `ComTask` pulling it back inside. Planted feet are held by
  high-weight tasks so the solver can't cheat by dragging them along. In double support, the weight starts shifting
  onto the upcoming stance foot `BALANCE_LOOKAHEAD` before the other foot lifts.
- **Smoothing.** A task pulls each frame's joints towards the previous frame's.

The support area uses the part of the sole that MuJoCo contact actually loads (x −0.05 to +0.067 m, y ±0.036 m in
the foot frame). That is noticeably smaller than the foot mesh's bounding box, because the toe curves up.

**Current status: it does not yet stop the robot falling in `--physics`.** Every single-support pose it produces
stands when held still in simulation, but playback still falls about 4 s into LAFAN1 `walk1_subject1` and
`aiming1_subject1`, even slowed down 8×. The cause is jerky output: the centre-of-mass target jumps whenever the
support area changes (a foot lands or lifts) and whenever the correction switches on or off, so joints can jump by
more than 1 rad in one frame. The servos can't follow those jumps, and the jolt tips the robot over. Tracking is also
looser than plain mode (about 5–6 cm mean keypoint error vs under 1 cm), since the upper body leans to stay balanced.

The likely fix is to plan one smooth centre-of-mass trajectory for the whole clip (e.g. a linear inverted pendulum
with preview control over the contact sequence) and have the IK track it, instead of correcting frame by frame.
Even then, open-loop playback of human-speed walking needs closed-loop balance on the robot.

## Limitations

- The output is **kinematic only**. It respects joint limits, but not balance, torque limits or contact forces.
  Standing upper-body motions can be played straight on the robot. Anything that moves the centre of mass (walking,
  kicking, getting up) needs a tracking controller, e.g. an RL policy trained in simulation on these clips.
- Feet are held flat (no heel-strike or toe-off). Without `--balance`, foot contact is not enforced, so some foot
  sliding is possible.
- There is no self-collision avoidance yet, so check clips with crossing limbs in the viewer.
- Head orientation is not retargeted; the head only follows a low-weight position target.
