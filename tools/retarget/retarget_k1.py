#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2026 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "mujoco>=3.2",
#     "mink>=0.0.10",
#     "qpsolvers[daqp]>=4.3",
#     "numpy",
#     "scipy",
# ]
# ///

"""
Retarget human motion capture (BVH from LAFAN1 or OptiTrack Motive) onto the Booster K1 using mink IK.

This is a standalone script, not a `./b` command: it needs mujoco/mink and (for --view) a display, neither of which
the build container provides. Run it on the host with uv, which installs the dependencies listed above:

    uv run tools/retarget/retarget_k1.py clip.bvh -o clip_k1.npz --view

Method (per output frame):
  1. Forward kinematics on the BVH gives world positions of a small set of human keypoints (pelvis, hips, knees,
     ankles, toes, shoulders, elbows, wrists, head), converted to Z-up.
  2. The keypoints are rebuilt as a tree from the pelvis, scaling each bone by robot_length / human_length, so the
     human's bone *directions* are kept but the K1's proportions are used. The result is shifted so the feet sit
     on the ground.
  3. mink solves differential IK for the floating base + 22 joints so K1 sites track those targets, within the
     joint limits, warm-started from the previous frame for smoothness.

The output is kinematic only. It respects joint limits but not balance or dynamics, so whole-body clips need a
tracking controller (e.g. an RL policy trained in sim) before they will work on the real robot.
"""

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mink
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[2]
K1_MODEL_DIR = REPO_ROOT / "shared" / "utility" / "platform" / "models" / "k1"

# ----------------------------------------------------------------------------------------------------------------------
# Keypoint definitions
# ----------------------------------------------------------------------------------------------------------------------

# Each keypoint: parent keypoint (bones are scaled parent -> child), K1 body the site is attached to, and the site's
# position in that body's frame. Sites sit on joint centres where the K1 has one, measured from the K1 meshes otherwise
# (foot sole is at z = -0.038, toe tip at x = +0.12; forearm mesh ends at |y| = 0.228).
KEYPOINTS = {
    #  name            parent        body                 site position in body
    "pelvis":         (None,         "Trunk",            (0.0, 0.0, -0.103)),
    "left_hip":       ("pelvis",     "Left_Hip_Roll",    (0.0, 0.0, 0.0)),
    "left_knee":      ("left_hip",   "Left_Shank",       (0.0, 0.0, 0.0)),
    "left_ankle":     ("left_knee",  "left_foot_link",   (0.0, 0.0, 0.0)),
    "left_toe":       ("left_ankle", "left_foot_link",   (0.09, 0.0, -0.03)),
    "right_hip":      ("pelvis",     "Right_Hip_Roll",   (0.0, 0.0, 0.0)),
    "right_knee":     ("right_hip",  "Right_Shank",      (0.0, 0.0, 0.0)),
    "right_ankle":    ("right_knee", "right_foot_link",  (0.0, 0.0, 0.0)),
    "right_toe":      ("right_ankle","right_foot_link",  (0.09, 0.0, -0.03)),
    "left_shoulder":  ("pelvis",     "Left_Arm_2",       (0.0, 0.0, 0.0)),
    "left_elbow":     ("left_shoulder", "left_hand_link", (0.0, 0.0, 0.0)),
    "left_wrist":     ("left_elbow", "left_hand_link",   (0.0, 0.20, 0.0)),
    "right_shoulder": ("pelvis",     "Right_Arm_2",      (0.0, 0.0, 0.0)),
    "right_elbow":    ("right_shoulder", "right_hand_link", (0.0, 0.0, 0.0)),
    "right_wrist":    ("right_elbow", "right_hand_link", (0.0, -0.20, 0.0)),
    "head":           ("pelvis",     "Head_2",           (0.01, 0.0, 0.08)),
}  # fmt: skip

# IK costs per keypoint: (position cost [1/m], orientation cost [1/rad]). Orientation targets only exist for the
# pelvis/trunk and the feet (see build_targets); everything else is position-only.
TASK_COSTS = {
    "pelvis": (10.0, 5.0),
    "left_hip": (1.0, 0.0),
    "right_hip": (1.0, 0.0),
    "left_knee": (2.0, 0.0),
    "right_knee": (2.0, 0.0),
    "left_ankle": (10.0, 2.0),
    "right_ankle": (10.0, 2.0),
    "left_toe": (2.0, 0.0),
    "right_toe": (2.0, 0.0),
    "left_shoulder": (1.0, 0.0),
    "right_shoulder": (1.0, 0.0),
    "left_elbow": (3.0, 0.0),
    "right_elbow": (3.0, 0.0),
    "left_wrist": (5.0, 0.0),
    "right_wrist": (5.0, 0.0),
    "head": (1.0, 0.0),
}

# Height of the ankle joint above the sole when the foot is flat (from the K1 foot mesh)
ANKLE_HEIGHT = 0.038

# Keypoints rigidly fixed to the K1 trunk (near enough). Their targets come from the trunk orientation target and the
# K1's own geometry rather than the human's spine, since the K1 has no waist to bend.
TRUNK_FIXED_KEYPOINTS = ("left_hip", "right_hip", "left_shoulder", "right_shoulder")

# Neutral pose the posture task pulls redundant DoFs towards (arms down; the K1's zero pose is a T-pose).
NEUTRAL_POSE = {
    "Left_Shoulder_Roll": -1.4,
    "Right_Shoulder_Roll": 1.4,
}
POSTURE_COST = 0.05

# Balance-aware retargeting (experimental, --balance). Feet that are on the ground are planted flat and held still for
# the whole contact, and the centre of mass is kept over the area the planted feet cover.
CONTACT_HEIGHT = 0.025  # an ankle within this of its flat-foot height [m]...
CONTACT_SPEED = 0.3  # ...and moving slower than this horizontally [m/s] counts as on the ground
CONTACT_MIN_FRAMES = 3  # shorter contacts are treated as noise
CONTACT_BLEND_FRAMES = 5  # frames either side of a contact over which the foot eases onto/off its planted pose
SOLE_X = (-0.05, 0.067)  # load-bearing sole in the foot frame [m]: where MuJoCo contact supports the K1 foot mesh
SOLE_HALF_WIDTH = 0.036  # [m], as above
SUPPORT_MARGIN = 0.03  # keep the centre of mass at least this far inside the support area [m]
COM_COST = 100.0  # cost on horizontal centre of mass error when correcting balance [1/m]
BALANCE_LOOKAHEAD = 0.3  # start shifting weight onto the stance foot this long before the other foot lifts [s]
SMOOTHING_COST = 1.0  # --balance only: cost pulling joints towards the previous frame's [1/rad], against jumps
STANCE_FOOT_COST = 1000.0  # cost holding planted feet in place while correcting balance (must dominate COM_COST)

# Physics playback (--physics). These are rough guesses, not identified from the real K1 servos.
PHYSICS_TIMESTEP = 0.002  # s
# Gains must be well above gravity's tipping stiffness at the ankle (m g h ~ 20 kg * 9.81 * 0.5 m ~ 100 Nm/rad), or the
# robot topples slowly even when just holding a pose.
SERVO_KP_PER_NM = 20.0  # position gain [Nm/rad] per Nm of joint torque limit (e.g. knee: 40 Nm -> kp 800)
SERVO_KV_RATIO = 0.05  # damping gain [Nm s/rad] as a fraction of kp
SERVO_ARMATURE = 0.01  # reflected rotor inertia [kg m^2], also keeps the simulation stable
FALLEN_HEIGHT = 0.25  # trunk below this height [m] counts as fallen

# BVH joint names for each keypoint, per skeleton convention. Matching ignores any "Skeleton_" / "name:" prefix.
JOINT_NAME_PRESETS = {
    "lafan1": {
        "pelvis": "Hips",
        "left_hip": "LeftUpLeg",
        "left_knee": "LeftLeg",
        "left_ankle": "LeftFoot",
        "left_toe": "LeftToe",
        "right_hip": "RightUpLeg",
        "right_knee": "RightLeg",
        "right_ankle": "RightFoot",
        "right_toe": "RightToe",
        "left_shoulder": "LeftArm",
        "left_elbow": "LeftForeArm",
        "left_wrist": "LeftHand",
        "right_shoulder": "RightArm",
        "right_elbow": "RightForeArm",
        "right_wrist": "RightHand",
        "head": "Head",
    },
    # Motive's own skeleton naming (Hip/Ab/Chest/..., exported as "<SkeletonName>_LThigh" etc.)
    "motive": {
        "pelvis": "Hip",
        "left_hip": "LThigh",
        "left_knee": "LShin",
        "left_ankle": "LFoot",
        "left_toe": "LToe",
        "right_hip": "RThigh",
        "right_knee": "RShin",
        "right_ankle": "RFoot",
        "right_toe": "RToe",
        "left_shoulder": "LUArm",
        "left_elbow": "LFArm",
        "left_wrist": "LHand",
        "right_shoulder": "RUArm",
        "right_elbow": "RFArm",
        "right_wrist": "RHand",
        "head": "Head",
    },
}
# Motive 3 can also export with the "standard"/FBX naming (Hips, LeftUpLeg, ...), which the lafan1 preset covers.

# ----------------------------------------------------------------------------------------------------------------------
# BVH loading
# ----------------------------------------------------------------------------------------------------------------------


@dataclass
class Bvh:
    names: list  # joint names, in file order
    parents: list  # parent index per joint (-1 for root)
    offsets: np.ndarray  # (J, 3) rest offsets from parent
    channels: list  # per joint: list of channel names
    frame_time: float
    motion: np.ndarray  # (F, total channels)


def load_bvh(path: Path) -> Bvh:
    tokens = Path(path).read_text().split()
    names, parents, offsets, channels = [], [], [], []
    stack = []
    i = 0
    assert tokens[i].upper() == "HIERARCHY", "Not a BVH file"
    i += 1
    while tokens[i].upper() != "MOTION":
        t = tokens[i].upper()
        if t in ("ROOT", "JOINT"):
            names.append(tokens[i + 1])
            parents.append(stack[-1] if stack else -1)
            offsets.append(np.zeros(3))
            channels.append([])
            i += 2
        elif t == "END":  # End Site: skip its block, it carries no channels
            depth = 0
            i += 2
            while True:
                if tokens[i] == "{":
                    depth += 1
                elif tokens[i] == "}":
                    depth -= 1
                    if depth == 0:
                        break
                i += 1
            i += 1
        elif t == "{":
            stack.append(len(names) - 1)
            i += 1
        elif t == "}":
            stack.pop()
            i += 1
        elif t == "OFFSET":
            offsets[-1] = np.array([float(x) for x in tokens[i + 1 : i + 4]])
            i += 4
        elif t == "CHANNELS":
            n = int(tokens[i + 1])
            channels[-1] = tokens[i + 2 : i + 2 + n]
            i += 2 + n
        else:
            raise ValueError(f"Unexpected BVH token {tokens[i]!r}")
    i += 1  # MOTION
    assert tokens[i].rstrip(":").upper() == "FRAMES"
    n_frames = int(tokens[i + 1])
    i += 2
    assert tokens[i].upper() == "FRAME" and tokens[i + 1].rstrip(":").upper() == "TIME"
    frame_time = float(tokens[i + 2])
    i += 3
    n_ch = sum(len(c) for c in channels)
    data = np.array(tokens[i : i + n_frames * n_ch], dtype=float)
    if data.size != n_frames * n_ch:
        n_frames = data.size // n_ch
        print(f"Warning: BVH motion section is short, using {n_frames} frames", file=sys.stderr)
        data = data[: n_frames * n_ch]
    return Bvh(names, parents, np.array(offsets), channels, frame_time, data.reshape(n_frames, n_ch))


def bvh_global_positions(bvh: Bvh) -> np.ndarray:
    """World position of every BVH joint in every frame, (F, J, 3), in the file's own axes and units."""
    n_frames, n_joints = bvh.motion.shape[0], len(bvh.names)
    pos = np.zeros((n_frames, n_joints, 3))
    rot = [None] * n_joints
    col = 0
    for j in range(n_joints):
        chans = bvh.channels[j]
        data = bvh.motion[:, col : col + len(chans)]
        col += len(chans)

        # Translation: position channels when present (LAFAN1 has them on every joint), otherwise the rest offset
        local_t = np.tile(bvh.offsets[j], (n_frames, 1))
        for axis, name in enumerate(("Xposition", "Yposition", "Zposition")):
            if name in chans:
                local_t[:, axis] = data[:, chans.index(name)]

        # Rotation: channels apply in the order listed, i.e. intrinsic Euler angles in that order
        rot_idx = [k for k, c in enumerate(chans) if c.endswith("rotation")]
        if rot_idx:
            order = "".join(chans[k][0].upper() for k in rot_idx)
            local_r = Rotation.from_euler(order, data[:, rot_idx], degrees=True)
        else:
            local_r = Rotation.identity(n_frames)

        p = bvh.parents[j]
        if p < 0:
            pos[:, j] = local_t
            rot[j] = local_r
        else:
            pos[:, j] = pos[:, p] + rot[p].apply(local_t)
            rot[j] = rot[p] * local_r
    return pos


def resolve_joint_names(bvh_names: list, preset: str) -> dict:
    """Map keypoint -> BVH joint index, choosing a naming preset automatically unless one is given."""

    def find(target):
        for idx, n in enumerate(bvh_names):
            if re.sub(r"^.*[:_]", "", n) == target or n == target:
                return idx
        return None

    presets = [preset] if preset != "auto" else list(JOINT_NAME_PRESETS)
    for p in presets:
        mapping = {k: find(v) for k, v in JOINT_NAME_PRESETS[p].items()}
        missing = [JOINT_NAME_PRESETS[p][k] for k, v in mapping.items() if v is None]
        if not missing:
            print(f"Using '{p}' joint naming")
            return mapping
        if preset != "auto":
            raise SystemExit(f"BVH is missing joints for preset '{p}': {missing}\nJoints in file: {bvh_names}")
    raise SystemExit(
        f"Could not match BVH joints to any preset {list(JOINT_NAME_PRESETS)}. Joints in file: {bvh_names}\n"
        "Add a preset to JOINT_NAME_PRESETS in this script."
    )


# ----------------------------------------------------------------------------------------------------------------------
# Robot model
# ----------------------------------------------------------------------------------------------------------------------


def build_k1_model(physics: bool = False) -> mujoco.MjModel:
    """
    Load the repo's K1 MJCF and adapt it for retargeting, without modifying the file on disk:
    wrap the trunk (which is welded to the world in mjmodel.xml) in a free-floating body, give it the URDF's trunk
    inertia, add the keypoint sites, and add a floor for the viewer.

    With physics=True it also adds position-controlled servos on every joint and robot/floor contact, for
    simulate(). Self-collision stays off, matching the IK, so limbs can pass through each other.
    """
    root = ET.parse(K1_MODEL_DIR / "mjmodel.xml").getroot()
    compiler = root.find("compiler")
    compiler.set("meshdir", str(K1_MODEL_DIR))

    # Trunk inertia from the URDF (mjmodel.xml has none, since its trunk is the world body)
    urdf = ET.parse(K1_MODEL_DIR / "robot.urdf").getroot()
    trunk_inertial = next(l for l in urdf.findall("link") if l.get("name") == "Trunk").find("inertial")
    i = trunk_inertial.find("inertia").attrib
    inertial = ET.Element(
        "inertial",
        pos=trunk_inertial.find("origin").get("xyz"),
        mass=trunk_inertial.find("mass").get("value"),
        fullinertia=" ".join(i[k] for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")),
    )

    world = root.find("worldbody")
    trunk = ET.Element("body", name="Trunk", pos="0 0 0.55")
    trunk.append(ET.Element("freejoint", name="root"))
    trunk.append(inertial)
    for child in list(world):
        world.remove(child)
        trunk.append(child)
    world.append(
        ET.Element(
            "geom", name="floor", type="plane", size="0 0 0.05", rgba="0.3 0.45 0.3 1", contype="0", conaffinity="0"
        )
    )
    world.append(ET.Element("light", pos="0 0 3", dir="0 0 -1", directional="true"))
    world.append(trunk)

    bodies = {b.get("name"): b for b in root.iter("body")}
    for name, (_, body, site_pos) in KEYPOINTS.items():
        bodies[body].append(
            ET.Element("site", name=name, pos=" ".join(map(str, site_pos)), size="0.012", rgba="0.1 0.4 1 0.8")
        )

    if physics:
        root.insert(1, ET.Element("option", timestep=str(PHYSICS_TIMESTEP), integrator="implicitfast"))
        # Robot geoms collide with the floor only: contact needs (contype1 & conaffinity2) or (contype2 & conaffinity1)
        for geom in trunk.iter("geom"):
            geom.set("contype", "1")
            geom.set("conaffinity", "0")
            geom.set("friction", "1.0 0.02 0.01")
        floor = world.find("geom[@name='floor']")
        floor.set("contype", "0")
        floor.set("conaffinity", "1")

        # Position servos with gains scaled by each joint's torque limit, which is also enforced as the force limit
        actuator = ET.SubElement(root, "actuator")
        for joint in trunk.iter("joint"):
            effort = float(joint.get("actuatorfrcrange").split()[1])
            kp = SERVO_KP_PER_NM * effort
            joint.set("armature", str(SERVO_ARMATURE))
            ET.SubElement(
                actuator,
                "position",
                name=joint.get("name"),
                joint=joint.get("name"),
                kp=f"{kp:g}",
                kv=f"{SERVO_KV_RATIO * kp:g}",
                ctrlrange=joint.get("range"),
                forcerange=f"{-effort:g} {effort:g}",
            )

    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def neutral_qpos(model: mujoco.MjModel) -> np.ndarray:
    q = model.qpos0.copy()
    for joint, value in NEUTRAL_POSE.items():
        q[model.jnt_qposadr[model.joint(joint).id]] = value
    return q


def robot_bones(model: mujoco.MjModel) -> dict:
    """Each keypoint bone (parent site -> child site) in the K1's zero pose, expressed in the trunk frame."""
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    mujoco.mj_kinematics(model, data)
    site = {name: data.site(name).xpos.copy() for name in KEYPOINTS}
    trunk_rot = data.body("Trunk").xmat.reshape(3, 3)
    return {k: trunk_rot.T @ (site[k] - site[p]) for k, (p, _, _) in KEYPOINTS.items() if p is not None}


# ----------------------------------------------------------------------------------------------------------------------
# Human -> robot targets
# ----------------------------------------------------------------------------------------------------------------------

# Cyclic axis permutation taking Y-up (BVH, Motive) to Z-up (MuJoCo): new (x, y, z) = old (z, x, y). Being a proper
# rotation it keeps the data right-handed, so left/right stay correct.
Y_UP_TO_Z_UP = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def _normalise(v):
    return v / np.linalg.norm(v, axis=-1, keepdims=True)


def _frame_from_xz(x, z):
    """Right-handed rotation matrices whose x axis is along x, with z as close to the given z as possible."""
    x = _normalise(x)
    y = _normalise(np.cross(z, x))
    z = np.cross(x, y)
    return np.stack([x, y, z], axis=-1)


def build_targets(human: dict, robot_bones: dict, ground_height: float):
    """
    human: keypoint -> (F, 3) Z-up positions. Returns keypoint -> (F, 3) robot-scale target positions, plus
    (F, 3, 3) target orientations for the oriented keypoints.
    """
    robot_lengths = {k: np.linalg.norm(v) for k, v in robot_bones.items()}
    human_lengths = {
        k: np.median(np.linalg.norm(human[k] - human[p], axis=-1)) for k, (p, _, _) in KEYPOINTS.items() if p
    }
    scale = {k: robot_lengths[k] / human_lengths[k] for k in robot_lengths if k not in TRUNK_FIXED_KEYPOINTS}

    # Pelvis translation scaled by leg length ratio, so stride length matches the K1's legs
    leg = ("left_knee", "left_ankle")  # hip -> knee -> ankle
    leg_scale = sum(robot_lengths[k] for k in leg) / sum(human_lengths[k] for k in leg)

    up = np.array([0.0, 0.0, 1.0])
    # Trunk: lateral axis from hips and shoulders together (the K1 has no waist joint), up from pelvis to shoulders
    lateral = _normalise(human["left_hip"] - human["right_hip"]) + _normalise(
        human["left_shoulder"] - human["right_shoulder"]
    )
    trunk_up = 0.5 * (human["left_shoulder"] + human["right_shoulder"]) - human["pelvis"]
    trunk_fwd = np.cross(lateral, trunk_up)
    orientations = {"pelvis": _frame_from_xz(trunk_fwd, _normalise(trunk_up))}
    # Feet: kept flat, yawed to match the human foot. The heading comes from the ankle's pitch axis (normal of the
    # shin/foot plane) rather than the flattened ankle -> toe vector, which shrinks to nothing and spins wildly when the
    # foot points down at toe-off.
    for side in ("left", "right"):
        shin = human[f"{side}_knee"] - human[f"{side}_ankle"]
        foot = human[f"{side}_toe"] - human[f"{side}_ankle"]
        foot_lateral = np.cross(shin, foot)
        fwd = np.cross(foot_lateral, up)
        orientations[f"{side}_ankle"] = _frame_from_xz(fwd, np.broadcast_to(up, fwd.shape))

    targets = {"pelvis": human["pelvis"] * leg_scale}
    for k, (p, _, _) in KEYPOINTS.items():  # dict order is parent-before-child
        if k in TRUNK_FIXED_KEYPOINTS:
            targets[k] = targets[p] + orientations["pelvis"] @ robot_bones[k]
        elif p is not None:
            targets[k] = targets[p] + scale[k] * (human[k] - human[p])

    # Put the feet on the ground: shift so the low ankle sits at the K1's standing ankle height for most frames
    low_ankle = np.minimum(targets["left_ankle"][:, 2], targets["right_ankle"][:, 2])
    dz = ground_height - np.percentile(low_ankle, 5)
    for k in targets:
        targets[k] = targets[k] + np.array([0.0, 0.0, dz])

    return targets, orientations, scale


def _runs(mask):
    """(start, end) index pairs of the contiguous True runs in a boolean array."""
    edges = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def detect_contacts(targets: dict, fps: float) -> dict:
    """Per side, (F,) bool: whether that foot is on the ground, judged from its retargeted ankle height and speed."""
    contacts = {}
    for side in ("left", "right"):
        ankle = targets[f"{side}_ankle"]
        speed = np.linalg.norm(np.gradient(ankle[:, :2], axis=0), axis=1) * fps
        contact = (ankle[:, 2] < ANKLE_HEIGHT + CONTACT_HEIGHT) & (speed < CONTACT_SPEED)
        for start, end in _runs(contact):
            if end - start < CONTACT_MIN_FRAMES:
                contact[start:end] = False
        contacts[side] = contact
    return contacts


def plant_feet(targets: dict, orientations: dict, contacts: dict):
    """
    For each contact, pin the foot flat on the floor at its mean position and heading over the contact, so stance feet
    don't slide or hover. The foot eases onto and off that pose over CONTACT_BLEND_FRAMES to avoid jumps.
    Modifies targets and orientations in place.
    """
    toe_site = np.array(KEYPOINTS["left_toe"][2])
    for side, contact in contacts.items():
        ankle, toe = targets[f"{side}_ankle"], targets[f"{side}_toe"]
        n_frames = len(ankle)
        correction = np.zeros((n_frames, 3))
        segments = _runs(contact)
        for start, end in segments:
            rot = orientations[f"{side}_ankle"][start:end]
            yaws = np.arctan2(rot[:, 1, 0], rot[:, 0, 0])
            yaw = np.arctan2(np.sin(yaws).mean(), np.cos(yaws).mean())  # circular mean
            planted_rot = Rotation.from_euler("z", yaw).as_matrix()
            planted = np.array([*ankle[start:end, :2].mean(axis=0), ANKLE_HEIGHT])
            correction[start:end] = planted - ankle[start:end]
            orientations[f"{side}_ankle"][start:end] = planted_rot
            toe[start:end] = planted + planted_rot @ toe_site - correction[start:end]  # correction is re-added below
        for start, end in segments:  # fade the correction in before and out after each contact
            for i in range(1, CONTACT_BLEND_FRAMES + 1):
                weight = 1.0 - i / (CONTACT_BLEND_FRAMES + 1)
                if start - i >= 0 and not contact[start - i]:
                    correction[start - i] = weight * correction[start]
                if end - 1 + i < n_frames and not contact[end - 1 + i]:
                    correction[end - 1 + i] = weight * correction[end - 1]
        ankle += correction
        toe += correction


def _support_polygon(feet) -> np.ndarray:
    """
    Counter-clockwise (N, 2) convex hull of the soles of the given feet, shrunk by SUPPORT_MARGIN.
    feet: (ankle position, ankle rotation matrix) per planted foot. These are the planted targets rather than the
    robot's current feet, so the IK can't "balance" by dragging the feet along under the centre of mass.
    """
    from scipy.spatial import ConvexHull

    x0, x1 = SOLE_X[0] + SUPPORT_MARGIN, SOLE_X[1] - SUPPORT_MARGIN
    y = SOLE_HALF_WIDTH - SUPPORT_MARGIN
    corners = np.array(
        [[x0, -y, -ANKLE_HEIGHT], [x1, -y, -ANKLE_HEIGHT], [x1, y, -ANKLE_HEIGHT], [x0, y, -ANKLE_HEIGHT]]
    )
    points = np.concatenate([pos + corners @ rot.T for pos, rot in feet])[:, :2]
    return points[ConvexHull(points).vertices]


def _inside(polygon: np.ndarray, p: np.ndarray) -> bool:
    edges = np.roll(polygon, -1, axis=0) - polygon
    rel = p - polygon
    return bool(np.all(edges[:, 0] * rel[:, 1] - edges[:, 1] * rel[:, 0] >= 0.0))


def _closest_on_polygon(polygon: np.ndarray, p: np.ndarray) -> np.ndarray:
    a, b = polygon, np.roll(polygon, -1, axis=0)
    t = np.clip(np.einsum("ij,ij->i", p - a, b - a) / np.einsum("ij,ij->i", b - a, b - a), 0.0, 1.0)
    candidates = a + t[:, None] * (b - a)
    return candidates[np.argmin(np.linalg.norm(candidates - p, axis=1))]


# ----------------------------------------------------------------------------------------------------------------------
# IK
# ----------------------------------------------------------------------------------------------------------------------


def retarget(model, targets, orientations, solver, max_iters, tol, restart_error, contacts=None, fps=50.0):
    """
    Solve IK for every frame. With contacts given (balance-aware mode), planted feet are held firmly, and any frame
    whose centre of mass falls outside the area under the planted feet is re-solved with a ComTask pulling it back
    inside, shifting weight onto the stance foot ahead of each lift-off.
    """
    lookahead = max(1, round(BALANCE_LOOKAHEAD * fps))
    n_frames = next(iter(targets.values())).shape[0]
    configuration = mink.Configuration(model)

    tasks = {}
    for name, (pos_cost, ori_cost) in TASK_COSTS.items():
        tasks[name] = mink.FrameTask(name, "site", position_cost=pos_cost, orientation_cost=ori_cost, lm_damping=1.0)
    posture = mink.PostureTask(model, cost=POSTURE_COST)
    q0 = neutral_qpos(model)
    posture.set_target(q0)
    limits = [mink.ConfigurationLimit(model)]
    all_tasks = [*tasks.values(), posture]
    com_task = mink.ComTask(cost=np.array([COM_COST, COM_COST, 0.0]))  # horizontal only
    # Temporal smoothing: feet landing/lifting and the balance correction switching on or off change the set of active
    # tasks, which otherwise makes joints jump between frames faster than any servo could follow
    smoothing = mink.PostureTask(model, cost=SMOOTHING_COST)
    stance_tasks = {
        side: mink.FrameTask(
            f"{side}_ankle", "site", position_cost=STANCE_FOOT_COST, orientation_cost=STANCE_FOOT_COST, lm_damping=1.0
        )
        for side in ("left", "right")
    }

    def neutral_at(f):
        """The neutral pose, placed and yawed onto frame f's pelvis target."""
        q = q0.copy()
        q[0:3] = targets["pelvis"][f] - np.array(KEYPOINTS["pelvis"][2])
        q[3:7] = Rotation.from_matrix(orientations["pelvis"][f]).as_quat(scalar_first=True)
        return q

    dt = 0.01  # IK integration step; without velocity limits each step is a full Gauss-Newton step regardless of dt

    def solve(f, iters, tasks_to_solve=all_tasks):
        for _ in range(iters):
            vel = mink.solve_ik(configuration, tasks_to_solve, dt, solver, damping=1e-3, limits=limits)
            configuration.integrate_inplace(vel, dt)
            if np.linalg.norm(vel) * dt < tol:
                break
        return np.array([np.linalg.norm(configuration.data.site(n).xpos - targets[n][f]) for n in KEYPOINTS])

    def single_support_ahead(f):
        """If both feet are down at f and one lifts within the look-ahead, (staying foot, frames until lift-off)."""
        for g in range(f + 1, min(f + 1 + lookahead, n_frames)):
            down = [side for side, contact in contacts.items() if contact[g]]
            if len(down) == 1:
                return down[0], g - f
            if not down:
                return None
        return None

    def set_frame_target(task, name, f):
        task.set_target(
            mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(orientations[name][f]), targets[name][f])
        )

    configuration.update(neutral_at(0))
    qpos = np.zeros((n_frames, model.nq))
    errors = np.zeros((n_frames, len(KEYPOINTS)))
    restarts = airborne = unbalanced = still_unbalanced = 0
    for f in range(n_frames):
        for name, task in tasks.items():
            rot = orientations[name][f] if name in orientations else np.eye(3)
            task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.from_matrix(rot), targets[name][f]))

        # Planted feet are held firmly (exact height, flat) rather than as one soft target among many: a stance foot
        # that's a few mm high or a degree off flat tilts the whole robot once it's flat on a real floor.
        feet = [side for side, contact in contacts.items() if contact[f]] if contacts is not None else []
        for side in feet:
            set_frame_target(stance_tasks[side], f"{side}_ankle", f)
        frame_tasks = all_tasks + [stance_tasks[side] for side in feet]
        if contacts is not None and f > 0:
            smoothing.set_target(qpos[f - 1])
            frame_tasks.append(smoothing)

        # First frame starts far from the solution, later frames are warm-started from the previous one
        err = solve(f, max_iters * 10 if f == 0 else max_iters, frame_tasks)

        # Warm starting can leave the solver stuck in a bad local minimum (e.g. crouched after the human does
        # something the K1 can't follow). If tracking is poor, also try from the neutral pose and keep the better one.
        if f > 0 and err.mean() > restart_error:
            warm_q = configuration.q.copy()
            configuration.update(neutral_at(f))
            fresh_err = solve(f, max_iters * 10, frame_tasks)
            if fresh_err.mean() < err.mean():
                err = fresh_err
                restarts += 1
            else:
                configuration.update(warm_q)

        # Balance: keep the centre of mass over the planted feet. Frames with no foot down (jumps, flight phases of a
        # run) can't be statically balanced, so they're left alone.
        if contacts is not None:
            if not feet:
                airborne += 1
            else:
                # Shift weight before lifting a foot, like a person does: in double support, ease the centre of mass
                # over the foot that's about to carry the robot, starting BALANCE_LOOKAHEAD before lift-off.
                support, weight = feet, 1.0
                ahead = single_support_ahead(f) if len(feet) == 2 else None
                if ahead is not None:
                    support = [ahead[0]]
                    x = 1.0 - ahead[1] / (lookahead + 1)
                    weight = x * x * (3.0 - 2.0 * x)  # smoothstep
                polygon = _support_polygon([(targets[f"{s}_ankle"][f], orientations[f"{s}_ankle"][f]) for s in support])
                com = configuration.data.subtree_com[1][:2]
                if not _inside(polygon, com):
                    unbalanced += 1
                    target = com + weight * (_closest_on_polygon(polygon, com) - com)
                    com_task.set_target(np.array([*target, 0.0]))
                    # Priority: stance feet stay put > centre of mass > everything else
                    err = solve(f, max_iters * 2, frame_tasks + [com_task])
                # Check against the feet actually down now. The ComTask is a soft cost, so allow a few mm short of
                # the shrunk edge (still inside the real sole).
                polygon = _support_polygon([(targets[f"{s}_ankle"][f], orientations[f"{s}_ankle"][f]) for s in feet])
                com = configuration.data.subtree_com[1][:2]
                if not _inside(polygon, com) and np.linalg.norm(_closest_on_polygon(polygon, com) - com) > 0.005:
                    still_unbalanced += 1

        qpos[f] = configuration.q
        errors[f] = err
        if f % 100 == 0 or f == n_frames - 1:
            print(f"\rframe {f + 1}/{n_frames}  mean keypoint error {errors[f].mean() * 100:.1f} cm", end="")
    print()
    if restarts:
        print(f"Restarted IK from the neutral pose on {restarts} frames where warm-started tracking was poor")
    if contacts is not None:
        print(
            f"Balance: centre of mass corrected back over the feet on {unbalanced}/{n_frames} frames "
            f"({still_unbalanced} still outside after correction); {airborne} frames with no foot on the ground"
        )
    return qpos, errors


# ----------------------------------------------------------------------------------------------------------------------
# Viewer
# ----------------------------------------------------------------------------------------------------------------------


def _draw_targets(viewer, targets, f):
    """Draw the retargeted keypoint targets for frame f as orange spheres (the robot's own sites are blue)."""
    scn = viewer.user_scn
    scn.ngeom = 0
    for name in KEYPOINTS:
        mujoco.mjv_initGeom(
            scn.geoms[scn.ngeom],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.015, 0, 0]),
            targets[name][f],
            np.eye(3).flatten(),
            np.array([1.0, 0.5, 0.0, 0.8], dtype=np.float32),
        )
        scn.ngeom += 1


def _open_viewer(model, data, key_callback=None):
    """
    MuJoCo's passive viewer, set up for watching rather than debugging: the side panels are hidden (Tab and Shift+Tab
    bring them back) and the camera follows the trunk, so it can be left alone. Mouse still works: left-drag rotates,
    right-drag pans, scroll zooms.
    """
    import mujoco.viewer

    viewer = mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback, show_left_ui=False, show_right_ui=False
    )
    with viewer.lock():
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = model.body("Trunk").id
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -15.0
    return viewer


def view(model, qpos, targets, fps):
    """Kinematic playback: pose the robot exactly as retargeted, frame by frame (no physics)."""
    import time

    data = mujoco.MjData(model)
    with _open_viewer(model, data) as viewer:
        f = 0
        while viewer.is_running():
            start = time.time()
            data.qpos[:] = qpos[f]
            mujoco.mj_kinematics(model, data)
            _draw_targets(viewer, targets, f)
            viewer.sync()
            f = (f + 1) % len(qpos)
            time.sleep(max(0.0, 1.0 / fps - (time.time() - start)))


def simulate(qpos, targets, fps):
    """
    Physics playback: start from the first retargeted frame, then send each frame's joint angles to position servos
    and let gravity and contacts do the rest, with no balance control. The orange spheres show where the reference
    motion goes, so you can watch the robot drift from it and fall. Restarts after a fall or at the end of the clip;
    press space to restart early.
    """
    import time

    model = build_k1_model(physics=True)
    data = mujoco.MjData(model)
    ctrl_qadr = model.jnt_qposadr[model.actuator_trnid[:, 0]]
    substeps = max(1, round(1.0 / (fps * model.opt.timestep)))
    trunk = model.body("Trunk").id
    restart = [True]

    def key_callback(keycode):
        if keycode == 32:  # space
            restart[0] = True

    with _open_viewer(model, data, key_callback) as viewer:
        while viewer.is_running():
            if restart[0]:
                restart[0] = False
                mujoco.mj_resetData(model, data)
                data.qpos[:] = qpos[0]
                if len(qpos) > 1:  # start moving with the reference's initial velocity
                    mujoco.mj_differentiatePos(model, data.qvel, 1.0 / fps, qpos[0], qpos[1])
                mujoco.mj_forward(model, data)
                f, fell_at = 0, None

            start = time.time()
            data.ctrl[:] = qpos[f][ctrl_qadr]
            for _ in range(substeps):
                mujoco.mj_step(model, data)
            _draw_targets(viewer, targets, f)
            viewer.sync()

            if fell_at is None and data.xpos[trunk][2] < FALLEN_HEIGHT:
                fell_at = f
                print(f"Fell over {f / fps:.1f} s into the clip")
            f += 1
            if f >= len(qpos) or (fell_at is not None and f - fell_at > 2 * fps):
                if fell_at is None:
                    print(f"Stayed up for the whole clip ({len(qpos) / fps:.1f} s)")
                restart[0] = True
            time.sleep(max(0.0, 1.0 / fps - (time.time() - start)))


# ----------------------------------------------------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "bvh",
        type=Path,
        help="Input BVH (LAFAN1, or a Motive skeleton export), or a previous output .npz to replay without re-solving",
    )
    parser.add_argument("-o", "--output", type=Path, help="Output .npz (default: <bvh name>_k1.npz next to the input)")
    parser.add_argument("--fps", type=float, default=50.0, help="Output frame rate (default 50)")
    parser.add_argument("--start", type=int, default=0, help="First BVH frame to use")
    parser.add_argument("--end", type=int, default=None, help="BVH frame to stop before")
    parser.add_argument("--up", choices=("y", "z"), default="y", help="Up axis of the BVH (LAFAN1 and Motive: y)")
    parser.add_argument("--names", choices=("auto", *JOINT_NAME_PRESETS), default="auto", help="BVH joint naming")
    parser.add_argument("--solver", default="daqp", help="qpsolvers backend for mink (default daqp)")
    parser.add_argument("--max-iters", type=int, default=20, help="IK iterations per frame")
    parser.add_argument("--tol", type=float, default=1e-4, help="Stop iterating when the IK step is below this")
    parser.add_argument(
        "--restart-error",
        type=float,
        default=None,
        help="Mean keypoint error (m) above which a frame is re-solved from the neutral pose "
        "(default 0.03, or 0.08 with --balance, which tracks less closely by design)",
    )
    parser.add_argument("--csv", action="store_true", help="Also write joint angles as CSV")
    parser.add_argument(
        "--balance",
        action="store_true",
        help="Experimental: plant stance feet and keep the centre of mass over them (see README for its limits)",
    )
    parser.add_argument("--view", action="store_true", help="Play the result in the MuJoCo viewer (kinematic)")
    parser.add_argument(
        "--physics",
        action="store_true",
        help="Play the result in the MuJoCo viewer with physics: joint angles go to position servos, no balance control",
    )
    args = parser.parse_args()

    if args.bvh.suffix == ".npz":
        saved = np.load(args.bvh)
        qpos = np.concatenate([saved["root_pos"], saved["root_quat_wxyz"], saved["joint_pos"]], axis=1)
        targets = {k: saved["keypoint_targets"][:, i] for i, k in enumerate(saved["keypoint_names"])}
        fps = float(saved["fps"])
        print(f"Loaded {args.bvh.name}: {len(qpos)} frames at {fps:g} fps")
        if args.physics:
            simulate(qpos, targets, fps)
        else:
            view(build_k1_model(), qpos, targets, fps)
        return

    bvh = load_bvh(args.bvh)
    src_fps = 1.0 / bvh.frame_time
    print(f"Loaded {args.bvh.name}: {bvh.motion.shape[0]} frames at {src_fps:.1f} fps, {len(bvh.names)} joints")
    bvh.motion = bvh.motion[args.start : args.end]

    joint_idx = resolve_joint_names(bvh.names, args.names)
    positions = bvh_global_positions(bvh)
    if args.up == "y":
        positions = positions @ Y_UP_TO_Z_UP.T

    # Resample keypoint positions to the output rate (positions interpolate cleanly, unlike Euler angles)
    t_src = np.arange(positions.shape[0]) / src_fps
    t_out = np.arange(0.0, t_src[-1] + 1e-9, 1.0 / args.fps)
    human = {
        k: np.stack([np.interp(t_out, t_src, positions[:, j, a]) for a in range(3)], axis=-1)
        for k, j in joint_idx.items()
    }

    model = build_k1_model()
    targets, orientations, scale = build_targets(human, robot_bones(model), ANKLE_HEIGHT)
    print("Bone scale factors (robot / human): " + ", ".join(f"{k} {v:.3g}" for k, v in scale.items()))

    if args.restart_error is None:
        args.restart_error = 0.08 if args.balance else 0.03
    contacts = None
    if args.balance:
        contacts = detect_contacts(targets, args.fps)
        plant_feet(targets, orientations, contacts)
        print(
            "Feet on the ground: "
            + ", ".join(
                f"{side} {c.mean() * 100:.0f}% of frames ({len(_runs(c))} contacts)" for side, c in contacts.items()
            )
        )

    qpos, errors = retarget(
        model, targets, orientations, args.solver, args.max_iters, args.tol, args.restart_error, contacts, args.fps
    )
    worst = np.argsort(errors.mean(0))[::-1][:3]
    print(
        f"Mean keypoint error {errors.mean() * 100:.1f} cm; worst keypoints: "
        + ", ".join(f"{list(KEYPOINTS)[i]} {errors[:, i].mean() * 100:.1f} cm" for i in worst)
    )

    # Joint names in qpos order (excluding the free joint)
    joint_names = [model.joint(j).name for j in range(model.njnt) if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE]
    output = args.output or args.bvh.with_name(args.bvh.stem + "_k1.npz")
    np.savez(
        output,
        fps=args.fps,
        joint_names=np.array(joint_names),
        root_pos=qpos[:, 0:3],
        root_quat_wxyz=qpos[:, 3:7],
        joint_pos=qpos[:, 7:],
        keypoint_names=np.array(list(KEYPOINTS)),
        **({f"contact_{side}": c for side, c in contacts.items()} if contacts else {}),
        keypoint_targets=np.stack([targets[k] for k in KEYPOINTS], axis=1),
        keypoint_errors=errors,
    )
    print(f"Wrote {output}")
    if args.csv:
        csv = output.with_suffix(".csv")
        header = "time," + ",".join(joint_names)
        np.savetxt(csv, np.column_stack([t_out, qpos[:, 7:]]), delimiter=",", header=header, comments="", fmt="%.6f")
        print(f"Wrote {csv}")

    if args.physics:
        simulate(qpos, targets, args.fps)
    elif args.view:
        view(model, qpos, targets, args.fps)


if __name__ == "__main__":
    main()
