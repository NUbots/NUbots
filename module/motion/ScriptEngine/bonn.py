#!/usr/bin/env python3

import yaml
import sys
import math

with open(sys.argv[1], 'r') as f:
    try:
        source = yaml.load(f)
        dest = []
    except yaml.YAMLError as exc:
        print(exc)

    for frame in source['motion']:

        targets = []
        for key in frame['joints']:
            joint = frame['joints'][key]
            name = {
                'right_hip_yaw': 'R_HIP_YAW',
                'right_hip_pitch': 'R_HIP_PITCH',
                'neck_yaw': 'HEAD_YAW',
                'right_ankle_roll': 'R_ANKLE_ROLL',
                'right_shoulder_roll': 'R_SHOULDER_ROLL',
                'head_pitch': 'HEAD_PITCH',
                'left_hip_pitch': 'L_HIP_PITCH',
                'left_hip_roll': 'L_HIP_ROLL',
                'right_elbow_pitch': 'R_ELBOW',
                'right_ankle_pitch': 'R_ANKLE_PITCH',
                'left_shoulder_pitch': 'L_SHOULDER_PITCH',
                'right_shoulder_pitch': 'R_SHOULDER_PITCH',
                'left_hip_yaw': 'L_HIP_YAW',
                'left_knee_pitch': 'L_KNEE',
                'left_ankle_roll': 'L_ANKLE_ROLL',
                'left_shoulder_roll': 'L_SHOULDER_ROLL',
                'right_knee_pitch': 'R_KNEE',
                'left_ankle_pitch': 'L_ANKLE_PITCH',
                'right_hip_roll': 'R_HIP_ROLL',
                'left_elbow_pitch': 'L_ELBOW',
            }[key]

            offset = {
                'HEAD_PITCH': math.pi/8.0,
                'L_SHOULDER_PITCH': math.pi/2.0,
                'R_SHOULDER_PITCH': math.pi/2.0,
            }.get(name, 0)

            targets.append({
                'id': name,
                'position': joint['position'] + offset,
                'gain': int(joint['effort'] * 100),
                'torque': 100,
            })

        dest.append({
            'duration': int(frame['duration'] * 1000),
            'targets': targets,
        })

with open(sys.argv[2], 'w') as f:
    yaml.dump(dest, f)
