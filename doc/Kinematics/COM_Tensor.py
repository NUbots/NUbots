#!/usr/bin/env python3
"""Takes the CoM for each component of each particle in the robot system and combines them into a single CoM for
each particle.

The particles are defined as head, upper arm, lower arm, torso, hip, upper leg, lower leg, ankle, and foot.

It is assumed that the particles are symmetric across the body so that we dont need a separate particle for the left
and right upper arms (for example)
"""
import json
import os
import sys

import numpy as np
import yaml

# CoM for each particle in the system
particles = {
    "torso": {"Mass": 0, "CoM": np.zeros((3,)), "Tensor": np.zeros((3, 3))},
    "head": {"Mass": 0, "CoM": np.zeros((3,)), "Tensor": np.zeros((3, 3))},
    "arm_upper": {"Mass": 0, "CoM": np.zeros((3,)), "Tensor": np.zeros((3, 3))},
    "arm_lower": {"Mass": 0, "CoM": np.zeros((3,)), "Tensor": np.zeros((3, 3))},
    "leg_upper": {"Mass": 0, "CoM": np.zeros((3,)), "Tensor": np.zeros((3, 3))},
    "leg_lower": {"Mass": 0, "CoM": np.zeros((3,)), "Tensor": np.zeros((3, 3))},
    "foot": {"Mass": 0, "CoM": np.zeros((3,)), "Tensor": np.zeros((3, 3))},
    "ankle_block": {"Mass": 0, "CoM": np.zeros((3,)), "Tensor": np.zeros((3, 3))},
    "hip_block": {"Mass": 0, "CoM": np.zeros((3,)), "Tensor": np.zeros((3, 3))},
}

# Load CoM data for all particles and their components
absolute_path = sys.argv[1] if len(sys.argv) == 2 else ""

with open(os.path.join(absolute_path, "mass.yaml"), "r") as f_mass:
    component_data = yaml.load(f_mass)

# Combine particle component CoMs into a single CoM for each particle
for particle in particles:
    with open(os.path.join(absolute_path, "Igus_{}.yaml".format(particle)), "r") as f:
        components = yaml.load(f)

    for component in components:
        comp_com = np.asarray(components[component]["CoM"])
        comp_mass = component_data[component]
        part_com = np.asarray(particles[particle]["CoM"])
        part_mass = particles[particle]["Mass"]
        particles[particle]["CoM"] = ((comp_com * comp_mass + part_com * part_mass) / (comp_mass + part_mass)).tolist()
        particles[particle]["Mass"] = part_mass + comp_mass

    (t_x, t_y, t_z) = particles[particle]["CoM"]

    t_com = 0

    for component in components:
        (x, y, z) = np.asarray(components[component]["CoM"])
        comp_mass = component_data[component]

        (d_x, d_y, d_z) = (t_x - x, t_y - y, t_z - z)

        # Skew-symmetric matrix for CoM displacement
        # https://en.wikipedia.org/wiki/Parallel_axis_theorem#Identities_for_a_skew-symmetric_matrix
        # This calculates -[d][d] = -[d]^2, where [d] is the skew-symmetric matrix constructed from (d_x, d_y, d_z)
        # fmt: off
        d = np.asarray(
            [
                [d_y ** 2 + d_z ** 2, -d_x * d_y,           -d_x * d_z],
                [-d_x * d_y,           d_x ** 2 + d_z ** 2, -d_y * d_z],
                [-d_x * d_z,          -d_y * d_z,            d_x ** 2 + d_y ** 2],
            ]
        )
        # fmt: on

        t_origin = np.asmatrix(components[component]["Tensor"])

        # https://en.wikipedia.org/wiki/Parallel_axis_theorem#Moment_of_inertia_matrix
        # Formula is [I_S] = [I_R] - M[d]^2
        # d = -[d][d] = -[d]^2
        # => [I_S] = [I_R] + Md
        # Where [I_S] is the inertia matrix relative to point S and [I_R] is the inertia matrix relative to the CoM
        # M is the mass of the particle
        # We want to transform the inertia matrix from point S to the CoM, so the formula should be
        # [I_R] = [I_S] - Md
        t_com += t_origin - comp_mass * d

    # Multiple by the mass of the particle to put the tensor into units of kg.m^2
    particles[particle]["Tensor"] = (t_com * particles[particle]["Mass"]).tolist()

# Print out the details
for particle in particles:
    print("{:11s}: {}".format(particle, particles[particle]))
    # print(
    #     "{:11s}: {}\n{}\n{}".format(
    #         particle, particles[particle]["CoM"], particles[particle]["Mass"], particles[particle]["Tensor"]
    #     )
    # )

# Print out the total weight for the robot
print(
    "{:11s}: {}".format(
        "Total",
        particles["torso"]["Mass"]
        + particles["head"]["Mass"]
        + 2.0 * particles["arm_upper"]["Mass"]
        + 2.0 * particles["arm_lower"]["Mass"]
        + 2.0 * particles["leg_upper"]["Mass"]
        + 2.0 * particles["leg_lower"]["Mass"]
        + 2.0 * particles["foot"]["Mass"]
        + 2.0 * particles["ankle_block"]["Mass"]
        + 2.0 * particles["hip_block"]["Mass"],
    )
)

with open("COM.yaml", "w") as f:
    f.write(yaml.dump(particles, default_flow_style=None))
