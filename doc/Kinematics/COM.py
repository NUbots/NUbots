#!/usr/bin/env python3
"""Takes the CoM for each component of each particle in the robot system and combines them into a single CoM for
each particle.

The particles are defined as head, upper arm, lower arm, torso, hip, upper leg, lower leg, ankle, and foot.

It is assumed that the particles are symmetric across the body so that we dont need a separate particle for the left
and right upper arms (for example)
"""
import yaml
import numpy as np

# TODO: Weigh these and incorporate them with the shelf CoM for the torso
NUC = 0.300
NUSENSE = 0.060

# CoM for each particle in the system
particles = {
    "torso": np.zeros((4,)),
    "head": np.zeros((4,)),
    "arm_upper": np.zeros((4,)),
    "arm_lower": np.zeros((4,)),
    "leg_upper": np.zeros((4,)),
    "leg_lower": np.zeros((4,)),
    "foot": np.zeros((4,)),
    "ankle_block": np.zeros((4,)),
    "hip_block": np.zeros((4,)),
}

# Load CoM data for all particles and their components
with open("COM.yaml") as f:
    data = yaml.load(f)

# Combine particle component CoMs into a single CoM for each particle
for particle in particles:
    for component in data[particle]:
        # Convert from mm to m
        comp_com = np.asarray(component[:3]) / 1000.0
        comp_mass = component[3]
        part_com = particles[particle][:3]
        part_mass = particles[particle][3]
        particles[particle][:3] = (comp_com * comp_mass + part_com * part_mass) / (comp_mass + part_mass)
        particles[particle][3] = part_mass + comp_mass

# Print out the details
for particle in particles:
    print("{:11s}: {}".format(particle, particles[particle]))

# Print out the total weight for the robot
print(
    "{:11s}: {}".format(
        "Total",
        particles["torso"][3]
        + particles["head"][3]
        + 2.0 * particles["arm_upper"][3]
        + 2.0 * particles["arm_lower"][3]
        + 2.0 * particles["leg_upper"][3]
        + 2.0 * particles["leg_lower"][3]
        + 2.0 * particles["foot"][3]
        + 2.0 * particles["ankle_block"][3]
        + 2.0 * particles["hip_block"][3]
        + NUC
        + NUSENSE,
    )
)
