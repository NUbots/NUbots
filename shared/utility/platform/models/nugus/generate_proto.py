#
# MIT License
#
# Copyright (c) 2023 NUbots
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
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pymeshlab
from scipy.spatial import ConvexHull
from urdf2webots.importer import convertUrdfFile

# Parse URDF file
tree = ET.parse("robot.urdf")
root = tree.getroot()

# Get all mesh filenames from URDF
mesh_files = []
collision_elements = []
for mesh in root.findall(".//mesh"):
    filename = mesh.attrib["filename"]

    # Remove 'package:///' prefix
    clean_filename = filename.replace("package:///", "")
    mesh.set("filename", clean_filename)

    mesh_files.append(clean_filename)
    if clean_filename.endswith("_collision.stl"):
        collision_elements.append(mesh)

# Simplify each STL file referenced in URDF
for file in mesh_files:
    # Load mesh
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(file)
    # Simplify mesh
    ms.meshing_decimation_quadric_edge_collapse(targetfacenum=21845)
    # Save simplified mesh
    ms.save_current_mesh(file)

# Find all collision STL files
collision_files = [file for file in mesh_files if file.endswith("_collision.stl")]

# Process collision elements
for collision in root.findall(".//collision"):
    mesh_element = collision.find(".//mesh")

    # Remove mesh element from geometry
    geometry_element = collision.find(".//geometry")

    if mesh_element is not None:
        filename = mesh_element.attrib["filename"].replace("package:///", "")

        # Load mesh
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(filename)

        # Compute the optimal bounding box
        bounding_box = ms.current_mesh().bounding_box()

        # Get min and max coordinates
        min_x = bounding_box.min()[0]
        max_x = bounding_box.max()[0]
        min_y = bounding_box.min()[1]
        max_y = bounding_box.max()[1]
        min_z = bounding_box.min()[2]
        max_z = bounding_box.max()[2]

        # Calculate bounding box dimensions
        x_size = max_x - min_x
        y_size = max_y - min_y
        z_size = max_z - min_z

        # Calculate center of bounding box
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        center_z = (min_z + max_z) / 2

        # Create or update the origin element
        origin_element = collision.find("./origin")
        if origin_element is None:
            origin_element = ET.SubElement(collision, "origin")
        origin_element.attrib["xyz"] = f"{center_x} {center_y} {center_z}"

        # Create box element
        box_element = ET.Element("box", size=f"{x_size} {y_size} {z_size}")

        # Find the parent geometry element
        if geometry_element is not None:
            # Remove mesh element from the geometry
            geometry_element.remove(mesh_element)

            # Add the box element to the geometry
            geometry_element.append(box_element)

# Save modified URDF
tree.write("robot.urdf")

# Delete all STL files not in URDF
for file in os.listdir("."):
    if file.endswith(".stl") and file not in mesh_files:
        os.remove(file)

# Delete all STL files used for collision
for file in os.listdir("."):
    if file.endswith("_collision.stl"):
        os.remove(file)

# Delete all .part files
for file in os.listdir("."):
    if file.endswith(".part"):
        os.remove(file)


urdf_file_path = "robot.urdf"
proto_file_path = "nugus.proto"

# Convert URDF to PROTO using urdf2webots library
convertUrdfFile(input=urdf_file_path, output=proto_file_path, boxCollision=False, normal=False)

# Read the existing proto file
with open(proto_file_path, "r") as file:
    filedata = file.read()

# Replace constants block
filedata = filedata.replace(
    """PROTO nugus [
  field  SFVec3f     translation     0 0 0
  field  SFRotation  rotation        0 0 1 0
  field  SFString    name            "nugus"  # Is `Robot.name`.
  field  SFString    controller      "void"   # Is `Robot.controller`.
  field  MFString    controllerArgs  []       # Is `Robot.controllerArgs`.
  field  SFString    customData      ""       # Is `Robot.customData`.
  field  SFBool      supervisor      FALSE    # Is `Robot.supervisor`.
  field  SFBool      synchronization TRUE     # Is `Robot.synchronization`.
  field  SFBool      selfCollision   FALSE    # Is `Robot.selfCollision`.
]""",
    """
EXTERNPROTO "JerseyBack.proto"
EXTERNPROTO "JerseyFront.proto"
PROTO nugus 
[
    field  SFVec3f     translation          0 0 0
    field  SFRotation  rotation             0 1 0 0
    field  SFString    name                 "nugus"            # Is `Robot.name`.
    # Find in "webots/projects/samples/contests/robocup/controllers/player"
    field  SFString    controller           "player"
    # Is `Robot.controllerArgs`.
    field  MFString    controllerArgs       []
    # Is `Robot.customData`.
    field  SFString    customData           ""
    # Is `Robot.supervisor`.
    field  SFBool      supervisor           FALSE
    # Is `Robot.synchronization`.
    field  SFBool      synchronization      TRUE
    # Is `Robot.selfCollision`.
    field  SFBool      selfCollision        FALSE
    # MOTOR PARAMETER: See section 2. of docs/Robot_Model_RoboCup_2021 for more information
    field  SFFloat     MX106-torque         10.00
    field  SFFloat     MX106-vel            10.00
    field  SFFloat     MX106-damping        1.23
    field  SFFloat     MX106-friction       2.55
    field  SFFloat     DYNAMIXEL-RESOLUTION 0.0015
    # CAMERA PARAMETERS: See docs for more information
    # Approximates PI/2 radians (90 degrees)
    field SFFloat      fieldOfView            1.5707
    field SFInt32      cameraWidth            640              # 640 pixels
    field SFInt32      cameraHeight           480              # 480 pixels
    # 1.98mm, as from our real cameras (https://www.lensation.de/pdf/BF10M19828S118C.pdf)
    field SFFloat      cameraLensFocalLength  1.98
    # Not much noise on real cameras
    field SFFloat      cameraNoise            0.000000001
    # With over 100fps, real cameras do not have much motion blur
    field SFFloat      cameraMotionBlur       10
    field MFColor      recognitionColors      [0 0 1]
    # Used in the vision data collection tool
    field SFFloat      height                 0.51
  ]""",
)

# Add recognition colors
filedata = filedata.replace(
    "selfCollision IS selfCollision",
    """selfCollision IS selfCollision
    recognitionColors IS recognitionColors""",
)

# Replace all servo parameters with constants and change to HingeJointWithBacklash
# TODO: Replace the joints with the correct constants (currently using MX106 only)
filedata = filedata.replace("maxVelocity 20.0", "maxVelocity IS MX106-vel")
filedata = filedata.replace("maxTorque 1.0", "maxTorque IS MX106-torque")
filedata = filedata.replace(
    "HingeJointParameters {",
    "HingeJointParameters {\n                    dampingConstant IS MX106-damping\n                    staticFriction IS MX106-friction",
)
filedata = filedata.replace("PositionSensor {", "PositionSensor {\n                 resolution IS DYNAMIXEL-RESOLUTION")

# Add gyro and accelerometer to torso
filedata = filedata.replace(
    """recognitionColors IS recognitionColors
    children [""",
    """recognitionColors IS recognitionColors
    children [
                            %{
                              if fields.name.value ~= '' then
                                -- name is supposed to be something like "red player 2" or "blue player 1"
                                local words = {}
                                for word in fields.name.value:gmatch("%w+") do table.insert(words, word) end
                                local color = words[1]
                                local number = words[3]
                            }%
                            Transform {
                              translation 0.00250000 0.000000 -0.0010000
                              rotation 0.000000 0.000000 1.000000 1.570790
                              children [
                                JerseyFront {
                                  jerseyFrontTexture [
                                  %{='"./NUgus_textures/NUgus_front_' .. tostring(color)  .. '_' .. tostring(number) .. '.jpg"'}%
                                  ]
                                }
                              ]
                            }

                            Transform {
                              translation -0.00250000 0.000000 -0.0010000
                              rotation 0.000000 0.000000 1.000000 1.570790
                              children [
                                JerseyBack {
                                  jerseyBackTexture [
                                  %{='"./NUgus_textures/NUgus_back_' .. tostring(color)  .. '_' .. tostring(number) .. '.jpg"'}%
                                  %{ end }%
                                  ]
                                }
                              ]
                            }
                            Transform {
                                # Accelerometer and Gyro are positioned 0.2m above torso base
                                translation 0.000000 0.000000 0.2
                                children [
                                #(See Section 1. of docs/Robot_Model_RoboCup_2021 for details on noise)
                                # Axis: x forward, y left, z up
                                # Units are [m/s^2]
                                # The acelerometer measures the reaction forces over 3 axes in [m/s^2], thus,
                                # at rest the accelerometer should read +9.81 [m/s^2] in the z axis.
                                # Range is -39.24 to 39.24 [m/s^2]
                                # Return value (2nd column of LUT) is offset by 100
                                Accelerometer {
                                    name "accelerometer"
                                    xAxis TRUE
                                    yAxis TRUE
                                    zAxis TRUE
                                    # Calculation used: (size of range)/4095
                                    resolution 0.01916
                                    lookupTable [
                                    -39.24 60.76 0.000704
                                    39.24  139.24 0.000307
                                    ]
                                }
                                # 'L3G4200D' Gyro
                                # (See Section 1. of docs/Robot_Model_RoboCup_2021 for details on noise)
                                # Axis: x forward, y left, z up
                                # Units are [rad/s]
                                # Range is -8.72665 to 8.72665 [rad/s]
                                # Return value (2nd column of LUT) is offset by 100
                                Gyro {
                                    name "gyroscope"
                                    xAxis TRUE
                                    yAxis TRUE
                                    zAxis TRUE
                                    # Calculation used: (size of range)/4095
                                    resolution 0.0042621
                                    lookupTable [
                                    -8.72665 91.238   0.0001151
                                    8.72665  108.762  0.000096541
                                    ]
                                }
                                ]
                            }
                        """,
)

# Add cameras to head
filedata = filedata.replace(
    ''']
                    name "right_camera"''',
    '''
                      DEF right_camera Camera {
                        name "right_camera"
                        translation 0.0 0.0 0.0
                        rotation 0.0 0.0 1.0 0.0
                        # Set parameters from fields
                        fieldOfView IS fieldOfView
                        width IS cameraWidth
                        height IS cameraHeight
                        noise IS cameraNoise
                        motionBlur IS cameraMotionBlur
                        # We are using a rectilinear lens because there is no true spherical lens
                        spherical FALSE
                        # Set the recognition parameters from the left camera
                        recognition USE recognition
                      }
                    ]
                    name "right_camera"''',
)
filedata = filedata.replace(
    ''']
                    name "left_camera"''',
    '''
                      DEF left_camera Camera {
                        name "left_camera"
                        translation 0.0 0.0 0.0
                        rotation 0.0 0.0 1.0 0.0
                        # Set parameters from fields
                        fieldOfView IS fieldOfView
                        width IS cameraWidth
                        height IS cameraHeight
                        noise IS cameraNoise
                        motionBlur IS cameraMotionBlur
                        # We are using a rectilinear lens because there is no true spherical lens
                        spherical FALSE
                        # Allows the robot to detect objects in the world
                        recognition DEF recognition Recognition {
                          frameThickness 0       # Remove bounding boxes
                          segmentation TRUE      # Add segmentation
                        }
                      }
                    ]
                    name "left_camera"''',
)

# Remove camera frames (causing physics issues in webots)
filedata = filedata.replace(
    """name "left_camera"
                    boundingObject Pose {
                      translation 0.000000 0.000000 0.000005
                      children [
                        Box {
                           size 0.000010 0.000010 0.000010
                        }
                      ]
                    }
                    physics Physics {
                      density -1
                      mass 0.000000
                      centerOfMass [ -0.000000 0.000000 0.000005 ]
                    }""",
    "",
)

filedata = filedata.replace(
    """name "right_camera"
                    boundingObject Pose {
                      translation 0.000000 0.000000 0.000005
                      children [
                        Box {
                           size 0.000010 0.000010 0.000010
                        }
                      ]
                    }
                    physics Physics {
                      density -1
                      mass 0.000000
                      centerOfMass [ -0.000000 0.000000 0.000005 ]
                    }""",
    "",
)

# Add touch sensors to feet
filedata = filedata.replace(
    ''']
                                        name "right_foot"''',
    '''
                                          # Define four touch sensors for the right foot
                                          # Back right touch sensor on right foot
                                          TouchSensor {
                                            translation 0.0379995 -0.0767345 -0.0867505
                                            name "right_touch_sensor_br"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            # Sensor Type
                                            # Bumper returns a 1 when collision is detected
                                            type "bumper"
                                          }
                                          
                                          # Back left touch sensor on right foot
                                          TouchSensor {
                                            translation 0.0379995 0.0548465  -0.0867505
                                            name "right_touch_sensor_bl"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                          # Front left touch sensor on right foot
                                          TouchSensor {
                                            translation 0.0379995 0.0548465 0.129071
                                            name "right_touch_sensor_fl"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                          # Front right touch sensor on right foot
                                          TouchSensor {
                                            translation 0.0379995  -0.0767345 0.129071
                                            name "right_touch_sensor_fr"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                        ]
                                        name "right_foot [foot]"''',
)

filedata = filedata.replace(
    ''']
                                        name "left_foot"''',
    '''
                                          # Define four touch sensors for the left foot
                                          # Back right touch sensor on left foot
                                          TouchSensor {
                                            translation 0.0379995 -0.0548465 -0.0867505
                                            name "left_touch_sensor_br"
                                            children [
                                              # Define shape for all touch sensors
                                              DEF touch_shape Shape{
                                                appearance PBRAppearance {
                                                  baseColor 0.125 0.125 0.125
                                                  transparency 0.000000
                                                  roughness 1.000000
                                                  metalness 0
                                                  emissiveColor 0.000000 0.000000 0.000000
                                                }
                                              }
                                            ]
                                            # Bounding object uses geometry from touch_shape
                                            boundingObject DEF touch_box Transform {
                                              translation 0.0 0.0 0.0
                                              rotation 0.0 0.0 1.0 1.571
                                              children [
                                                Capsule {
                                                  bottom      TRUE   # {TRUE, FALSE}
                                                  height      0.001  # [0, inf)
                                                  radius      0.009  # [0, inf)
                                                  side        TRUE   # {TRUE, FALSE}
                                                  top         FALSE  # {TRUE, FALSE}
                                                  subdivision 12     # [3, inf)
                                                }
                                              ]
                                            }
                                            # Define physics for all touch sensors
                                            physics DEF touch_physics Physics{
                                              density -1
                                              mass 0.02
                                              centerOfMass 0.0 0.0 0.0
                                            }
                                            type "bumper"
                                          }
                                          # Back left touch sensor on left foot
                                          TouchSensor {
                                            translation 0.0379995 0.0767345 -0.0867505
                                            name "left_touch_sensor_bl"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                          # Front left touch sensor on left foot
                                          TouchSensor {
                                            translation 0.0379995 0.0767345 0.129071
                                            name "left_touch_sensor_fl"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                          # Front right touch sensor on left foot
                                          TouchSensor {
                                            translation 0.0379995 -0.0548465 0.129071
                                            name "left_touch_sensor_fr"
                                            boundingObject USE touch_box
                                            physics USE touch_physics
                                            type "bumper"
                                          }
                                        ]
                                        name "left_foot [foot]"''',
)
# Rename limbs
filedata = filedata.replace('''name "right_shoulder_pitch"''', '''name "right_shoulder_pitch [shoulder]"''')
filedata = filedata.replace('''name "left_shoulder_pitch"''', '''name "left_shoulder_pitch [shoulder]"''')
filedata = filedata.replace('''name "right_hip_roll"''', '''name "right_hip_roll [hip]"''')
filedata = filedata.replace('''name "left_hip_roll"''', '''name "left_hip_roll [hip]"''')

# Fix naming issue of bounding object caused by urdf2webots tool
filedata = filedata.replace("boundingObject Pose", "boundingObject Transform")

# Fix issue with mass of bounding object
filedata = filedata.replace("mass 0.000000", "mass 1e-8")
filedata = filedata.replace("mass -1", "mass 1e-8")

# Update colours
filedata = filedata.replace("baseColor 0.286275 0.286275 0.286275", "baseColor 0.125 0.125 0.125")

# Write the update proto file
with open(proto_file_path, "w") as file:
    file.write(filedata)

print("Proto file updated")
