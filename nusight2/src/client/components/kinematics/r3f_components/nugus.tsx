import React from "react";
import { observer } from "mobx-react";
import * as THREE from "three";
import URDFLoader, { URDFRobot } from "urdf-loader";

import { KinematicsRobotModel } from "../robot_model";

const nugusUrdfPath = "/robot-models/nugus/robot.urdf";

// Mapping from joint names to servo IDs
const jointToServoId: { [key: string]: number } = {
  head_pitch: 19, // HEAD_PITCH
  left_ankle_pitch: 15, // L_ANKLE_PITCH
  left_ankle_roll: 17, // L_ANKLE_ROLL
  left_elbow_pitch: 5, // L_ELBOW
  left_hip_pitch: 11, // L_HIP_PITCH
  left_hip_roll: 9, // L_HIP_ROLL
  left_hip_yaw: 7, // L_HIP_YAW
  left_knee_pitch: 13, // L_KNEE
  left_shoulder_pitch: 1, // L_SHOULDER_PITCH
  left_shoulder_roll: 3, // L_SHOULDER_ROLL
  neck_yaw: 18, // HEAD_YAW
  right_ankle_pitch: 14, // R_ANKLE_PITCH
  right_ankle_roll: 16, // R_ANKLE_ROLL
  right_elbow_pitch: 4, // R_ELBOW
  right_hip_pitch: 10, // R_HIP_PITCH
  right_hip_roll: 8, // R_HIP_ROLL
  right_hip_yaw: 6, // R_HIP_YAW
  right_knee_pitch: 12, // R_KNEE
  right_shoulder_pitch: 0, // R_SHOULDER_PITCH
  right_shoulder_roll: 2, // R_SHOULDER_ROLL
};

export const Nugus = observer(({ model }: { model: KinematicsRobotModel }) => {
  const robotRef = React.useRef<URDFRobot | null>(null);

  // Load the URDF model only once
  React.useEffect(() => {
    const loader = new URDFLoader();
    loader.load(nugusUrdfPath, (robot: URDFRobot) => {
      // Add the loaded robot to the wrapper
      if (robotRef.current) {
        robotRef.current.add(robot);
      }
    });
  }, [nugusUrdfPath]);

  const rotation = model.Hft.decompose().rotation;
  const motors = model.motors;

  React.useEffect(() => {
    // Update robot's pose
    if (robotRef.current) {
      const joints = (robotRef.current?.children[0] as any)?.joints;
      // Update robot's joints
      if (joints) {
        joints?.head_pitch.setJointValue(motors.headTilt.angle);
        joints?.left_ankle_pitch.setJointValue(motors.leftAnklePitch.angle);
        joints?.left_ankle_roll.setJointValue(motors.leftAnkleRoll.angle);
        joints?.left_elbow_pitch.setJointValue(motors.leftElbow.angle);
        joints?.left_hip_pitch.setJointValue(motors.leftHipPitch.angle);
        joints?.left_hip_roll.setJointValue(motors.leftHipRoll.angle);
        joints?.left_hip_yaw.setJointValue(motors.leftHipYaw.angle);
        joints?.left_knee_pitch.setJointValue(motors.leftKnee.angle);
        joints?.left_shoulder_pitch.setJointValue(motors.leftShoulderPitch.angle);
        joints?.left_shoulder_roll.setJointValue(motors.leftShoulderRoll.angle);
        joints?.neck_yaw.setJointValue(motors.headPan.angle);
        joints?.right_ankle_pitch.setJointValue(motors.rightAnklePitch.angle);
        joints?.right_ankle_roll.setJointValue(motors.rightAnkleRoll.angle);
        joints?.right_elbow_pitch.setJointValue(motors.rightElbow.angle);
        joints?.right_hip_pitch.setJointValue(motors.rightHipPitch.angle);
        joints?.right_hip_roll.setJointValue(motors.rightHipRoll.angle);
        joints?.right_hip_yaw.setJointValue(motors.rightHipYaw.angle);
        joints?.right_knee_pitch.setJointValue(motors.rightKnee.angle);
        joints?.right_shoulder_pitch.setJointValue(motors.rightShoulderPitch.angle);
        joints?.right_shoulder_roll.setJointValue(motors.rightShoulderRoll.angle);
      }
    }
  }, [rotation, motors]);

  // Update materials based on temperature - apply on every render like localisation version
  const normalMaterial = new THREE.MeshStandardMaterial({
    color: "#666666",
    roughness: 0.5,
    metalness: 0.2,
  });

  const hotMaterial = new THREE.MeshStandardMaterial({
    color: "#FF4444",
    roughness: 0.5,
    metalness: 0.2,
    emissive: "#440000",
    emissiveIntensity: 0.2,
  });

  const errorMaterial = new THREE.MeshStandardMaterial({
    color: "#FF8800",
    roughness: 0.5,
    metalness: 0.2,
    emissive: "#442200",
    emissiveIntensity: 0.3,
  });

  if (robotRef.current) {
    robotRef.current.traverse((child) => {
      if (child.type === "URDFVisual" && child.children.length > 0) {
        const mesh = child.children[0] as THREE.Mesh;

        // Traverse up the hierarchy to find the joint
        let current = child.parent;
        let jointName = null;

        while (current) {
          if (current.type === "URDFJoint") {
            jointName = current.name;
            break;
          }
          current = current.parent;
        }

        if (jointName && jointToServoId[jointName] !== undefined) {
          const servoId = jointToServoId[jointName];
          const temperature = model.servoTemperatures.get(servoId);
          const error = model.servoErrors.get(servoId);
          const isOverLimit = temperature !== undefined && temperature > 50;
          const hasError = error !== undefined && error !== 0;

          // Priority: Error > Temperature > Normal
          if (hasError) {
            mesh.material = errorMaterial;
          } else if (isOverLimit) {
            mesh.material = hotMaterial;
          } else {
            mesh.material = normalMaterial;
          }
        } else {
          // Default material for non-joint parts
          mesh.material = normalMaterial;
        }

        mesh.castShadow = true;
        mesh.receiveShadow = true;
      }
    });
  }

  return <object3D ref={robotRef} />;
});
