import React from "react";
import * as THREE from "three";
import URDFLoader, { URDFRobot } from "urdf-loader";

import { KinematicsRobotModel } from "../../robot_model";

const nugusUrdfPath = "/robot-models/nugus/robot.urdf";

export const Nugus = ({ model }: { model: KinematicsRobotModel }) => {
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
      robotRef.current.quaternion.copy(new THREE.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));
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

  // Update the material of the robot
  const material = new THREE.MeshStandardMaterial({
    color: "#666666",
    roughness: 0.5,
    metalness: 0.2,
  });
  if (robotRef.current) {
    robotRef.current.traverse((child) => {
      if (child.type === "URDFVisual" && child.children.length > 0) {
        const mesh = child.children[0] as THREE.Mesh;
        mesh.material = material;
      }
    });
  }

  return <object3D ref={robotRef} />;
};
