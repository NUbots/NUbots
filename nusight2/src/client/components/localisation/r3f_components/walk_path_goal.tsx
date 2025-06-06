import React from "react";
import * as THREE from "three";
import URDFLoader, { URDFRobot } from "urdf-loader";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { ServoMotorSet } from "../robot_model";

interface WalkPathGoalProps {
  Hfd: Matrix4;
  Hft: Matrix4;
  motors: ServoMotorSet;
}

const nugusUrdfPath = "/robot-models/nugus/robot.urdf";

export const WalkPathGoal: React.FC<WalkPathGoalProps> = ({ Hfd, Hft, motors }) => {
  const robotRef = React.useRef<URDFRobot | null>(null);

  // Load the URDF model only once
  React.useEffect(() => {
    const loader = new URDFLoader();
    loader.load(nugusUrdfPath, (robot: URDFRobot) => {
      if (robotRef.current) {
        robotRef.current.add(robot);
      }
    });
  }, []);

  const rDFf = Hfd.decompose().translation;
  const rTFf = Hft.decompose().translation;
  const Rfd_quat = new THREE.Quaternion(
    Hfd.decompose().rotation.x,
    Hfd.decompose().rotation.y,
    Hfd.decompose().rotation.z,
    Hfd.decompose().rotation.w,
  );
  const Rft_quat = new THREE.Quaternion(
    Hft.decompose().rotation.x,
    Hft.decompose().rotation.y,
    Hft.decompose().rotation.z,
    Hft.decompose().rotation.w,
  );

  // Get euler angles from quaternion
  const Rfz_euler = new THREE.Euler().setFromQuaternion(Rfd_quat, "ZYX");
  const Rft_euler = new THREE.Euler().setFromQuaternion(Rft_quat, "ZYX");
  // Fuse the euler angles into a single quaternion
  const rotation = new THREE.Quaternion().setFromEuler(new THREE.Euler(Rft_euler.x, Rft_euler.y, Rfz_euler.z, "ZYX"));
  const position = new THREE.Vector3(rDFf?.x, rDFf?.y, rTFf.z);

  // Update the position of the robot to match the walk path goal
  React.useEffect(() => {
    if (robotRef.current) {
      robotRef.current.position.set(position.x, position.y, position.z);
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
      robotRef.current.traverse((child) => {
        if (child instanceof THREE.Mesh) {
          // Set opacity for all mesh children
          child.material.transparent = true;
          child.material.color.setStyle("rgb(0, 100, 100)");
          child.material.opacity = 0.2;
        }
      });
    }
  });

  return <object3D ref={robotRef} />;
};
