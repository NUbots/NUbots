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

const k1UrdfPath = "/robot-models/k1/robot.urdf";

const setJointValue = (joints: any, names: string[], value: number) => {
  for (const name of names) {
    joints?.[name]?.setJointValue(value);
  }
};

export const WalkPathGoal: React.FC<WalkPathGoalProps> = ({ Hfd, Hft, motors }) => {
  const robotRef = React.useRef<URDFRobot | null>(null);

  // Load the URDF model only once
  React.useEffect(() => {
    const loader = new URDFLoader();
    loader.load(k1UrdfPath, (robot: URDFRobot) => {
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
        setJointValue(joints, ["Head_pitch", "head_pitch"], motors.headTilt.angle);
        setJointValue(joints, ["AAHead_yaw", "neck_yaw"], motors.headPan.angle);

        setJointValue(joints, ["ALeft_Shoulder_Pitch", "left_shoulder_pitch"], motors.leftShoulderPitch.angle);
        setJointValue(joints, ["Left_Shoulder_Roll", "left_shoulder_roll"], motors.leftShoulderRoll.angle);
        setJointValue(joints, ["Left_Elbow_Pitch", "left_elbow_pitch"], motors.leftElbow.angle);

        setJointValue(joints, ["ARight_Shoulder_Pitch", "right_shoulder_pitch"], motors.rightShoulderPitch.angle);
        setJointValue(joints, ["Right_Shoulder_Roll", "right_shoulder_roll"], motors.rightShoulderRoll.angle);
        setJointValue(joints, ["Right_Elbow_Pitch", "right_elbow_pitch"], motors.rightElbow.angle);

        setJointValue(joints, ["Left_Hip_Pitch", "left_hip_pitch"], motors.leftHipPitch.angle);
        setJointValue(joints, ["Left_Hip_Roll", "left_hip_roll"], motors.leftHipRoll.angle);
        setJointValue(joints, ["Left_Hip_Yaw", "left_hip_yaw"], motors.leftHipYaw.angle);
        setJointValue(joints, ["Left_Knee_Pitch", "left_knee_pitch"], motors.leftKnee.angle);
        setJointValue(joints, ["Left_Ankle_Pitch", "left_ankle_pitch"], motors.leftAnklePitch.angle);
        setJointValue(joints, ["Left_Ankle_Roll", "left_ankle_roll"], motors.leftAnkleRoll.angle);

        setJointValue(joints, ["Right_Hip_Pitch", "right_hip_pitch"], motors.rightHipPitch.angle);
        setJointValue(joints, ["Right_Hip_Roll", "right_hip_roll"], motors.rightHipRoll.angle);
        setJointValue(joints, ["Right_Hip_Yaw", "right_hip_yaw"], motors.rightHipYaw.angle);
        setJointValue(joints, ["Right_Knee_Pitch", "right_knee_pitch"], motors.rightKnee.angle);
        setJointValue(joints, ["Right_Ankle_Pitch", "right_ankle_pitch"], motors.rightAnklePitch.angle);
        setJointValue(joints, ["Right_Ankle_Roll", "right_ankle_roll"], motors.rightAnkleRoll.angle);
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
