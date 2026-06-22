import React from "react";
import * as THREE from "three";
import URDFLoader, { URDFRobot } from "urdf-loader";

import { LocalisationRobotModel } from "../robot_model";

const k1UrdfPath = "/robot-models/k1/robot.urdf";

/** Renders XYZ axes (red/green/blue) at the camera pose in field space. */
const CameraAxes = ({ model }: { model: LocalisationRobotModel }) => {
  const axesRef = React.useRef<THREE.AxesHelper>(null);

  React.useEffect(() => {
    if (!axesRef.current) return;
    const Hfc = model.Hfc;
    const { translation, rotation } = Hfc.decompose();
    axesRef.current.position.set(translation.x, translation.y, translation.z);
    axesRef.current.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
  }, [model.Hfc]);

  return <primitive ref={axesRef} object={new THREE.AxesHelper(0.15)} />;
};

const setJointValue = (joints: any, names: string[], value: number) => {
  for (const name of names) {
    joints?.[name]?.setJointValue(value);
  }
};

const updateK1Joints = (joints: any, model: LocalisationRobotModel) => {
  const motors = model.motors;

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
};

export const K1 = ({ model }: { model: LocalisationRobotModel }) => {
  const robotRef = React.useRef<URDFRobot | null>(null);

  React.useEffect(() => {
    const loader = new URDFLoader();
    loader.load(k1UrdfPath, (robot: URDFRobot) => {
      if (robotRef.current) {
        robotRef.current.add(robot);
      }
    });
  }, []);

  const position = model.Hft.decompose().translation;
  const rotation = model.Hft.decompose().rotation;

  React.useEffect(() => {
    if (robotRef.current) {
      robotRef.current.position.copy(new THREE.Vector3(position.x, position.y, position.z));
      robotRef.current.quaternion.copy(new THREE.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w));

      const joints = (robotRef.current?.children[0] as any)?.joints;
      if (joints) {
        updateK1Joints(joints, model);
      }
    }
  }, [position, rotation, model]);

  const material = new THREE.MeshStandardMaterial({
    color: "#888888",
    roughness: 0.5,
    metalness: 0.2,
  });

  if (robotRef.current) {
    robotRef.current.traverse((child) => {
      if (child.type === "URDFVisual" && child.children.length > 0) {
        const mesh = child.children[0] as THREE.Mesh;
        mesh.material = material;
        mesh.castShadow = true;
        mesh.receiveShadow = true;
      }
    });
  }

  return (
    <>
      <object3D ref={robotRef} />
      <CameraAxes model={model} />
    </>
  );
};
