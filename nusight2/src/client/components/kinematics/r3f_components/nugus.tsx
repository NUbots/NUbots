import React from "react";
import { observer } from "mobx-react";
import * as THREE from "three";
import URDFLoader, { URDFRobot } from "urdf-loader";

import { KinematicsRobotModel } from "../robot_model";

const nugusUrdfPath = "/robot-models/nugus/robot.urdf";

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
    if (robotRef.current) {
      const joints = (robotRef.current?.children[0] as any)?.joints;
      if (joints) {
        const apply = (jointKey: string, angle: number) => {
          const joint = joints[jointKey];
          joint?.setJointValue(angle);

          joint?.traverse((child: any) => {
            if (child.type === "URDFVisual" && child.children.length > 0) {
              const mesh = child.children[0] as THREE.Mesh;

              if (mesh?.material && !Array.isArray(mesh.material) && "color" in mesh.material) {
                if (!mesh.userData.customizedMaterial) {
                  mesh.material = mesh.material.clone();
                  mesh.userData.customizedMaterial = true;
                }

                const material = mesh.material as THREE.Material & { color: THREE.Color };
                const isZero = Math.abs(angle) < 1e-4;
                material.color.set(isZero ? "#FFCC00" : "#A2A2A2");
              }
            }
          });
        };

        apply("head_pitch", motors.headTilt.angle);
        apply("left_ankle_pitch", motors.leftAnklePitch.angle);
        apply("left_ankle_roll", motors.leftAnkleRoll.angle);
        apply("left_elbow_pitch", motors.leftElbow.angle);
        apply("left_hip_pitch", motors.leftHipPitch.angle);
        apply("left_hip_roll", motors.leftHipRoll.angle);
        apply("left_hip_yaw", motors.leftHipYaw.angle);
        apply("left_knee_pitch", motors.leftKnee.angle);
        apply("left_shoulder_pitch", motors.leftShoulderPitch.angle);
        apply("left_shoulder_roll", motors.leftShoulderRoll.angle);
        apply("neck_yaw", motors.headPan.angle);
        apply("right_ankle_pitch", motors.rightAnklePitch.angle);
        apply("right_ankle_roll", motors.rightAnkleRoll.angle);
        apply("right_elbow_pitch", motors.rightElbow.angle);
        apply("right_hip_pitch", motors.rightHipPitch.angle);
        apply("right_hip_roll", motors.rightHipRoll.angle);
        apply("right_hip_yaw", motors.rightHipYaw.angle);
        apply("right_knee_pitch", motors.rightKnee.angle);
        apply("right_shoulder_pitch", motors.rightShoulderPitch.angle);
        apply("right_shoulder_roll", motors.rightShoulderRoll.angle);
      }
    }
  }, [rotation, motors]);


  return <object3D ref={robotRef} />;
});
