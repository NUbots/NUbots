import { useMemo } from "react";
import React from "react";
import { useThree } from "@react-three/fiber";
import { useLoader } from "@react-three/fiber";
import { autorun } from "mobx";
import { observer } from "mobx-react";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";

import { compose } from "../../../../../../shared/base/compose";
import { LocalisationRobotModel } from "../../../robot_model";

import url from "./config/nugus.glb?url";

export const NUgusView = observer(({ model }: { model: LocalisationRobotModel }) => {
  const { scene: originalScene } = useLoader(GLTFLoader, url);
  // Clone so that each instance has its own copy that it may mutate.
  const scene = useMemo(() => originalScene.clone(), [originalScene]);
  const three = useThree();
  React.useEffect(() => {
    const material = new THREE.MeshStandardMaterial({ color: "#666666", roughness: 0.5, metalness: 0.2 });
    scene.traverse((o: THREE.Object3D) => {
      if (o instanceof THREE.Mesh) {
        o.material = material;
      }
    });
    return compose([
      () => material.dispose(),
      autorun(() => {
        const robot = findMesh(scene, "Torso");
        const motors = model.motors;
        // TODO (Annable): Baking the offsets into the model geometries would be ideal.
        const PI = Math.PI;
        const PI_2 = PI / 2;
        robot.matrix = model.Hft.toThree().multiply(new THREE.Matrix4().makeRotationX(PI_2));
        robot.matrixAutoUpdate = false;
        findMesh(robot, "R_Shoulder").rotation.set(0, 0, PI_2 - motors.rightShoulderPitch.angle);
        findMesh(robot, "L_Shoulder").rotation.set(0, 0, -PI_2 - motors.leftShoulderPitch.angle);
        findMesh(robot, "R_Arm_Upper").rotation.set(0, PI_2, motors.rightShoulderRoll.angle);
        findMesh(robot, "L_Arm_Upper").rotation.set(0, PI_2, -motors.leftShoulderRoll.angle);
        findMesh(robot, "R_Arm_Lower").rotation.set(motors.rightElbow.angle, 0, 0);
        findMesh(robot, "L_Arm_Lower").rotation.set(motors.leftElbow.angle, 0, 0);
        findMesh(robot, "R_Hip_Yaw").rotation.set(0, motors.rightHipYaw.angle - PI_2, -PI_2);
        findMesh(robot, "L_Hip_Yaw").rotation.set(0, PI_2 + motors.leftHipYaw.angle, -PI_2);
        findMesh(robot, "R_Hip").rotation.set(0, 0, PI_2 - motors.rightHipRoll.angle);
        findMesh(robot, "L_Hip").rotation.set(0, 0, motors.leftHipRoll.angle - PI_2);
        findMesh(robot, "R_Upper_Leg").rotation.set(-motors.rightHipPitch.angle, 0, PI);
        findMesh(robot, "L_Upper_Leg").rotation.set(-motors.leftHipPitch.angle, 0, PI);
        findMesh(robot, "R_Lower_Leg").rotation.set(motors.rightKnee.angle, 0, 0);
        findMesh(robot, "L_Lower_Leg").rotation.set(motors.leftKnee.angle, 0, 0);
        findMesh(robot, "R_Ankle").rotation.set(motors.rightAnklePitch.angle, 0, 0);
        findMesh(robot, "L_Ankle").rotation.set(motors.leftAnklePitch.angle, 0, 0);
        findMesh(robot, "R_Foot").rotation.set(0, 0, -motors.rightAnkleRoll.angle);
        findMesh(robot, "L_Foot").rotation.set(0, 0, motors.leftAnkleRoll.angle);
        findMesh(robot, "Neck").rotation.set(0, PI_2 + motors.headPan.angle, 0);
        findMesh(robot, "Head").rotation.set(motors.headTilt.angle, 0, 0);
        three.invalidate();
      }),
    ]);
  }, [scene]);
  return <primitive object={scene} key={model.id} />;
});

const findMesh = (root: THREE.Object3D, name: string): THREE.Mesh => {
  const object = root.getObjectByName(name);
  if (!object || !(object instanceof THREE.Mesh)) {
    throw new Error();
  }
  return object;
};
