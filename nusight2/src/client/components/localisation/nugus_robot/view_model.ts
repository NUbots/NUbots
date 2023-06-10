import { computed } from "mobx";
import { lazyObservable } from "mobx-utils";
import { createTransformer } from "mobx-utils";
import { Matrix4 } from "three";
import { Mesh } from "three";
import { Object3D } from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";

import { LocalisationRobotModel } from "../robot_model";

import url from "./config/nugus.glb?url";

export class NUgusViewModel {
  constructor(private readonly model: LocalisationRobotModel) {}

  static of = createTransformer((model: LocalisationRobotModel) => {
    return new NUgusViewModel(model);
  });

  @computed({ equals: () => false })
  get robot(): Object3D {
    const robot = this.robotObject;
    if (!robot) {
      // Model has not loaded yet, show a dummy object until it loads.
      return this.dummyObject;
    }
    const motors = this.model.motors;
    // TODO (Annable): Baking the offsets into the model geometries would be ideal.
    const PI = Math.PI;
    const PI_2 = PI / 2;
    robot.matrix = this.model.Hft.toThree().multiply(new Matrix4().makeRotationX(PI_2));
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
    findMesh(robot, "Neck").rotation.set(0, PI + motors.headPan.angle, 0);
    findMesh(robot, "Head").rotation.set(0, 0, motors.headTilt.angle);
    return robot;
  }

  @computed
  private get robotObject() {
    const current = NUgusViewModel.robotObjectBase.current();
    return current
      ? current.clone() // Clone so that each view model instance has its own copy that it may mutate.
      : undefined;
  }

  private static robotObjectBase = lazyObservable<Object3D | undefined>((sink) => {
    new GLTFLoader().load(url, (gltf) => {
      // TODO (Annable): Baking this rotation into the model geometry would be ideal.
      findMesh(gltf.scene, "Head").geometry.applyMatrix4(new Matrix4().makeRotationY(-Math.PI / 2));
      sink(findMesh(gltf.scene, "Torso"));
    });
  });

  @computed
  get dummyObject() {
    return new Object3D();
  }
}

const findMesh = (root: Object3D, name: string): Mesh => {
  const object = root.getObjectByName(name);
  if (!object || !(object instanceof Mesh)) {
    throw new Error();
  }
  return object;
};
