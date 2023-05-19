import { observable } from "mobx";
import { computed } from "mobx";

import { Quaternion } from "../../../shared/math/quaternion";
import { Vector3 } from "../../../shared/math/vector3";
import { memoize } from "../../base/memoize";
import { RobotModel } from "../robot/model";

class ServoMotor {
  @observable angle: number;

  constructor({ angle }: ServoMotor) {
    this.angle = angle;
  }

  static of() {
    return new ServoMotor({ angle: 0 });
  }
}

export class ServoMotorSet {
  @observable rightShoulderPitch: ServoMotor;
  @observable leftShoulderPitch: ServoMotor;
  @observable rightShoulderRoll: ServoMotor;
  @observable leftShoulderRoll: ServoMotor;
  @observable rightElbow: ServoMotor;
  @observable leftElbow: ServoMotor;
  @observable rightHipYaw: ServoMotor;
  @observable leftHipYaw: ServoMotor;
  @observable rightHipRoll: ServoMotor;
  @observable leftHipRoll: ServoMotor;
  @observable rightHipPitch: ServoMotor;
  @observable leftHipPitch: ServoMotor;
  @observable rightKnee: ServoMotor;
  @observable leftKnee: ServoMotor;
  @observable rightAnklePitch: ServoMotor;
  @observable leftAnklePitch: ServoMotor;
  @observable rightAnkleRoll: ServoMotor;
  @observable leftAnkleRoll: ServoMotor;
  @observable headPan: ServoMotor;
  @observable headTilt: ServoMotor;

  constructor({
    rightShoulderPitch,
    leftShoulderPitch,
    rightShoulderRoll,
    leftShoulderRoll,
    rightElbow,
    leftElbow,
    rightHipYaw,
    leftHipYaw,
    rightHipRoll,
    leftHipRoll,
    rightHipPitch,
    leftHipPitch,
    rightKnee,
    leftKnee,
    rightAnklePitch,
    leftAnklePitch,
    rightAnkleRoll,
    leftAnkleRoll,
    headPan,
    headTilt,
  }: ServoMotorSet) {
    this.rightShoulderPitch = rightShoulderPitch;
    this.leftShoulderPitch = leftShoulderPitch;
    this.rightShoulderRoll = rightShoulderRoll;
    this.leftShoulderRoll = leftShoulderRoll;
    this.rightElbow = rightElbow;
    this.leftElbow = leftElbow;
    this.rightHipYaw = rightHipYaw;
    this.leftHipYaw = leftHipYaw;
    this.rightHipRoll = rightHipRoll;
    this.leftHipRoll = leftHipRoll;
    this.rightHipPitch = rightHipPitch;
    this.leftHipPitch = leftHipPitch;
    this.rightKnee = rightKnee;
    this.leftKnee = leftKnee;
    this.rightAnklePitch = rightAnklePitch;
    this.leftAnklePitch = leftAnklePitch;
    this.rightAnkleRoll = rightAnkleRoll;
    this.leftAnkleRoll = leftAnkleRoll;
    this.headPan = headPan;
    this.headTilt = headTilt;
  }

  static of() {
    return new ServoMotorSet({
      rightShoulderPitch: ServoMotor.of(),
      leftShoulderPitch: ServoMotor.of(),
      rightShoulderRoll: ServoMotor.of(),
      leftShoulderRoll: ServoMotor.of(),
      rightElbow: ServoMotor.of(),
      leftElbow: ServoMotor.of(),
      rightHipYaw: ServoMotor.of(),
      leftHipYaw: ServoMotor.of(),
      rightHipRoll: ServoMotor.of(),
      leftHipRoll: ServoMotor.of(),
      rightHipPitch: ServoMotor.of(),
      leftHipPitch: ServoMotor.of(),
      rightKnee: ServoMotor.of(),
      leftKnee: ServoMotor.of(),
      rightAnklePitch: ServoMotor.of(),
      leftAnklePitch: ServoMotor.of(),
      rightAnkleRoll: ServoMotor.of(),
      leftAnkleRoll: ServoMotor.of(),
      headPan: ServoMotor.of(),
      headTilt: ServoMotor.of(),
    });
  }
}

export class LocalisationRobotModel {
  @observable private model: RobotModel;
  @observable name: string;
  @observable color?: string;
  @observable rWTt: Vector3; // Torso to world translation in torso space.
  @observable Rwt: Quaternion; // Torso to world rotation.
  @observable motors: ServoMotorSet;

  constructor({
    model,
    name,
    color,
    rWTt,
    Rwt,
    motors,
  }: {
    model: RobotModel;
    name: string;
    color?: string;
    rWTt: Vector3;
    Rwt: Quaternion;
    motors: ServoMotorSet;
  }) {
    this.model = model;
    this.name = name;
    this.color = color;
    this.rWTt = rWTt;
    this.Rwt = Rwt;
    this.motors = motors;
  }

  static of = memoize((model: RobotModel): LocalisationRobotModel => {
    return new LocalisationRobotModel({
      model,
      name: model.name,
      rWTt: Vector3.of(),
      Rwt: Quaternion.of(),
      motors: ServoMotorSet.of(),
    });
  });

  @computed get visible() {
    return this.model.enabled;
  }
}
