import { observable } from "mobx";
import { computed } from "mobx";

import { Matrix4 } from "../../../shared/math/matrix4";
import { Quaternion } from "../../../shared/math/quaternion";
import { memoize } from "../../base/memoize";
import { RobotModel } from "../robot/model";
import { ServoNames } from "./model";

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

export class KinematicsRobotModel {
  @observable private model: RobotModel;
  @observable name: string;
  @observable Htw: Matrix4; // World to torso
  @observable Hfw: Matrix4; // World to field
  @observable Rwt: Quaternion; // Torso to world rotation.
  @observable motors: ServoMotorSet;
  @observable servoTemperatures: Map<number, number> = new Map();

  constructor({
    model,
    name,
    Htw,
    Hfw,
    Rwt,
    motors,
  }: {
    model: RobotModel;
    name: string;
    Htw: Matrix4;
    Hfw: Matrix4;
    Rwt: Quaternion;
    motors: ServoMotorSet;
  }) {
    this.model = model;
    this.name = name;
    this.Htw = Htw;
    this.Hfw = Hfw;
    this.Rwt = Rwt;
    this.motors = motors;
  }

  static of = memoize((model: RobotModel): KinematicsRobotModel => {
    return new KinematicsRobotModel({
      model,
      name: model.name,
      Htw: Matrix4.of(),
      Hfw: Matrix4.of(),
      Rwt: Quaternion.of(),
      motors: ServoMotorSet.of(),
    });
  });

  @computed get robotModel() {
    return this.model;
  }

  @computed get id() {
    return this.model.id;
  }

  @computed get visible() {
    return this.model.enabled;
  }

  /** Torso to field transformation */
  @computed
  get Hft(): Matrix4 {
    return this.Hfw.multiply(this.Htw.invert());
  }

  @computed
  get averageTemperature(): number {
    const temps = Array.from(this.servoTemperatures.values());
    return temps.length > 0 ? temps.reduce((a, b) => a + b) / temps.length : 0;
  }

  @computed
  get highestTemperature(): number {
    return Math.max(...Array.from(this.servoTemperatures.values()), 0);
  }

  @computed
  get highestTemperatureServo(): { id: number; name: string; temperature: number } | null {
    if (this.servoTemperatures.size === 0) return null;
    const entries = Array.from(this.servoTemperatures.entries());
    const [id, temp] = entries.reduce((max, current) => (current[1] > max[1] ? current : max));
    return {
      id,
      name: ServoNames[id] || `Servo ${id}`,
      temperature: temp,
    };
  }
}
