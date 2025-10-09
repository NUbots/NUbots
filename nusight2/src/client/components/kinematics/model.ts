import { computed } from "mobx";
import { observable } from "mobx";

import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";
import { RobotModel } from "../robot/model";

import { KinematicsRobotModel } from "./robot_model";

export const ServoNames: { [key: number]: string } = {
  0: "R_SHOULDER_PITCH",
  1: "L_SHOULDER_PITCH",
  2: "R_SHOULDER_ROLL",
  3: "L_SHOULDER_ROLL",
  4: "R_ELBOW",
  5: "L_ELBOW",
  6: "R_HIP_YAW",
  7: "L_HIP_YAW",
  8: "R_HIP_ROLL",
  9: "L_HIP_ROLL",
  10: "R_HIP_PITCH",
  11: "L_HIP_PITCH",
  12: "R_KNEE",
  13: "L_KNEE",
  14: "R_ANKLE_PITCH",
  15: "L_ANKLE_PITCH",
  16: "R_ANKLE_ROLL",
  17: "L_ANKLE_ROLL",
  18: "HEAD_YAW",
  19: "HEAD_PITCH",
};

// Hardware error constants from RawSensors.proto
export const HardwareError = {
  HARDWARE_OK: 0,
  INPUT_VOLTAGE: 1,
  UNUSED_BIT_1: 2,
  OVERHEATING: 4,
  MOTOR_ENCODER: 8,
  ELECTRICAL_SHOCK: 16,
  OVERLOAD: 32,
} as const;

export const getErrorDescription = (errorCode: number): string[] => {
  const errors: string[] = [];

  if (errorCode === 0) return ["No errors"];

  if (errorCode & HardwareError.INPUT_VOLTAGE) {
    errors.push("Input Voltage Error");
  }
  if (errorCode & HardwareError.OVERHEATING) {
    errors.push("Overheating");
  }
  if (errorCode & HardwareError.MOTOR_ENCODER) {
    errors.push("Motor Encoder Malfunction");
  }
  if (errorCode & HardwareError.ELECTRICAL_SHOCK) {
    errors.push("Electrical Shock/Insufficient Power");
  }
  if (errorCode & HardwareError.OVERLOAD) {
    errors.push("Overload");
  }

  return errors.length > 0 ? errors : ["Unknown Error"];
};

export class KinematicsModel {
  @observable selectedRobot?: KinematicsRobotModel;

  constructor(private appModel: AppModel) {}

  static of = memoize((appModel: AppModel) => {
    return new KinematicsModel(appModel);
  });

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter((r) => r.enabled);
  }
}
