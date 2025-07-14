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

export class KinematicsModel {
  @observable selectedRobot?: KinematicsRobotModel;

  constructor(private appModel: AppModel) { }

  static of = memoize((appModel: AppModel) => {
    return new KinematicsModel(appModel);
  });

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter((r) => r.enabled);
  }
}
