import { computed } from "mobx";
import { observable } from "mobx";

import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";

import { KinematicsRobotModel } from "./robot_model";

export class KinematicsModel {
  @observable.ref selectedRobot?: KinematicsRobotModel;

  constructor(private appModel: AppModel) {}

  static of = memoize((appModel: AppModel) => {
    return new KinematicsModel(appModel);
  });

  @computed
  get robots(): KinematicsRobotModel[] {
    return this.appModel.robots.filter((robot) => robot.enabled).map((robot) => KinematicsRobotModel.of(robot));
  }
}
