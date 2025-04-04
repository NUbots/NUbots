import { computed } from "mobx";
import { observable } from "mobx";

import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";
import { RobotModel } from "../robot/model";

import { KinematicsRobotModel } from "./robot_model";

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
