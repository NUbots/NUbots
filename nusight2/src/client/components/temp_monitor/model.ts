import { computed } from "mobx";
import { observable } from "mobx";

import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";
import { RobotModel } from "../robot/model";

export class TempMonitorModel {
  @observable.ref selectedRobot?: TempMonitorRobotModel;

  constructor(private appModel: AppModel) {}

  static of = memoize((appModel: AppModel) => {
    return new TempMonitorModel(appModel);
  });

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter((r) => r.enabled);
  }
}

export class TempMonitorRobotModel {
  robotModel: RobotModel;

  constructor(robotModel: RobotModel) {
    this.robotModel = robotModel;
  }

  static of = memoize((robotModel: RobotModel) => {
    return new TempMonitorRobotModel(robotModel);
  });
}
