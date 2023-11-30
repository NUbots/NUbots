import { computed } from "mobx";

import { memoize } from "../../base/memoize";
import { RobotModel } from "../robot/model";
import { AppModel } from "../app/model";

export class LogsModel {
  private appModel: AppModel;

  constructor(appModel: AppModel) {
    this.appModel = appModel;
  }

  static of = memoize((appModel: AppModel) => {
    return new LogsModel(appModel);
  });

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter((robot) => robot.enabled);
  }
}
