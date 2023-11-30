import { computed, observable } from "mobx";

import { memoize } from "../../base/memoize";
import { RobotModel } from "../robot/model";
import { AppModel } from "../app/model";

export interface LogMessage {
  message: string;
}

export class LogsModel {
  private appModel: AppModel;

  @observable.ref selectedRobot?: RobotModel;

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

  @computed
  get logsRobots(): LogsRobotModel[] {
    return this.robots.map(LogsRobotModel.of);
  }

  @computed
  get selectedLogsRobot(): LogsRobotModel | undefined {
    return this.selectedRobot ? LogsRobotModel.of(this.selectedRobot) : undefined;
  }
}

export class LogsRobotModel {
  robotModel: RobotModel;

  @observable.shallow messages: LogMessage[] = [];

  constructor(robotModel: RobotModel) {
    this.robotModel = robotModel;
  }

  static of = memoize((robot: RobotModel): LogsRobotModel => {
    return new LogsRobotModel(robot);
  });
}
