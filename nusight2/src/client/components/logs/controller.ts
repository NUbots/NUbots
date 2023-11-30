import { action } from "mobx";

import { LogsModel } from "./model";
import { RobotModel } from "../robot/model";

export class LogsController {
  private model: LogsModel;

  constructor(model: LogsModel) {
    this.model = model;
  }

  static of(model: LogsModel): LogsController {
    return new LogsController(model);
  }

  @action.bound
  onSelectRobot(robot?: RobotModel) {
    this.model.selectedRobot = robot;
  }
}
