import { action } from "mobx";

import { FilterLevels, LogsModel, LogsRobotModel } from "./model";
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

  @action.bound
  setFilter(model: LogsRobotModel, filter: keyof FilterLevels, value: boolean) {
    model.filters.levels[filter] = value;
  }

  @action.bound
  setSearch(model: LogsRobotModel, search: string) {
    model.filters.search = search;
  }

  @action.bound
  setShowTimestamps(model: LogsRobotModel, showTimestamps: boolean) {
    model.showTimestamps = showTimestamps;
  }
}
