import { computed } from "mobx";
import { observable } from "mobx";

import { RobotModel } from "../robot/model";

import { DashboardRobotModel } from "./dashboard_robot/model";
import { FieldModel } from "./field/model";

export class DashboardModel {
  @observable private robotModels: RobotModel[];

  constructor(robotModels: RobotModel[]) {
    this.robotModels = robotModels;
  }

  static of(robots: RobotModel[]): DashboardModel {
    return new DashboardModel(robots);
  }

  @computed
  get field(): FieldModel {
    return FieldModel.of(this.robots);
  }

  @computed
  get robots(): DashboardRobotModel[] {
    return this.robotModels.map((robot) => DashboardRobotModel.of(robot));
  }
}
