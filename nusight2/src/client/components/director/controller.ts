import { action } from "mobx";

import { RobotModel } from "../robot/model";
import { DirectorModel } from "./model";

export class DirectorController {
  private model: DirectorModel;

  constructor(model: DirectorModel) {
    this.model = model;
  }

  static of(model: DirectorModel): DirectorController {
    return new DirectorController(model);
  }

  @action.bound
  onSelectRobot(robot?: RobotModel) {
    this.model.selectedRobot = robot;
  }
}
