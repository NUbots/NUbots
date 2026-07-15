import { RobotModel } from "../robot/model";

import { DirectorModel } from "./model";

export class DirectorController {
  static of(): DirectorController {
    return new DirectorController();
  }

  onSelectRobot(model: DirectorModel, robot?: RobotModel) {
    model.selectedRobot = robot;
  }
}
