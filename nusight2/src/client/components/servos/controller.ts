import { RobotModel } from "../robot/model";

import { ServosModel } from "./model";
import { ServosRobotModel } from "./robot_model";

export class ServosController {
  static of(): ServosController {
    return new ServosController();
  }

  onSelectRobot(model: ServosModel, robot?: RobotModel) {
    model.selectedRobot = robot && ServosRobotModel.of(robot);
  }
}
