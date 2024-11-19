import { RobotModel } from "../robot/model";

import { KinematicsModel } from "./model";
import { KinematicsRobotModel } from "./model";

export class KinematicsController {
  static of() {
    return new KinematicsController();
  }

  onSelectRobot(model: KinematicsModel, robot?: RobotModel) {
    model.selectedRobot = robot && KinematicsRobotModel.of(robot);
  }
}
