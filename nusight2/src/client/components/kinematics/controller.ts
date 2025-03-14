import { RobotModel } from "../robot/model";

import { KinematicsModel } from "./model";
import { KinematicsRobotModel } from "./robot_model";

export class KinematicsController {
  static of(): KinematicsController {
    return new KinematicsController();
  }

  onSelectRobot(model: KinematicsModel, robot?: RobotModel) {
    model.selectedRobot = robot && KinematicsRobotModel.of(robot);
  }
}
