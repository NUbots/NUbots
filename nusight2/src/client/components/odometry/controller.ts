import { RobotModel } from "../robot/model";

import { OdometryRobotModel } from "./model";
import { OdometryModel } from "./model";

export class OdometryController {
  static of() {
    return new OdometryController();
  }

  onSelectRobot(model: OdometryModel, robot?: RobotModel) {
    model.selectedRobot = robot && OdometryRobotModel.of(robot);
  }
}
