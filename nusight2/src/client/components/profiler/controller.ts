import { RobotModel } from "../robot/model";

import { ProfilerRobotModel } from "./model";
import { ProfilerModel } from "./model";

export class ProfilerController {
  static of() {
    return new ProfilerController();
  }

  onSelectRobot(model: ProfilerModel, robot?: RobotModel) {
    model.selectedRobot = robot && ProfilerRobotModel.of(robot);
  }
}
