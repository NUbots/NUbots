import { action } from "mobx";

import { RobotModel } from "../robot/model";

import { TempMonitorRobotModel } from "./model";
import { TempMonitorModel } from "./model";

export class TempMonitorController {
  static of() {
    return new TempMonitorController();
  }

  onSelectRobot(model: TempMonitorModel, robot?: RobotModel) {
    model.selectedRobot = robot && TempMonitorRobotModel.of(robot);
  }
}
