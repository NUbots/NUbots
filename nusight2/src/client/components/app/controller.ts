import { action } from "mobx";

import { RobotModel } from "../robot/model";

export class AppController {
  static of() {
    return new AppController();
  }

  @action
  toggleRobotEnabled(model: RobotModel) {
    model.enabled = !model.enabled;
  }
}
