import { action } from 'mobx'

import { RobotModel } from '../robot/model'

export class AppController {
  public static of() {
    return new AppController()
  }

  @action
  public toggleRobotEnabled(model: RobotModel) {
    model.enabled = !model.enabled
  }
}
