import { RobotModel } from '../robot/model'
import { ReactionModel, ReactionRobotModel } from './model'

export class ReactionController {
  static of() {
    return new ReactionController()
  }

  onSelectRobot(model: ReactionModel, robot?: RobotModel) {
    model.selectedRobot = robot && ReactionRobotModel.of(robot)
  }
}
