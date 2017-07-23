import { createTransformer } from 'mobx'
import { computed } from 'mobx'
import { DashboardRobotViewModel } from '../dashboard_robot/view_model'
import { GroundViewModel } from '../ground/view_model'
import { FieldModel } from './model'

export class FieldViewModel {
  public constructor(private model: FieldModel) {
  }

  public static of = createTransformer((model: FieldModel): FieldViewModel => {
    return new FieldViewModel(model)
  })

  @computed
  public get scene() {
    return [
      this.ground,
      this.robots,
    ]
  }

  @computed
  private get ground() {
    return GroundViewModel.of(this.model.ground).ground
  }

  @computed
  private get robots() {
    return this.model.robots
      .filter(robot => robot.visible)
      .map(robot => DashboardRobotViewModel.of(robot).robot)
  }
}
