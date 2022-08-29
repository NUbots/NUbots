import { action, computed } from 'mobx'
import { createTransformer } from 'mobx-utils'

import { Transform } from '../../../math/transform'
import { Group } from '../../../render2d/object/group'
import { SwitchesMenuOption } from '../../switches_menu/view'
import { DashboardRobotViewModel } from '../dashboard_robot/view_model'
import { GroundViewModel } from '../ground/view_model'

import { FieldModel } from './model'

export class FieldViewModel {
  constructor(private model: FieldModel) {}

  static of = createTransformer((model: FieldModel): FieldViewModel => {
    return new FieldViewModel(model)
  })

  @computed
  get scene(): Group {
    return Group.of({
      transform: Transform.of({
        // TODO (Annable): move camera to the view model and put this transform there.
        rotate: this.model.orientation === 'left' ? Math.PI : 0,
      }),
      children: [this.ground, this.robots],
    })
  }

  @computed
  get camera(): Transform {
    return Transform.of({
      scale: { x: 1.0 / this.model.fieldLength, y: 1.0 / this.model.fieldLength },
    })
  }

  @computed
  get aspectRatio(): number {
    return this.model.fieldLength / this.model.fieldWidth
  }

  @computed
  private get ground() {
    return GroundViewModel.of(this.model.ground).ground
  }

  @computed
  private get robots() {
    return Group.of({
      children: this.model.robots
        .filter(robot => robot.enabled && robot.connected)
        .map(robot => DashboardRobotViewModel.of(robot).robot),
    })
  }

  @computed
  get drawOptions(): SwitchesMenuOption[] {
    return this.model.robots.flatMap(robot => {
      return [
        {
          label: `${robot.name} - Ball std 1`,
          enabled: robot.drawOptions.drawStd1,
          toggle: action(() => (robot.drawOptions.drawStd1 = !robot.drawOptions.drawStd1)),
        },
        {
          label: `${robot.name} - Ball std 2`,
          enabled: robot.drawOptions.drawStd2,
          toggle: action(() => (robot.drawOptions.drawStd2 = !robot.drawOptions.drawStd2)),
        },
        {
          label: `${robot.name} - Ball std 3`,
          enabled: robot.drawOptions.drawStd3,
          toggle: action(() => (robot.drawOptions.drawStd3 = !robot.drawOptions.drawStd3)),
        },
      ]
    })
  }
}
