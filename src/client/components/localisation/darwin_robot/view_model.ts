import { computed } from 'mobx'
import { createTransformer } from 'mobx'
import { Object3D } from 'three'
import { BodyViewModel } from './body/view_model'
import { LocalisationRobotModel } from './model'

export const HIP_TO_FOOT = 0.2465

export class RobotViewModel {
  public constructor(private model: LocalisationRobotModel) {
  }

  public static of = createTransformer((model: LocalisationRobotModel): RobotViewModel => {
    return new RobotViewModel(model)
  })

  @computed
  public get robot(): Object3D {
    const robot = new Object3D()
    robot.rotation.y = this.model.heading
    robot.position.x = this.model.position.x
    robot.position.y = this.model.position.y + HIP_TO_FOOT
    robot.position.z = this.model.position.z
    robot.add(BodyViewModel.of(this.model).body)
    return robot
  }
}
