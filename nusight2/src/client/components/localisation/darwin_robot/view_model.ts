import { computed } from 'mobx'
import { createTransformer } from 'mobx-utils'
import { Object3D } from 'three'
import { Quaternion } from 'three'

import { BodyViewModel } from './body/view_model'
import { LocalisationRobotModel } from './model'

export const HIP_TO_FOOT = 0.2465

export class RobotViewModel {
  constructor(private model: LocalisationRobotModel) {}

  static of = createTransformer(
    (model: LocalisationRobotModel): RobotViewModel => {
      return new RobotViewModel(model)
    },
  )

  @computed
  get robot(): Object3D {
    const robot = new Object3D()
    robot.position.x = this.model.rTFf.x
    robot.position.y = this.model.rTFf.y
    robot.position.z = this.model.rTFf.z
    const rotation = new Quaternion(
      this.model.Rtf.x,
      this.model.Rtf.y,
      this.model.Rtf.z,
      this.model.Rtf.w,
    )
    robot.setRotationFromQuaternion(rotation)
    robot.add(BodyViewModel.of(this.model).body)
    return robot
  }
}
