import { computed } from 'mobx'
import { createTransformer } from 'mobx'
import { Object3D } from 'three'
import { Quaternion } from 'three'
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
    robot.position.x = this.model.rWTt.x
    robot.position.y = this.model.rWTt.y + HIP_TO_FOOT
    robot.position.z = this.model.rWTt.z
    const rotation = new Quaternion(this.model.Rwt.x, this.model.Rwt.y, this.model.Rwt.z, this.model.Rwt.w)
    robot.setRotationFromQuaternion(rotation)
    robot.add(BodyViewModel.of(this.model).body)
    return robot
  }
}
