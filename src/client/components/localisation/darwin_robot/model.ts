import { observable } from 'mobx'
import { computed } from 'mobx'

import { memoize } from '../../../base/memoize'
import { Vector3 } from '../../../math/vector3'
import { RobotModel } from '../../robot/model'
import { Quaternion } from '../model'

export class LocalisationRobotModel {
  @observable private model: RobotModel
  @observable name: string
  @observable color?: string
  @observable rWTt: Vector3 // Torso to world translation in torso space.
  @observable Rwt: Quaternion // Torso to world rotation.
  @observable motors: DarwinMotorSet

  constructor(model: RobotModel, opts: Partial<LocalisationRobotModel>) {
    this.model = model
    Object.assign(this, opts)
  }

  static of = memoize((model: RobotModel): LocalisationRobotModel => {
    return new LocalisationRobotModel(model, {
      name: model.name,
      rWTt: Vector3.of(),
      Rwt: Quaternion.of(),
      motors: DarwinMotorSet.of(),
    })
  })

  @computed get visible() {
    return this.model.enabled
  }
}

export class DarwinMotorSet {
  @observable rightShoulderPitch: DarwinMotor
  @observable leftShoulderPitch: DarwinMotor
  @observable rightShoulderRoll: DarwinMotor
  @observable leftShoulderRoll: DarwinMotor
  @observable rightElbow: DarwinMotor
  @observable leftElbow: DarwinMotor
  @observable rightHipYaw: DarwinMotor
  @observable leftHipYaw: DarwinMotor
  @observable rightHipRoll: DarwinMotor
  @observable leftHipRoll: DarwinMotor
  @observable rightHipPitch: DarwinMotor
  @observable leftHipPitch: DarwinMotor
  @observable rightKnee: DarwinMotor
  @observable leftKnee: DarwinMotor
  @observable rightAnklePitch: DarwinMotor
  @observable leftAnklePitch: DarwinMotor
  @observable rightAnkleRoll: DarwinMotor
  @observable leftAnkleRoll: DarwinMotor
  @observable headPan: DarwinMotor
  @observable headTilt: DarwinMotor

  constructor(opts: DarwinMotorSet) {
    Object.assign(this, opts)
  }

  static of() {
    return new DarwinMotorSet({
      rightShoulderPitch: DarwinMotor.of(),
      leftShoulderPitch: DarwinMotor.of(),
      rightShoulderRoll: DarwinMotor.of(),
      leftShoulderRoll: DarwinMotor.of(),
      rightElbow: DarwinMotor.of(),
      leftElbow: DarwinMotor.of(),
      rightHipYaw: DarwinMotor.of(),
      leftHipYaw: DarwinMotor.of(),
      rightHipRoll: DarwinMotor.of(),
      leftHipRoll: DarwinMotor.of(),
      rightHipPitch: DarwinMotor.of(),
      leftHipPitch: DarwinMotor.of(),
      rightKnee: DarwinMotor.of(),
      leftKnee: DarwinMotor.of(),
      rightAnklePitch: DarwinMotor.of(),
      leftAnklePitch: DarwinMotor.of(),
      rightAnkleRoll: DarwinMotor.of(),
      leftAnkleRoll: DarwinMotor.of(),
      headPan: DarwinMotor.of(),
      headTilt: DarwinMotor.of(),
    })
  }
}

class DarwinMotor {
  @observable angle: number

  constructor(opts: DarwinMotor) {
    Object.assign(this, opts)
  }

  static of() {
    return new DarwinMotor({ angle: 0 })
  }
}
