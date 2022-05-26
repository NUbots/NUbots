import { observable } from 'mobx'
import { computed } from 'mobx'

import { memoize } from '../../../base/memoize'
import { Quaternion } from '../../../math/quaternion'
import { Vector3 } from '../../../math/vector3'
import { RobotModel } from '../../robot/model'

class DarwinMotor {
  @observable angle: number

  constructor({ angle }: DarwinMotor) {
    this.angle = angle
  }

  static of() {
    return new DarwinMotor({ angle: 0 })
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

  constructor({
    rightShoulderPitch,
    leftShoulderPitch,
    rightShoulderRoll,
    leftShoulderRoll,
    rightElbow,
    leftElbow,
    rightHipYaw,
    leftHipYaw,
    rightHipRoll,
    leftHipRoll,
    rightHipPitch,
    leftHipPitch,
    rightKnee,
    leftKnee,
    rightAnklePitch,
    leftAnklePitch,
    rightAnkleRoll,
    leftAnkleRoll,
    headPan,
    headTilt,
  }: DarwinMotorSet) {
    this.rightShoulderPitch = rightShoulderPitch
    this.leftShoulderPitch = leftShoulderPitch
    this.rightShoulderRoll = rightShoulderRoll
    this.leftShoulderRoll = leftShoulderRoll
    this.rightElbow = rightElbow
    this.leftElbow = leftElbow
    this.rightHipYaw = rightHipYaw
    this.leftHipYaw = leftHipYaw
    this.rightHipRoll = rightHipRoll
    this.leftHipRoll = leftHipRoll
    this.rightHipPitch = rightHipPitch
    this.leftHipPitch = leftHipPitch
    this.rightKnee = rightKnee
    this.leftKnee = leftKnee
    this.rightAnklePitch = rightAnklePitch
    this.leftAnklePitch = leftAnklePitch
    this.rightAnkleRoll = rightAnkleRoll
    this.leftAnkleRoll = leftAnkleRoll
    this.headPan = headPan
    this.headTilt = headTilt
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

export class LocalisationRobotModel {
  @observable private model: RobotModel
  @observable name: string
  @observable color?: string
  @observable rWTt: Vector3 // Torso to world translation in torso space.
  @observable Rwt: Quaternion // Torso to world rotation.
  @observable motors: DarwinMotorSet

  constructor({
    model,
    name,
    color,
    rWTt,
    Rwt,
    motors,
  }: {
    model: RobotModel
    name: string
    color?: string
    rWTt: Vector3
    Rwt: Quaternion
    motors: DarwinMotorSet
  }) {
    this.model = model
    this.name = name
    this.color = color
    this.rWTt = rWTt
    this.Rwt = Rwt
    this.motors = motors
  }

  static of = memoize((model: RobotModel): LocalisationRobotModel => {
    return new LocalisationRobotModel({
      model,
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
