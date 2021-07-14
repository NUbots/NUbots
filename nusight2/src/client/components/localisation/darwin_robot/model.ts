import { observable } from 'mobx'
import { computed } from 'mobx'
import * as THREE from 'three'

import { memoize } from '../../../base/memoize'
import { Matrix4 } from '../../../math/matrix4'
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
  @observable Htw: Matrix4 // World to torso
  @observable Hfw: Matrix4 // World to field
  @observable motors: DarwinMotorSet

  constructor({
    model,
    name,
    color,
    Htw,
    Hfw,
    motors,
  }: {
    model: RobotModel
    name: string
    color?: string
    Htw: Matrix4
    Hfw: Matrix4
    motors: DarwinMotorSet
  }) {
    this.model = model
    this.name = name
    this.color = color
    this.Htw = Htw
    this.Hfw = Hfw
    this.motors = motors
  }

  static of = memoize(
    (model: RobotModel): LocalisationRobotModel => {
      return new LocalisationRobotModel({
        model,
        name: model.name,
        Htw: Matrix4.of(),
        Hfw: Matrix4.of(),
        motors: DarwinMotorSet.of(),
      })
    },
  )

  @computed get visible() {
    return this.model.enabled
  }

  /** Field to torso translation in field space. */
  @computed get rTFf(): Vector3 {
    return this.position.rTFf
  }

  /* Field to torso rotation in field space. */
  @computed get Rtf(): Quaternion {
    return this.position.Rtf
  }

  @computed private get position() {
    const Hwf = new THREE.Matrix4().getInverse(this.Hfw.toThree())
    const Htf = this.Htw.toThree().multiply(Hwf)
    const { rotation: Rtf } = decompose(Htf)
    const Hft = new THREE.Matrix4().getInverse(Htf)
    const { translation: rTFf } = decompose(Hft)
    return { Htf, rTFf, Rtf }
  }
}

function decompose(
  m: THREE.Matrix4,
): { translation: Vector3; rotation: Quaternion; scale: Vector3 } {
  const translation = new THREE.Vector3()
  const rotation = new THREE.Quaternion()
  const scale = new THREE.Vector3()
  m.decompose(translation, rotation, scale)
  return {
    translation: Vector3.from(translation),
    rotation: Quaternion.from(rotation),
    scale: Vector3.from(scale),
  }
}
