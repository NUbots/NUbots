import { action } from 'mobx'
import { observable } from 'mobx'
import { Vector3 } from '../model'

interface Motor {
  angle: number
  setAngle(angle: number): void
}

interface MotorSet {
  rightShoulderPitch: Motor
  leftShoulderPitch: Motor
  rightShoulderRoll: Motor
  leftShoulderRoll: Motor
  rightElbow: Motor
  leftElbow: Motor
  rightHipYaw: Motor
  leftHipYaw: Motor
  rightHipRoll: Motor
  leftHipRoll: Motor
  rightHipPitch: Motor
  leftHipPitch: Motor
  rightKnee: Motor
  leftKnee: Motor
  rightAnklePitch: Motor
  leftAnklePitch: Motor
  rightAnkleRoll: Motor
  leftAnkleRoll: Motor
  headPan: Motor
  headTilt: Motor
}

export class RobotModel {
  @observable public id: number
  @observable public name: number
  @observable public color: string
  @observable public heading: number
  @observable public position: Vector3
  @observable public motors: MotorSet

  public constructor({ id, name, color, heading, position, motors }) {
    this.id = id
    this.name = name
    this.color = color
    this.heading = heading
    this.position = position
    this.motors = motors
  }

  public static of({ id, name, color, heading }) {
    return new RobotModel({
      id,
      name,
      color,
      heading,
      position: Vector3.of(),
      motors: DarwinMotorSet.of(),
    })
  }

  @action
  public setColor(color) {
    this.color = color
  }

  @action
  public setHeading(heading) {
    this.heading = heading
  }
}

class DarwinMotorSet implements MotorSet {
  @observable public rightShoulderPitch: DarwinMotor
  @observable public leftShoulderPitch: DarwinMotor
  @observable public rightShoulderRoll: DarwinMotor
  @observable public leftShoulderRoll: DarwinMotor
  @observable public rightElbow: DarwinMotor
  @observable public leftElbow: DarwinMotor
  @observable public rightHipYaw: DarwinMotor
  @observable public leftHipYaw: DarwinMotor
  @observable public rightHipRoll: DarwinMotor
  @observable public leftHipRoll: DarwinMotor
  @observable public rightHipPitch: DarwinMotor
  @observable public leftHipPitch: DarwinMotor
  @observable public rightKnee: DarwinMotor
  @observable public leftKnee: DarwinMotor
  @observable public rightAnklePitch: DarwinMotor
  @observable public leftAnklePitch: DarwinMotor
  @observable public rightAnkleRoll: DarwinMotor
  @observable public leftAnkleRoll: DarwinMotor
  @observable public headPan: DarwinMotor
  @observable public headTilt: DarwinMotor

  public constructor(opts) {
    this.rightShoulderPitch = opts.rightShoulderPitch
    this.leftShoulderPitch = opts.leftShoulderPitch
    this.rightShoulderRoll = opts.rightShoulderRoll
    this.leftShoulderRoll = opts.leftShoulderRoll
    this.rightElbow = opts.rightElbow
    this.leftElbow = opts.leftElbow
    this.rightHipYaw = opts.rightHipYaw
    this.leftHipYaw = opts.leftHipYaw
    this.rightHipRoll = opts.rightHipRoll
    this.leftHipRoll = opts.leftHipRoll
    this.rightHipPitch = opts.rightHipPitch
    this.leftHipPitch = opts.leftHipPitch
    this.rightKnee = opts.rightKnee
    this.leftKnee = opts.leftKnee
    this.rightAnklePitch = opts.rightAnklePitch
    this.leftAnklePitch = opts.leftAnklePitch
    this.rightAnkleRoll = opts.rightAnkleRoll
    this.leftAnkleRoll = opts.leftAnkleRoll
    this.headPan = opts.headPan
    this.headTilt = opts.headTilt
  }

  public static of() {
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

class DarwinMotor implements Motor {
  @observable public angle

  public constructor({ angle }) {
    this.angle = angle
  }

  public static of() {
    return new DarwinMotor({ angle: 0 })
  }

  @action
  public setAngle(angle: number) {
    this.angle = angle
  }
}
