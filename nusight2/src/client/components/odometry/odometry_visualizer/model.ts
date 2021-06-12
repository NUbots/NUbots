import { observable } from 'mobx'
import { Matrix4 } from '../../../math/matrix4'
import { Vector3 } from '../../../math/vector3'

export class OdometryVisualizerModel {
  @observable.ref Hwt: Matrix4
  @observable.ref accelerometer: Vector3
  @observable.ref camera: OdometryCamera
  // TODO: make optional?
  @observable.ref leftFoot: OdometryFootModel
  @observable.ref rightFoot: OdometryFootModel

  constructor({
    Hwt,
    accelerometer,
    camera,
    leftFoot,
    rightFoot,
  }: {
    Hwt: Matrix4
    accelerometer: Vector3
    camera: OdometryCamera
    leftFoot: OdometryFootModel
    rightFoot: OdometryFootModel
  }) {
    this.Hwt = Hwt
    this.accelerometer = accelerometer
    this.camera = camera
    this.leftFoot = leftFoot
    this.rightFoot = rightFoot
  }

  static of({
    Hwt = Matrix4.of(),
    accelerometer,
    leftFoot,
    rightFoot,
  }: {
    Hwt?: Matrix4
    accelerometer: Vector3
    leftFoot: OdometryFootModel
    rightFoot: OdometryFootModel
  }) {
    return new OdometryVisualizerModel({
      Hwt,
      accelerometer,
      camera: OdometryCamera.of({ distance: 2 }),
      leftFoot,
      rightFoot,
    })
  }
}

export class OdometryCamera {
  @observable.ref distance: number
  @observable.ref pitch: number
  @observable.ref yaw: number

  constructor({ distance, pitch, yaw }: { distance: number; pitch: number; yaw: number }) {
    this.distance = distance
    this.pitch = pitch
    this.yaw = yaw
  }

  static of({ distance }: { distance: number }) {
    return new OdometryCamera({ distance, pitch: 0, yaw: 0 })
  }
}

interface OdometryFootModel {
  readonly down: boolean
  readonly Hwf: Matrix4
  readonly Htf: Matrix4
}
