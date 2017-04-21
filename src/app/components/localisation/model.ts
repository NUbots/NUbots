import { action, observable } from 'mobx'
import { computed } from 'mobx'
import { RobotModel } from './darwin_robot/model'
import { FieldModel } from './field/model'

export class TimeModel {
  // seconds
  @observable public time: number

  // seconds
  @observable public lastRenderTime: number

  constructor({ time, lastTime }) {
    this.time = time
    this.lastRenderTime = lastTime
  }

  public static of() {
    return new TimeModel({
      time: 0,
      lastTime: 0,
    })
  }

  @action
  public setTime(time: number) {
    this.time = time
  }

  @action
  public setLastRenderTime(lastRenderTime: number) {
    this.lastRenderTime = lastRenderTime
  }

  @computed get timeSinceLastRender() {
    return this.time - this.lastRenderTime
  }
}

export enum ViewMode {
  NO_CLIP,
  FIRST_PERSON,
  THIRD_PERSON,
}

export class LocalisationModel {
  @observable public aspect: number
  @observable public robots: RobotModel[]
  @observable public field: FieldModel
  @observable public camera: CameraModel
  @observable public locked: boolean
  @observable public controls: ControlsModel
  @observable public viewMode: ViewMode
  @observable public target?: RobotModel
  @observable public time: TimeModel

  constructor({ aspect, field, camera, controls, viewMode, target, time }) {
    this.aspect = aspect
    this.field = field
    this.camera = camera
    this.robots = []
    this.locked = false
    this.controls = controls
    this.viewMode = viewMode
    this.target = target
    this.time = time
  }

  public static of(): LocalisationModel {
    const aspect = 300 / 150
    const field = FieldModel.of()
    const camera = CameraModel.of()
    const controls = ControlsModel.of()
    const viewMode = ViewMode.NO_CLIP
    const target = null
    const time = TimeModel.of()
    return new LocalisationModel({ aspect, field, camera, controls, viewMode, target, time })
  }

  @action
  public setAspect(aspect): LocalisationModel {
    this.aspect = aspect
    return this
  }

  @action
  public addRobot(robot: RobotModel): LocalisationModel {
    this.robots.push(robot)
    return this
  }

  @action
  public setLocked(locked: boolean): LocalisationModel {
    this.locked = locked
    return this
  }

  @action
  public setViewMode(viewMode: ViewMode): LocalisationModel {
    this.viewMode = viewMode
    return this
  }

  @action
  public setTarget(target: RobotModel): LocalisationModel {
    this.target = target
    return this
  }
}

class CameraModel {
  @observable public position: Vector3
  @observable public yaw: number
  @observable public pitch: number
  @observable public distance: number

  constructor({ position, yaw, pitch, distance }) {
    this.position = position
    this.yaw = yaw
    this.pitch = pitch
    this.distance = distance
  }

  public static of() {
    return new CameraModel({
      position: Vector3.of(),
      yaw: 0,
      pitch: 0,
      distance: 0.5,
    })
  }

  @action
  public setYaw(yaw): CameraModel {
    this.yaw = yaw
    return this
  }

  @action
  public setPitch(pitch): CameraModel {
    this.pitch = pitch
    return this
  }

  @action
  public setDistance(distance): CameraModel {
    this.distance = distance
    return this
  }
}

export class ControlsModel {
  @observable public forward: boolean
  @observable public left: boolean
  @observable public right: boolean
  @observable public back: boolean
  @observable public up: boolean
  @observable public down: boolean
  @observable public pitch: number
  @observable public yaw: number

  constructor({ forward, left, right, back, up, down, pitch, yaw }) {
    this.forward = forward
    this.left = left
    this.right = right
    this.back = back
    this.up = up
    this.down = down
    this.pitch = pitch
    this.yaw = yaw
  }

  public static of() {
    return new ControlsModel({
      forward: false,
      left: false,
      right: false,
      back: false,
      up: false,
      down: false,
      pitch: 0,
      yaw: 0,
    })
  }

  @action
  public setForward(forward: boolean): ControlsModel {
    this.forward = forward
    return this
  }

  @action
  public setLeft(left: boolean): ControlsModel {
    this.left = left
    return this
  }

  @action
  public setRight(right: boolean): ControlsModel {
    this.right = right
    return this
  }

  @action
  public setBack(back: boolean): ControlsModel {
    this.back = back
    return this
  }

  @action
  public setUp(up: boolean): ControlsModel {
    this.up = up
    return this
  }

  @action
  public setDown(down: boolean): ControlsModel {
    this.down = down
    return this
  }

  @action
  public setPitch(pitch: number): ControlsModel {
    this.pitch = pitch
    return this
  }

  @action
  public setYaw(yaw: number): ControlsModel {
    this.yaw = yaw
    return this
  }
}

export class Vector3 {
  @observable public x: number
  @observable public y: number
  @observable public z: number

  public constructor(x: number, y: number, z: number) {
    this.x = x
    this.y = y
    this.z = z
  }

  public static of() {
    return new Vector3(0, 0, 0)
  }

  @computed get length(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z)
  }

  @action
  public set(x, y, z): Vector3 {
    this.x = x
    this.y = y
    this.z = z
    return this
  }

  @action
  public clone(): Vector3 {
    return new Vector3(this.x, this.y, this.z)
  }

  @action
  public copy(v: Vector3): Vector3 {
    this.x = v.x
    this.y = v.y
    this.z = v.z
    return this
  }

  @action
  public normalize(): Vector3 {
    return this.divideScalar(this.length)
  }

  @action
  public multiplyScalar(scalar: number): Vector3 {
    this.x *= scalar
    this.y *= scalar
    this.z *= scalar
    return this
  }

  @action
  public divideScalar(scalar: number): Vector3 {
    if (scalar !== 0) {
      const invScalar = 1 / scalar
      this.x *= invScalar
      this.y *= invScalar
      this.z *= invScalar
    } else {
      this.x = 0
      this.y = 0
      this.z = 0
    }
    return this
  }

  @action
  public setX(x): Vector3 {
    this.x = x
    return this
  }

  @action
  public setY(y): Vector3 {
    this.y = y
    return this
  }

  @action
  public setZ(z): Vector3 {
    this.z = z
    return this
  }

  @action
  public add(movement: Vector3): Vector3 {
    this.x += movement.x
    this.y += movement.y
    this.z += movement.z
    return this
  }
}
