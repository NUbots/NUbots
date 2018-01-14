import { action, observable } from 'mobx'
import { computed } from 'mobx'

import { memoize } from '../../base/memoize'
import { Vector3 } from '../../math/vector3'
import { AppModel } from '../app/model'

import { LocalisationRobotModel } from './darwin_robot/model'
import { FieldModel } from './field/model'
import { SkyboxModel } from './skybox/model'

export class TimeModel {
  @observable time: number // seconds
  @observable lastRenderTime: number // seconds

  constructor(opts: Partial<TimeModel>) {
    Object.assign(this, opts)
  }

  static of() {
    return new TimeModel({
      time: 0,
      lastRenderTime: 0,
    })
  }

  @computed get timeSinceLastRender() {
    return this.time - this.lastRenderTime
  }
}

export enum ViewMode {
  FreeCamera,
  FirstPerson,
  ThirdPerson,
}

export class LocalisationModel {
  @observable private appModel: AppModel
  @observable aspect: number
  @observable field: FieldModel
  @observable skybox: SkyboxModel
  @observable camera: CameraModel
  @observable locked: boolean
  @observable controls: ControlsModel
  @observable viewMode: ViewMode
  @observable target?: LocalisationRobotModel
  @observable time: TimeModel

  constructor(appModel: AppModel, opts: Partial<LocalisationModel>) {
    this.appModel = appModel
    Object.assign(this, opts)
  }

  static of = memoize((appModel: AppModel): LocalisationModel => {
    return new LocalisationModel(appModel, {
      aspect: 300 / 150,
      field: FieldModel.of(),
      skybox: SkyboxModel.of(),
      camera: CameraModel.of(),
      locked: false,
      controls: ControlsModel.of(),
      viewMode: ViewMode.FreeCamera,
      time: TimeModel.of(),
    })
  })

  @computed get robots(): LocalisationRobotModel[] {
    return this.appModel.robots.map(robot => LocalisationRobotModel.of(robot))
  }
}

class CameraModel {
  @observable position: Vector3
  @observable yaw: number
  @observable pitch: number
  @observable distance: number

  constructor(opts: CameraModel) {
    Object.assign(this, opts)
  }

  static of() {
    return new CameraModel({
      position: new Vector3(-1, 0, 1),
      yaw: 0,
      pitch: -Math.PI / 4,
      distance: 0.5,
    })
  }
}

export class ControlsModel {
  @observable forward: boolean
  @observable left: boolean
  @observable right: boolean
  @observable back: boolean
  @observable up: boolean
  @observable down: boolean
  @observable pitch: number
  @observable yaw: number

  constructor(opts: ControlsModel) {
    Object.assign(this, opts)
  }

  static of() {
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
}

export class Quaternion {
  @observable x: number
  @observable y: number
  @observable z: number
  @observable w: number

  constructor(x: number, y: number, z: number, w: number) {
    this.x = x
    this.y = y
    this.z = z
    this.w = w
  }

  static of() {
    return new Quaternion(0, 0, 0, 1)
  }

  set(x: number, y: number, z: number, w: number): Quaternion {
    this.x = x
    this.y = y
    this.z = z
    this.w = w
    return this
  }
}
