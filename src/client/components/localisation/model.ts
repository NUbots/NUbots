import { action, observable } from 'mobx'
import { computed } from 'mobx'
import { memoize } from '../../base/memoize'
import { Vector3 } from '../../math/vector3'
import { AppModel } from '../app/model'
import { LocalisationRobotModel } from './darwin_robot/model'
import { FieldModel } from './field/model'
import { SkyboxModel } from './skybox/model'

export class TimeModel {
  @observable public time: number // seconds
  @observable public lastRenderTime: number // seconds

  constructor(opts: Partial<TimeModel>) {
    Object.assign(this, opts)
  }

  public static of() {
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
  @observable public aspect: number
  @observable public field: FieldModel
  @observable public skybox: SkyboxModel
  @observable public camera: CameraModel
  @observable public locked: boolean
  @observable public controls: ControlsModel
  @observable public viewMode: ViewMode
  @observable public target?: LocalisationRobotModel
  @observable public time: TimeModel

  constructor(appModel: AppModel, opts: Partial<LocalisationModel>) {
    this.appModel = appModel
    Object.assign(this, opts)
  }

  public static of = memoize((appModel: AppModel): LocalisationModel => {
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
  @observable public position: Vector3
  @observable public yaw: number
  @observable public pitch: number
  @observable public distance: number

  constructor(opts: CameraModel) {
    Object.assign(this, opts)
  }

  public static of() {
    return new CameraModel({
      position: new Vector3(-1, 0, 1),
      yaw: 0,
      pitch: -Math.PI / 4,
      distance: 0.5,
    })
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

  constructor(opts: ControlsModel) {
    Object.assign(this, opts)
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
}

export class Quaternion {
  @observable public x: number
  @observable public y: number
  @observable public z: number
  @observable public w: number

  public constructor(x: number, y: number, z: number, w: number) {
    this.x = x
    this.y = y
    this.z = z
    this.w = w
  }

  public static of() {
    return new Quaternion(0, 0, 0, 1)
  }

  public set(x: number, y: number, z: number, w: number): Quaternion {
    this.x = x
    this.y = y
    this.z = z
    this.w = w
    return this
  }
}
