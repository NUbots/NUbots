import { action } from 'mobx'
import { computed, observable } from 'mobx'
import { memoize } from '../../base/memoize'
import { Vector3 } from '../../math/vector3'
import { AppModel } from '../app/model'
import { RobotModel } from '../robot/model'
import { KinematicsRobotModel } from './darwin_robot/model'

export class TimeModel {
  @observable time: number // seconds
  @observable lastPhysicsUpdate: number // seconds

  constructor({ time, lastPhysicsUpdate }: { time: number; lastPhysicsUpdate: number }) {
    this.time = time
    this.lastPhysicsUpdate = lastPhysicsUpdate
  }

  static of() {
    return new TimeModel({
      time: 0,
      lastPhysicsUpdate: 0,
    })
  }

  @computed get timeSinceLastPhysicsUpdate() {
    return this.time - this.lastPhysicsUpdate
  }
}

export enum ViewMode {
  FreeCamera,
  FirstPerson,
  ThirdPerson,
}

class CameraModel {
  @observable position: Vector3
  @observable yaw: number
  @observable pitch: number
  @observable distance: number

  constructor({ position, yaw, pitch, distance }: CameraModel) {
    this.position = position
    this.yaw = yaw
    this.pitch = pitch
    this.distance = distance
  }

  static of() {
    return new CameraModel({
      position: new Vector3(-1, 0, 1),
      yaw: 0,
      pitch: -Math.PI / 4,
      distance: 2,
    })
  }
}

export class ControlsModel {
  @observable pitch: number
  @observable yaw: number

  constructor({ pitch, yaw }: ControlsModel) {
    this.pitch = pitch
    this.yaw = yaw
  }

  static of() {
    return new ControlsModel({
      pitch: 0,
      yaw: 0,
    })
  }
}

export class KinematicsModel {
  @observable private appModel: AppModel
  @observable selectedRobot?: KinematicsRobotModel
  @observable enabledJoints: { [key: string]: boolean }
  @observable camera: CameraModel
  @observable locked: boolean
  @observable controls: ControlsModel
  @observable viewMode: ViewMode
  @observable target?: KinematicsRobotModel
  @observable time: TimeModel

  constructor(
    appModel: AppModel,
    {
      enabledJoints,
      camera,
      locked,
      controls,
      viewMode,
      target,
      time,
    }: {
      enabledJoints: { [key: string]: boolean }
      camera: CameraModel
      locked: boolean
      controls: ControlsModel
      viewMode: ViewMode
      target?: KinematicsRobotModel
      time: TimeModel
    },
  ) {
    this.appModel = appModel
    this.enabledJoints = enabledJoints
    this.camera = camera
    this.locked = locked
    this.controls = controls
    this.viewMode = viewMode
    this.target = target
    this.time = time
    console.log(JSON.stringify(this))
  }

  static of = memoize((appModel: AppModel): KinematicsModel => {
    return new KinematicsModel(appModel, {
      enabledJoints: meshes,
      camera: CameraModel.of(),
      locked: false,
      controls: ControlsModel.of(),
      viewMode: ViewMode.ThirdPerson,
      time: TimeModel.of(),
    })
  })

  @computed get robots(): RobotModel[] {
    return this.appModel.robots.filter(r => r.enabled)
  }

  @action
  setMeshes = (joint: string) => {
    console.log('VIEW', JSON.stringify(this))
    if (Object.keys(this.enabledJoints).includes(joint))
      this.enabledJoints[joint] = !this.enabledJoints[joint]
  }
}

const meshes = {
  R_Shoulder: true,
  L_Shoulder: true,
  R_Arm_Upper: true,
  L_Arm_Upper: true,
  R_Arm_Lower: true,
  L_Arm_Lower: true,
  R_Hip_Yaw: true,
  L_Hip_Yaw: true,
  R_Hip: true,
  L_Hip: true,
  R_Upper_Leg: true,
  L_Upper_Leg: true,
  R_Lower_Leg: true,
  L_Lower_Leg: true,
  R_Ankle: true,
  L_Ankle: true,
  R_Foot: true,
  L_Foot: true,
  Neck: true,
  Head: true,
}
