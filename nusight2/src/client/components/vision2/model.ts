import { computed } from 'mobx'
import { observable } from 'mobx'

import { memoize } from '../../base/memoize'
import { AppModel } from '../app/model'
import { RobotModel } from '../robot/model'

import { CameraModel } from './camera/model'

export class VisionModel {
  @observable.ref selectedRobot?: VisionRobotModel

  constructor(private appModel: AppModel) {}

  static of = memoize((appModel: AppModel) => {
    return new VisionModel(appModel)
  })

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter(r => r.enabled)
  }

  @computed
  get visionRobots(): VisionRobotModel[] {
    return this.robots.map(VisionRobotModel.of)
  }
}

export class VisionRobotModel {
  @observable cameras: Map<number, CameraModel> = new Map()

  constructor(readonly robotModel: RobotModel) {}

  static of = memoize((robotModel: RobotModel) => {
    return new VisionRobotModel(robotModel)
  })

  @computed
  get id() {
    return this.robotModel.id
  }

  @computed
  get name() {
    return this.robotModel.name
  }

  @computed
  get visible() {
    return this.robotModel.enabled
  }

  @computed
  get cameraList() {
    const cameras = Array.from(this.cameras.values())
    cameras.sort((a, b) => a.name.localeCompare(b.name))
    return cameras
  }
}
