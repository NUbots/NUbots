import { computed } from 'mobx'
import { observable } from 'mobx'

import { memoize } from '../../base/memoize'
import { AppModel } from '../app/model'
import { RobotModel } from '../robot/model'

import { CameraModel } from './camera/model'

export class VisionModel {
  constructor(private appModel: AppModel) {
  }

  static of = memoize((appModel: AppModel) => {
    return new VisionModel(appModel)
  })

  @computed
  get robots(): VisionRobotModel[] {
    return this.appModel.robots.map(VisionRobotModel.of)
  }
}

export class VisionRobotModel {

  @observable cameras: Map<number, CameraModel> = new Map()

  constructor(private robotModel: RobotModel) {
  }

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
}
