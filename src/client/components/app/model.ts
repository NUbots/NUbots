import { observable } from 'mobx'

import { RobotModel } from '../robot/model'

export class AppModel {
  @observable robots: RobotModel[]

  constructor(opts: AppModel) {
    Object.assign(this, opts)
  }

  static of(options: { robots: RobotModel[] } = { robots: [] }) {
    return new AppModel(options)
  }
}
