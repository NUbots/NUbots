import { observable } from 'mobx'

export class RobotModel {
  @observable id: string
  @observable connected: boolean
  @observable enabled: boolean
  @observable name: string
  @observable address: string
  @observable port: number

  constructor(opts: RobotModel) {
    Object.assign(this, opts)
  }

  static of(opts: RobotModel) {
    return new RobotModel(opts)
  }
}
