import { observable } from 'mobx'

export class RobotModel {
  @observable public id: string
  @observable public connected: boolean
  @observable public enabled: boolean
  @observable public name: string
  @observable public address: string
  @observable public port: number

  public constructor(opts: RobotModel) {
    Object.assign(this, opts)
  }

  public static of(opts: RobotModel) {
    return new RobotModel(opts)
  }
}
