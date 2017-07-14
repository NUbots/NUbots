import { observable } from 'mobx'

export class RobotModel {
  @observable public enabled: boolean
  @observable public name: string
  @observable public host: string

  public constructor(opts: RobotModel) {
    Object.assign(this, opts)
  }

  public static of(opts: RobotModel) {
    return new RobotModel(opts)
  }
}
