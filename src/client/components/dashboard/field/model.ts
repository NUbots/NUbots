import { computed } from 'mobx'
import { observable } from 'mobx'
import { memoize } from '../../../base/memoize'
import { Transform } from '../../../math/transform'
import { DashboardRobotModel } from '../dashboard_robot/model'
import { GroundModel } from '../ground/model'

export type FieldModelOpts = {
  camera: Transform
  ground: GroundModel
  robots: DashboardRobotModel[]
}

export class FieldModel {
  @observable public camera: Transform
  @observable public ground: GroundModel
  @observable public robots: DashboardRobotModel[]

  constructor(opts: FieldModelOpts) {
    this.camera = opts.camera
    this.ground = opts.ground
    this.robots = opts.robots
  }

  public static of = memoize((robots: DashboardRobotModel[]): FieldModel => {
    return new FieldModel({
      camera: Transform.of(),
      ground: GroundModel.of(),
      robots,
    })
  })

  @computed
  public get fieldLength() {
    return this.ground.dimensions.fieldLength
      + (this.ground.dimensions.goalDepth * 2)
      + (this.ground.dimensions.borderStripMinWidth * 2)
  }

  @computed
  public get fieldWidth() {
    return this.ground.dimensions.fieldWidth + (this.ground.dimensions.borderStripMinWidth * 2)
  }
}
