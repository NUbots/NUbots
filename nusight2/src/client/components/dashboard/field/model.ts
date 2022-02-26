import { computed } from 'mobx'
import { observable } from 'mobx'

import { memoize } from '../../../base/memoize'
import { DashboardRobotModel } from '../dashboard_robot/model'
import { GroundModel } from '../ground/model'

export type FieldModelOpts = {
  orientation: 'left' | 'right'
  ground: GroundModel
  robots: DashboardRobotModel[]
}

export class FieldModel {
  @observable orientation: 'left' | 'right'
  @observable ground: GroundModel
  @observable robots: DashboardRobotModel[]

  constructor(opts: FieldModelOpts) {
    this.orientation = opts.orientation
    this.ground = opts.ground
    this.robots = opts.robots
  }

  static of = memoize((robots: DashboardRobotModel[]): FieldModel => {
    return new FieldModel({
      orientation: 'right',
      ground: GroundModel.of(),
      robots,
    })
  })

  @computed
  get fieldLength() {
    return (
      this.ground.dimensions.fieldLength +
      this.ground.dimensions.goalDepth * 2 +
      this.ground.dimensions.borderStripMinWidth * 2
    )
  }

  @computed
  get fieldWidth() {
    return this.ground.dimensions.fieldWidth + this.ground.dimensions.borderStripMinWidth * 2
  }
}
