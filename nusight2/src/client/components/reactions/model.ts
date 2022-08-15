import { computed, observable } from 'mobx'
import { memoize } from '../../base/memoize'
import { AppModel } from '../app/model'
import { RobotModel } from '../robot/model'

export interface ReactionStats {
  name: string
  triggerName: string
  functionName: string
  reactionId: number | Long
  taskId: number | Long
  causeReactionId: number | Long
  causeTaskId: number | Long
  emitted: number | Long
  started: number | Long
  finished: number | Long
}

export class ReactionModel {
  @observable.ref selectedRobot?: ReactionRobotModel

  constructor(private appModel: AppModel) {} //, reaction: ReactionStats) { this.reaction = reaction }

  static of = memoize((appModel: AppModel) => {
    return new ReactionModel(appModel)
  })

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter(r => r.enabled)
  }
}

export class ReactionRobotModel {
  @observable.ref lastReaction?: ReactionStats

  constructor(readonly robotModel: RobotModel) {}

  static of = memoize((robotModel: RobotModel) => {
    return new ReactionRobotModel(robotModel)
  })
}
