import { computed, observable } from 'mobx'
import { memoize } from '../../base/memoize'
import { AppModel } from '../app/model'
import { RobotModel } from '../robot/model'
import { OdometryVisualizerModel } from './odometry_visualizer/model'

export class OdometryModel {
  @observable.ref selectedRobot?: OdometryRobotModel
  //@observable.ref reaction: ReactionStats

  constructor(private appModel: AppModel) { }//, reaction: ReactionStats) { this.reaction = reaction }

  static of = memoize((appModel: AppModel) => {
    return new OdometryModel(appModel)
  })

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter(r => r.enabled)
  }
}

export class OdometryRobotModel {
  constructor(readonly robotModel: RobotModel, readonly visualizerModel: OdometryVisualizerModel) { }

  static of = (robotModel: RobotModel, visualizerModel: OdometryVisualizerModel) => {
    //const reaction = visualizerModel.reaction
    return new OdometryRobotModel(
      robotModel,
      OdometryVisualizerModel.of(visualizerModel)
    )
  }
}
