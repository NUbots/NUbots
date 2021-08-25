import { action } from 'mobx'
import { message } from '../../../shared/messages'
import { Network } from '../../network/network'
import { NUsightNetwork } from '../../network/nusight_network'
import { RobotModel } from '../robot/model'
import { OdometryRobotModel } from './model'
import { OdometryVisualizerModel } from './odometry_visualizer/model'
import { ReactionStats } from './odometry_visualizer/view'

export class OdometryNetwork {
  constructor(private network: Network) {
    this.network.on(message.support.nuclear.ReactionStatistics, this.onReactionStats)
  }

  static of(nusightNetwork: NUsightNetwork): OdometryNetwork {
    const network = Network.of(nusightNetwork)
    return new OdometryNetwork(network)
  }

  destroy = () => {
    this.network.off()
  }

  @action.bound
  private onReactionStats(robotModel: RobotModel, packet: message.support.nuclear.ReactionStatistics) {

    const reaction: ReactionStats = {
      name: packet.name,
      trigger_name: packet.triggerName,
      function_name: packet.functionName,
      reaction_id: packet.reactionId,
      task_id: packet.taskId,
      cause_reaction_id: packet.causeReactionId,
      cause_task_id: packet.causeTaskId,
      emitted: packet.emitted,
      started: packet.started,
      finished: packet.finished
    }
    const visualizerModel = new OdometryVisualizerModel({ reaction })
    const robot = OdometryRobotModel.of(robotModel, visualizerModel)
  }
}
