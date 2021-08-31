import { action } from 'mobx'
import { message } from '../../../shared/messages'
import { Network } from '../../network/network'
import { NUsightNetwork } from '../../network/nusight_network'
import { RobotModel } from '../robot/model'
import { ReactionRobotModel, ReactionStats } from './model'

import ReactionStatistics = message.support.nuclear.ReactionStatistics

export class ReactionNetwork {
  constructor(private network: Network) {
    this.network.on(ReactionStatistics, this.onReactionStats)
  }

  static of(nusightNetwork: NUsightNetwork): ReactionNetwork {
    const network = Network.of(nusightNetwork)
    return new ReactionNetwork(network)
  }

  destroy = () => {
    this.network.off()
  }

  @action.bound
  private onReactionStats(robotModel: RobotModel, packet: ReactionStatistics) {
    console.log('got reaction stats', packet)

    const reaction: ReactionStats = {
      name: packet.name,
      triggerName: packet.triggerName,
      functionName: packet.functionName,
      reactionId: packet.reactionId,
      taskId: packet.taskId,
      causeReactionId: packet.causeReactionId,
      causeTaskId: packet.causeTaskId,
      emitted: packet.emitted,
      started: packet.started,
      finished: packet.finished
    }

    const robot = ReactionRobotModel.of(robotModel)

    robot.lastReaction = reaction;
  }
}
