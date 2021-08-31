import { autorun } from 'mobx'
import { SeededRandom } from '../../shared/base/random/seeded_random'
import { message } from '../../shared/messages'
import { NUClearNetClient } from '../../shared/nuclearnet/nuclearnet_client'
import { Message, Simulator } from '../simulator'
import { periodic } from './periodic'

import ReactionStatistics = message.support.nuclear.ReactionStatistics

export class ReactionStatsSimulator extends Simulator {
  constructor(
    nuclearnetClient: NUClearNetClient,
    private readonly robotIndex: number,
    private readonly numRobots: number,
    private readonly random: SeededRandom,
  ) {
    super(nuclearnetClient)
  }

  static of({
    nuclearnetClient,
    robotIndex,
    numRobots,
  }: {
    nuclearnetClient: NUClearNetClient
    robotIndex: number
    numRobots: number
  }) {
    return new ReactionStatsSimulator(
      nuclearnetClient,
      robotIndex,
      numRobots,
      SeededRandom.of('overview_simulator'),
    )
  }

  start() {
    return autorun(() => this.send(this.reactionStats))
  }

  get reactionStats(): Message {
    const time = periodic(2)

    const messageType = 'message.support.nuclear.ReactionStatistics'

    const t = time / 10 - this.robotIndex

    const buffer = ReactionStatistics.encode({
      name: 'test-test',
      reactionId: 0,
      causeReactionId: 0,
      causeTaskId: 0,
      emitted: this.random.integer(0, 100),
      finished: 0,
      functionName: '',
      started: 0,
      taskId: t,
      triggerName: '',
    }).finish()

    const message = { messageType, buffer }

    return message
  }
}
