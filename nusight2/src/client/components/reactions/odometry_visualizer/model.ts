import { observable } from 'mobx'
import { ReactionStats } from '../model'

export class ReactionVisualizerModel {
  @observable.ref reaction: ReactionStats

  constructor({ reaction }: { reaction: ReactionStats }) {
    this.reaction = reaction
  }

  static of({ reaction }: { reaction: ReactionStats }) {
    return new ReactionVisualizerModel({
      reaction: {
        name: reaction.name,
        triggerName: reaction.triggerName,
        functionName: reaction.functionName,
        reactionId: reaction.reactionId,
        taskId: reaction.taskId,
        causeReactionId: reaction.causeReactionId,
        causeTaskId: reaction.causeReactionId,
        emitted: reaction.emitted,
        started: reaction.started,
        finished: reaction.finished,
      },
    })
  }
}
