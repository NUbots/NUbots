import { observable } from 'mobx'
import { ReactionStats } from './view'

export class ReactionVisualizerModel {
  @observable.ref reaction : ReactionStats

  constructor({
    reaction
  }: {
    reaction: ReactionStats
  }) {
    this.reaction = reaction
  }

  static of({reaction}: { reaction: ReactionStats}) {
    return new ReactionVisualizerModel({
     reaction : {
       name : reaction.name,
       trigger_name: reaction.trigger_name,
       function_name: reaction.function_name,
       reaction_id: reaction.reaction_id,
       task_id: reaction.task_id,
       cause_reaction_id: reaction.cause_reaction_id,
       cause_task_id: reaction.cause_task_id,
       emitted: reaction.emitted,
       started: reaction.started,
       finished: reaction.finished
     }
    })
  }
}
