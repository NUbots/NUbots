import { ReactionVisualizerModel } from './model'

export class ReactionVisualizerViewModel {
  constructor(private readonly model: ReactionVisualizerModel) {}

  static of(model: ReactionVisualizerModel) {
    return new ReactionVisualizerViewModel(model)
  }

}
