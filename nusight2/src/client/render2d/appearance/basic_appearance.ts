import { observable } from 'mobx'

type Fill = { color: string; alpha: number }
type Stroke = { color: string; alpha: number; width: number }

export type BasicAppearanceOpts = {
  fill?: Partial<Fill>
  stroke?: Partial<Stroke>
}

export class BasicAppearance {
  @observable fill?: Fill
  @observable stroke?: Stroke

  constructor({ fill, stroke }: { fill?: Fill; stroke?: Stroke }) {
    this.fill = fill
    this.stroke = stroke
  }

  static of({ fill, stroke }: BasicAppearanceOpts = {}): BasicAppearance {
    return new BasicAppearance({
      fill: fill && { color: '#000000', alpha: 1, ...fill },
      stroke: stroke && { color: '#000000', alpha: 1, width: 1, ...stroke },
    })
  }
}
