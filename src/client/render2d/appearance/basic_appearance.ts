import { observable } from 'mobx'

export class BasicAppearance {
  @observable fillStyle: string
  @observable lineWidth: number
  @observable strokeStyle: string

  constructor(opts: BasicAppearance) {
    this.fillStyle = opts.fillStyle
    this.lineWidth = opts.lineWidth
    this.strokeStyle = opts.strokeStyle
  }

  static of({
    fillStyle = '#000000',
    lineWidth = 1,
    strokeStyle = '#000000',
  }: Partial<BasicAppearance> = {}): BasicAppearance {
    return new BasicAppearance({
      fillStyle,
      lineWidth,
      strokeStyle,
    })
  }
}
