import { observable } from 'mobx'

export class LineAppearance {
  @observable lineCap: 'butt' | 'round' | 'square'
  @observable lineDashOffset: number
  @observable lineJoin: 'bevel' | 'round' | 'miter'
  @observable lineWidth: number
  @observable strokeStyle: string

  constructor(opts: LineAppearance) {
    this.lineCap = opts.lineCap
    this.lineDashOffset = opts.lineDashOffset
    this.lineJoin = opts.lineJoin
    this.lineWidth = opts.lineWidth
    this.strokeStyle = opts.strokeStyle
  }

  static of({
    lineCap = 'butt',
    lineDashOffset = 0,
    lineJoin = 'miter',
    lineWidth = 1,
    strokeStyle = '#000',
  }: Partial<LineAppearance> = {}): LineAppearance {
    return new LineAppearance({
      lineCap,
      lineDashOffset,
      lineJoin,
      lineWidth,
      strokeStyle,
    })
  }
}
