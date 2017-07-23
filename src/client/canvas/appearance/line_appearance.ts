import { observable } from 'mobx'

export class LineAppearance {
  @observable public lineCap: 'butt' | 'round' | 'square'
  @observable public lineDashOffset: number
  @observable public lineJoin: 'bevel' | 'round' | 'miter'
  @observable public lineWidth: number
  @observable public strokeStyle: string

  constructor(opts: LineAppearance) {
    this.lineCap = opts.lineCap
    this.lineDashOffset = opts.lineDashOffset
    this.lineJoin = opts.lineJoin
    this.lineWidth = opts.lineWidth
    this.strokeStyle = opts.strokeStyle
  }

  public static of({
    lineCap = 'butt',
    lineDashOffset = 0,
    lineJoin = 'miter',
    lineWidth = 1,
    strokeStyle = '#000'
  }: Partial<LineAppearance> = {}): LineAppearance {
    return new LineAppearance({
      lineCap,
      lineDashOffset,
      lineJoin,
      lineWidth,
      strokeStyle
    })
  }
}
