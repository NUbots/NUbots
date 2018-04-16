import { observable } from 'mobx'

export class TextGeometry {
  @observable alignToView: boolean
  @observable fontFamily: string
  @observable maxWidth: number
  @observable text: string
  @observable textAlign: 'start' | 'middle' | 'end' | 'left' | 'right' | 'center'
  @observable textBaseline: 'top' | 'hanging' | 'middle' | 'alphabetic' | 'ideographic' | 'bottom'
  @observable x: number
  @observable y: number

  constructor(opts: TextGeometry) {
    this.alignToView = opts.alignToView
    this.fontFamily = opts.fontFamily
    this.maxWidth = opts.maxWidth
    this.text = opts.text
    this.textAlign = opts.textAlign
    this.textBaseline = opts.textBaseline
    this.x = opts.x
    this.y = opts.y
  }

  static of({
    alignToView = true,
    fontFamily = 'sans-serif',
    maxWidth = 0.5,
    text = '',
    textAlign = 'start',
    textBaseline = 'alphabetic',
    x = 0,
    y = 0,
  }: Partial<TextGeometry> = {}): TextGeometry {
    return new TextGeometry({
      alignToView,
      fontFamily,
      maxWidth,
      text,
      textAlign,
      textBaseline,
      x,
      y,
    })
  }
}
