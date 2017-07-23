import { observable } from 'mobx'
import { Object2d } from '../object/object2d'

export class TextGeometry implements Object2d {
  @observable public alignToView: boolean
  @observable public fontFamily: string
  @observable public maxWidth: number
  @observable public rotate: number
  @observable public scale: { x: number, y: number }
  @observable public text: string
  @observable public textAlign: 'start' | 'end' | 'left' | 'right' | 'center'
  @observable public textBaseline: 'top' | 'hanging' | 'middle' | 'alphabetic' | 'ideographic' | 'bottom'
  @observable public translate: { x: number, y: number }
  @observable public x: number
  @observable public y: number

  constructor(opts: TextGeometry) {
    this.alignToView = opts.alignToView
    this.fontFamily = opts.fontFamily
    this.maxWidth = opts.maxWidth
    this.rotate = opts.rotate
    this.scale = opts.scale
    this.text = opts.text
    this.textAlign = opts.textAlign
    this.textBaseline = opts.textBaseline
    this.translate = opts.translate
    this.x = opts.x
    this.y = opts.y
  }

  public static of({
    alignToView = true,
    fontFamily = 'sans-serif',
    maxWidth = 0.5,
    rotate = 0,
    scale = { x: 1, y: 1 },
    text = '',
    textAlign = 'start',
    textBaseline = 'alphabetic',
    translate = { x: 0, y: 0 },
    x = 0,
    y = 0,
  }: Partial<TextGeometry> = {}): TextGeometry {
    return new TextGeometry({
      alignToView,
      fontFamily,
      maxWidth,
      rotate,
      scale,
      text,
      textAlign,
      textBaseline,
      translate,
      x,
      y,
    })
  }
}
