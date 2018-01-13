import { observable } from 'mobx'

import { Vector2 } from '../../math/vector2'

export class ArcGeometry {
  @observable public origin: Vector2
  @observable public radius: number
  @observable public startAngle: number
  @observable public endAngle: number
  @observable public anticlockwise: boolean

  constructor(opts: ArcGeometry) {
    this.origin = opts.origin
    this.radius = opts.radius
    this.startAngle = opts.startAngle
    this.endAngle = opts.endAngle
    this.anticlockwise = opts.anticlockwise
  }

  public static of({
                     origin = Vector2.of(0, 0),
                     radius = 1,
                     startAngle = 0,
                     endAngle = 2 * Math.PI,
                     anticlockwise = false,
                   }: Partial<ArcGeometry> = {}): ArcGeometry {
    return new ArcGeometry({
      origin,
      radius,
      startAngle,
      endAngle,
      anticlockwise,
    })
  }
}
