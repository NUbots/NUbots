import { observable } from 'mobx'
import { Vector2 } from '../../math/vector2'

export class LineGeometry {
  @observable public origin: Vector2
  @observable public target: Vector2

  constructor(opts: LineGeometry) {
    this.origin = opts.origin
    this.target = opts.target
  }

  public static of({
    origin = Vector2.of(0, 0),
    target = Vector2.of(1, 1),
  }: Partial<LineGeometry> = {}): LineGeometry {
    return new LineGeometry({
      origin,
      target,
    })
  }
}
