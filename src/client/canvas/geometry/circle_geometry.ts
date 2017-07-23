import { observable } from 'mobx'

export class CircleGeometry {
  @observable public radius: number
  @observable public x: number
  @observable public y: number

  constructor(opts: CircleGeometry) {
    this.radius = opts.radius
    this.x = opts.x
    this.y = opts.y
  }

  public static of({
    radius = 1, 
    x = 0, 
    y = 0,
  }: Partial<CircleGeometry> = {}): CircleGeometry {
    return new CircleGeometry({
      radius,
      x,
      y,
    })
  }
}
