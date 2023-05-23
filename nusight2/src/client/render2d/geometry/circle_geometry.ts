import { observable } from "mobx";

export class CircleGeometry {
  @observable radius: number;
  @observable x: number;
  @observable y: number;

  constructor(opts: CircleGeometry) {
    this.radius = opts.radius;
    this.x = opts.x;
    this.y = opts.y;
  }

  static of({ radius = 1, x = 0, y = 0 }: Partial<CircleGeometry> = {}): CircleGeometry {
    return new CircleGeometry({
      radius,
      x,
      y,
    });
  }
}
