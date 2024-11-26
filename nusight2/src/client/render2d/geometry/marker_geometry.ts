import { observable } from "mobx";

import { Vector2 } from "../../../shared/math/vector2";

export class MarkerGeometry {
  @observable accessor heading: Vector2;
  @observable accessor radius: number;
  @observable accessor x: number;
  @observable accessor y: number;

  constructor(opts: MarkerGeometry) {
    this.heading = opts.heading;
    this.radius = opts.radius;
    this.x = opts.x;
    this.y = opts.y;
  }

  static of({ heading = Vector2.of(1, 0), radius = 1, x = 0, y = 0 }: Partial<MarkerGeometry> = {}): MarkerGeometry {
    return new MarkerGeometry({
      heading,
      radius,
      x,
      y,
    });
  }
}
