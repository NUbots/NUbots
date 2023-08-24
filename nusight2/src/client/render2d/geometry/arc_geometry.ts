import { observable } from "mobx";

import { Vector2 } from "../../../shared/math/vector2";

export class ArcGeometry {
  @observable origin: Vector2;
  @observable radius: number;
  @observable startAngle: number;
  @observable endAngle: number;
  @observable anticlockwise: boolean;

  constructor(opts: ArcGeometry) {
    this.origin = opts.origin;
    this.radius = opts.radius;
    this.startAngle = opts.startAngle;
    this.endAngle = opts.endAngle;
    this.anticlockwise = opts.anticlockwise;
  }

  static of({
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
    });
  }
}
