import { observable } from "mobx";

import { Vector2 } from "../../../shared/math/vector2";

export class ArcGeometry {
  @observable accessor origin: Vector2;
  @observable accessor radius: number;
  @observable accessor startAngle: number;
  @observable accessor endAngle: number;
  @observable accessor anticlockwise: boolean;

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
