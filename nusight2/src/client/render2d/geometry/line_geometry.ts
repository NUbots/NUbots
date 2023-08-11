import { observable } from "mobx";

import { Vector2 } from "../../../shared/math/vector2";

export class LineGeometry {
  @observable origin: Vector2;
  @observable target: Vector2;

  constructor(opts: LineGeometry) {
    this.origin = opts.origin;
    this.target = opts.target;
  }

  static of({ origin = Vector2.of(0, 0), target = Vector2.of(1, 1) }: Partial<LineGeometry> = {}): LineGeometry {
    return new LineGeometry({
      origin,
      target,
    });
  }
}
