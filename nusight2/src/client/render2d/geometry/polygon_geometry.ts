import { observable } from "mobx";

import { Vector2 } from "../../../shared/math/vector2";

export class PolygonGeometry {
  @observable points: Vector2[];

  constructor(opts: PolygonGeometry) {
    const points = opts.points;
    if (points.length < 3) {
      throw new Error(`Polygon must have 3 or more points, ${points.length} points given.`);
    }
    this.points = points;
  }

  static of(points: Vector2[]): PolygonGeometry {
    return new PolygonGeometry({ points });
  }
}
