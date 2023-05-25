import { observable } from "mobx";

import { Vector2 } from "../../../shared/math/vector2";

export class PathGeometry {
  @observable points: Vector2[];

  constructor(opts: PathGeometry) {
    this.points = opts.points;
  }

  static of(points: Vector2[]): PathGeometry {
    return new PathGeometry({ points });
  }
}
