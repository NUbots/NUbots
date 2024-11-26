import { observable } from "mobx";

import { Vector2 } from "../../../shared/math/vector2";

export class ArrowGeometry {
  @observable accessor direction: Vector2;
  @observable accessor headLength: number;
  @observable accessor headWidth: number;
  @observable accessor length: number;
  @observable accessor origin: Vector2;
  @observable accessor width: number;

  constructor(opts: ArrowGeometry) {
    this.direction = opts.direction;
    this.headLength = opts.headLength;
    this.headWidth = opts.headWidth;
    this.length = opts.length;
    this.origin = opts.origin;
    this.width = opts.width;
  }

  static of({
    length = 1,
    direction = Vector2.of(1, 0),
    headLength = 0.2 * length,
    headWidth = 0.04 * length,
    origin = Vector2.of(),
    width = 1,
  }: Partial<ArrowGeometry> = {}): ArrowGeometry {
    return new ArrowGeometry({
      direction,
      headLength,
      headWidth,
      length,
      origin,
      width,
    });
  }
}
