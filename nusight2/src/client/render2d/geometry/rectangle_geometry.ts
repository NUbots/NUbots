import { observable } from "mobx";

export class RectangleGeometry {
  @observable x: number;
  @observable y: number;
  @observable width: number;
  @observable height: number;
  @observable borderRadius: number | "full";

  constructor(opts: { x: number; y: number; width: number; height: number; borderRadius: number | "full" }) {
    this.x = opts.x;
    this.y = opts.y;
    this.width = opts.width;
    this.height = opts.height;
    this.borderRadius = opts.borderRadius;
  }

  static of(x: number, y: number, width: number, height: number, borderRadius?: number | "full"): RectangleGeometry {
    return new RectangleGeometry({ x, y, width, height, borderRadius: borderRadius ?? 0 });
  }
}
