import { Vector2 } from "./vector2";

export class Matrix2 {
  constructor(
    readonly x: Vector2,
    readonly y: Vector2,
  ) {}

  static of() {
    return new Matrix2(new Vector2(1, 0), new Vector2(0, 1));
  }

  static from(
    mat?: {
      x?: { x?: number | null; y?: number | null } | null;
      y?: { x?: number | null; y?: number | null } | null;
    } | null,
  ): Matrix2 {
    if (!mat) {
      return Matrix2.of();
    }
    return new Matrix2(Vector2.from(mat.x), Vector2.from(mat.y));
  }

  get trace(): number {
    return this.x.x + this.y.y;
  }

  toString() {
    return [`${format(this.x.x)} ${format(this.y.x)}`, `${format(this.x.y)} ${format(this.y.y)}`].join("\n");
  }
}

const format = (x: number) => x.toFixed(2).padStart(7);
