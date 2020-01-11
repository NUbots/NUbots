import { Vector4 } from './vector4'

export class Matrix4 {
  constructor(
    readonly x: Vector4,
    readonly y: Vector4,
    readonly z: Vector4,
    readonly t: Vector4,
  ) {
  }

  static of() {
    return new Matrix4(
      new Vector4(1, 0, 0, 0),
      new Vector4(0, 1, 0, 0),
      new Vector4(0, 0, 1, 0),
      new Vector4(0, 0, 0, 1),
    )
  }

  static from(mat?: {
    x?: { x?: number | null, y?: number | null, z?: number | null, t?: number | null } | null,
    y?: { x?: number | null, y?: number | null, z?: number | null, t?: number | null } | null,
    z?: { x?: number | null, y?: number | null, z?: number | null, t?: number | null } | null,
    t?: { x?: number | null, y?: number | null, z?: number | null, t?: number | null } | null
  } | null): Matrix4 {
    if (!mat) {
      return Matrix4.of()
    }
    return new Matrix4(Vector4.from(mat.x), Vector4.from(mat.y), Vector4.from(mat.z), Vector4.from(mat.t))
  }

  get trace(): number {
    return this.x.x + this.y.y + this.z.z + this.t.t
  }
}
