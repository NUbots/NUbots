import { Vector3 } from './vector3'

export class Matrix3 {
  constructor(
    readonly x: Vector3,
    readonly y: Vector3,
    readonly z: Vector3,
  ) {
  }

  static of() {
    return new Matrix3(
      new Vector3(1, 0, 0),
      new Vector3(0, 1, 0),
      new Vector3(0, 0, 1),
    )
  }

  static from(mat?: {
    x?: { x?: number | null, y?: number | null, z?: number | null } | null,
    y?: { x?: number | null, y?: number | null, z?: number | null } | null,
    z?: { x?: number | null, y?: number | null, z?: number | null } | null
  } | null): Matrix3 {
    if (!mat) {
      return Matrix3.of()
    }
    return new Matrix3(Vector3.from(mat.x), Vector3.from(mat.y), Vector3.from(mat.z))
  }

  get trace(): number {
    return this.x.x + this.y.y + this.z.z
  }
}
