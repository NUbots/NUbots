import { computed } from 'mobx'
import { observable } from 'mobx'

import { Vector4 } from './vector4'

export class Matrix4 {
  @observable x: Vector4
  @observable y: Vector4
  @observable z: Vector4
  @observable t: Vector4

  constructor(x: Vector4, y: Vector4, z: Vector4, t: Vector4) {
    this.x = x
    this.y = y
    this.z = z
    this.t = t
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

  @computed
  get trace(): number {
    return this.x.x + this.y.y + this.z.z + this.t.t
  }

  set(x: Vector4, y: Vector4, z: Vector4, t: Vector4): Matrix4 {
    this.x = x
    this.y = y
    this.z = z
    this.t = t
    return this
  }

  clone(): Matrix4 {
    return new Matrix4(this.x.clone(), this.y.clone(), this.z.clone(), this.t.clone())
  }

  copy(m: Matrix4): Matrix4 {
    this.x.copy(m.x)
    this.y.copy(m.y)
    this.z.copy(m.z)
    this.t.copy(m.t)
    return this
  }
}
