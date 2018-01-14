import { computed } from 'mobx'
import { observable } from 'mobx'

import { Vector3 } from './vector3'

export class Matrix3 {
  @observable x: Vector3
  @observable y: Vector3
  @observable z: Vector3

  constructor(x: Vector3, y: Vector3, z: Vector3) {
    this.x = x
    this.y = y
    this.z = z
  }

  static of() {
    return new Matrix3(new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1))
  }

  static from(mat?: {
    x?: { x?: number, y?: number, z?: number },
    y?: { x?: number, y?: number, z?: number },
    z?: { x?: number, y?: number, z?: number }
  } | null): Matrix3 {
    if (!mat) {
      return Matrix3.of()
    }
    return new Matrix3(Vector3.from(mat.x), Vector3.from(mat.y), Vector3.from(mat.z))
  }

  @computed get trace(): number {
    return this.x.x + this.y.y + this.z.z
  }

  set(x: Vector3, y: Vector3, z: Vector3): Matrix3 {
    this.x = x
    this.y = y
    this.z = z
    return this
  }

  clone(): Matrix3 {
    return new Matrix3(this.x.clone(), this.y.clone(), this.z.clone())
  }

  copy(m: Matrix3): Matrix3 {
    this.x.copy(m.x)
    this.y.copy(m.y)
    this.z.copy(m.z)
    return this
  }
}
