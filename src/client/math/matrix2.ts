import { computed } from 'mobx'
import { observable } from 'mobx'

import { Vector2 } from './vector2'

export class Matrix2 {
  @observable x: Vector2
  @observable y: Vector2

  constructor(x: Vector2, y: Vector2) {
    this.x = x
    this.y = y
  }

  static of() {
    return new Matrix2(new Vector2(1, 0), new Vector2(0, 1))
  }

  static from(mat?: {
    x?: { x?: number | null, y?: number | null } | null,
    y?: { x?: number | null, y?: number | null } | null
  } | null): Matrix2 {
    if (!mat) {
      return Matrix2.of()
    }
    return new Matrix2(Vector2.from(mat.x), Vector2.from(mat.y))
  }

  @computed get trace(): number {
    return this.x.x + this.y.y
  }

  set(x: Vector2, y: Vector2): Matrix2 {
    this.x = x
    this.y = y
    return this
  }

  clone(): Matrix2 {
    return new Matrix2(this.x.clone(), this.y.clone())
  }

  copy(m: Matrix2): Matrix2 {
    this.x.copy(m.x)
    this.y.copy(m.y)
    return this
  }
}
