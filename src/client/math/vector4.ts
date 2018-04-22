import { computed } from 'mobx'
import { observable } from 'mobx'

export class Vector4 {
  @observable x: number
  @observable y: number
  @observable z: number
  @observable t: number

  constructor(x: number, y: number, z: number, t: number) {
    this.x = x
    this.y = y
    this.z = z
    this.t = t
  }

  static of() {
    return new Vector4(0, 0, 0, 0)
  }

  static from(vec?: { x?: number | null, y?: number | null, z?: number | null, t?: number | null } | null): Vector4 {
    if (!vec) {
      return Vector4.of()
    }
    return new Vector4(vec.x || 0, vec.y || 0, vec.z || 0, vec.t || 0)
  }

  @computed get length(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.t * this.t)
  }

  set(x: number, y: number, z: number, t: number): Vector4 {
    this.x = x
    this.y = y
    this.z = z
    this.t = t
    return this
  }

  clone(): Vector4 {
    return new Vector4(this.x, this.y, this.z, this.t)
  }

  copy(v: Vector4): Vector4 {
    this.x = v.x
    this.y = v.y
    this.z = v.z
    this.t = v.t
    return this
  }

  normalize(): Vector4 {
    return this.divideScalar(this.length)
  }

  multiplyScalar(scalar: number): Vector4 {
    this.x *= scalar
    this.y *= scalar
    this.z *= scalar
    this.t *= scalar
    return this
  }

  divideScalar(scalar: number): Vector4 {
    if (scalar !== 0) {
      const invScalar = 1 / scalar
      this.x *= invScalar
      this.y *= invScalar
      this.z *= invScalar
      this.t *= invScalar
    } else {
      this.x = 0
      this.y = 0
      this.z = 0
      this.t = 0
    }
    return this
  }

  add(v: Vector4): Vector4 {
    this.x += v.x
    this.y += v.y
    this.z += v.z
    this.t += v.t
    return this
  }

  subtract(v: Vector4): Vector4 {
    this.x -= v.x
    this.y -= v.y
    this.z -= v.z
    this.t -= v.t
    return this
  }
}
