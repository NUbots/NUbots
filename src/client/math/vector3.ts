import { computed } from 'mobx'
import { observable } from 'mobx'

export class Vector3 {
  @observable public x: number
  @observable public y: number
  @observable public z: number

  public constructor(x: number, y: number, z: number) {
    this.x = x
    this.y = y
    this.z = z
  }

  public static of() {
    return new Vector3(0, 0, 0)
  }

  public static from(vec?: { x?: number, y?: number, z?: number } | null): Vector3 {
    if (!vec) {
      return Vector3.of()
    }
    return new Vector3(vec.x || 0, vec.y || 0, vec.z || 0)
  }

  @computed get length(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z)
  }

  public set(x: number, y: number, z: number): Vector3 {
    this.x = x
    this.y = y
    this.z = z
    return this
  }

  public clone(): Vector3 {
    return new Vector3(this.x, this.y, this.z)
  }

  public copy(v: Vector3): Vector3 {
    this.x = v.x
    this.y = v.y
    this.z = v.z
    return this
  }

  public normalize(): Vector3 {
    return this.divideScalar(this.length)
  }

  public multiplyScalar(scalar: number): Vector3 {
    this.x *= scalar
    this.y *= scalar
    this.z *= scalar
    return this
  }

  public divideScalar(scalar: number): Vector3 {
    if (scalar !== 0) {
      const invScalar = 1 / scalar
      this.x *= invScalar
      this.y *= invScalar
      this.z *= invScalar
    } else {
      this.x = 0
      this.y = 0
      this.z = 0
    }
    return this
  }

  public add(v: Vector3): Vector3 {
    this.x += v.x
    this.y += v.y
    this.z += v.z
    return this
  }

  public subtract(v: Vector3): Vector3 {
    this.x -= v.x
    this.y -= v.y
    this.z -= v.z
    return this
  }
}

