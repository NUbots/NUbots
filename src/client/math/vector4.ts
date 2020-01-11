export class Vector4 {
  constructor(
    readonly x: number,
    readonly y: number,
    readonly z: number,
    readonly t: number,
  ) {
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

  get length(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.t * this.t)
  }

  normalize(): Vector4 {
    return this.divideScalar(this.length)
  }

  multiplyScalar(scalar: number): Vector4 {
    return new Vector4(this.x * scalar, this.y * scalar, this.z * scalar, this.t * scalar)
  }

  divideScalar(scalar: number): Vector4 {
    if (scalar !== 0) {
      const invScalar = 1 / scalar
      return new Vector4(this.x * invScalar, this.y * invScalar, this.z * invScalar, this.t * invScalar)
    } else {
      return new Vector4(0, 0, 0, 0) // TODO (Annable): This should throw
    }
  }

  add(v: Vector4): Vector4 {
    return new Vector4(this.x + v.x, this.y + v.y, this.z + v.z, this.t + v.t)
  }

  subtract(v: Vector4): Vector4 {
    return new Vector4(this.x - v.x, this.y - v.y, this.z - v.z, this.t - v.t)
  }
}
