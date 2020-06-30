import * as THREE from 'three'

export class Vector3 {
  constructor(readonly x: number, readonly y: number, readonly z: number) {}

  static of() {
    return new Vector3(0, 0, 0)
  }

  static from(vec?: { x?: number | null; y?: number | null; z?: number | null } | null): Vector3 {
    if (!vec) {
      return Vector3.of()
    }
    return new Vector3(vec.x || 0, vec.y || 0, vec.z || 0)
  }

  static fromScalar(scalar: number) {
    return new Vector3(scalar, scalar, scalar)
  }

  get length(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z)
  }

  normalize(): Vector3 {
    return this.divideScalar(this.length)
  }

  multiplyScalar(scalar: number): Vector3 {
    return new Vector3(this.x * scalar, this.y * scalar, this.z * scalar)
  }

  divideScalar(scalar: number): Vector3 {
    if (scalar !== 0) {
      const invScalar = 1 / scalar
      return new Vector3(this.x * invScalar, this.y * invScalar, this.z * invScalar)
    } else {
      return new Vector3(0, 0, 0) // TODO (Annable): This should throw
    }
  }

  add(v: Vector3): Vector3 {
    return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z)
  }

  subtract(v: Vector3): Vector3 {
    return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z)
  }

  static fromThree(vec3: THREE.Vector3 | THREE.Euler): Vector3 {
    return new Vector3(vec3.x, vec3.y, vec3.z)
  }

  toThree(): THREE.Vector3 {
    return new THREE.Vector3(this.x, this.y, this.z)
  }

  toString() {
    return `(${this.x}, ${this.y}, ${this.z})`
  }
}
