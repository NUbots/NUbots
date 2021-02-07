import * as THREE from 'three'

import { Transform } from './transform'

export class Vector2 {
  constructor(readonly x: number, readonly y: number) {}

  static of(x?: number, y?: number): Vector2 {
    return new Vector2(x || 0, y || 0)
  }

  static from(vec?: { x?: number | null; y?: number | null } | null): Vector2 {
    if (!vec) {
      return Vector2.of()
    }
    return new Vector2(vec.x || 0, vec.y || 0)
  }

  static fromPolar(radius: number, angle: number): Vector2 {
    return Vector2.of(radius * Math.cos(angle), radius * Math.sin(angle))
  }

  get length(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y)
  }

  transform(transform: Transform): Vector2 {
    const { rotate, scale, translate } = transform

    const theta = rotate * (transform.anticlockwise ? 1 : -1)

    const cosTheta = Math.cos(theta)
    const sinTheta = Math.sin(theta)

    const rotationMatrix = [cosTheta, -sinTheta, sinTheta, cosTheta]

    return new Vector2(
      scale.x * (this.x * rotationMatrix[0] + this.y * rotationMatrix[1]) + translate.x,
      scale.y * (this.x * rotationMatrix[2] + this.y * rotationMatrix[3]) + translate.y,
    )
  }

  normalize(): Vector2 {
    return this.divideScalar(this.length)
  }

  multiplyScalar(scalarX: number, scalarY: number = scalarX): Vector2 {
    return new Vector2(this.x * scalarX, this.y * scalarY)
  }

  divideScalar(scalarX: number, scalarY: number = scalarX): Vector2 {
    if (scalarX !== 0) {
      if (scalarY !== 0) {
        return new Vector2(this.x / scalarX, this.y / scalarY)
      } else {
        return new Vector2(this.x, 0) // TODO (Annable): This should throw
      }
    } else {
      return new Vector2(0, this.y) // TODO (Annable): This should throw
    }
  }

  add(v: Vector2): Vector2 {
    return new Vector2(this.x + v.x, this.y + v.y)
  }

  subtract(v: Vector2): Vector2 {
    return new Vector2(this.x - v.x, this.y - v.y)
  }

  static fromThree(vec2: THREE.Vector2): Vector2 {
    return new Vector2(vec2.x, vec2.y)
  }

  toThree(): THREE.Vector2 {
    return new THREE.Vector2(this.x, this.y)
  }

  toString() {
    return `(${this.x}, ${this.y})`
  }
}
