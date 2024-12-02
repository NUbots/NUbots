import * as THREE from "three";

import { Matrix4 } from "./matrix4";
import { Vector } from "./vector";

export class Vector3 extends Vector {
  constructor(
    readonly x: number,
    readonly y: number,
    readonly z: number,
  ) {
    super();
  }

  static of(x?: number, y?: number, z?: number) {
    return new Vector3(x ?? 0, y ?? 0, z ?? 0);
  }

  static from(vec?: { x?: number | null; y?: number | null; z?: number | null } | null): Vector3 {
    if (!vec) {
      return new Vector3(0, 0, 0);
    }
    return new Vector3(vec.x ?? 0, vec.y ?? 0, vec.z ?? 0);
  }

  static fromScalar(scalar: number) {
    return new Vector3(scalar, scalar, scalar);
  }

  copy(): Vector3 {
    return new Vector3(this.x, this.y, this.z);
  }

  multiplyScalar(scalar: number): Vector3 {
    return new Vector3(this.x * scalar, this.y * scalar, this.z * scalar);
  }

  divideScalar(scalar: number): Vector3 {
    if (scalar !== 0) {
      const invScalar = 1 / scalar;
      return new Vector3(this.x * invScalar, this.y * invScalar, this.z * invScalar);
    } else {
      return new Vector3(0, 0, 0); // TODO (Annable): This should throw
    }
  }

  add(v: Vector3): Vector3 {
    return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z);
  }

  subtract(v: Vector3): Vector3 {
    return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z);
  }

  dot(v: Vector3): number {
    return this.x * v.x + this.y * v.y + this.z * v.z;
  }

  applyMatrix4(m: Matrix4): Vector3 {
    return Vector3.from(this.toThree().applyMatrix4(m.toThree()));
  }

  cross(v: Vector3): Vector3 {
    // prettier-ignore
    return new Vector3(
      this.y * v.z - this.z * v.y,
      this.z * v.x - this.x * v.z,
      this.x * v.y - this.y * v.x,
    );
  }

  static fromThree(vec3: THREE.Vector3 | THREE.Euler): Vector3 {
    return new Vector3(vec3.x, vec3.y, vec3.z);
  }

  toThree(): THREE.Vector3 {
    return new THREE.Vector3(this.x, this.y, this.z);
  }

  toString() {
    return `(${this.x}, ${this.y}, ${this.z})`;
  }

  toArray(): [number, number, number] {
    return [this.x, this.y, this.z];
  }
}
