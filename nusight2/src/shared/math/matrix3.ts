import * as THREE from "three";

import { Vector3 } from "./vector3";

export class Matrix3 {
  constructor(
    readonly x: Vector3,
    readonly y: Vector3,
    readonly z: Vector3,
  ) {}

  static of() {
    return new Matrix3(new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1));
  }

  static from(
    mat?: {
      x?: { x?: number | null; y?: number | null; z?: number | null } | null;
      y?: { x?: number | null; y?: number | null; z?: number | null } | null;
      z?: { x?: number | null; y?: number | null; z?: number | null } | null;
    } | null,
  ): Matrix3 {
    if (!mat) {
      return Matrix3.of();
    }
    return new Matrix3(Vector3.from(mat.x), Vector3.from(mat.y), Vector3.from(mat.z));
  }

  static Identity() {
    return new Matrix3(new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1));
  }

  get trace(): number {
    return this.x.x + this.y.y + this.z.z;
  }

  multiply(m: Matrix3): Matrix3 {
    return Matrix3.fromThree(this.toThree().multiply(m.toThree()));
  }

  invert(): Matrix3 {
    return Matrix3.fromThree(this.toThree().invert());
  }

  transpose() {
    return Matrix3.fromThree(this.toThree().transpose());
  }

  static fromThree(mat4: THREE.Matrix3) {
    return new Matrix3(
      new Vector3(mat4.elements[0], mat4.elements[1], mat4.elements[2]),
      new Vector3(mat4.elements[3], mat4.elements[4], mat4.elements[5]),
      new Vector3(mat4.elements[6], mat4.elements[7], mat4.elements[8]),
    );
  }

  toThree(): THREE.Matrix3 {
    // prettier-ignore
    return new THREE.Matrix3().set(
      this.x.x, this.y.x, this.z.x,
      this.x.y, this.y.y, this.z.y,
      this.x.z, this.y.z, this.z.z,
    )
  }

  toString() {
    return [
      `${format(this.x.x)} ${format(this.y.x)} ${format(this.z.x)}`,
      `${format(this.x.y)} ${format(this.y.y)} ${format(this.z.y)}`,
      `${format(this.x.z)} ${format(this.y.z)} ${format(this.z.z)}`,
    ].join("\n");
  }

  toArray() {
    // prettier-ignore
    return [
      this.x.x, this.x.y, this.x.z,
      this.y.x, this.y.y, this.y.z,
      this.z.x, this.z.y, this.z.z,
    ]
  }
}

const format = (x: number) => x.toFixed(2).padStart(7);
