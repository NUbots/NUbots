import * as THREE from "three";

import { Quaternion } from "./quaternion";
import { Vector3 } from "./vector3";
import { Vector4 } from "./vector4";

export class Matrix4 {
  constructor(
    readonly x: Vector4,
    readonly y: Vector4,
    readonly z: Vector4,
    readonly t: Vector4,
  ) {}

  static of() {
    return new Matrix4(
      new Vector4(1, 0, 0, 0),
      new Vector4(0, 1, 0, 0),
      new Vector4(0, 0, 1, 0),
      new Vector4(0, 0, 0, 1),
    );
  }

  static from(
    mat?: {
      x?: { x?: number | null; y?: number | null; z?: number | null; t?: number | null } | null;
      y?: { x?: number | null; y?: number | null; z?: number | null; t?: number | null } | null;
      z?: { x?: number | null; y?: number | null; z?: number | null; t?: number | null } | null;
      t?: { x?: number | null; y?: number | null; z?: number | null; t?: number | null } | null;
    } | null,
  ): Matrix4 {
    if (!mat) {
      return Matrix4.of();
    }
    return new Matrix4(Vector4.from(mat.x), Vector4.from(mat.y), Vector4.from(mat.z), Vector4.from(mat.t));
  }

  get trace(): number {
    return this.x.x + this.y.y + this.z.z + this.t.t;
  }

  multiply(m: Matrix4): Matrix4 {
    return Matrix4.fromThree(this.toThree().multiply(m.toThree()));
  }

  invert(): Matrix4 {
    return Matrix4.fromThree(this.toThree().invert());
  }

  decompose(): {
    translation: Vector3;
    rotation: Quaternion;
    scale: Vector3;
  } {
    const translation = new THREE.Vector3();
    const rotation = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    this.toThree().decompose(translation, rotation, scale);
    return {
      translation: Vector3.fromThree(translation),
      rotation: Quaternion.fromThree(rotation),
      scale: Vector3.fromThree(scale),
    };
  }

  static fromThree(mat4: THREE.Matrix4) {
    return new Matrix4(
      new Vector4(mat4.elements[0], mat4.elements[1], mat4.elements[2], mat4.elements[3]),
      new Vector4(mat4.elements[4], mat4.elements[5], mat4.elements[6], mat4.elements[7]),
      new Vector4(mat4.elements[8], mat4.elements[9], mat4.elements[10], mat4.elements[11]),
      new Vector4(mat4.elements[12], mat4.elements[13], mat4.elements[14], mat4.elements[15]),
    );
  }

  toThree(): THREE.Matrix4 {
    // prettier-ignore
    return new THREE.Matrix4().set(
      this.x.x, this.y.x, this.z.x, this.t.x,
      this.x.y, this.y.y, this.z.y, this.t.y,
      this.x.z, this.y.z, this.z.z, this.t.z,
      this.x.t, this.y.t, this.z.t, this.t.t,
    )
  }

  toString() {
    return [
      `${format(this.x.x)} ${format(this.y.x)} ${format(this.z.x)} ${format(this.t.x)}`,
      `${format(this.x.y)} ${format(this.y.y)} ${format(this.z.y)} ${format(this.t.y)}`,
      `${format(this.x.z)} ${format(this.y.z)} ${format(this.z.z)} ${format(this.t.z)}`,
      `${format(this.x.t)} ${format(this.y.t)} ${format(this.z.t)} ${format(this.t.t)}`,
    ].join("\n");
  }
}

const format = (x: number) => x.toFixed(2).padStart(7);
