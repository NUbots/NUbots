import * as THREE from "three";

export class Quaternion {
  constructor(readonly x: number, readonly y: number, readonly z: number, readonly w: number) {}

  static of() {
    return new Quaternion(0, 0, 0, 1);
  }

  static from(q?: { x?: number | null; y?: number | null; z?: number | null; w?: number | null } | null): Quaternion {
    if (!q) {
      return Quaternion.of();
    }
    return new Quaternion(q.x || 0, q.y || 0, q.z || 0, q.w || 0);
  }

  static fromThree(q: THREE.Quaternion) {
    return new Quaternion(q.x, q.y, q.z, q.w);
  }

  toThree() {
    return new THREE.Quaternion(this.x, this.y, this.z, this.w);
  }
}
