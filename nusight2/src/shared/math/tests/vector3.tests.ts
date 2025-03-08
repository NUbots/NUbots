import * as THREE from "three";
import { describe, expect, it } from "vitest";

import { Vector3 } from "../vector3";

describe("Vector3", () => {
  it("can be constructed from x, y, z parameters", () => {
    const vec3 = Vector3.of(1, 2, 3);
    expect(vec3).toEqual({ x: 1, y: 2, z: 3 });
  });

  it("can be constructed from an object with x, y, z keys", () => {
    const vec3 = Vector3.from({ x: 1, y: 2, z: 3 });
    expect(vec3).toEqual({ x: 1, y: 2, z: 3 });
  });

  it("can be constructed from a single scalar", () => {
    const vec3 = Vector3.fromScalar(1);
    expect(vec3).toEqual({ x: 1, y: 1, z: 1 });
  });

  it("can be constructed from THREE.Vector3", () => {
    const threeVec3 = new THREE.Vector3(1, 2, 3);
    const vec3 = Vector3.fromThree(threeVec3);
    expect(vec3).toEqual({ x: 1, y: 2, z: 3 });
  });

  it("can be converted to THREE.Vector3", () => {
    const vec3 = Vector3.of(1, 2, 3);
    const threeVec3 = vec3.toThree();
    expect(threeVec3).toEqual({ x: 1, y: 2, z: 3 });
  });

  it("can be converted to string", () => {
    const vec3 = Vector3.of(1, 2, 3);
    expect(vec3.toString()).toBe("(1, 2, 3)");
  });

  it("can be converted to array", () => {
    const vec3 = Vector3.of(1, 2, 3);
    expect(vec3.toArray()).toEqual([1, 2, 3]);
  });

  it("copy() produces a new vector with the same values", () => {
    const vec3 = Vector3.of(1, 2, 3);
    const copy = vec3.copy();

    expect(copy.x).toBe(vec3.x);
    expect(copy.y).toBe(vec3.y);
    expect(copy.z).toBe(vec3.z);

    expect(copy).not.toBe(vec3);
  });

  it("multiplyScalar() produces a new vector with each component multiplied by a scalar", () => {
    const vec3 = Vector3.of(1, 2, 3);
    const result = vec3.multiplyScalar(2);

    // The result should be correct
    expect(result).toEqual({ x: 2, y: 4, z: 6 });

    // The original vector should not be modified
    expect(vec3).toEqual({ x: 1, y: 2, z: 3 });
  });

  it("divideScalar() produces a new vector with each component divided by a scalar", () => {
    const vec3 = Vector3.of(2, 4, 6);
    const result = vec3.divideScalar(2);

    // The result should be correct
    expect(result).toEqual({ x: 1, y: 2, z: 3 });

    // The original vectors should not be modified
    expect(vec3).toEqual({ x: 2, y: 4, z: 6 });
    expect(vec3.divideScalar(0)).toEqual({ x: 0, y: 0, z: 0 });
  });

  it("add() produces a new vector by adding the components of another vector", () => {
    const vec3 = Vector3.of(1, 2, 3);
    const other = Vector3.of(1, 2, 3);
    const result = vec3.add(other);

    // The result should be correct
    expect(result).toEqual({ x: 2, y: 4, z: 6 });

    // The original vectors should not be modified
    expect(vec3).toEqual({ x: 1, y: 2, z: 3 });
    expect(other).toEqual({ x: 1, y: 2, z: 3 });
  });

  it("subtract() produces a new vector by adding the components of another vector", () => {
    const vec3 = Vector3.of(3, 3, 3);
    const other = Vector3.of(2, 1, 0);
    const result = vec3.subtract(other);

    // The result should be correct
    expect(result).toEqual({ x: 1, y: 2, z: 3 });

    // The original vectors should not be modified
    expect(vec3).toEqual({ x: 3, y: 3, z: 3 });
    expect(other).toEqual({ x: 2, y: 1, z: 0 });
  });

  it("dot() produces the dot product of the vector and another vector", () => {
    const vec3 = Vector3.of(1, 2, 3);
    const other = Vector3.of(4, 5, 6);
    const result = vec3.dot(other);

    expect(result).toBe(32);
  });

  it("cross() produces a new vector that is the cross product of the vector and another vector", () => {
    const vec3 = Vector3.of(1, 0, 0);
    const other = Vector3.of(0, 1, 0);
    const result = vec3.cross(other);

    // The result should be correct
    expect(result).toEqual({ x: 0, y: 0, z: 1 });

    // The original vectors should not be modified
    expect(vec3).toEqual({ x: 1, y: 0, z: 0 });
    expect(other).toEqual({ x: 0, y: 1, z: 0 });
  });
});
