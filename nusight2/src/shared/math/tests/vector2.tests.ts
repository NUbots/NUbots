import { describe, expect, it } from "vitest";
import { describe, expect, it } from "vitest";

import { Transform } from "../transform";
import { Vector2 } from "../vector2";

describe("Vector2", () => {
  it("should apply transform rotate correctly", () => {
    const vec2 = Vector2.of(2, 3);
    const transform = Transform.of({ rotate: Math.PI });

    const actual = vec2.transform(transform);
    const expected = Vector2.of(-2, -3);
    expect(actual.x).toBeCloseTo(expected.x);
    expect(actual.y).toBeCloseTo(expected.y);
  });

  it("should apply transform scale correctly", () => {
    const vec2 = Vector2.of(2, 3);
    const transform = Transform.of({ scale: Vector2.of(2, 2) });

    const actual = vec2.transform(transform);
    const expected = Vector2.of(4, 6);
    expect(actual.x).toBeCloseTo(expected.x);
    expect(actual.y).toBeCloseTo(expected.y);
  });

  it("should apply transform translate correctly", () => {
    const vec2 = Vector2.of(2, 3);
    const transform = Transform.of({ translate: { x: 1, y: 1 } });

    const actual = vec2.transform(transform);
    const expected = Vector2.of(3, 4);
    expect(actual.x).toBeCloseTo(expected.x);
    expect(actual.y).toBeCloseTo(expected.y);
  });

  it("should apply all transforms correctly", () => {
    const vec2 = Vector2.of(2, 3);
    const transform = Transform.of({
      rotate: Math.PI,
      scale: Vector2.of(2, 2),
      translate: { x: 1, y: 1 },
    });

    const actual = vec2.transform(transform);
    const expected = Vector2.of(-3, -5);
    expect(actual.x).toBeCloseTo(expected.x);
    expect(actual.y).toBeCloseTo(expected.y);
  });

  it("should multiply scalar correctly", () => {
    const vec1 = Vector2.of(1, 4);
    const scalar = 2;

    const result = vec1.multiplyScalar(scalar);
    const expected = Vector2.of(2, 8);

    expect(result.x).toBe(expected.x);
    expect(result.y).toBe(expected.y);
  });

  it("should interpolate correctly", () => {
    const vec1 = Vector2.of(1, 4);
    const vec2 = Vector2.of(3, 8);

    const result = vec1.lerpTo(vec2, 0.5);
    const expected = Vector2.of(2, 6);
    expect(result.x).toBe(expected.x);
    expect(result.y).toBe(expected.y);

    const result2 = vec1.lerpTo(vec2, 0);
    expect(result2.x).toBe(vec1.x);
    expect(result2.y).toBe(vec1.y);

    const result3 = vec1.lerpTo(vec2, 1);
    expect(result3.x).toBe(vec2.x);
    expect(result3.y).toBe(vec2.y);
  });

  it("should slerp correctly", () => {
    //
    const vec1 = Vector2.of(1, 0);
    const vec2 = Vector2.of(0, 1);

    const result = vec1.slerpTo(vec2, 0.5);
    const expected = Vector2.of(1, 0).transform(Transform.of({ rotate: Math.PI / 4 }));
    expect(result.x).toBeCloseTo(expected.x, 3);
    expect(result.y).toBeCloseTo(expected.y, 3);

    const result2 = vec1.slerpTo(vec2, 0);
    expect(result2.x).toBe(vec1.x);
    expect(result2.y).toBe(vec1.y);

    const result3 = vec1.slerpTo(vec2, 1);
    expect(result3.x).toBe(vec2.x);
    expect(result3.y).toBe(vec2.y);
  });
});
