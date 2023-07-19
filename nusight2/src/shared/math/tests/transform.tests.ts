import { Transform } from "../transform";

describe("Transform", () => {
  describe("#then", () => {
    it("composes rotations", () => {
      const transform1 = Transform.of({ rotate: Math.PI });
      const transform2 = Transform.of({ rotate: Math.PI / 2 });

      const actual = transform1.then(transform2);
      const expected = Transform.of({ rotate: Math.PI * 1.5 });
      expect(actual).toEqual(expected);
    });

    it("composes scales", () => {
      const transform1 = Transform.of({ scale: { x: 2, y: 3 } });
      const transform2 = Transform.of({ scale: { x: 4, y: -5 } });

      const actual = transform1.then(transform2);
      const expected = Transform.of({ scale: { x: 8, y: -15 } });
      expect(actual).toEqual(expected);
    });

    it("composes translations", () => {
      const transform1 = Transform.of({ translate: { x: 2, y: 3 } });
      const transform2 = Transform.of({ translate: { x: 4, y: -5 } });

      const actual = transform1.then(transform2);
      const expected = Transform.of({ translate: { x: 6, y: -2 } });
      expect(actual).toEqual(expected);
    });

    it("composes rotations, scales and translations together", () => {
      const transform1 = Transform.of({
        rotate: Math.PI / 2,
        scale: { x: 2, y: 2 },
        translate: { x: 2, y: 1 },
      });
      const transform2 = Transform.of({
        rotate: Math.PI,
        scale: { x: 1, y: 1 },
        translate: { x: 1, y: -2 },
      });

      const actual = transform1.then(transform2);
      const expected = Transform.of({
        rotate: (3 * Math.PI) / 2,
        scale: { x: 2, y: 2 },
        translate: { x: 6, y: 3 },
      });
      expect(actual).toEqual(expected);
    });
  });

  it("can inverse correctly", () => {
    // Test this by transforming a transform by its inverse,
    // and making sure it results in the identity matrix.
    const transform = Transform.of({
      rotate: Math.PI / 2,
      scale: { x: 2, y: 3 },
      translate: { x: 10, y: -5 },
    });

    const result = transform.then(transform.inverse());
    expect(result.isIdentity()).toBe(true);
  });
});
