import { describe, expect, it } from "vitest";

import { hexToRGB } from "../rendering";

describe("hexToRGB", () => {
  it("converts hex colours to rgb", () => {
    expect(hexToRGB("#AABBCC")).toEqual({
      r: 0xaa,
      g: 0xbb,
      b: 0xcc,
    });
  });

  it("throws if not a hex colour", () => {
    expect(() => hexToRGB("#FFF")).toThrowErrorMatchingSnapshot();
    expect(() => hexToRGB("red")).toThrowErrorMatchingSnapshot();
  });
});
