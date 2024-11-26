import { describe, expect, it } from "vitest";
import { describe, expect, it } from "vitest";
import { hashType } from "../hash_type";

describe("hashType()", () => {
  it("matches previous snapshots", () => {
    expect(hashType("foo")).toMatchSnapshot();
    expect(hashType("bar")).toMatchSnapshot();
    expect(hashType("baz")).toMatchSnapshot();
    expect(hashType("cheesecake")).toMatchSnapshot();
  });

  it("ensures it returns 8 bytes", () => {
    expect(hashType("foo").byteLength).toBe(8);
    expect(hashType("bar").byteLength).toBe(8);
    expect(hashType("baz").byteLength).toBe(8);
    // "cheesecake" happens to hash to 7 bytes without padding.
    expect(hashType("cheesecake").byteLength).toBe(8);
  });
});
