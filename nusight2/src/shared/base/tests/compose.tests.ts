import { describe, expect, it } from "vitest";
import { describe, expect, it, vi } from "vitest";

import { compose } from "../compose";

describe("compose", () => {
  it("returns a function which calls all functions", () => {
    const a = vi.fn();
    const b = vi.fn();
    const c = vi.fn();
    const f = compose([a, b, c]);
    f();
    expect(a).toBeCalledTimes(1);
    expect(b).toBeCalledTimes(1);
    expect(c).toBeCalledTimes(1);
  });

  it("calls functions in the order they are given", () => {
    const values: string[] = [];
    const a = () => values.push("a");
    const b = () => values.push("b");
    const c = () => values.push("c");
    const f = compose([a, b, c]);
    f();
    expect(values).toEqual(["a", "b", "c"]);
  });
});
