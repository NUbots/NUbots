import { describe, expect, it } from "vitest";

import { parseEventString } from "../parse_event_string";

describe.only("parseEventString()", () => {
  it("parses event without subtype", () => {
    expect(parseEventString("my.type")).toStrictEqual({
      type: "my.type",
      subtype: undefined,
    });

    expect(parseEventString("my.type#")).toStrictEqual({
      type: "my.type",
      subtype: undefined,
    });
  });

  it("parses event with valid subtype", () => {
    expect(parseEventString("my.type#0")).toStrictEqual({
      type: "my.type",
      subtype: 0,
    });

    expect(parseEventString("my.type#20")).toStrictEqual({
      type: "my.type",
      subtype: 20,
    });
  });

  it("throws for event with invalid subtype by default", () => {
    expect(() => {
      parseEventString("my.type#not-a-number");
    }).toThrowErrorMatchingSnapshot();
  });

  it("returns empty subtype for event with invalid subtype when configured to not throw", () => {
    expect(parseEventString("my.type#not-a-number", { throwOnInvalidSubtype: false })).toStrictEqual({
      type: "my.type",
      subtype: undefined,
    });
  });
});
