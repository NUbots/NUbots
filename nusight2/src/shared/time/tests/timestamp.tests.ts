import { beforeEach, describe, expect, it } from "vitest";
import { beforeEach, describe, expect, it } from "vitest";
import { SeededRandom } from "../../../shared/base/random/seeded_random";
import { TimestampObject } from "../timestamp";

describe("TimestampObject", () => {
  let random: SeededRandom;

  beforeEach(() => {
    random = SeededRandom.of("timestamp");
  });

  it("converts seconds to timestamp objects", () => {
    const timestamp = TimestampObject.fromSeconds(1000.0001);
    expect(timestamp.seconds).toBeCloseTo(1000);
    expect(timestamp.nanos).toBeCloseTo(100000);
  });

  it("converts timestamp objects to seconds", () => {
    const seconds = TimestampObject.toSeconds({
      seconds: 12345,
      nanos: 54321,
    });
    expect(seconds).toBeCloseTo(12345.000054321);
  });

  it("performs roundtrips between timestamp objects and seconds", () => {
    for (let i = 0; i < 10; i++) {
      const t = random.float() * 1e5;
      expect(TimestampObject.toSeconds(TimestampObject.fromSeconds(t))).toBeCloseTo(t);
    }
  });

  it("converts milliseconds to timestamp objects", () => {
    const timestamp = TimestampObject.fromMillis(1000000.0001);
    expect(timestamp.seconds).toBeCloseTo(1000);
    expect(timestamp.nanos).toBeCloseTo(100);
  });

  it("converts timestamp objects to seconds", () => {
    const milliseconds = TimestampObject.toMillis({
      seconds: 123,
      nanos: 54321224,
    });
    expect(milliseconds).toBeCloseTo(123054.321224);
  });

  it("performs roundtrips between timestamp objects and milliseconds", () => {
    for (let i = 0; i < 10; i++) {
      const t = random.float() * 1e8;
      expect(TimestampObject.toMillis(TimestampObject.fromMillis(t))).toBeCloseTo(t);
    }
  });

  it("converts nanoseconds to timestamp objects", () => {
    const timestamp = TimestampObject.fromNanos(1234000000123n);
    expect(timestamp.seconds).toBe(1234);
    expect(timestamp.nanos).toBe(123);
  });

  it("converts timestamp objects to nanoseconds", () => {
    const nanoseconds = TimestampObject.toNanos({
      seconds: 12653,
      nanos: 144151545,
    });
    expect(nanoseconds).toBe(12653144151545n);
  });

  it("performs roundtrips between timestamp objects and nanoseconds", () => {
    for (let i = 0; i < 10; i++) {
      const t = BigInt(Math.floor(random.float() * 1e13));
      expect(TimestampObject.toNanos(TimestampObject.fromNanos(t))).toBe(t);
    }
  });
});
