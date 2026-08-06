import { Timestamp as TimestampMessage } from "@proto/google/protobuf/timestamp";
import { beforeEach, describe, expect, it } from "vitest";

import { SeededRandom } from "../../../shared/base/random/seeded_random";
import { Timestamp } from "../timestamp";

describe("Timestamp", () => {
  let random: SeededRandom;

  beforeEach(() => {
    random = SeededRandom.of("timestamp");
  });

  it("constructs Timestamp from null or undefined", () => {
    // null
    expect(new Timestamp(null)).toEqual({ seconds: 0, nanos: 0 });

    // undefined
    expect(new Timestamp(undefined)).toEqual({ seconds: 0, nanos: 0 });
  });

  it("constructs Timestamp from a number or bigint of nanoseconds since epoch", () => {
    // Number
    expect(new Timestamp(10_000_000_000)).toEqual({ seconds: 10, nanos: 0 });

    // BigInt
    expect(new Timestamp(10_000_000_000n)).toEqual({ seconds: 10, nanos: 0 });
  });

  it("constructs Timestamp from an object with seconds and nanos fields", () => {
    // Where seconds is a number
    expect(new Timestamp({ seconds: 10, nanos: 100 })).toEqual({ seconds: 10, nanos: 100 });

    // Where seconds is a bigint
    expect(new Timestamp({ seconds: 10n, nanos: 100 })).toEqual({ seconds: 10, nanos: 100 });
  });

  it("constructs Timestamp from a number of milliseconds since epoch", () => {
    // BigInt
    expect(Timestamp.fromMillis(10_000n)).toEqual({ seconds: 10, nanos: 0 });

    // Number
    expect(Timestamp.fromMillis(10_000)).toEqual({ seconds: 10, nanos: 0 });

    // Fractional
    expect(Timestamp.fromMillis(10_000.001)).toEqual({ seconds: 10, nanos: 1000 });
  });

  it("constructs Timestamp from a number of seconds since epoch", () => {
    // BigInt
    expect(Timestamp.fromSeconds(10n)).toEqual({ seconds: 10, nanos: 0 });

    // Number
    expect(Timestamp.fromSeconds(10)).toEqual({ seconds: 10, nanos: 0 });

    // Fractional
    expect(Timestamp.fromSeconds(10.001)).toEqual({ seconds: 10, nanos: 1_000_000 });
  });

  it("roundtrips nanoseconds", () => {
    for (let i = 0; i < 10; i++) {
      const t = BigInt(Math.floor(random.float() * 1e13));
      expect(new Timestamp(t).toNanos()).toBe(t);
    }
  });

  it("roundtrips milliseconds", () => {
    for (let i = 0; i < 10; i++) {
      const t = random.float() * 1e8;
      expect(Timestamp.fromMillis(t).toMillis()).toBeCloseTo(t);
    }
  });

  it("roundtrips seconds", () => {
    for (let i = 0; i < 10; i++) {
      const t = random.float() * 1e5;
      expect(Timestamp.fromSeconds(t).toSeconds()).toBeCloseTo(t);
    }
  });

  it("can be converted to a Timestamp message", () => {
    const timestamp = new Timestamp({ seconds: 10, nanos: 200 });
    const message = timestamp.toMessage();

    expect(message).toBeInstanceOf(TimestampMessage);
    expect(message).toEqual({
      $typeName: TimestampMessage.typeName,
      seconds: 10n,
      nanos: 200,
    });
  });

  it("can be converted to a formatted date string", () => {
    const timestamp = Timestamp.fromMillis(new Date(2025, 0, 1, 12, 30, 10, 200).getTime());
    expect(timestamp.toFormattedDate()).toBe("01/01/2025 12:30:10.200");
  });

  it("can be converted to a formatted duration string", () => {
    const timestamp = new Timestamp({ seconds: 3_950, nanos: 2_000_000_00 });
    expect(timestamp.toFormattedDuration()).toBe("01:05:50.200");
  });

  it("roundtrips nanoseconds without constructing a temporary Timestamp instance", () => {
    const timestamp = { seconds: 10, nanos: 0 };
    const t = BigInt(Math.floor(timestamp.seconds * 1e9));

    expect(Timestamp.toNanos(timestamp)).toBe(t);
  });

  it("roundtrips milliseconds without constructing a temporary Timestamp instance", () => {
    const timestamp = { seconds: 10, nanos: 0 };
    const t = timestamp.seconds * 1e3;

    expect(Timestamp.toMillis(timestamp)).toBe(t);
  });

  it("roundtrips seconds without constructing a temporary Timestamp instance", () => {
    const timestamp = { seconds: 10, nanos: 0 };
    const t = timestamp.seconds;

    expect(Timestamp.toSeconds(timestamp)).toBe(t);
  });

  it("can be converted to a Timestamp message without constructing a temporary Timestamp instance", () => {
    const timestamp = { seconds: 10, nanos: 200 };
    const message1 = Timestamp.toMessage(timestamp);

    const nanos = 2_000_000_150n;
    const message2 = Timestamp.toMessage(nanos);

    const nanosInNumber = 2_000_000_500;
    const message3 = Timestamp.toMessage(nanosInNumber);

    expect(message1).toBeInstanceOf(TimestampMessage);
    expect(message1).toEqual({
      $typeName: TimestampMessage.typeName,
      seconds: 10n,
      nanos: 200,
    });

    expect(message2).toBeInstanceOf(TimestampMessage);
    expect(message2).toEqual({
      $typeName: TimestampMessage.typeName,
      seconds: 2n,
      nanos: 150,
    });

    expect(message3).toEqual({
      $typeName: TimestampMessage.typeName,
      seconds: 2n,
      nanos: 500,
    });
  });

  it("can be converted to a formatted date string without constructing a temporary Timestamp instance", () => {
    const nanos = new Date(2025, 0, 1, 12, 30, 10, 200).getTime() * 1e6;
    expect(Timestamp.toFormattedDate(nanos)).toBe("01/01/2025 12:30:10.200");

    const timestamp = Timestamp.fromMillis(nanos / 1e6);
    expect(Timestamp.toFormattedDate(timestamp)).toBe("01/01/2025 12:30:10.200");
  });

  it("can be converted to a formatted duration string without constructing a temporary Timestamp instance", () => {
    const timestamp = new Timestamp({ seconds: 3_950, nanos: 2_000_000_00 });
    expect(Timestamp.toFormattedDuration(timestamp)).toBe("01:05:50.200");

    const nanos = Timestamp.toNanos(timestamp);
    expect(Timestamp.toFormattedDuration(nanos)).toBe("01:05:50.200");
  });

  it("can be compared to other timestamps", () => {
    const a = new Timestamp({ seconds: 10, nanos: 0 });
    const b = new Timestamp({ seconds: 20, nanos: 0 });

    expect(a < b).toBe(true);
    expect(a > b).toBe(false);
    expect(a <= b).toBe(true);
    expect(a >= b).toBe(false);

    expect(b < a).toBe(false);
    expect(b > a).toBe(true);
    expect(b <= a).toBe(false);
    expect(b >= a).toBe(true);
  });
});
