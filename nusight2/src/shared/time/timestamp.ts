import { Timestamp as TimestampMessage } from "@proto/google/protobuf/timestamp";

export type NbsTimestamp = { seconds: number; nanos: number };

type TimestampType = number | bigint | { seconds: number | bigint; nanos: number };
/**
 * A class that represents a Protobuf timestamp with nanosecond precision,
 * with methods to convert to and from various formats.
 *
 * The static conversion methods avoid allocating a new instance when converting,
 * and should be preferred unless an instance is also needed for other purposes.
 */
export class Timestamp implements NbsTimestamp {
  seconds: number;
  nanos: number;

  /**
   * Create a new Timestamp instance from a number of nanoseconds since
   * Unix epoch or from an object with seconds and nanos fields.
   */
  constructor(nanosOrTimestamp?: null | TimestampType) {
    if (nanosOrTimestamp === null || nanosOrTimestamp === undefined) {
      this.seconds = 0;
      this.nanos = 0;
    } else if (typeof nanosOrTimestamp === "bigint") {
      this.seconds = Number(nanosOrTimestamp / BigInt(1e9));
      this.nanos = Number(nanosOrTimestamp % BigInt(1e9));
    } else if (typeof nanosOrTimestamp === "number") {
      this.seconds = Math.floor(nanosOrTimestamp / 1e9);
      this.nanos = Math.floor(nanosOrTimestamp % 1e9);
    } else if (typeof nanosOrTimestamp.seconds === "bigint") {
      this.seconds = Number(nanosOrTimestamp.seconds);
      this.nanos = nanosOrTimestamp.nanos;
    } else {
      this.seconds = nanosOrTimestamp.seconds;
      this.nanos = nanosOrTimestamp.nanos;
    }
  }

  /** Create a new timestamp instance from a number of milliseconds since Unix epoch **/
  static fromMillis(value?: number | bigint | null): Timestamp {
    if (value === null || value === undefined) {
      return new Timestamp(0);
    }

    value = typeof value === "number" ? value : Number(value);

    const seconds = Math.floor(value / 1e3);
    const nanos = Math.floor((value * 1e6) % 1e9);

    return new Timestamp({ seconds, nanos });
  }

  /** Create a new Timestamp instance from a number of seconds since Unix epoch **/
  static fromSeconds(value?: number | bigint | null): Timestamp {
    if (value === null || value === undefined) {
      return new Timestamp(0);
    }

    value = typeof value === "number" ? value : Number(value);

    const seconds = Math.floor(value);
    const nanos = Math.floor((value * 1e9) % 1e9);

    return new Timestamp({ seconds, nanos });
  }

  /** Convert the Timestamp to a number of nanoseconds since Unix epoch **/
  toNanos(): bigint {
    return BigInt(this.seconds) * BigInt(1e9) + BigInt(this.nanos);
  }

  /** Convert the Timestamp to a number of milliseconds since Unix epoch **/
  toMillis(): number {
    return this.seconds * 1e3 + this.nanos * 1e-6;
  }

  /** Convert the Timestamp to a number of seconds since Unix epoch **/
  toSeconds(): number {
    return this.seconds + this.nanos * 1e-9;
  }

  /** Convert the Timestamp to a wrapped `google.protobuf.Timestamp` message **/
  toMessage(): TimestampMessage {
    return new TimestampMessage({
      seconds: BigInt(this.seconds),
      nanos: this.nanos,
    });
  }

  /** Convert the Timestamp to a JavaScript Date object **/
  toDate(): Date {
    return new Date(this.toMillis());
  }

  /** Convert the Timestamp to a string with the format `dd/mm/yyyy hh:mm:ss.mmm` **/
  toFormattedDate(): string {
    return formatDate(this.toNanos());
  }

  /** Convert the Timestamp to a duration string with the format `hh:mm:ss.mmm` **/
  toFormattedDuration(): string {
    return formatDuration(this.toNanos());
  }

  valueOf(): bigint {
    return this.toNanos();
  }

  /**
   * Convert the given timestamp object to a number of nanoseconds since Unix epoch
   * without creating a new Timestamp instance.
   */
  static toNanos(timestamp?: null | { seconds: number | bigint; nanos: number }): bigint {
    if (timestamp === null || timestamp === undefined) {
      return 0n;
    }

    return BigInt(timestamp.seconds) * BigInt(1e9) + BigInt(timestamp.nanos);
  }

  /**
   * Convert the given timestamp object to a number of milliseconds since Unix epoch
   * without creating a new Timestamp instance.
   */
  static toMillis(timestamp?: null | { seconds: number | bigint; nanos: number }): number {
    if (timestamp === null || timestamp === undefined) {
      return 0;
    }

    return Number(timestamp.seconds) * 1e3 + timestamp.nanos * 1e-6;
  }

  /**
   * Convert the given timestamp object to a number of milliseconds since Unix epoch
   * without creating a new Timestamp instance.
   */
  static toSeconds(timestamp?: null | { seconds: number | bigint; nanos: number }): number {
    if (timestamp === null || timestamp === undefined) {
      return 0;
    }

    return Number(timestamp.seconds) + timestamp.nanos * 1e-9;
  }

  /**
   * Convert the given timestamp object to a wrapped `google.protobuf.Timestamp` message
   * without creating a new Timestamp instance.
   */
  static toMessage(
    nanosOrTimestamp?: null | { seconds: number | bigint; nanos: number } | bigint | number,
  ): TimestampMessage {
    if (nanosOrTimestamp === null || nanosOrTimestamp === undefined) {
      return new TimestampMessage({ seconds: 0n, nanos: 0 });
    } else if (typeof nanosOrTimestamp === "bigint") {
      return new TimestampMessage({
        seconds: nanosOrTimestamp / BigInt(1e9),
        nanos: Number(nanosOrTimestamp % BigInt(1e9)),
      });
    } else if (typeof nanosOrTimestamp === "number") {
      return new TimestampMessage({
        seconds: BigInt(Math.floor(nanosOrTimestamp / 1e9)),
        nanos: Math.floor(nanosOrTimestamp % 1e9),
      });
    } else if (typeof nanosOrTimestamp.seconds === "number") {
      return new TimestampMessage({
        seconds: BigInt(nanosOrTimestamp.seconds),
        nanos: nanosOrTimestamp.nanos,
      });
    } else {
      return new TimestampMessage({ seconds: nanosOrTimestamp.seconds, nanos: nanosOrTimestamp.nanos });
    }
  }

  /**
   * Convert the given timestamp to a string with the format `dd/mm/yyyy hh:mm:ss.mmm`
   * without creating a new Timestamp instance.
   */
  static toFormattedDate(nanosOrTimestamp: TimestampType): string {
    return formatDate(
      typeof nanosOrTimestamp === "object" ? Timestamp.toNanos(nanosOrTimestamp) : BigInt(nanosOrTimestamp),
    );
  }

  /**
   * Convert the given timestamp to a duration string with the format `hh:mm:ss.mmm`
   * without creating a new Timestamp instance.
   */
  static toFormattedDuration(nanosOrTimestamp: TimestampType): string {
    return formatDuration(
      typeof nanosOrTimestamp === "object" ? Timestamp.toNanos(nanosOrTimestamp) : BigInt(nanosOrTimestamp),
    );
  }

  /**
   * Convert the given timestamp to a string with the given format
   * without creating a new Timestamp instance.
   */
  static format(nanosOrTimestamp: TimestampType, format: string): string {
    return formatDateCustom(
      typeof nanosOrTimestamp === "object" ? Timestamp.toNanos(nanosOrTimestamp) : BigInt(nanosOrTimestamp),
      format,
    );
  }
}

/** Format a number of nanoseconds (since Unix epoch) to `dd/mm/yyyy hh:mm:ss.mmm` */
function formatDate(nanos: bigint): string {
  const date = new Date(Number(nanos / BigInt(1e6)));

  const d = String(date.getDate()).padStart(2, "0");
  const m = String(date.getMonth() + 1).padStart(2, "0");
  const y = String(date.getFullYear()).padStart(4, "0");

  const hr = String(date.getHours()).padStart(2, "0");
  const min = String(date.getMinutes()).padStart(2, "0");
  const sec = String(date.getSeconds()).padStart(2, "0");
  const ms = String(date.getMilliseconds()).padStart(3, "0");

  return `${d}/${m}/${y} ${hr}:${min}:${sec}.${ms}`;
}

/** Format a number of nanoseconds to `hh:mm:ss.mmm` */
function formatDuration(nanos: bigint): string {
  const isNegative = nanos < 0n;
  const milliseconds = (nanos * (isNegative ? -1n : 1n)) / BigInt(1e6);

  const h = milliseconds / 1000n / 60n / 60n;
  const m = (milliseconds / 1000n / 60n) % 60n;
  const s = (milliseconds / 1000n) % 60n;
  const ms = milliseconds % 1000n;

  return `${isNegative ? "-" : ""}${padTo(h, 2)}:${padTo(m, 2)}:${padTo(s, 2)}.${padTo(ms, 3)}`;
}

function formatDateCustom(nanos: bigint, format: string): string {
  const dateValue = new Date(Number(nanos / BigInt(1e6)));

  const hour = String(dateValue.getHours());
  const minute = String(dateValue.getMinutes());
  const second = String(dateValue.getSeconds());
  const milliseconds = String(dateValue.getMilliseconds());

  const date = String(dateValue.getDate());
  const day = dateValue.toLocaleString("en-US", { weekday: "long" });
  const shortDay = dateValue.toLocaleString("en-US", { weekday: "short" });
  const month = String(dateValue.getMonth() + 1);
  const longMonth = dateValue.toLocaleString("en-US", { month: "long" });
  const shortMonth = dateValue.toLocaleString("en-US", { month: "short" });
  const year = String(dateValue.getFullYear());

  // Define patterns with their replacements, ordered by length descending to handle longer matches first
  const patterns = [
    ["hh", hour.padStart(2, "0")],
    ["h", hour],
    ["mm", minute.padStart(2, "0")],
    ["m", minute],
    ["ss", second.padStart(2, "0")],
    ["s", second],
    ["ffff", Number(milliseconds).toString().padStart(4, "0")],
    ["fff", milliseconds.padStart(3, "0")],
    [
      "ff",
      Math.floor(Number(milliseconds) / 10)
        .toString()
        .padStart(2, "0"),
    ],
    ["f", Math.floor(Number(milliseconds) / 100).toString()],
    ["dd", date.padStart(2, "0")],
    ["d", date],
    ["DD", day],
    ["D", shortDay],
    ["MMMM", longMonth],
    ["MMM", shortMonth],
    ["MM", month.padStart(2, "0")],
    ["M", month],
    ["YYYY", year.padStart(4, "0")],
    ["YY", year.slice(-2)],
  ];

  let result = format;
  for (const [pattern, replacement] of patterns) {
    // Replace only if it exists in the original format (to avoid replacing patterns that have already been replaced)
    if (format.includes(pattern)) {
      format = format.replace(pattern, "");
      result = result.replace(pattern, replacement);
    }
  }

  return result;
}

/** Pad from the start with 0, up to to the given width */
function padTo(value: bigint, width: number) {
  return String(value).padStart(width, "0");
}
