import { google } from "../messages";

import ITimestamp = google.protobuf.ITimestamp;

export type NbsTimestamp = { seconds: number; nanos: number };

/** Pad from the start with 0, up to to the given width */
function padTo(value: bigint, width: number) {
  return String(value).padStart(width, "0");
}

/** Simple container with helper methods for converting between NBS/Protobuf timestamps and time units */
export class TimestampObject {
  /** Convert the given NBS timestamp object to a float value of seconds */
  static toSeconds(timestamp?: ITimestamp | null): number {
    return (timestamp?.seconds ?? 0) + (timestamp?.nanos ?? 0) * 1e-9;
  }

  /** Convert a float value of seconds to an NBS timestamp object */
  static fromSeconds(seconds: number): NbsTimestamp {
    return {
      seconds: Math.floor(seconds),
      nanos: Math.floor((seconds * 1e9) % 1e9),
    };
  }

  /** Convert the given NBS timestamp object to a float value of milliseconds */
  static toMillis(timestamp?: ITimestamp | null): number {
    return (timestamp?.seconds ?? 0) * 1e3 + (timestamp?.nanos ?? 0) * 1e-6;
  }

  /** Convert a float value of milliseconds to an NBS timestamp object */
  static fromMillis(millis: number): NbsTimestamp {
    return {
      seconds: Math.floor(millis / 1e3),
      nanos: Math.floor((millis * 1e6) % 1e9),
    };
  }

  /** Convert the given NBS timestamp object to a bigint of nanoseconds */
  static toNanos(timestamp?: ITimestamp | null): bigint {
    return BigInt(timestamp?.seconds ?? 0) * BigInt(1e9) + BigInt(timestamp?.nanos ?? 0);
  }

  /** Convert a nanoseconds bigint to an NBS timestamp object */
  static fromNanos(nanos: bigint): NbsTimestamp {
    return {
      seconds: Number(nanos / BigInt(1e9)),
      nanos: Number(nanos % BigInt(1e9)),
    };
  }

  /** Format a number of nanoseconds (since Unix epoch) to 'dd/mm/yyyy hh:mm:ss.mmm' */
  static formatDate(nanos: bigint): string {
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

  /** Format a number of nanoseconds to 'hh:mm:ss.mmm' */
  static formatDuration(nanos: bigint): string {
    const isNegative = nanos < 0n;
    const milliseconds = (nanos * (isNegative ? -1n : 1n)) / BigInt(1e6);

    const h = milliseconds / 1000n / 60n / 60n;
    const m = (milliseconds / 1000n / 60n) % 60n;
    const s = (milliseconds / 1000n) % 60n;
    const ms = milliseconds % 1000n;

    return `${isNegative ? "-" : ""}${padTo(h, 2)}:${padTo(m, 2)}:${padTo(s, 2)}.${padTo(ms, 3)}`;
  }
}
