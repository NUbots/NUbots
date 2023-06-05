import { NbsTimestamp } from "nbsdecoder.js";

/** Convert the given timestamp object to a BigInt of nanoseconds */
export function timestampObjectToNanos(timestamp: NbsTimestamp): bigint {
  return BigInt(timestamp.seconds) * BigInt(1e9) + BigInt(timestamp.nanos);
}

/** Convert the given BigInt of nanoseconds to a timestamp object */
export function nanosToTimestampObject(nanos: bigint): NbsTimestamp {
  return {
    seconds: Number(nanos / BigInt(1e9)),
    nanos: Number(nanos % BigInt(1e9)),
  };
}

/** Convert the given percentage to a timestamp in nanoseconds */
export function percentageToTimestamp(percentage: number, rangeStart: bigint, rangeEnd: bigint): bigint {
  return rangeStart + BigInt(Math.floor(percentage * Number(rangeEnd - rangeStart)));
}

/** Convert the given timestamp in nanoseconds to a percentage of the given range */
export function timestampToPercentage(timestamp: bigint, rangeStart: bigint, rangeEnd: bigint): number {
  return Number(timestamp - rangeStart) / Number(rangeEnd - rangeStart);
}
