import { NbsTimestamp } from "nbsdecoder.js";

/** Convert the given NBS timestamp object to a BigInt of nanoseconds */
export function timestampObjectToNanos(timestamp: NbsTimestamp): bigint {
  return BigInt(timestamp.seconds) * BigInt(1e9) + BigInt(timestamp.nanos);
}

/** Convert the given nanoseconds to an NBS timestamp object */
export function nanosToTimestampObject(nanos: bigint): NbsTimestamp {
  return {
    seconds: Number(nanos / BigInt(1e9)),
    nanos: Number(nanos % BigInt(1e9)),
  };
}
